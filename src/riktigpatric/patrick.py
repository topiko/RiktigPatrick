"""
This is the module that contains RiktigPatrick!
"""

import time
import logging

import numpy as np
import pandas as pd

from riktigpatric.servo import Servo
from relay.conversions import make_ctrl
from relay.conversions import depack
from relay.conversions import np2bytes
from filters.mahony import Mahony


LOG = logging.getLogger('rp_logger')

SPEEDSCALE = 5
MINTHETA = -28
MAXTHETA = 50
MINPHI = -40
MAXPHI = 40

htheta_params = {'name':'head_theta',
                 'max_speed': (MAXTHETA - MINTHETA)*SPEEDSCALE, #[deg/sec]
                 'minlim': MINTHETA,
                 'maxlim': MAXTHETA,
                 'idx':1,
                 'a':-15,
                 'b':1611,
                 }

hphi_params = {'name':'head_phi',
               'max_speed': (MAXPHI-MINPHI)*SPEEDSCALE, #[deg/sec]
               'minlim': MINPHI,
               'maxlim': MAXPHI,
               'idx':0,
               'a': -13,
               'b':1664,
               }


class RPHead():

    def __init__(self):
        self.phiservo = Servo(**hphi_params)
        self.thetaservo = Servo(**htheta_params)
        self._servo_init = 0
        self._servosinited = False

    @property
    def state(self):
        return {'phiservo':self.phiservo.state,
                'thetaservo':self.thetaservo.state,
                'servo_init':self._servo_init}

    @property
    def servo_init_cmds(self) -> list:
        cmd_tuples = list(self.thetaservo.init_dict.values()) \
                + list(self.phiservo.init_dict.values())

        return [make_ctrl(*t) for t in cmd_tuples]

    @property
    def target_phi(self):
        return self.phiservo.target_angle

    @target_phi.setter
    def target_phi(self, tandangle):
        self.phiservo.target_angle = tandangle

    @property
    def target_theta(self):
        return self.thetaservo.target_angle

    @target_theta.setter
    def target_theta(self, tandangle):
        self.thetaservo.target_angle = tandangle

    @property
    def pulse_phi(self):
        return self.phiservo.pulse

    @pulse_phi.setter
    def pulse_phi(self, pulse):
        self.phiservo.pulse = pulse

    @property
    def pulse_theta(self):
        return self.thetaservo.pulse

    @pulse_theta.setter
    def pulse_theta(self, pulse):
        self.thetaservo.pulse = pulse

    def get_ctrl(self, deltaT=None, phispeed=None, thetaspeed=None):
        phispeed = self.phiservo.speed2int(deltaT)
        thetaspeed = self.thetaservo.speed2int(deltaT)
        return make_ctrl(16, phispeed, thetaspeed)

    @property
    def asarray(self):
        return np.concatenate((self.phiservo.asarray,
                               self.thetaservo.asarray))

    @property
    def arrayheader(self):
        return self.phiservo.arrayheader + self.thetaservo.arrayheader

    def __repr__(self):
        add = '  '
        repr_  = 'Head:\n'
        repr_ += f'Phiservo\n'
        repr_ += self.phiservo.__repr__().replace('\n', f'\n{add*2}')
        repr_ += f'\nThetaservo\n'
        repr_ += self.thetaservo.__repr__().replace('\n', f'\n{add*2}')
        repr_ += '\n'
        return repr_

class RPatrick():

    def __init__(self, report_sock=None):

        self.head = RPHead()
        self._ahrs = Mahony()

        self.imu_a = np.zeros(3)
        self.imu_w = np.zeros(3)
        self.rpy  = np.zeros(3) # ahrs roll, pitch, yaw
        self.dt = 0
        self.dt_mean = 0
        self.rptime = 0
        self.rpmode = 0
        self._target_mode = 0
        self._count = 0
        self._mode = 0
        self.mytime = time.time()
        self.report_sock = report_sock

        self._n_collect = 100
        if self._n_collect > -1:
            self.df = pd.DataFrame(data=np.zeros((self._n_collect, len(self.arrayheader))),
                                columns=self.arrayheader)

    def __repr__(self):
        add = '  '
        repr_  = 'RiktigPatrick:\n'
        for k,v in self.state.items():
            if k=='head': continue
            repr_ += f'{add}{k} : {v}\n'
        repr_ += self.head.__repr__().replace('\n', f'\n{add}')
        return repr_

    @property
    def asarray(self):
        return np.concatenate((np.array([self.rptime, self.mytime]),
                               self.imu_a,
                               self.imu_w,
                               self.rpy,
                               self.head.asarray))

    @property
    def arrayheader(self):
        xyz = 'xyz'
        timehead = ['rptime', 'mytime']
        ahead = [f'a_{k}' for k in xyz]
        whead = [f'w_{k}' for k in xyz]
        rpyhead = ['roll', 'pitch', 'yaw']

        return timehead + ahead + whead + rpyhead + self.head.arrayheader

    def send2monitor(self):
        if self.report_sock is None:
            return

        if self._count%5==0:

            arr = np.concatenate((np.array([self.rptime,
                                            self.head.phiservo.angle,
                                            self.head.phiservo.target_angle,
                                            self.head.thetaservo.angle,
                                            self.head.thetaservo.target_angle]),
                                  self.rpy), dtype='f')


            if arr.dtype != 'f':
                return


            self.report_sock.send(arr.tobytes())
            #self.report_sock.send(np2bytes(arr, fmt='f'))

    @property
    def state(self):
        return {'a':self.imu_a,
                'w':self.imu_w,
                'rpy':self.rpy,
                'rptime':self.rptime,
                'rpmode':self.rpmode,
                'head':self.head.state}

    @state.setter
    def state(self, keydata): #data : bytearray):

        key, data = keydata
        if key == 'measurements':
            LOG.debug(data)
            self._count += 1
            rptime, self.imu_a, \
                    self.imu_w, self.head.pulse_phi, \
                    self.head.pulse_theta, self.rpmode = data


            rptime /= 1e6
            if (rptime - self.rptime) > .1:
                LOG.warning(f'Long break {rptime - self.rptime} s')
            else:
                self.dt = rptime - self.rptime
                # Update average on the fly
                self.dt_mean = self.dt_mean \
                        + (self.dt - self.dt_mean)/self._count
            self.rptime = rptime
            self._ahrs.update(self.imu_a/np.linalg.norm(self.imu_a),
                              self.imu_w/180*np.pi,
                              self.dt)
            self.rpy = self._ahrs.eul.copy()
            self.mytime = time.time()

            if self._n_collect > -1:
                self.df.loc[self._count-1, :] = self.asarray


        elif key=='external_input':
            LOG.info(f'External input : phi = {data[0]}, theta = {data[1]}')
            self.head.target_phi, self.head.target_theta = data
        elif key=='setmode-0':
            self._mode = 0
        elif key=='setmode-1':
            self._mode = 1
        else:
            raise KeyError(f'Invalid key "{key}" to update state.')

        # TESTING AREA
        # ======================
        if self._count%100 == 0:
            for s in self.__repr__().split('\n'): LOG.info(s)

        if self._count==self._n_collect:
            self._count = 0
            print(self.df.head())
            self.df.to_hdf('data/rp.hdf5', key='data')

        if self._count > 10:
            self.send2monitor()
        # ======================

    def get_ctrl(self):


        if not self.head._servosinited:
            LOG.warning(f'Servos not inited - curinit: {self.head._servo_init}...')
            cmd = self.head.servo_init_cmds[self.head._servo_init]
            self.head._servo_init += 1
            if self.head._servo_init == len(self.head.servo_init_cmds):
                self.head._servosinited = True
            return cmd

        if self._mode != self.rpmode:
            LOG.info(f'Update operation mode to {self._mode}')
            return make_ctrl(self._mode, 0, 0)


        # TODO: implement the controls...
        LOG.info(f'Set targets t={self.rptime}')
        self.head.target_phi = (self.rptime, -self.rpy[0])
        self.head.target_theta = (self.rptime, -self.rpy[1])

        # TODO: think about this dt scheme a bit.
        cmd = self.head.get_ctrl(self.dt_mean)
        LOG.debug('Control input: ')
        return cmd




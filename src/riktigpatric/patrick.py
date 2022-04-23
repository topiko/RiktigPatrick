"""
This is the module that contains RiktigPatrick!
"""

import time
import logging

import numpy as np

from riktigpatric.servo import Servo
from relay.conversions import make_ctrl
from relay.conversions import depack
from filters.mahony import Mahony
from typing import Union

P = 5; D=0;

LOG = logging.getLogger()

COMMPERIOD = 20e-3

SPEEDSCALE = 4
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
                 'b':1670,
                 }

hphi_params = {'name':'head_phi',
               'max_speed': (MAXPHI-MINPHI)*SPEEDSCALE, #[deg/sec]
               'minlim': MINPHI,
               'maxlim': MAXPHI,
               'idx':0,
               'a': -13,
               'b':1600,
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
    def target_phi(self, angle):
        self.phiservo.target_angle = angle

    @property
    def target_theta(self):
        return self.thetaservo.target_angle

    @target_theta.setter
    def target_theta(self, angle):
        self.thetaservo.target_angle = angle

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

    def get_ctrl(self, phispeed=None, thetaspeed=None):
        phispeed = self.phiservo.speed2int()
        thetaspeed = self.thetaservo.speed2int()
        return make_ctrl(16, phispeed, thetaspeed)

    def asarray(self):
        return np.concatenate((self.phiservo.asarray,
                               self.thetaservo.asarray))


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

    def __init__(self):

        self.head = RPHead()
        self._ahrs = Mahony()

        self.imu_a = np.zeros(3)
        self.imu_w = np.zeros(3)
        self.rpy  = np.zeros(3) # ahrs roll, pitch, yaw
        self.dt = 0
        self.rptime = 0
        self.rpmode = 0
        self._target_mode = 0
        self._count = 0
        self._mode = 0


    def __repr__(self):
        add = '  '
        repr_  = 'RiktigPatrick:\n'
        for k,v in self.state.items():
            repr_ += f'{add}{k} : {v}\n'
        repr_ += self.head.__repr__().replace('\n', f'\n{add}')
        return repr_

    @property
    def state(self):
        return {'a':self.imu_a,
                'w':self.imu_w,
                'rpy':self.rpy,
                'rptime':self.rptime,
                'rpmode':self.rpmode,
                'head':self.head.state}

    @state.setter
    def state(self, data : bytearray):

        key, data = depack(data)
        if key == 'measurements':
            LOG.debug(data)
            rptime, self.imu_a, \
                    self.imu_w, self.head.pulse_phi, \
                    self.head.pulse_theta, self.rpmode = data
            rptime /= 1e6
            self.dt = min(.1, rptime - self.rptime)
            self.rptime = rptime
            self._ahrs.update(self.imu_a/np.linalg.norm(self.imu_a),
                              self.imu_w/180*np.pi,
                              self.dt)
            self.rpy = self._ahrs.eul.copy()
            self._send_control = True
            self._count += 1
        elif key=='external_input':
            LOG.info(f'External input : phi = {data[0]}, theta = {data[1]}')
            self.head.target_phi, self.head.target_theta = data
            self._send_control = False
        elif key=='setmode-0':
            self._target_mode = 0
        elif key=='setmode-1':
            self._target_mode = 1
        else:
            raise KeyError(f'Invalid key "{key}" to update state.')

        if self._count%100 == 0:
            for s in self.__repr__().split('\n'): LOG.info(s)


    def get_ctrl(self):

        if not self._send_control:
            return b''

        if not self.head._servosinited:
            LOG.warning(f'Servos not inited - curinit: {self.head._servo_init}...')
            cmd = self.head.servo_init_cmds[self.head._servo_init]
            self.head._servo_init += 1
            if self.head._servo_init == len(self.head.servo_init_cmds):
                self.head._servosinited = True
            return cmd

        if self._mode != self.rpmode:
            LOG.info('Update operation mode to {self._mode}')
            return make_ctrl(self._mode, 0, 0)


        self.head.target_phi = -self.rpy[0]
        self.head.target_theta = -self.rpy[1]
        cmd = self.head.get_ctrl()
        LOG.debug('Control input: ')
        return cmd




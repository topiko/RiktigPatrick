"""
This is the module that contains RiktigPatrick!
"""

import time
import numpy as np
import logging

from riktigpatric.servo import ServoDriver
from relay.conversions import make_ctrl
from relay.conversions import depack
from typing import Union

P = 5; D=0;

LOG = logging.getLogger()

COMMPERIOD = 20e-3

SPEEDSCALE = 1
MINTHETA = -50
MAXTHETA = 28
MINPHI = -40
MAXPHI = 40

htheta_params = {'name':'head_theta',
                 'max_speed': (MAXTHETA - MINTHETA)*SPEEDSCALE, #[deg/sec]
                 'minlim': MINTHETA,
                 'maxlim': MAXTHETA,
                 'idx':1,
                 'a':15,
                 'b':1500,
                 'P':P,
                 'D':D
                 }

hphi_params = {'name':'head_theta',
               'max_speed': (MAXPHI-MINPHI)*SPEEDSCALE, #[deg/sec]
               'minlim': MINPHI,
               'maxlim': MAXPHI,
               'idx':0,
               'a': -13,
               'b':1500,
               'P':P,
               'D':D
               }



class Head():

    def __init__(self, saverun=False):
        self.headphi = ServoDriver(hphi_params)
        self.headtheta = ServoDriver(htheta_params)

        self.updateperiod = COMMPERIOD
        self.prevupdate = 0
        self.saverun = saverun
        self.servomode = 'speed'

        self._phipulse = None
        self._thetapulse = None

        if self.saverun:
            self.store_arr = np.zeros((1000, 8))
            self.nstore = 0
            self.store_vec = np.zeros(8)

        self.servo_init = 0
        self.servosinited = False
        self.set_speeds(0, 0)

    @property
    def initlists(self):
        cmds_theta = [make_ctrl(*t) for _, t in self.headtheta.get_init_dict().items()]
        cmds_phi = [make_ctrl(*t) for _, t in self.headphi.get_init_dict().items()]

        return cmds_phi + cmds_theta


    @property
    def servomode(self):
        return self._servomode

    @servomode.setter
    def servomode(self, mode : str):
        if mode in ['speed', 'pos']:
            self._servomode=mode
        else:
            raise KeyError('Invalid servomode key')

    def report(self):
        print('Head report:')
        self.headphi.report()
        self.headtheta.report()
        print()


    def get_ctrl(self):
        """
        Actions performed at each loop iter.
        """

        if self.servomode == 'pos':
            self.headphi.update_speed()
            self.headtheta.update_speed()

        phi_speed_int = self.headphi.speed_to_int()
        theta_speed_int = self.headtheta.speed_to_int()

        ctrl = make_ctrl(16, phi_speed_int, theta_speed_int)


        if self.saverun:
            self.store_state(1) #sentcmd)

        return ctrl


    def set_speeds(self, speedphi, speedtheta):
        self.headphi.set_speed(speedphi)
        self.headtheta.set_speed(speedtheta)

    def set_targets(self, targetphi, targettheta):
        self.headphi.set_target_angle(targetphi)
        self.headtheta.set_target_angle(targettheta)


    @property
    def servopulses(self):
        return self._phipulse, self._thetapulse

    @servopulses.setter
    def servopulses(self, pulses):
        """
        Set the current pulses read from
        """
        self._phipulse = pulses[0]
        self._thetapulse = pulses[1]
        self.headphi.set_angle(pulses[0], key='pulse')
        self.headtheta.set_angle(pulses[1], key='pulse')

    @property
    def state(self):
        return {'servomode': self.servomode,
                'phiservo':self.headphi.report(),
                'thetaservo':self.headtheta.report(),
               }


    def store_state(self, sentcmd, save=False):

        if self.nstore==len(self.store_arr)-1:
            save = True


        self.store_vec[0] = time.time()
        self.store_vec[1] = sentcmd
        self.store_vec[2] = self.headphi.angle
        self.store_vec[3] = self.headphi.target_angle
        self.store_vec[4] = self.headphi.speed
        self.store_vec[5] = self.headtheta.angle
        self.store_vec[6] = self.headtheta.target_angle
        self.store_vec[7] = self.headtheta.speed

        self.store_arr[self.nstore, :] = self.store_vec

        self.nstore += 1

        if save:
            np.save('rundat/head_dat.npy', self.store_arr)
            self.nstore=0




class RPatrick():

    def __init__(self, save=False):

        self.head = Head(saverun=save)
        self._state = {}
        self._send_control = False
        self._mode = 0
    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, data = Union[bytearray, np.ndarray, dict]):

        key, data = depack(data)
        if key=='measurements':
            avec, wvec, head_phi_p, head_theta_p, mode = data
            self._state['a'] = avec
            self._state['w'] = wvec
            self.head.servopulses = (head_phi_p, head_theta_p)
            self._state['head'] = self.head.state
            self._state['mode'] = mode

            self._send_control = True

        elif key=='external_input':
            self.head.set_speeds(*data)
            self._state['head'] = self.head.state
            print(self.state)
            self._send_control = False
        elif key=='setmode-0':
            self._mode = 0
        elif key=='setmode-1':
            self._mode = 1
        else:
            raise KeyError(f'Invalid key "{key}" to update state.')


    def get_ctrl(self) -> bytearray:

        if not self.head.servosinited:
            LOG.warning('Servos not inited - curinit: {self.head.servo_init}...')
            cmd = self.head.initlists[self.head.servo_init]
            self.head.servo_init += 1
            if self.head.servo_init == len(self.head.initlists):
                self.head.servosinited = True
            return cmd

        if self._mode != self.state['mode']:
            return make_ctrl(self._mode, 0, 0)
        if self._send_control:
            cmd = self.head.get_ctrl()
            LOG.debug('Control input')
            return cmd #make_ctrl(1,0,0)
        else:
            return b''

        #return self.head.get_ctrl()


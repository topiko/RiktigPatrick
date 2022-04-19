"""
This is the module that contains RiktigPatrick!
"""

import time
import numpy as np

from servo import ServoDriver
from communication import write_arduino, make_ctrl, init_servo

P = 5; D=0;

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
                 'b':1800,
                 'P':P,
                 'D':D
                 }

hphi_params = {'name':'head_theta',
               'max_speed': (MAXPHI-MINPHI)*SPEEDSCALE, #[deg/sec]
               'minlim': MINPHI,
               'maxlim': MAXPHI,
               'idx':0,
               'a': -13,
               'b':1480,
               'P':P,
               'D':D
               }



class Head():

    def __init__(self, I2Cbus, store=False):
        self.I2Cbus = I2Cbus
        self.headphi = ServoDriver(hphi_params)
        self.headtheta = ServoDriver(htheta_params)

        self.updateperiod = COMMPERIOD
        self.prevupdate = 0
        self.store = store

        if self.store:
            self.store_arr = np.zeros((1000, 8))
            self.nstore = 0
            self.store_vec = np.zeros(8)

        # Init servos:
        init_servo(self.I2Cbus, self.headphi.make_init_list())
        init_servo(self.I2Cbus, self.headtheta.make_init_list())

    def report(self):
        print('Head report:')
        self.headphi.report()
        self.headtheta.report()
        print()

    def run(self, pulses=None):
        """
        Actions performed at each loop iter.
        Attr:
            pulses : (phipulse, thetapulse)
        """

        if pulses is not None:
            self.set_pulses(*pulses)

        self.headphi.update_speed()
        self.headtheta.update_speed()

        sentcmd = 0
        if (time.time() - self.prevupdate) > self.updateperiod:
            phi_speed_int = self.headphi.speed_to_int()
            theta_speed_int = self.headtheta.speed_to_int()

            ctrl = make_ctrl(16, phi_speed_int, theta_speed_int)
            write_arduino(self.I2Cbus, ctrl)
            self.prevupdate = time.time()
            sentcmd=1


        if self.store:
            self.store_state(sentcmd)



    def set_speeds(self, speedphi, speedtheta):
        self.headphi.set_speed(speedphi)
        self.headtheta.set_speed(speedtheta)

    def set_targets(self, targetphi, targettheta):
        self.headphi.set_target_angle(targetphi)
        self.headtheta.set_target_angle(targettheta)

    def set_pulses(self, phipulse, thetapulse):
        """
        Set the current pulses read from
        """
        self.headphi.set_angle(phipulse, key='pulse')
        self.headtheta.set_angle(thetapulse, key='pulse')

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

    def __init__(self,):
        pass


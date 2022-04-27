import logging

import numpy as np

LOG = logging.getLogger()

class Servo():

    def __init__(self,
                 name : str,
                 max_speed : float,
                 minlim : float,
                 maxlim : float,
                 idx : int,
                 a : float,
                 b: float,
                 ):

        self.name = name
        self.maxspeed = max_speed
        self.minlim = minlim
        self.maxlim = maxlim
        self.idx = idx

        self.a = a
        self.b = b
        self._angle = None
        self._target_angle = None
        self._pulse = None
        self._target_pulse = None
        self._speed = None
        self._deltaT = None
        self._target_angle_arr = np.zeros((3,2))
        self.fac = 1 if self.a > 0 else -1 # If negative slope we need to send negative value.

    def pulse2angle(self, pulse):
        "pulse [ms] to [deg]"
        LOG.debug(f'{self.name} pulse2angle {pulse}-->angle')
        if pulse is None:
            return None
        return (pulse - self.b)/self.a

    def angle2pulse(self, angle):
        "[deg] to pulse [ms]"
        LOG.debug(f'{self.name} angle2pulse {angle}-->pulse')
        if angle is None:
            return None
        return int(self.a * angle + self.b)

    @property
    def state(self):
        return {'name': self.name,
                'idx': self.idx,
                'minlim': self.minlim,
                'maxlim':self.maxlim,
                'speed':self.speed,
                'target_angle':self.target_angle,
                'angle':self.angle,
                'target_pulse':self.target_pulse,
                'pulse':self.pulse}

    def __repr__(self):
        return '\n'.join([f'{k} : {v}' for k,v in self.state.items()])

    @property
    def asarray(self):
        return np.array([self._speed, self.target_angle, self.angle, self.target_pulse, self.pulse])

    @property
    def estim_target_speed(self):
        times = self._target_angle_arr[:, 0]
        times -= times[0]
        t1, t2, t3 = times
        a1, a2, a3 = self._target_angle_arr[:, 1]

        if (t1==t2) or (t2==t3):
            return 0
        # Second order fit:
        # 1   t1   t1^2  | a    a1
        # 1   t2   t2^2  | b  = a2
        # 1   t3   t3^2  | c    a3
        # -->
        # TODO: check these!
        a = a1/(t1*t1 - t1*t2 - t1*t3 + t2*t3) \
                + a2/(-t1*t2 + t1*t3 + t2*t2 - t2*t3) \
                + a3/(t1*t2 - t1*t3 - t2*t3 + t3*t3)

        b = - (t2*a1)/(t1*t1 - t1*t2 - t1*t3 + t2*t3) \
                - (t3*a1)/(t1*t1 - t1*t2 - t1*t3 + t2*t3) \
                - (t1*a2)/(-t1*t2 + t1*t3 + t2*t2 - t2*t3) \
                - (t3*a2)/(-t1*t2 + t1*t3 + t2*t2 - t2*t3) \
                - (t1*a3)/(t1*t2 - t1*t3 - t2*t3 + t3*t3) \
                - (t2*a3)/(t1*t2 - t1*t3 - t2*t3 + t3*t3)

        c = (t2*t3*a1)/(t1*t1 - t1*t2 - t1*t3 + t2*t3) \
                + (t1*t3*a2)/(-t1*t2 + t1*t3 + t2*t2 - t2*t3) \
                + (t1*t2*a3)/(t1*t2 - t1*t3 - t2*t3 + t3*t3)

        if self._deltaT is not None:
            # TODO: icorrect time?
            return 2*a*self._deltaT*0 + b

        LOG.warning(f'{self.name} could not estimate next target.')
        return 0

    @property
    def arrayheader(self):
        keys = ['speed', 'target_angle', 'angle', 'target_pulse', 'pulse']
        return [f'{self.name}_{k}' for k in keys]

    @property
    def speed(self) -> float:
        "[deg/s]"
        LOG.debug(f'{self.name} speed: {self.target_angle} - {self.angle}')

        # TODO: fix some nice speed update here:
        if any(a is None for a in (self.target_angle, self.angle)):
            return None

        try:
            estim_cur_angle =  self.angle #+ self._speed*self._deltaT
        except TypeError:
            LOG.warning(f'{self.name} issues in determining cur angle.')
            estim_cur_angle = 0


        if self._speed is None:
            LOG.warning(f'{self.name} no speed set --> 0.')
            self._speed = 0

        # TODO: fix these...
        dangle = self.target_angle - estim_cur_angle
        dangledot = self.estim_target_speed - self._speed

        P = 30
        D = 2

        speed = self._speed
        speed += P * dangle  + D * dangledot #/ self._deltaT
        if abs(speed) > self.maxspeed:
            LOG.warning(f'{self.name} requesting larger speed ({speed} deg/s) than available {self.maxspeed} deg/s.')
            speed = np.clip(speed, -self.maxspeed, self.maxspeed)


        self._speed = speed

        return self._speed

    @property
    def angle(self) -> float:
        "[deg]"
        return self._angle

    @property
    def pulse(self) -> float:
        "[ms]"
        return self._pulse

    @pulse.setter
    def pulse(self, pulse) -> None:
        "[ms]"
        LOG.debug(f'{self.name} pulse: {self.pulse} --> {pulse}')
        self._pulse = pulse
        self._angle = self.pulse2angle(pulse)


    @property
    def target_pulse(self) -> float:
        "[ms]"
        return self._target_pulse

    @target_pulse.setter
    def target_pulse(self, pulse) -> None:
        "[ms]"
        t, pulse = tandpulse
        pulse = int(pulse)
        LOG.debug(f'{self.name} target_pulse: {self.target_pulse} --> {pulse}')
        self._target_angle = self.pulse2angle(pulse)
        self._target_pulse = pulse


    @property
    def target_angle(self) -> float:
        "[deg]"
        return self._target_angle

    @target_angle.setter
    def target_angle(self, tandangle) -> None:
        "[deg]"
        t, angle = tandangle
        LOG.debug(f'{self.name} target_angle: {self.target_angle} --> {angle}')
        if not np.isfinite(angle):
            LOG.warning(f'Infinity/NaN encountered in setting angle for {self.name}')
            return

        angle_ = np.clip(angle, self.minlim, self.maxlim)
        if not np.isclose(angle, angle_):
            LOG.warning(f'Angle ({angle}) out of range [{self.minlim}, {self.maxlim}] for {self.name}')


        self._target_pulse = self.angle2pulse(angle_)
        self._target_angle = angle_

        self._target_angle_arr[1:] = self._target_angle_arr[:-1]
        self._target_angle_arr[0,:] = t, self._target_angle

    def speed2int(self, deltaT=None):
        """
        Comver current speed to value sent to arduino.
        Attr:
            speed : [deg/sec] (None -> use self.speed)
        Returns:
            speed_int : fraction of speed to maxspeed * 2**15.
        """

        self._deltaT = deltaT

        speed = self.speed
        if speed is None:
            speed = 0

        speed_frac = speed / self.maxspeed

        #if abs(speed_frac)>1:
        #    LOG.warning(f'{self.name} requesting larger speed ({speed} deg/s) than available {self.maxspeed} deg/s.')
        #    speed_frac = np.clip(speed_frac, -1, 1)
            #if   speed_frac < -1: speed_frac = -1
            #elif speed_frac >  1: speed_frac =  1

        speed_int = int(self.fac * speed_frac * 32768) # This is then sent to arduino.

        return speed_int

    @property
    def init_dict(self):
        """Produce set of commands setn to arduino to init the servo params."""

        init_dict = {}

        lim1 = self.angle2pulse(self.minlim)
        lim2 = self.angle2pulse(self.maxlim)

        if self.fac == 1:
            minlim = lim1
            maxlim = lim2
        elif self.fac == -1:
            minlim = lim2
            maxlim = lim1


        init_dict['minlim'] = (64,
                               self.idx,
                               minlim)

        init_dict['maxlim'] = (65,
                               self.idx,
                               maxlim)

        maxspeed_pulse = abs(int(self.a * self.maxspeed))
        init_dict['maxspeed'] = (66,
                                 self.idx,
                                 maxspeed_pulse)
        return init_dict



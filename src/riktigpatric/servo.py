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
                 b: float):

        self.name = name
        self.maxspeed = max_speed
        self.minlim = minlim
        self.maxlim = maxlim
        self.idx = idx

        self.a = a
        self.b = b
        self._angle = None
        self._target_angle = None
        self.target_angle = 0
        self._pulse = None
        self._target_pulse = None
        self._speed = None
        self._deltaT = None
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
        return np.array([self.speed, self.target_angle, self.angle, self.target_pulse, self.pulse])

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
            add_angle =  self._speed*self._deltaT
        except TypeError:
            add_angle = 0

        dangle = self.target_angle - (self.angle + add_angle)

        sign = np.sign(dangle)
        self._speed = sign*dangle**2


        return self._speed # dangle**2 / 10

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
        pulse = int(pulse)
        LOG.debug(f'{self.name} target_pulse: {self.target_pulse} --> {pulse}')
        self._target_angle = self.pulse2angle(pulse)
        self._target_pulse = pulse

    @property
    def target_angle(self) -> float:
        "[deg]"
        return self._target_angle

    @target_angle.setter
    def target_angle(self, angle) -> None:
        "[deg]"
        LOG.debug(f'{self.name} target_angle: {self.target_angle} --> {angle}')
        if not np.isfinite(angle):
            LOG.warning(f'Infinity/NaN encountered in setting angle for {self.name}')
            return

        angle_ = np.clip(angle, self.minlim, self.maxlim)
        if not np.isclose(angle, angle_):
            LOG.warning(f'Angle ({angle}) out of range [{self.minlim}, {self.maxlim}] for {self.name}')


        self._target_pulse = self.angle2pulse(angle_)
        self._target_angle = angle_

    def speed2int(self, deltaT=None, speed=None):
        """
        Comver current speed to value sent to arduino.
        Attr:
            speed : [deg/sec] (None -> use self.speed)
        Returns:
            speed_int : fraction of speed to maxspeed * 2**15.
        """

        self._deltaT = deltaT

        if speed is None:
            speed = self.speed
        if speed is None:
            speed = 0

        speed_frac = (speed / self.maxspeed)
        if abs(speed_frac)>1:
            LOG.warning(f'{self.name} requesting larger speed ({speed} deg/s) than available {self.maxspeed} deg/s.')
            speed_frac = np.clip(speed_frac, -1, 1)
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



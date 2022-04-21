import numpy as np

class ServoCalc():

    def __init__(self, params):
        """
        Attr:
            params : Either dict of params or path to params file. Check params below.
        """

        if isinstance(params, str):
            params = np.load(params + 'npy').item()
        elif isinstance(params, dict):
            pass
        else:
            raise TypeError('Params need to be either path to servo params file. Or params dict.')

        self.params = params
        self.name = params['name']
        self.speed = 0
        self.maxspeed = params['max_speed']
        self.minlim = params['minlim']
        self.maxlim = params['maxlim']
        self.idx = params['idx']

        self.a = params['a']
        self.b = params['b']
        self.angle = self.b

        self.showwarning = False
        self.fac = 1 if self.a > 0 else -1 # If negative slope we need to send negative value.

    def set_speed(self, speed):
        """
        Attr:
            speed : [deg/sec]
        """
        self.speed=speed

    def get_speed(self):
        """
        Return:
            speed : [deg/sec]
        """
        return self.speed

    def get_angle(self):
        """
        Return:
            angle : deg
        """
        return self.angle

    def set_angle(self, val, key='pulse'):
        """
        """
        if key=='pulse':
            self.angle = (val - self.b)/self.a
        elif key=='deg':
            self.angle = val
        else:
            raise KeyError('Invalid key: {}'.format(key))

        return self.angle




    def speed_to_int(self, speed=None):
        """
        Comver current speed to value sent to arduino.
        Attr:
            speed : [deg/sec] (None -> use self.speed)
        Returns:
            speed_int : fraction of speed to maxspeed * 2**15.
        """

        if speed is None: speed = self.speed

        speed_frac = (speed / self.maxspeed)
        if abs(speed_frac)>1:
            if self.showwarning: print('WARNING: requesting larger speed than available.')
            if   speed_frac < -1: speed_frac = -1
            elif speed_frac >  1: speed_frac =  1

        speed_int = self.fac * ( speed / self.maxspeed ) * 32768 # This is then sent to arduino.

        return int(speed_int)

    def angle_to_pulse(self, angle=None):

        if angle is None: angle = self.angle

        return int(self.a * angle + self.b)

    def pulse_to_angle(self, pulse):
        return (pulse - self.b)/self.a

    def get_init_dict(self):
        """Produce set of commands setn to arduino to init the servo params."""

        init_dict = {}

        lim1 = self.angle_to_pulse(self.minlim)
        lim2 = self.angle_to_pulse(self.maxlim)

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

class ServoDriver(ServoCalc):

    def __init__(self, params, P=None, D=None):

        super().__init__(params);

        self.P = self.params['P'] if P is None else P
        self.D = self.params['D'] if D is None else D

        self.target_angle = None
        self.target_pulse = None

    def set_target_angle(self, angle):

        if angle<self.minlim:
            print('WARNING: setting anlgle outside boundaries: {:.2f}<{:.2f}(min)'.format(angle, self.minlim))
        if angle>self.maxlim:
            print('WARNING: setting anlgle outside boundaries: {:.2f}>{:.2f}(max)'.format(angle, self.maxlim))
        self.target_angle = angle
        self.target_pulse = self.angle_to_pulse(angle=angle)

    def set_target_pulse(self, pulse):

        self.target_pulse = pulse
        self.set_target_angle(self.pulse_to_angle(pulse))

    def update_speed(self, target_angle=None, pulse=None):

        if target_angle is not None:
            self.set_target_angle(target_angle)
        if pulse is not None:
            self.set_angle(pulse, key='pulse')

        dangle = self.target_angle - self.angle

        # TODO: this is incorrect and should be replaced with accelaration
        # based updates on the velocity.

        self.speed = self.P * dangle - self.D * self.speed

    def report(self, show=False):

        reportlist = {'Name': self.name,
                      'Angle': self.angle,
                      'Target angle': self.target_angle,
                      'Speed': self.speed,
                      'Speed int': self.speed_to_int(),
                      'Pulse': self.angle_to_pulse(),
                      'Target pulse': self.target_pulse}

        fstr = '{:<25s}: {}'
        if show:
            print("SERVO REPORT")
            for key, val in reportlist.items():
                print(fstr.format(key, val))
            print("END")

        return reportlist

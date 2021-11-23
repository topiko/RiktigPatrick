
class ServoDriver():

    def __init__(self, servo, P=1, D=0):
        self.P = P
        self.D = D
        self.servo = servo
        self.target_pos = 90

    def pdspeed(self, delta_pos):
        return  self.P * delta_pos - self.D * self.servo.get_speed()

    def update_servo_speed(self, pos=None, pos_pulse=None):

        if pos_pulse is not None:
            pos = self.servo.set_pos_from_pulse(pos_pulse)
        speed = self.pdspeed(self.target_pos - pos)
        self.servo.set_speed(speed)

    def get_ctrl_int(self):
        return self.servo.speed_to_sendspeed()

class Servo():

    def __init__(self, name, idx, a=None, b=None):
        """
        Attr:
            name : name of the servo for book keeping.
            idx : idx in arduino servo table for writing params.
        """
        self.name = name
        self.speed = 0
        self.maxspeed = 90
        self.minlim = -20
        self.maxlim = 20
        self.idx = idx

        self.coefs_set = False

        if (a is not None) and (b is not None):
            self.set_coefs(a, b)

    def set_speed(self, speed):
        self.speed=speed

    def get_speed(self):
        return self.speed

    def get_pos(self):
        return self.pos

    def set_pos_from_pulse(self, pulse):
        self.check_coefs()
        self.pos = (pulse - self.b)/self.a
        return self.pos

    def set_coefs(self, a, b):
        self.a = a
        self.b = b
        self.coefs_set = True

    def check_coefs(self):

        if not self.coefs_set:
            raise ValueError('You have not supplied to conversion parameters a and b.')

        return True


    def speed_to_sendspeed(self, speed=None):
        self.check_coefs()

        if speed is None: speed = self.speed

        speed_int = ( speed / self.maxspeed ) * 32768

        return int(speed_int)

    def pos_to_pulse(self, pos=None):
        self.check_coefs()

        if pos is None: pos = self.pos

        return self.a * pos + self.b

    def make_init_list(self):
        """Produce set of commands setn to arduino to init the servo params."""

        key = 64
        init_dict = {}

        init_dict['minlim'] = (64,
                              self.idx,
                              self.pos_to_pulse(self.minlim))

        init_dict['maxlim'] = (65,
                              self.idx,
                              self.pos_to_pulse(self.maxlim))

        maxspeed_pulse = int(self.a * self.maxspeed)
        init_dict['maxspeed'] = (66,
                                 self.idx,
                                 maxspeed_pulse)
        return init_dict

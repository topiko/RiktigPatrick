import time
import socket

from pyPS4Controller.controller import Controller

HOSTNAME = socket.gethostname()
HOST = socket.gethostbyname('{HOSTNAME}.local') #'192.168.0.45'
PORT = 1024


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

class RPController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        self.ctrl = [0,0]
        self.tlastinput=time.time()
        self.period = 1./30 # Period for control inputs.

    def make_ctrl_input(self, value, ch):

        value = value/32768


        self.ctrl[ch] = value
        strctrl = [str(val) for val in self.ctrl]
        print("Ctrl: ")
        print("theta : {}".format(self.ctrl[0]))
        print("phi   : {}".format(self.ctrl[1]))

        return ','.join(strctrl)

    def decide_input(self, value, ch):

        major_change = abs(self.ctrl[ch] - value/32768) > .1
        if ((time.time()-self.tlastinput)>self.period) or (value==0) or major_change:
            self.tlastinput=time.time()
            return self.make_ctrl_input(value, ch)


        return None


    def handle_joystic(self, value, ch):
        ctrl = self.decide_input(value, ch)
        if ctrl is not None:
            pass
            #sock.
            #client.publish("riktigpatrick/remote/head",
            #               payload=ctrl,
            #               qos=0,
            #               retain=False)


    def on_L3_up(self, value):
        self.handle_joystic(-value, 0)

    def on_L3_down(self, value):
        self.handle_joystic(-value, 0)

    def on_L3_y_at_rest(self):
        self.handle_joystic(0, 0)

    def on_L3_left(self, value):
        self.handle_joystic(-value, 1)

    def on_L3_right(self, value):
        self.handle_joystic(-value, 1)

    def on_L3_x_at_rest(self):
        self.handle_joystic(0, 1)





controller = RPController(interface="/dev/input/js0", connecting_using_ds4drv=False)


try:
    controller.listen()
except KeyboardInterrupt:
    pass
finally:
    sock.send(b'')





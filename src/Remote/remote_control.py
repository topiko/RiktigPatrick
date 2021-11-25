import paho.mqtt.client as mqtt
from utils import makemqttclient
from pyPS4Controller.controller import Controller
import time


q, client = makemqttclient([])

class RPController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

        self.ctrl = [0,0]
        self.tlastinput=time.time()
        self.period = 1./20 # Period for control inputs.

    def make_ctrl_input(self, value, ch):

        value = value/32768


        self.ctrl[ch] = value
        strctrl = [str(val) for val in self.ctrl]
        print("Ctrl: ")
        print("theta : {}".format(self.ctrl[0]))
        print("phi   : {}".format(self.ctrl[1]))

        return ','.join(strctrl)

    def decide_input(self, value, ch):

        if ((time.time()-self.tlastinput)>self.period) or (value==0):
            self.tlastinput=time.time()
            return self.make_ctrl_input(value, ch)


        return None


    def handle_joystic(self, value, ch):
        ctrl = self.decide_input(value, ch)
        if ctrl is not None:
            client.publish("riktigpatric/control/head",
                           payload=ctrl,
                           qos=0,
                           retain=False)


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
controller.listen()





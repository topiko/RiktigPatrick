import time

from smbus2 import SMBus, i2c_msg
from communication import read_arduino, make_ctrl, write_arduino
from patrick import Head
import patrick as rp
from mqttutils import makemqttclient
from math import sin

P = 20; D=0;

PERIOD = 3

theta_amp = 5 #D(maxtheta - mintheta)/2
phi_amp = 5

queue, client = makemqttclient(['riktigpatrick/remote/head/'], host='192.168.0.13')

def handle_mqtt_message():
    while not queue.empty():
        if queue.qsize()!=1: print("LONG QUEUE")
        message = q.get()

        msg_topic = message[0]
        message = message[1]
        message = message.decode("utf-8")


        if msg_topic=="riktigpatric/remote/head":
            target_theta, target_phi = message.split(',')
            print(target_phi, target_theta)

        target_phi = (rp.MAXPHI - rp.MINPHI)*target_phi + rp.MINPHI
        target_theta = (rp.MAXTHETA - rp.MINTHETA)*target_theta + rp.MINTHETA
        return target_phi, target_theta



# Createthe I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    rphead = Head(I2Cbus, True)

    t1 = time.time()
    n = 0
    while True:

        target_theta = 0
        target_phi = 0

        a = client.loop(timeout=0.001)
        target_phi_, target_theta_ = handle_mqtt_message()

        ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode = read_arduino(I2Cbus)




        if (mode==0):
            ctrl_input = make_ctrl(1,0,0)
            write_arduino(I2Cbus, ctrl_input)
        else:
            rphead.set_targets(target_phi, target_theta)
            rphead.run(pulses=(head_phi_pulse, head_theta_pulse))

            print(target_phi_, target_theta_)
            if (n%1000)==0:
                print(ax, ay, az, wx, wy, wz, mode)
                print('Freq: {:.2f}'.format(n/(time.time() - t1)))
                rphead.report()



        n += 1


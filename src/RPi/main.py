import time

from smbus2 import SMBus, i2c_msg
from communication import read_arduino, make_ctrl, write_arduino
from patrick import Head
import patrick as rp
from mqttutils import makemqttclient
from math import sin


queue, client = makemqttclient(['riktigpatrick/remote/head'], host='192.168.0.13')
def handle_mqtt_message(queue, target_phi, target_theta):

    update = False
    while not queue.empty():


        if queue.qsize()!=1: print("LONG QUEUE")
        message = queue.get()

        msg_topic = message[0]
        message = message[1]
        message = message.decode("utf-8")


        if msg_topic=="riktigpatrick/remote/head":
            target_theta, target_phi = message.split(',')

        target_phi = (rp.MAXPHI - rp.MINPHI)*(float(target_phi)+1)/2 + rp.MINPHI
        target_theta = (rp.MAXTHETA - rp.MINTHETA)*(-float(target_theta)+1)/2 + rp.MINTHETA


        update = True

    return target_phi, target_theta, update




target_phi = 0
target_theta = 0

# Createthe I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    rphead = Head(I2Cbus, True)
    rphead.set_targets(target_phi, target_theta)

    t1 = time.time()
    n = 0
    while True:

        client.loop(timeout=0.01)
        target_phi, target_theta, targetschanged = handle_mqtt_message(queue, target_phi, target_theta)
        ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode = read_arduino(I2Cbus)




        if (mode==0):
            ctrl_input = make_ctrl(1,0,0)
            write_arduino(I2Cbus, ctrl_input)
        else:
            if targetschanged:
                rphead.set_targets(target_phi, target_theta)

            rphead.run(pulses=(head_phi_pulse, head_theta_pulse))


            if (n%1000)==0:
                print(ax, ay, az, wx, wy, wz, mode)
                print('Freq: {:.2f}'.format(n/(time.time() - t1)))
                rphead.report()



        n += 1


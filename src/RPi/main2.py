import time

from smbus2 import SMBus, i2c_msg
from communication import read_arduino, make_ctrl, write_arduino
from patrick import Head
from math import sin

P = 20; D=0;

PERIOD = 3

theta_amp = 5 #D(maxtheta - mintheta)/2
phi_amp = 5



# Createthe I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    rphead = Head(I2Cbus, True)

    t1 = time.time()
    n = 0
    while True:

        target_theta = sin((time.time() - t1)/PERIOD * 6.28) * theta_amp
        target_phi = -sin((time.time() - t1)/PERIOD * 6.28) * phi_amp


        ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode = read_arduino(I2Cbus)




        if (mode==0):
            ctrl_input = make_ctrl(1,0,0)
            write_arduino(I2Cbus, ctrl_input)
        else:
            rphead.set_targets(target_phi, target_theta)
            rphead.run(pulses=(head_phi_pulse, head_theta_pulse))

            if (n%1000)==0:
                print(ax, ay, az, wx, wy, wz, mode)
                print('Freq: {:.2f}'.format(n/(time.time() - t1)))
                rphead.report()



        n += 1


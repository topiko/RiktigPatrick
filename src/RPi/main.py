import time
from smbus2 import SMBus, i2c_msg
from communication import read_arduino, make_ctrl, write_arduino
from servo import ServoDriver
from math import sin

P = 20; D=0;

PERIOD = 3
COMMPERIOD = 20e-3

mintheta = -.45
maxtheta = .25
minphi = -40
maxphi = 40
htheta_params = {'name':'head_theta',
                 'max_speed': 120, #[deg/sec]
                 'minlim': mintheta,
                 'maxlim': maxtheta,
                 'idx':1,
                 'a':15,
                 'b':1800,
                 'P':P,
                 'D':D
                 }

hphi_params = {'name':'head_theta',
               'max_speed': 120, #[deg/sec]
               'minlim': minphi,
               'maxlim': maxphi,
               'idx':0,
               'a': -13,
               'b':1520,
               'P':P,
               'D':D
               }


headtheta= ServoDriver(htheta_params)
headphi= ServoDriver(hphi_params)

init_dict_head_theta = headtheta.make_init_list()
init_dict_head_phi = headphi.make_init_list()



def init_servo(I2Cbus, init_dict):

    print(init_dict)
    for _, tup in init_dict.items():
        send = make_ctrl(*tup)
        write_arduino(I2Cbus, send)



# Createthe I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    t1 = time.time()
    prev_write = t1
    n = 0

    init_servo(I2Cbus, headphi.make_init_list())
    init_servo(I2Cbus, headtheta.make_init_list())

    while True:

        target_theta = sin((time.time() - t1)/PERIOD * 6.28) * (maxtheta - mintheta)/2 + (maxtheta + mintheta)/2
        target_phi = -sin((time.time() - t1)/PERIOD * 6.28) * (maxphi - minphi)/2 + (maxphi + minphi)/2

        ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode = read_arduino(I2Cbus)

        theta_speed = headtheta.update_speed_int(target_theta, head_theta_pulse)
        phi_speed = headphi.update_speed_int(target_phi, head_phi_pulse)

        ctrl_input = make_ctrl(16, phi_speed, theta_speed)

        if (n%1000==0):
            print(ax, ay, az, wx, wy, wz)
            print('Freq: {:.2f}'.format(n/(time.time() - t1)))
            print()
            print('Arduino mode:  ', mode)
            print('Time: {:.2f}'.format(time.time() - t1))
            print('THETA:')
            print('Arduino pulse theta: {:5d} / {:<5d}'.format(head_theta_pulse, headtheta.target_pulse))
            print('Theta angle:         {:5.2f} / {:5.2f} / {:5.2f}'.format(headtheta.get_angle(), headtheta.target_angle, target_theta))
            print('Send speed theta:    {:5d} / {:5.4f}'.format(headtheta.speed_to_int(), headtheta.get_speed()))
            print('PHI:')
            print('Arduino pulse phi:   {:5d} / {:<5d}'.format(head_phi_pulse, headphi.target_pulse))
            print('Phi angle:           {:5.2f} / {:5.2f} / {:5.2f}'.format(headphi.get_angle(), headphi.target_angle, target_phi))
            print('Send speed phi:      {:5d} / {:5.4f}'.format(headphi.speed_to_int(), headphi.get_speed()))
            print()


        if (mode==0): ctrl_input = make_ctrl(1,0,0)

        if (time.time() - prev_write) > COMMPERIOD:
            write_arduino(I2Cbus, ctrl_input)
            prev_write = time.time()

        n += 1

        #time.sleep(.02)

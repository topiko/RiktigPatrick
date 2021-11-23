from smbus2 import SMBus, i2c_msg
from communication import read_arduino, make_ctrl, write_arduino
from servo import Servo, ServoDriver
from math import sin
import time


head_theta_servo = Servo('headtheta', 1, a=10, b=1505)
head_phi_servo = Servo('headphi', 0, a=10, b=1505)


init_dict_head_phi = head_phi_servo.make_init_list()

turn_phi = ServoDriver(head_phi_servo, P=.5)


def init_servo(I2Cbus, init_dict):

    print(init_dict)
    for k, tup in init_dict.items():
        send = make_ctrl(*tup)
        write_arduino(I2Cbus, send)

    return
# (selector, head_phi, head_theta)

# Create the I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    t1 = time.time()
    n = 0

    init_servo(I2Cbus, init_dict_head_phi)

    while True:

        ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode = read_arduino(I2Cbus);

        print(ax, ay, az, wx, wy, wz)
        print('Arduino mode: ', mode)
        print('Arduino pulse: ', head_phi_pulse)
        print('Send speed: ', head_phi_servo.speed_to_sendspeed())

        turn_phi.target_pos = 40*sin((t1 - time.time())/1)
        head_phi_servo.set_pos_from_pulse(head_phi_pulse)
        turn_phi.update_servo_speed(pos_pulse=head_phi_pulse)
        ctrl_input = make_ctrl(16, head_phi_servo.speed_to_sendspeed(), 0)
        print(ctrl_input)
        if (n==0): ctrl_input = make_ctrl(1,0,0)
        elif ((n!=0) & (n%1000==0)): ctrl_input = make_ctrl(8,0,0)

        write_arduino(I2Cbus, ctrl_input)

        n += 1;

        time.sleep(.1)

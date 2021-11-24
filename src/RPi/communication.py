
import struct
from smbus2 import i2c_msg


# Slave Addresses
I2C_SLAVE_ADDRESS = 11 #0x0b ou 11

def make_ctrl(i,j,k):

    def constrain(i):
        return int(max(-32768, min(i, 32768-1)))

    def input2bytes(i,j,k):

        i = constrain(i).to_bytes(1, byteorder='little')
        j = constrain(j).to_bytes(2, byteorder='big', signed=True)
        k = constrain(k).to_bytes(2, byteorder='big', signed=True)
        return i+j+k

    return input2bytes(i,j,k)

def write_arduino(I2Cbus, ctrl_input):
    I2Cbus.write_i2c_block_data(I2C_SLAVE_ADDRESS, 0x00, ctrl_input)

def read_arduino(I2Cbus):

    NBYTESI2C = 29

    msg = i2c_msg.read(I2C_SLAVE_ADDRESS, NBYTESI2C)
    I2Cbus.i2c_rdwr(msg)


    return depack_arduino_i2c(list(msg))


def depack_arduino_i2c(intarr):

    def unpack_to_float(valarr):
        b = bytearray(valarr)
        return struct.unpack('f', b)[0]

    def unpack_to_int(valarr):
        b = bytearray(valarr)
        return struct.unpack('H', b)[0]

    # Accel [m/s**2]:
    G=9.81 #[m/s**2] earths gravit accel.
    ax = unpack_to_float(intarr[0:4])*G
    ay = unpack_to_float(intarr[4:8])*G
    az = unpack_to_float(intarr[8:12])*G

    # Gyroscope [deg/s] (source Arduino example for readGyroscope)??:
    wx = unpack_to_float(intarr[12:16])
    wy = unpack_to_float(intarr[16:20])
    wz = unpack_to_float(intarr[20:24])


    # pos servos:
    head_phi_pulse = unpack_to_int(intarr[24:26])
    head_theta_pulse = unpack_to_int(intarr[26:28])

    # mode:
    mode = intarr[28]

    return ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode



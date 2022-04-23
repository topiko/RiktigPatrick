import struct
import numpy as np
import logging

from remote.keyboard_remote import depack_KB

LOG = logging.getLogger()

def depack(bytearr_ : bytearray) -> dict:

    commsel = struct.unpack('>H', b'\x00' + bytearr_[-1:])[0]

    if commsel in [0, 1]:
        LOG.debug('IMU input')
        return 'measurements', depack_IMU(bytearr_)
    elif commsel==128:
        LOG.debug('External control input.')
        return 'external_input', depack_KB(bytearr_)
    elif commsel==129:
        LOG.debug('External control: mode --> 0.')
        return 'setmode-0', None
    elif commsel==130:
        LOG.debug('External control: mode --> 1.')
        return 'setmode-1', None

    else:
        raise KeyError(f'Invalid commsel key: {commsel}')

def depack_IMU(bytearr_):
    """
    Attr:
        IMU+Servo+TIME(?) bytearray from arduino.
    Returns:
        [ax, ay, az], [wx, wy, wz], head_phi_pulse, head_theta_pulse, mode
    """

    def unpack_to_float(b):
        #b = bytearray(valarr)
        return struct.unpack('f', b)[0]

    def unpack_to_int(b, fmt='H'):
        #b = bytearray(valarr)
        return struct.unpack(fmt, b)[0]

    # Accel [m/s**2]:
    G=9.81 #[m/s**2] earths gravit accel.
    ax = unpack_to_float(bytearr_[0:4])*G
    ay = unpack_to_float(bytearr_[4:8])*G
    az = unpack_to_float(bytearr_[8:12])*G

    # Gyroscope [deg/s] (source Arduino example for readGyroscope)??:
    wx = unpack_to_float(bytearr_[12:16])
    wy = unpack_to_float(bytearr_[16:20])
    wz = unpack_to_float(bytearr_[20:24])


    # pos servos:
    head_phi_pulse = unpack_to_int(bytearr_[24:26])
    head_theta_pulse = unpack_to_int(bytearr_[26:28])

    # readtime:
    readtime = unpack_to_int(bytearr_[28:32], 'I')

    # mode:
    mode = bytearr_[32]

    return readtime, np.array([ax, ay, az]), np.array([wx, wy, wz]), head_phi_pulse, head_theta_pulse, mode

def make_ctrl(select, val1, val2, val3=0, val4=0):

    def constrain(i):
        return int(max(-32768, min(i, 32768-1)))

    def input2bytes(i,j,k,l,m):

        b = constrain(i).to_bytes(1, byteorder='little')
        for v in [val1, val2, val3, val4]:
            b += constrain(v).to_bytes(2, byteorder='big', signed=True)
        #j = constrain(j).to_bytes(2, byteorder='big', signed=True)
        #k = constrain(k).to_bytes(2, byteorder='big', signed=True)
        return b #i+j+k

    return input2bytes(select,val1,val2,val3, val4)




import struct

def depack(bytearr_):

    def unpack_to_float(b):
        #b = bytearray(valarr)
        return struct.unpack('f', b)[0]

    def unpack_to_int(b):
        #b = bytearray(valarr)
        return struct.unpack('H', b)[0]

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

    # mode:
    mode = bytearr_[28]

    return ax, ay, az, wx, wy, wz, head_phi_pulse, head_theta_pulse, mode

def make_ctrl(i,j,k):

    def constrain(i):
        return int(max(-32768, min(i, 32768-1)))

    def input2bytes(i,j,k):

        i = constrain(i).to_bytes(1, byteorder='little')
        j = constrain(j).to_bytes(2, byteorder='big', signed=True)
        k = constrain(k).to_bytes(2, byteorder='big', signed=True)
        return i+j+k

    return input2bytes(i,j,k)




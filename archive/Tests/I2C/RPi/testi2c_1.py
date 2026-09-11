from smbus2 import SMBus, i2c_msg
import time
import struct
import numpy as np

# Slave Addresses
I2C_SLAVE_ADDRESS = 11 #0x0b ou 11


def depack_i2c(intarr):

    def unpack_to_float(valarr):
        b = bytearray(valarr)
        return struct.unpack('f', b)[0]

    ax = unpack_to_float(intarr[0:4])
    ay = unpack_to_float(intarr[4:8])
    az = unpack_to_float(intarr[8:12])
    print(ax, ay, az)

# Create the I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    while True:
        for i in range(2**8-1):
            #print("Sending ", str(i))
            #I2Cbus.write_byte(I2C_SLAVE_ADDRESS, ord("a"))
            #time.sleep(.5)

            #I2Cbus.write_i2c_block_data(I2C_SLAVE_ADDRESS, 0x00, 2*[ord("a")])
            #time.sleep(.5)


            #TODO: why does not reading work?
            msg = i2c_msg.read(I2C_SLAVE_ADDRESS, 32)
            I2Cbus.i2c_rdwr(msg)
            depack_i2c(list(msg))


            time.sleep(.5)

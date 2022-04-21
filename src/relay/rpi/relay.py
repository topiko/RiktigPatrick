import time
import socket

from smbus2 import SMBus, i2c_msg

NBYTESI2C = 29
I2C_SLAVE_ADDRESS = 11
HOST = socket.gethostbyname('topikone.local') #'192.168.0.45'
PORT = 1024

def read_arduino(I2Cbus):

    msg = i2c_msg.read(I2C_SLAVE_ADDRESS, NBYTESI2C)
    I2Cbus.i2c_rdwr(msg)

    return bytearray(list(msg))

def write_arduino(I2Cbus, ctrl_input):
    I2Cbus.write_i2c_block_data(I2C_SLAVE_ADDRESS,
                                0x00,
                                ctrl_input)



# Create the I2C bus
with SMBus(1) as I2Cbus:
    I2Cbus.pec=1
    i = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        t0 = time.time()
        while True:
            try:
                statebytes = read_arduino(I2Cbus)

                nsent = s.send(statebytes, NBYTESI2C+1) # There is a ending byte?

                if nsent==NBYTESI2C:
                    ctrl_input = bytearray([])

                    # Wait for the full ctrl_input to be received:
                    while len(ctrl_input) != 5:
                        ctrl_input += s.recv(5)

                    # Write it to arduino:
                    write_arduino(I2Cbus, ctrl_input)
                else:
                    raise ValueError(f'Incorrect number of bytes sent: {nsent} != {NBYTESI2C}')
            except KeyboardInterrupt:
                break

            i += 1
        t1 = time.time()

        #s.send(b"")

print('Period: {:.3f} ms'.format((t1-t0)/i*1000))




import time
import socket
import struct

HOST = socket.gethostbyname('topikone.local') #'192.168.0.45'
PORT = 1024
INC = 10
tobytes = lambda val, fmt='f': bytearray(struct.pack(fmt, val))

def depack_KB(bytearray_ : bytearray) -> tuple[float, float]:

    phispeed = struct.unpack('f', bytearray_[0:4])[0]
    thetaspeed = struct.unpack('f', bytearray_[4:8])[0]
    return phispeed, thetaspeed

if __name__ == '__main__':
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        phi, theta = 0., 0.
        exit = False
        mode = 1
        try:
            while not exit:
                ctrl = input('Q - quit, s-stop, [h,j,k,l] to steer: ')
                if ctrl == 'h':
                    phi -= INC
                elif ctrl == 'l':
                    phi += INC
                elif ctrl == 'j':
                    theta -= INC
                elif ctrl == 'k':
                    theta += INC
                elif ctrl == 'Q':
                    phi, theta = 0, 0
                    exit = True
                elif ctrl == 's':
                    phi, theta = 0, 0
                elif ctrl == 'm':
                    mode = not mode

                if ctrl != 'm':
                    bytearray_ = tobytes(phi) + tobytes(theta) + tobytes(128, '>H')
                else:
                    if mode:
                        bytearray_ = tobytes(130, '>H')
                    else:
                        bytearray_ = tobytes(129, '>H')
                s.sendall(bytearray_)

        except KeyboardInterrupt:
            print('Quit')
        finally:
            # Disconnect:
            s.send(b'')

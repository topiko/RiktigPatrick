import socket
import selectors
import types
import struct
import logging

from relay.conversions import depack, make_ctrl
from riktigpatric.patrick import RPatrick

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)

LOG = logging.Logger('rp_logger')

sel = selectors.DefaultSelector()

HOST = socket.gethostbyname('topikone.local') # "192.168.0.13"
PORT = 1024
NBYTES = 29
MAXBYTES = 128

lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
lsock.bind((HOST, PORT))
lsock.listen()
print(f'listening at: {HOST}:{PORT}')
lsock.setblocking(False)
sel.register(lsock, selectors.EVENT_READ, data=None)

# Thsi is the RP!
rp = RPatrick()


def accept_wrapper(sock : socket.socket):
    conn, addr = sock.accept()
    print(f'Accepting connection: {addr}')
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)


def service_connection(key, mask):
    sock = key.fileobj
    data = key.data

    if mask & selectors.EVENT_READ:

        #recv_data = bytearray()
        #while len(recv_data) < NBYTES:
        recv_data = sock.recv(MAXBYTES)
        #    if recv_data == b'':
                # on empty byte temirnate.
        #        break

        if recv_data:
            # Set state and fetch control
            rp.state = recv_data
            data.outb = rp.get_ctrl()
        else:
            print(f'Empty message -> closing connection {data.addr}')
            sel.unregister(sock)
            sock.close()
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            sent = sock.send(data.outb)
            data.outb = b''


if __name__ == "__main__":
    try:
        while True:
            events = sel.select(timeout=None)
            for key, mask in events:
                if key.data is None:
                    accept_wrapper(key.fileobj)
                else:
                    service_connection(key, mask)
    except KeyboardInterrupt or Exception as e:
        print('Exit', e)
    finally:
        sel.close()


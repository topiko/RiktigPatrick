import socket
import selectors
import types
import logging
import argparse

from relay.conversions import depack
from monitor.monitor import get_monitor_socket

from riktigpatric.patrick import RPatrick

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)

LOG = logging.getLogger()

# Arguments:
parser = argparse.ArgumentParser(description='Run the base service for RiktigPatrick')
parser.add_argument('--monitor', type=bool, default=False, help='report to monitor service.')
args = parser.parse_args()


def accept_wrapper(sock : socket.socket):
    conn, addr = sock.accept()
    LOG.info(f'Accepting connection: {addr}')
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)


def service_connection(key, mask):
    sock = key.fileobj
    data = key.data

    if mask & selectors.EVENT_READ:

        recv_data = sock.recv(MAXBYTES)

        if recv_data:
            # Set state and fetch control
            key, depacked, resp = depack(recv_data)
            rp.state = key, depacked

            if resp:
                data.outb = rp.get_ctrl()
            else:
                data.outb = b''
        else:
            print(f'Empty message -> closing connection {data.addr}')
            sel.unregister(sock)
            sock.close()

    if mask & selectors.EVENT_WRITE:
        if data.outb:
            sent = sock.send(data.outb)
            data.outb = b''



sel = selectors.DefaultSelector()

HOSTNAME = socket.gethostname() # "192.168.0.13"
HOST = socket.gethostbyname('f{HOSTNAME}.local') #'192.168.0.45'
PORT = 1024
MAXBYTES = 128

lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
lsock.bind((HOST, PORT))
lsock.listen()
LOG.info(f'Listening at: {HOST}:{PORT}')
lsock.setblocking(False)
sel.register(lsock, selectors.EVENT_READ, data=None)


# Thsi is the RP!

if args.monitor:
    report_sock = get_monitor_socket(LOG)
else:
    report_sock = None
rp = RPatrick(report_sock=report_sock)


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
        report_sock.send(b'')
        sel.close()

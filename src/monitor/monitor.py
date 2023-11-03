import socket
import logging
import time

import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)

LOG = logging.getLogger('monitor_logger')

HOSTNAME = socket.gethostname()
HOST = socket.gethostbyname('{HOSTNAME}.local') #'192.168.0.45'
REPORTPORT = 1026


def get_monitor_socket(log):
    report_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
        try:
            report_sock.connect((HOST, REPORTPORT))
            break
        except ConnectionRefusedError:
            log.error('Failed to connect monitor. Retrying!')
            time.sleep(2)

    log.info(f'Report socket bound to "{HOST}:{REPORTPORT}".')
    return report_sock


if __name__ == '__main__':
    LOG.info('Running at: {}'.format(socket.gethostname()))
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, REPORTPORT))
        s.listen()
        LOG.info(f'Monitor: listening at "{HOST}:{REPORTPORT}"')

        conn, addr = s.accept()
        with conn:
            LOG.info('Accepting connection from {}'.format(socket.gethostbyaddr(addr[0])))
            while True:
                bytes_ = conn.recv(1024)
                if bytes_!=b'':
                    arr = np.frombuffer(bytes_, 'f')
                    print(arr)

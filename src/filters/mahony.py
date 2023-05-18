import numpy as np
import quaternion as qt
import logging
from filters.qutils import eul2q, q2eul

LOG = logging.getLogger()


class Mahony:
    def __init__(self):
        self._qhat = None
        self.bhat = None  # set in reset
        self.g = np.array([0, 0, -1])
        self.kp = 1
        self.ki = 0.3
        self.reset()

    def update(self, acc: np.ndarray, gyro: np.ndarray, dt: float):
        assert dt > 0

        if not np.isclose(np.linalg.norm(acc), 1):
            acc_ = acc/np.linalg.norm(acc)
        else:
            acc_ = acc

        v = acc_
        R = qt.as_rotation_matrix(self.qhat)
        vhat = -R.T @ self.g

        w_mes = np.cross(v, vhat)

        # P quaternion:
        p = qt.quaternion(0)
        p.vec = gyro - self.bhat + self.kp * w_mes

        # Q dot:
        qhatdot = 1 / 2.0 * self._qhat * p

        # bhat dot:
        bhatdot = -self.ki * w_mes

        # Update:
        self._qhat += dt * qhatdot

        # assert np.isclose(self.qhat.norm(), 1)

        self._qhat /= self.qhat.norm()

        self.bhat += dt * bhatdot
        self._euler = q2eul(self.qhat)

    @property
    def qhat(self) -> qt.quaternion:
        return self._qhat

    @property
    def eul(self) -> np.ndarray:
        return self._euler / np.pi * 180

    def reset(self):
        self.bhat = np.zeros(3)
        self._qhat = eul2q(0, 0, 0)
        self._euler = np.zeros(3)

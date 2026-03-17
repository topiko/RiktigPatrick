import logging

import numpy as np

from filters.qutils import Quaternion, as_rotation_matrix, eul2q, q2eul

LOG = logging.getLogger()


class Mahony:
    def __init__(self):
        self._qhat = None
        self.bhat = None
        self.g = np.array([0, 0, -1])
        self.kp = 1
        self.ki = 0.3
        self.reset()

    def update(self, acc: np.ndarray, gyro: np.ndarray, dt: float):
        assert dt > 0

        if not np.isclose(np.linalg.norm(acc), 1):
            acc_ = acc / np.linalg.norm(acc)
        else:
            acc_ = acc

        v = acc_
        R = as_rotation_matrix(self.qhat)
        vhat = -R.T @ self.g

        w_mes = np.cross(v, vhat)

        p_vec = gyro - self.bhat + self.kp * w_mes
        p = Quaternion(0, p_vec[0], p_vec[1], p_vec[2])

        qhatdot = self._qhat * p * 0.5

        bhatdot = -self.ki * w_mes

        self._qhat = self._qhat + dt * qhatdot

        self._qhat = self._qhat.normalize()

        self.bhat = self.bhat + dt * bhatdot
        self._euler = q2eul(self._qhat)

    @property
    def qhat(self):
        return self._qhat

    @property
    def eul(self) -> np.ndarray:
        return self._euler / np.pi * 180

    def reset(self):
        self.bhat = np.zeros(3)
        self._qhat = eul2q(0, 0, 0)
        self._euler = np.zeros(3)

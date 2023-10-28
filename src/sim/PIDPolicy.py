import numpy as np


class PIDPolicy:
    def __init__(
        self, kp: float, ki: float, kd: float, dt: float, lock_head: bool = True
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._dt = dt
        self._lock_head = lock_head

    def sample_action(self, obs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        P = obs["filter/rp_pitch"] + .85
        D = obs["sens/gyro"][1]
        I = 0

        print(P, D)

        a = self._kp * P + self._ki * I + self._kd * D
        v = obs["sens/left_wheel_vel"][0] + a * self._dt
        v = np.array([v])

        d = {
            "act/left_wheel": v,
            "act/right_wheel": v,
        }
        if self._lock_head:
            return d

        d["act/head_pitch"] = np.array([0])
        d["act/head_turn"] = np.array([0])

        return d

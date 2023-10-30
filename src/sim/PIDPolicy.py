import numpy as np


class PIDPolicy:
    def __init__(
        self, kp: float, ki: float, kd: float, dt: float, lock_head: bool = True
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._dt = dt
        self._stable_pitch = -3.0
        self._lock_head = lock_head

    def sample_action(
        self, obs: dict[str, np.ndarray], target_pitch: float = 0
    ) -> dict[str, np.ndarray]:
        P = obs["filter/rp_pitch"] - self._stable_pitch - target_pitch
        try:
            D = obs["sens/gyro"][1]
        except KeyError:
            D = 0
        I = 0

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

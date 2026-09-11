import numpy as np
import torch

from sim.envs.rp_env import MAXV, GymRP
from sim.nets import AccelPolicyNetwork, VelocityPolicyNetwork
from sim.sim_config import MODEL_INPUT, N_ACTIONS, POLICY_TYPE


class NetPolicy:
    def __init__(self):
        if POLICY_TYPE == "velocity":
            self._net = VelocityPolicyNetwork.from_file()
        elif POLICY_TYPE == "acceleration":
            self._net = AccelPolicyNetwork.from_file()
        self._lock_head = True

    def sample_action(self, obs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        obs_t = torch.concatenate([torch.Tensor(obs[k]) for k in MODEL_INPUT]).view(
            1, -1
        )

        if POLICY_TYPE == "velocity":
            action_means, _, _ = self._net(obs_t)
            action = action_means.detach().numpy().astype(float)
            action = action[0]
            if any(abs(action) > MAXV):
                raise ValueError()
            return {
                "act/velocity_left_wheel": action[0:1].reshape(1, 1),
                "act/velocity_right_wheel": action[1:2].reshape(1, 1),
            }

        elif POLICY_TYPE == "acceleration":
            left_logits, right_logits, _ = self._net(obs_t)
            left_idx = torch.argmax(left_logits, dim=-1).cpu().numpy()
            right_idx = torch.argmax(right_logits, dim=-1).cpu().numpy()
            return {
                "act/acceleration_left_wheel": left_idx,
                "act/acceleration_right_wheel": right_idx,
            }


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

        if POLICY_TYPE == "velocity":
            v = obs["sens/left_wheel_vel"][0] + a * self._dt
            v = np.array([v])
            np.clip(v, -MAXV, MAXV, out=v)
            return {
                "act/velocity_left_wheel": v.reshape(1, 1),
                "act/velocity_right_wheel": v.reshape(1, 1),
            }

        elif POLICY_TYPE == "acceleration":
            idx = int((a / MAXV + 1) * (N_ACTIONS - 1) / 2)
            idx = max(0, min(N_ACTIONS - 1, idx))
            return {
                "act/acceleration_left_wheel": np.array([idx]),
                "act/acceleration_right_wheel": np.array([idx]),
            }

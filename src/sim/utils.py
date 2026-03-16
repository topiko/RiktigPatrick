from __future__ import annotations

import gymnasium as gym
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import torch
from gymnasium.envs.registration import register
from gymnasium import spaces

from sim.sim_config import POLICY_TYPE


def model_indim(
    rpenv: gym.Env | gym.vector.AsyncVectorEnv, model_input: list[str]
) -> int:
    if isinstance(rpenv, gym.vector.AsyncVectorEnv):
        obs_space = rpenv.single_observation_space
    else:
        obs_space = rpenv.observation_space

    return sum(v.shape[0] for k, v in obs_space.items() if k in model_input)


def actiondim(rpenv: gym.Env | gym.vector.AsyncVectorEnv) -> int:
    if POLICY_TYPE == "velocity":
        if isinstance(rpenv, gym.vector.AsyncVectorEnv):
            act_space = rpenv.single_action_space
        else:
            act_space = rpenv.action_space
        return sum(v.shape[0] for v in act_space.values())
    elif POLICY_TYPE == "acceleration":
        return 2  # Two wheels


def register_and_make_env(
    env_config: dict,
    obs_space: list[str],
    vector_env: bool = False,
    batch_size: int = 1,
) -> gym.Env | gym.vector.AsyncVectorEnv:
    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs=env_config,
    )

    if vector_env:
        return gym.vector.AsyncVectorEnv(
            [
                lambda: gym.make(
                    "RiktigPatrick-v0",
                    state_keys=obs_space,
                    render_mode="rgb_array",
                    disable_env_checker=True,
                )
                for _ in range(batch_size)
            ]
        )

    return gym.make(
        "RiktigPatrick-v0",
        state_keys=obs_space,
        render_mode="rgb_array",
        disable_env_checker=True,
    )


class Tape:
    def __init__(self, idx: int):
        self.idx = idx
        self.probs = []
        self.values = []
        self.rewards = []
        self.entropies = []
        self._is_ready = False

    @property
    def ep_return(self) -> float:
        return sum(self.rewards)

    def build(self) -> Tape:
        self._is_ready = True
        self.probs = torch.stack(self.probs, axis=0)
        self.rewards = np.array(self.rewards)
        self.values = torch.concat(self.values, axis=0)
        self.entropies = torch.stack(self.entropies, axis=0)
        return self

    def __len__(self) -> int:
        return len(self.probs)


def dict2tensor(obs_d: dict[str, np.ndarray]) -> torch.Tensor:
    ndim = list(obs_d.values())[0].ndim
    if ndim == 1:
        obs_arr = np.concatenate(list(obs_d.values()), axis=0)
        obs_arr = obs_arr[np.newaxis, :]  # Add batch dimension
    elif ndim == 2:
        obs_arr = np.concatenate(list(obs_d.values()), axis=1)
    else:
        raise TypeError()

    return torch.Tensor(obs_arr)

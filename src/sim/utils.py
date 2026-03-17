from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch
from gymnasium.envs.registration import register
from omegaconf import DictConfig


def register_and_make_env(
    cfg: DictConfig,
) -> gym.Env | gym.vector.AsyncVectorEnv:
    env_config = dict(cfg.env)
    n_parallel = env_config.pop("n_parallel", 1)

    env_config["actions"] = list(cfg.policy.actions)

    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs=env_config,
    )

    if n_parallel> 1:
        return gym.vector.AsyncVectorEnv(
            [
                lambda: gym.make(
                    "RiktigPatrick-v0",
                    disable_env_checker=True,
                    **env_config,
                )
                for _ in range(n_parallel)
            ]
        )

    return gym.make(
        "RiktigPatrick-v0",
        disable_env_checker=True,
        **env_config,
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
        self.probs = torch.stack(self.probs, dim=0)
        self.rewards = np.array(self.rewards)
        self.values = torch.cat(self.values, dim=0)
        self.entropies = torch.stack(self.entropies, dim=0)
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

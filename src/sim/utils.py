from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch
from gymnasium.envs.registration import register
from omegaconf import DictConfig

from riktigpatric.patrick import Actions, Observables


def register_and_make_env(
    cfg: DictConfig,
) -> gym.Env | gym.vector.AsyncVectorEnv:
    env_config = dict(cfg.env)
    n_parallel = env_config.pop("n_parallel", 1)

    # Extract action keys from new config format
    # Actions are now dicts like: {'act/accelerate_both_wheels': {'type': 'discrete', ...}}
    action_keys = [list(action_item.keys())[0] for action_item in cfg.policy.actions]
    env_config["actions"] = action_keys
    env_config["max_wheel_vel"] = cfg.env.max_wheel_vel
    env_config["max_wheel_acc"] = cfg.env.max_wheel_acc

    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs=env_config,
    )

    if n_parallel > 1:
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


class Episode:
    """Episode data container with attribute-based access.

    Provides direct access to observations, actions, and rewards using enum names.
    All arrays have shape: (num_steps, num_envs, dim)

    Example:
        eps = Episode(obs_l, action_l, rewards_l)
        pitch = eps.RP_PITCH           # (num_steps, num_envs, 1)
        gyro = eps.GYRO                # (num_steps, num_envs, 3)
        wheel_acc = eps.ACC_BOTH_WHEELS  # (num_steps, num_envs, 1)
        rewards = eps.rewards           # (num_steps, num_envs)

    Available observation attributes (from Observables enum):
        .ACC, .GYRO, .HEAD_PITCH, .HEAD_TURN, .LEFT_WHEEL_VEL,
        .RIGHT_WHEEL_VEL, .RP_PITCH, .TRUE_PITCH, .OBS_TIME

    Available action attributes (from Actions enum):
        .ACC_BOTH_WHEELS, .VEL_HEAD_PITCH, etc.
    """

    def __init__(
        self,
        obs_l: list[dict[Observables, np.ndarray]],
        action_l: list[dict[str, np.ndarray]],
        rewards_l: list[dict[str, np.ndarray]],
    ):
        """Initialize episode from rollout data.

        Args:
            obs_l: List of observation dicts (keys are Observables enum)
            action_l: List of action dicts (keys are strings)
            rewards_l: List of reward_info dicts (keys are strings)
        """
        # Stack observations over time: (num_steps, num_envs, obs_dim)
        if obs_l:
            for obs_key in obs_l[0].keys():
                # obs_key is Observables.ACC, use obs_key.name to get "ACC"
                obs_array = np.stack([obs[obs_key] for obs in obs_l], axis=0)
                setattr(self, obs_key.name, obs_array)

        # Stack actions over time: (num_steps, num_envs, action_dim)
        if action_l:
            for action_key_str in action_l[0].keys():
                action_array = np.stack(
                    [act[action_key_str] for act in action_l], axis=0
                )

                # Try to find matching Actions enum for nice attribute name
                action_enum = None
                for a in Actions:
                    if a.value == action_key_str:
                        action_enum = a
                        break

                if action_enum:
                    # Use enum name: "ACC_BOTH_WHEELS", "VEL_HEAD_PITCH"
                    setattr(self, action_enum.name, action_array)
                else:
                    # Fallback: sanitize string
                    # "act/accelerate_both_wheels" -> "accelerate_both_wheels"
                    attr_name = action_key_str.replace("act/", "").replace("/", "_")
                    setattr(self, attr_name, action_array)

        # Stack reward components over time
        self.reward_components = {}
        if rewards_l:
            for reward_key in rewards_l[0].keys():
                reward_array = np.stack([r[reward_key] for r in rewards_l], axis=0)
                self.reward_components[reward_key] = reward_array

            # Store total reward if available
            if "step_reward" in self.reward_components:
                self.rewards = self.reward_components["step_reward"]

    def __repr__(self) -> str:
        """Return string representation listing available attributes."""
        obs_attrs = [
            attr for attr in dir(self) if not attr.startswith("_") and attr.isupper()
        ]
        action_attrs = [
            attr
            for attr in dir(self)
            if not attr.startswith("_") and attr.isupper() and hasattr(Actions, attr)
        ]

        return f"Episode(observations={obs_attrs}, actions={action_attrs}, num_steps={getattr(self, obs_attrs[0], np.array([])).shape[0] if obs_attrs else 0})"

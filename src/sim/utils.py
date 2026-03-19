from __future__ import annotations

from enum import Enum

import gymnasium as gym
import numpy as np
import torch
from gymnasium.envs.registration import register
from omegaconf import DictConfig

from riktigpatric.patrick import Actions, Observables


class MiscKeys(str, Enum):
    """Miscellaneous constants that don't fit into Actions or Observables."""

    # Time step of the environment (seconds)
    VALUES_ESTIM = "value_estimates"
    RETURNS = "returns"
    ADVANTAGES = "advantages"

    @classmethod
    def from_str(cls, label: str) -> MiscKeys:
        """Convert string to MiscKeys enum, case-insensitive."""
        label = label.upper()
        for key in MiscKeys:
            if key.value.upper() == label:
                return key
        raise ValueError(f"Unknown MiscKeys value: {label}")


def _add_batch_dim(
    data: dict | np.ndarray | torch.Tensor | float | bool,
) -> dict | np.ndarray | torch.Tensor:
    """Add batch dimension to data if needed.

    Args:
        data: dict, np.ndarray, torch.Tensor, or scalar value

    Returns:
        Data with batch dimension added where ndim == 1
        - dict: Recursively add batch dim to values
        - np.ndarray with ndim==1: shape (N,) -> (1, N)
        - torch.Tensor with ndim==1: shape (N,) -> (1, N)
        - scalar: converted to np.array([scalar])
    """
    if isinstance(data, dict):
        return {k: _add_batch_dim(v) for k, v in data.items()}
    elif isinstance(data, np.ndarray):
        if data.ndim == 1:
            return data[np.newaxis, :]
        else:
            return data
    elif isinstance(data, torch.Tensor):
        if data.ndim == 1:
            return data.unsqueeze(0)
        else:
            return data
    else:
        # Scalar (float, bool, int) - convert to array with batch dim
        return np.array([data])


def _remove_batch_dim(
    data: dict | np.ndarray | torch.Tensor,
) -> dict | np.ndarray | torch.Tensor | float:
    """Remove batch dimension from data (assumes batch size = 1).

    Args:
        data: dict, np.ndarray, or torch.Tensor with batch dimension

    Returns:
        Data with batch dimension removed
        - dict: Recursively remove batch dim from values
        - np.ndarray with shape[0]==1: shape (1, N) -> (N,)
        - torch.Tensor with shape[0]==1: shape (1, N) -> (N,)
        - Otherwise: return as-is
    """
    if isinstance(data, dict):
        return {k: _remove_batch_dim(v) for k, v in data.items()}
    elif isinstance(data, np.ndarray):
        if data.shape[0] == 1:
            return data[0]
        else:
            return data
    elif isinstance(data, torch.Tensor):
        if data.shape[0] == 1:
            return data[0]
        else:
            return data
    else:
        return data


class SingleEnvWrapper:
    """Minimal wrapper to add/remove batch dimension for single environment.

    Makes single env compatible with rollout that expects batched format.
    Only wraps step() to handle batch dimension - other methods pass through.

    Usage:
        single_env = gym.make("MyEnv-v0")
        batched_env = SingleEnvWrapper(single_env)
        # Now returns batched format: obs (1, dim), reward (1,), etc.
    """

    def __init__(self, env):
        self.env = env
        self.num_envs = 1  # Signal that this provides batched format

    def reset(self, seed=None):
        """Add batch dimension to reset output.

        Args:
            seed: Random seed

        Returns:
            Batched obs_d (1, dim) and info
        """
        obs_d, info = self.env.reset(seed=seed)
        obs_d_batched = _add_batch_dim(obs_d)
        return obs_d_batched, info

    def step(self, action_d: dict) -> tuple:
        """Remove batch dim before step, add it back after.

        Args:
            action_d: Dict with batched actions, shape (1, dim)

        Returns:
            Batched outputs: obs_d (1, dim), reward (1,), terminated (1,), truncated (1, 1), reward_info
        """
        # Remove batch dimension: (1, dim) -> (dim)
        action_single = _remove_batch_dim(action_d)

        # Call underlying single env
        obs_d, reward, terminated, truncated, reward_info = self.env.step(action_single)

        # Add batch dimension back
        obs_d_batched = _add_batch_dim(obs_d)
        reward_batched = _add_batch_dim(reward)
        terminated_batched = _add_batch_dim(terminated)
        truncated_batched = _add_batch_dim(truncated)

        # Add batch dimension to reward_info values for consistent Episode stacking
        reward_info_batched = {k: _add_batch_dim(v) for k, v in reward_info.items()}

        return (
            obs_d_batched,
            reward_batched,
            terminated_batched,
            truncated_batched,
            reward_info_batched,
        )

    def __getattr__(self, name):
        """Pass through any other attributes/methods to underlying env."""
        return getattr(self.env, name)

    def __setattr__(self, name, value):
        """Set attributes - special handling for 'env' and 'num_envs', pass others through."""
        if name in ("env", "num_envs"):
            # Set on wrapper itself
            object.__setattr__(self, name, value)
        else:
            # Pass through to underlying env (e.g., RecordVideo.name_prefix)
            setattr(self.env, name, value)


def register_and_make_env(
    cfg: DictConfig, force_single_env: bool = False
) -> gym.Env | gym.vector.AsyncVectorEnv:
    env_config = dict(cfg.env)
    n_parallel = env_config.pop("n_parallel", 1)
    if force_single_env:
        n_parallel = 1

    env_config["actions"] = list(cfg.policy.actions.keys())

    config_ = env_config.copy()
    # Map the config str values to Observables enum keys for reward scales
    config_["reward_scales"] = {
        Observables.from_str(k): v for k, v in dict(cfg.reward).items()
    }

    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs=config_,
    )

    if n_parallel > 1:
        return gym.vector.AsyncVectorEnv(
            [
                lambda: gym.make(
                    "RiktigPatrick-v0",
                    disable_env_checker=True,
                    **config_,
                )
                for _ in range(n_parallel)
            ]
        )

    return gym.make(
        "RiktigPatrick-v0",
        disable_env_checker=True,
        **config_,
    )


class Episode:
    """Episode data container for SINGLE episode with attribute-based access.

    Provides direct access to observations, actions, and rewards using enum names.
    All arrays have shape: (num_steps, dim) - NO batch/env dimension!

    IMPORTANT: Episode represents a SINGLE episode. If you have data from multiple
    environments, create separate Episode objects for each.

    Example:
        eps = Episode(obs_l, action_l, rewards_l)
        pitch = eps.RP_PITCH           # (num_steps, 1)
        gyro = eps.GYRO                # (num_steps, 3)
        wheel_acc = eps.ACC_BOTH_WHEELS  # (num_steps, 1)
        rewards = eps.rewards           # (num_steps,)

    Available observation attributes (from Observables enum):
        .ACC, .GYRO, .HEAD_PITCH, .HEAD_TURN, .LEFT_WHEEL_VEL,
        .RIGHT_WHEEL_VEL, .RP_PITCH, .TRUE_PITCH, .OBS_TIME

    Available action attributes (from Actions enum):
        .ACC_BOTH_WHEELS, .VEL_HEAD_PITCH, etc.
    """

    def __init__(
        self,
        obs_l: list[dict[Observables, np.ndarray]],
        action_l: list[dict[Actions, np.ndarray]],
        value_estimates: np.ndarray | None = None,
        returns: np.ndarray | None = None,
        advantages: np.ndarray | None = None,
    ):
        """Initialize episode from rollout data for SINGLE episode.

        Args:
            obs_l: List of observation dicts (keys are Observables enum)
                   Each obs value should have shape (dim,) for single episode
            action_l: List of action dicts (keys are strings)
                      Each action value should have shape (dim,) for single episode

        Raises:
            ValueError: If input data contains batch dimension (multiple environments)
        """
        # Check that we have single episode data, not batched
        first_obs_value = list(obs_l[0].values())[0]
        if first_obs_value.shape[0] != 1:
            raise ValueError(
                f"Episode expects single episode data, but got batched data with "
                f"shape {first_obs_value.shape}. Extract single environment first."
            )

        # Stack over time: (T, dim_)
        for obs_key in obs_l[0].keys():
            # (T, obs_dim)
            obs_array = np.concat([obs[obs_key] for obs in obs_l], axis=0)

            setattr(self, f"OBS_{obs_key.name}", obs_array)

        # Stack actions over time: (num_steps, action_dim)
        for action_key in action_l[0].keys():
            action_array = np.concat([act[action_key] for act in action_l], axis=0)

            # (T, action_dim)
            setattr(self, f"ACT_{action_key.name}", action_array)

        for key, arr in (
            (MiscKeys.VALUES_ESTIM, value_estimates),
            (MiscKeys.RETURNS, returns),
            (MiscKeys.ADVANTAGES, advantages),
        ):
            if arr is None:
                continue

            if arr.shape[0] != 1:
                raise ValueError(
                    f"Expected {key} to have batch size 1, got shape {key.shape}"
                )

            arr = arr.T  # (T, 1)

            if arr.shape[0] != action_array.shape[0]:
                raise ValueError(
                    f"{key} length {arr.shape[0]} does not match number of steps {action_array.shape[0]}"
                )

            # (T, 1)
            setattr(self, f"MISC_{key.name}", arr)

    def get_data(self, key: Actions | Observables | MiscKeys) -> np.ndarray:
        if isinstance(key, Observables):
            return self.get_observable(key)
        if isinstance(key, Actions):
            return self.get_action(key)
        if isinstance(key, MiscKeys):
            return self.get_misc(key)

        raise ValueError(
            f"Key must be an instance of Observables or Actions enum, got {type(key)}"
        )

    def get_observable(self, obs: Observables) -> np.ndarray:
        """Get observable array by enum key."""
        return getattr(self, f"OBS_{obs.name}")

    def get_action(self, act: Actions) -> np.ndarray:
        """Get action array by enum key."""
        return getattr(self, f"ACT_{act.name}")

    def get_misc(self, misc_key: MiscKeys) -> np.ndarray:
        """Get miscellaneous data array by enum key."""
        return getattr(self, f"MISC_{misc_key.name}")

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

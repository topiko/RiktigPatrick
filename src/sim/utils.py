from __future__ import annotations

import gymnasium as gym
import numpy as np
import torch
from gymnasium.envs.registration import register
from omegaconf import DictConfig

from riktigpatric.patrick import Actions, Observables


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
        action_l: list[dict[str, np.ndarray]],
        rewards_l: list[dict[str, np.ndarray]],
    ):
        """Initialize episode from rollout data for SINGLE episode.

        Args:
            obs_l: List of observation dicts (keys are Observables enum)
                   Each obs value should have shape (dim,) for single episode
            action_l: List of action dicts (keys are strings)
                      Each action value should have shape (dim,) for single episode
            rewards_l: List of reward_info dicts (keys are strings)
                       Each reward value should be scalar for single episode

        Raises:
            ValueError: If input data contains batch dimension (multiple environments)
        """
        # Check that we have single episode data, not batched
        if obs_l:
            first_obs_value = list(obs_l[0].values())[0]
            if first_obs_value.ndim > 1 and first_obs_value.shape[0] > 1:
                raise ValueError(
                    f"Episode expects single episode data, but got batched data with "
                    f"shape {first_obs_value.shape}. Extract single environment first."
                )

        # Stack observations over time: (num_steps, obs_dim)
        if obs_l:
            for obs_key in obs_l[0].keys():
                # obs_key is Observables.ACC, use obs_key.name to get "ACC"
                obs_array = np.stack([obs[obs_key] for obs in obs_l], axis=0)
                # Remove batch dim if present (shape (num_steps, 1, dim) -> (num_steps, dim))
                if obs_array.ndim == 3 and obs_array.shape[1] == 1:
                    obs_array = obs_array[:, 0, :]
                setattr(self, obs_key.name, obs_array)

        # Stack actions over time: (num_steps, action_dim)
        if action_l:
            for action_key_str in action_l[0].keys():
                action_array = np.stack(
                    [act[action_key_str] for act in action_l], axis=0
                )
                # Remove batch dim if present
                if action_array.ndim == 3 and action_array.shape[1] == 1:
                    action_array = action_array[:, 0, :]

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

        # Stack reward components over time: (num_steps,)
        self.reward_components = {}
        if rewards_l:
            for reward_key in rewards_l[0].keys():
                reward_array = np.stack([r[reward_key] for r in rewards_l], axis=0)
                # Remove batch dim if present
                if reward_array.ndim == 2 and reward_array.shape[1] == 1:
                    reward_array = reward_array[:, 0]
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

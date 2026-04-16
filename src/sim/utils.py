from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from functools import wraps

import gymnasium as gym
import numpy as np
import torch
from gymnasium.envs.registration import register
from omegaconf import DictConfig

from riktigpatric.patrick import Actions, Observable, StateVarKey


class MiscKeys(str, Enum):
    """Miscellaneous constants that don't fit into Actions or Observables."""

    REWARDS = "rewards"
    LOGPS = "log_probs"

    VALUE_ESTIM = "value_estimates"
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

    def reset(self, seed=None, options=None):
        """Add batch dimension to reset output.

        Args:
            seed: Random seed
            options: Unused, kept for VectorEnv.reset compatibility

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
    cfg: DictConfig, force_single_env: bool = False, force_non_random: bool = False
) -> gym.Env | gym.vector.VectorEnv:
    env_config = dict(cfg.env)
    n_parallel = env_config.pop("n_parallel", 1)
    if force_single_env:
        n_parallel = 1
    if force_non_random:
        env_config["randomize"] = False

    env_config["actions"] = list(cfg.policy.actions.keys())

    config_ = env_config.copy()
    # Map the config str values to Observables enum keys for reward scales
    config_["reward_scales"] = {
        Observable.from_str(k): v for k, v in dict(cfg.reward).items()
    }

    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs=config_,
    )

    if n_parallel > 1:
        return gym.vector.SyncVectorEnv(
            [
                lambda: gym.make(
                    "RiktigPatrick-v0",
                    disable_env_checker=True,
                    **config_,
                )
                for _ in range(n_parallel)
            ],
            autoreset_mode=gym.vector.AutoresetMode.NEXT_STEP,
        )

    return gym.make(
        "RiktigPatrick-v0",
        disable_env_checker=True,
        **config_,
    )


def flatten_dict(d, parent_key="", sep="_"):
    """Flatten nested dict for MLflow params."""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def npd2tensord(np_dict: dict) -> dict:
    return {k: torch.from_numpy(v).float() for k, v in np_dict.items()}


def tensord2npd(tensor_dict: dict) -> dict:
    return {k: v.cpu().numpy() for k, v in tensor_dict.items()}


def get_returns(rewards: torch.Tensor, discount: float) -> torch.Tensor:
    if rewards.ndim == 1:
        return get_returns(rewards.unsqueeze(0), discount)[0]

    if rewards.ndim != 2:
        raise ValueError(f"Expected 1D or 2D rewards, got shape {rewards.shape}")

    returns = torch.zeros_like(rewards)
    running_return = torch.zeros(rewards.shape[0], device=rewards.device)
    for i in reversed(range(rewards.shape[1])):
        running_return = rewards[:, i] + discount * running_return
        returns[:, i] = running_return
    return returns


def get_advantages(returns: torch.Tensor, values: torch.Tensor) -> torch.Tensor:
    if returns.shape != values.shape:
        raise ValueError(
            f"Returns and values must have the same shape, got {returns.shape} and {values.shape}"
        )
    advantages = returns - values
    return advantages


def _verify_len(kind: str, with_key: bool = False, offset: int = 0):
    def decorator(fun):
        @wraps(fun)
        def wrapper(self, *args, **kwargs):
            arr = fun(self, *args, **kwargs)
            expected_len = self.seq_len + offset
            if len(arr) != expected_len:
                if with_key and args:
                    key = args[0]
                    raise ValueError(
                        f"Expected {kind} {key} length {expected_len}, got {len(arr)}"
                    )
                raise ValueError(
                    f"Expected {kind} length {expected_len}, got {len(arr)}"
                )

            return arr

        return wrapper

    return decorator


@dataclass
class EpisodeBuffer:
    """Step-wise storage for one completed episode."""

    obs_l: list[dict[StateVarKey, np.ndarray]] = field(default_factory=list)
    action_l: list[dict[Actions, np.ndarray]] = field(default_factory=list)
    rewards_l: list[float] = field(default_factory=list)
    logps_l: list[torch.Tensor] = field(default_factory=list)
    values_l: list[torch.Tensor] = field(default_factory=list)
    finished: bool = False

    def add_step(
        self,
        obs_t: dict[StateVarKey, np.ndarray],
        action_t: dict[Actions, np.ndarray],
        reward_t: float,
        logp_t: torch.Tensor,
        value_t: torch.Tensor,
    ) -> None:
        if len(self.obs_l) > 0:
            # The reward at step t=0 is a dummy

            self.rewards_l.append(reward_t)
        self.obs_l.append(obs_t)
        self.action_l.append(action_t)
        self.logps_l.append(logp_t)
        self.values_l.append(value_t)

    def finish(self, final_obs: dict[StateVarKey, np.ndarray], reward: float) -> None:
        self.obs_l.append(final_obs)
        self.rewards_l.append(reward)

        self.finished = True

        if len(self.rewards_l) != len(self.obs_l) - 1:
            raise ValueError(
                f"Expected rewards length {len(self.obs_l) - 1}, got {len(self.rewards_l)}"
            )

        # Trigger decorated length checks.
        self.get_stvar_dict()
        self.get_action_dict()
        self.get_rewards()
        self.get_logps()
        self.get_values()

    @property
    def seq_len(self) -> int:
        if not self.finished:
            raise ValueError("EpisodeBuffer must be finished to get sequence length")
        return len(self.rewards_l)

    def get_action_dict(self) -> dict[Actions, np.ndarray]:
        d = {}
        for k in self.action_l[0].keys():
            d[k] = self.get_action(k)
        return d

    @_verify_len(kind="action", with_key=True)
    def get_action(self, key: Actions) -> np.ndarray:
        return np.stack([act_d[key] for act_d in self.action_l], axis=0)

    def get_stvar_dict(self) -> dict[Observable, np.ndarray]:
        d = {}
        for k in self.obs_l[0].keys():
            d[k] = self.get_observable(k)
        return d

    @_verify_len(kind="statevar", with_key=True, offset=1)
    def get_observable(self, key: StateVarKey) -> np.ndarray:
        return np.stack([obs_d[key] for obs_d in self.obs_l], axis=0)

    @_verify_len(kind="rewards")
    def get_rewards(self) -> torch.Tensor:
        return torch.tensor(self.rewards_l)

    @_verify_len(kind="logps")
    def get_logps(self) -> torch.Tensor:
        return torch.stack(self.logps_l)

    @_verify_len(kind="values")
    def get_values(self) -> torch.Tensor:
        return torch.stack(self.values_l)


def ebufs2batchd(
    ebuf_l: list[EpisodeBuffer], device: torch.DeviceObjType = "cpu"
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, np.ndarray, torch.Tensor]:
    maxlen = max(buf.seq_len for buf in ebuf_l)
    bs = len(ebuf_l)

    values = torch.zeros((bs, maxlen), device=device)
    logps = torch.zeros((bs, maxlen), device=device)
    valid_mask = torch.zeros((bs, maxlen), device=device)
    rewards = torch.zeros((bs, maxlen))
    seq_lens = np.zeros(bs)

    for i, buf in enumerate(ebuf_l):
        seq_len = buf.seq_len
        values[i, :seq_len] = buf.get_values()
        logps[i, :seq_len] = buf.get_logps().cpu()
        rewards[i, :seq_len] = buf.get_rewards()
        seq_lens[i] = seq_len
        valid_mask[i, :seq_len] = 1.0

    return logps, rewards, values, seq_lens, valid_mask


class Episode:
    """Episode data container for SINGLE finished episode with attribute access.

    Provides direct access to observations, actions, and rewards using enum names.
    All arrays have shape: (num_steps, dim) - NO batch/env dimension!

    IMPORTANT: FinishedEpisode represents a SINGLE episode. If you have data from
    multiple environments, create separate FinishedEpisode objects for each.

    Example:
        eps = FinishedEpisode(obs_l, action_l, rewards_l)
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
        epbuffer: EpisodeBuffer,
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
        if not epbuffer.finished:
            raise ValueError("EpisodeBuffer must be finished to create Episode")

        # Stack over time: (T, dim_)
        for obs_key, obs_arr in epbuffer.get_stvar_dict().items():
            # (T, obs_dim), (T,) if scalar obs like time

            setattr(self, f"STVAR_{obs_key.name}", obs_arr)

        for act_key, act_arr in epbuffer.get_action_dict().items():
            # (T, action_dim), (T,) if scalar action

            setattr(self, f"ACT_{act_key.name}", act_arr)

        for key, arr in (
            (MiscKeys.VALUE_ESTIM, value_estimates),
            (MiscKeys.RETURNS, returns),
            (MiscKeys.ADVANTAGES, advantages),
        ):
            if arr is None:
                continue

            if arr.shape[0] != 1:
                raise ValueError(
                    f"Expected {key} to have batch size 1, got shape {arr.shape}"
                )

            arr = arr[0]  # Remove batch dimension: (1, T) -> (T,)
            if arr.shape[0] != epbuffer.seq_len:
                raise ValueError(
                    f"{key} length {arr.shape[0]} does not match number of steps {epbuffer.seq_len}"
                )

            # (T,)
            setattr(self, f"MISC_{key.name}", arr)

    def get_data(self, key: Actions | StateVarKey | MiscKeys) -> np.ndarray:
        if isinstance(key, StateVarKey):
            return self.get_observable(key)
        if isinstance(key, Actions):
            return self.get_action(key)
        if isinstance(key, MiscKeys):
            return self.get_misc(key)

        raise ValueError(
            f"Key must be an instance of StateVarKey, Actions, or MiscKeys, got {type(key)}"
        )

    def get_observable(self, obs: StateVarKey) -> np.ndarray:
        """Get observable array by enum key."""
        return getattr(self, f"STVAR_{obs.name}")

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

        return (
            f"Episode(observations={obs_attrs}, actions={action_attrs}, "
            f"num_steps={getattr(self, obs_attrs[0], np.array([])).shape[0] if obs_attrs else 0})"
        )

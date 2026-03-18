"""Plotting utilities for Episode data visualization."""

import matplotlib.pyplot as plt
import numpy as np

from riktigpatric.patrick import Actions, Observables
from sim.utils import Episode


def plot_observations(eps: Episode, env_idx: int = 0, figsize=None):
    """Plot all observations for a single environment.

    Args:
        eps: Episode object with observation data
        env_idx: Which environment to plot (default: 0)
        figsize: Figure size tuple

    Returns:
        Figure and axes objects
    """
    # Collect available observations (filter out dunder methods and non-Observable attrs)
    obs_names = []
    for attr in dir(eps):
        if attr.startswith("_"):
            continue
        try:
            # Check if Observables has this enum member
            obs_enum = getattr(Observables, attr, None)
            if obs_enum is not None and isinstance(obs_enum, Observables):
                obs_names.append(attr)
        except:
            pass

    if not obs_names:
        print("No observations found in episode")
        return None, None

    # Create subplots - one per observation (each row 3x taller)
    n_obs = len(obs_names)
    row_height = 3  # Height per row
    if figsize is None:
        figsize = (12, n_obs * row_height)

    fig, axes = plt.subplots(n_obs, 1, figsize=figsize, sharex=True)
    if n_obs == 1:
        axes = [axes]

    # Get time array for x-axis - MUST be present
    if not hasattr(eps, "OBS_TIME"):
        raise ValueError("Episode must have OBS_TIME attribute for time axis")

    time_data = eps.OBS_TIME[:, env_idx, 0]  # Shape: (num_steps,)

    for idx, obs_name in enumerate(obs_names):
        obs_data = getattr(eps, obs_name)  # Shape: (num_steps, num_envs, dim)

        # Get data for this environment
        env_data = obs_data[:, env_idx, :]  # Shape: (num_steps, dim)

        # Plot each dimension with actual time
        if env_data.shape[1] == 1:
            # Scalar observation
            axes[idx].plot(time_data, env_data[:, 0], label=obs_name)
        else:
            # Vector observation (e.g., gyro with 3 channels)
            for dim in range(env_data.shape[1]):
                axes[idx].plot(time_data, env_data[:, dim], label=f"{obs_name}[{dim}]")

        axes[idx].set_ylabel(obs_name)
        axes[idx].legend(loc="upper right")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].spines["top"].set_visible(False)
        axes[idx].spines["right"].set_visible(False)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Observations - Environment {env_idx}")
    fig.tight_layout()

    return fig, axes


def plot_actions(eps: Episode, env_idx: int = 0, figsize=None):
    """Plot all actions for a single environment.

    Args:
        eps: Episode object with action data
        env_idx: Which environment to plot (default: 0)
        figsize: Figure size tuple

    Returns:
        Figure and axes objects
    """
    # Collect available actions (filter out non-Action attrs)
    action_names = []
    for attr in dir(eps):
        if attr.startswith("_"):
            continue
        try:
            # Check if Actions has this enum member
            action_enum = getattr(Actions, attr, None)
            if action_enum is not None and isinstance(action_enum, Actions):
                action_names.append(attr)
        except:
            pass

    # Create subplots (each row 3x taller)
    n_actions = len(action_names)
    if n_actions == 0:
        print("No actions found in episode")
        return None, None

    row_height = 3  # Height per row
    if figsize is None:
        figsize = (10, n_actions * row_height)

    fig, axes = plt.subplots(n_actions, 1, figsize=figsize, sharex=True)
    if n_actions == 1:
        axes = [axes]

    # Get time array for x-axis - MUST be present (use TIME action or OBS_TIME)
    if hasattr(eps, "TIME"):
        time_data = eps.TIME[:, env_idx, 0]
    elif hasattr(eps, "OBS_TIME"):
        time_data = eps.OBS_TIME[:, env_idx, 0]
    else:
        raise ValueError("Episode must have TIME or OBS_TIME attribute for time axis")

    for idx, action_name in enumerate(action_names):
        action_data = getattr(eps, action_name)  # Shape: (num_steps, num_envs, dim)

        # Get data for this environment
        env_data = action_data[:, env_idx, :]  # Shape: (num_steps, dim)

        # Plot each dimension with actual time
        if env_data.shape[1] == 1:
            axes[idx].plot(
                time_data, env_data[:, 0], label=action_name, marker="o", markersize=3
            )
        else:
            for dim in range(env_data.shape[1]):
                axes[idx].plot(
                    time_data,
                    env_data[:, dim],
                    label=f"{action_name}[{dim}]",
                    marker="o",
                    markersize=3,
                )

        axes[idx].set_ylabel(action_name)
        axes[idx].legend(loc="upper right")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].axhline(y=0, color="k", linestyle="--", alpha=0.3)
        axes[idx].spines["top"].set_visible(False)
        axes[idx].spines["right"].set_visible(False)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Actions - Environment {env_idx}")
    fig.tight_layout()

    return fig, axes


def plot_rewards(eps: Episode, env_idx: int = 0, figsize=None):
    """Plot reward components for a single environment.

    Args:
        eps: Episode object with reward data
        env_idx: Which environment to plot (default: 0)
        figsize: Figure size tuple

    Returns:
        Figure and axes objects
    """
    if not hasattr(eps, "reward_components") or not eps.reward_components:
        print("No reward components found in episode")
        return None, None

    # Get reward component names
    reward_keys = list(eps.reward_components.keys())

    # Create subplots (each row 3x taller)
    n_rewards = len(reward_keys)
    row_height = 3
    if figsize is None:
        figsize = (10, n_rewards * row_height)

    fig, axes = plt.subplots(n_rewards, 1, figsize=figsize, sharex=True)
    if n_rewards == 1:
        axes = [axes]

    # Get time array for x-axis
    if hasattr(eps, "TIME"):
        time_data = eps.TIME[:, env_idx, 0]
    elif hasattr(eps, "OBS_TIME"):
        time_data = eps.OBS_TIME[:, env_idx, 0]
    else:
        raise ValueError("Episode must have TIME or OBS_TIME attribute for time axis")

    for idx, reward_key in enumerate(reward_keys):
        reward_data = eps.reward_components[reward_key]  # Shape: (num_steps, num_envs)

        # Get data for this environment
        env_data = reward_data[:, env_idx]  # Shape: (num_steps,)

        axes[idx].plot(time_data, env_data, label=reward_key)
        axes[idx].set_ylabel(reward_key)
        axes[idx].legend(loc="upper right")
        axes[idx].grid(True, alpha=0.3)
        axes[idx].spines["top"].set_visible(False)
        axes[idx].spines["right"].set_visible(False)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Rewards - Environment {env_idx}")
    fig.tight_layout()

    return fig, axes


def plot_episode(eps: Episode, env_idx: int = 0, figsize=None):
    """Plot complete episode: observations, actions, and rewards in one figure.

    Creates a multi-row figure with:
    - Top rows: Key observations (pitch, wheel velocities, etc.)
    - Middle rows: Actions
    - Bottom rows: Reward components

    Args:
        eps: Episode object
        env_idx: Which environment to plot (default: 0)
        figsize: Figure size tuple

    Returns:
        Figure object
    """
    # Collect available data
    obs_names = []
    for attr in dir(eps):
        if not attr.startswith("_"):
            try:
                obs_enum = getattr(Observables, attr, None)
                if obs_enum is not None and isinstance(obs_enum, Observables):
                    obs_names.append(attr)
            except:
                pass

    action_names = []
    for attr in dir(eps):
        if not attr.startswith("_"):
            try:
                action_enum = getattr(Actions, attr, None)
                if action_enum is not None and isinstance(action_enum, Actions):
                    action_names.append(attr)
            except:
                pass

    reward_keys = (
        list(eps.reward_components.keys()) if hasattr(eps, "reward_components") else []
    )

    total_plots = len(obs_names) + len(action_names) + len(reward_keys)

    if total_plots == 0:
        print("No data found in episode")
        return None

    # Each row 3x taller
    row_height = 3
    if figsize is None:
        figsize = (14, total_plots * row_height)

    fig, axes = plt.subplots(total_plots, 1, figsize=figsize, sharex=True)
    if total_plots == 1:
        axes = [axes]

    # Get time array for x-axis
    if hasattr(eps, "TIME"):
        time_data = eps.TIME[:, env_idx, 0]
    elif hasattr(eps, "OBS_TIME"):
        time_data = eps.OBS_TIME[:, env_idx, 0]
    else:
        raise ValueError("Episode must have TIME or OBS_TIME attribute for time axis")

    plot_idx = 0

    # Plot observations
    for obs_name in obs_names:
        obs_data = getattr(eps, obs_name)[:, env_idx, :]

        if obs_data.shape[1] == 1:
            axes[plot_idx].plot(time_data, obs_data[:, 0], label=obs_name)
        else:
            for dim in range(obs_data.shape[1]):
                axes[plot_idx].plot(
                    time_data, obs_data[:, dim], label=f"{obs_name}[{dim}]", alpha=0.7
                )

        axes[plot_idx].set_ylabel(obs_name, fontsize=9)
        axes[plot_idx].legend(loc="upper right", fontsize=7)
        axes[plot_idx].grid(True, alpha=0.3)
        axes[plot_idx].spines["top"].set_visible(False)
        axes[plot_idx].spines["right"].set_visible(False)
        plot_idx += 1

    # Plot actions
    for action_name in action_names:
        action_data = getattr(eps, action_name)[:, env_idx, :]

        if action_data.shape[1] == 1:
            axes[plot_idx].plot(
                time_data,
                action_data[:, 0],
                label=action_name,
                marker="o",
                markersize=2,
            )
        else:
            for dim in range(action_data.shape[1]):
                axes[plot_idx].plot(
                    time_data,
                    action_data[:, dim],
                    label=f"{action_name}[{dim}]",
                    marker="o",
                    markersize=2,
                )

        axes[plot_idx].set_ylabel(action_name, fontsize=9)
        axes[plot_idx].legend(loc="upper right", fontsize=7)
        axes[plot_idx].grid(True, alpha=0.3)
        axes[plot_idx].axhline(y=0, color="k", linestyle="--", alpha=0.3)
        axes[plot_idx].spines["top"].set_visible(False)
        axes[plot_idx].spines["right"].set_visible(False)
        plot_idx += 1

    # Plot rewards
    for reward_key in reward_keys:
        reward_data = eps.reward_components[reward_key][:, env_idx]

        axes[plot_idx].plot(time_data, reward_data, label=reward_key)
        axes[plot_idx].set_ylabel(reward_key, fontsize=9)
        axes[plot_idx].legend(loc="upper right", fontsize=7)
        axes[plot_idx].grid(True, alpha=0.3)
        axes[plot_idx].spines["top"].set_visible(False)
        axes[plot_idx].spines["right"].set_visible(False)
        plot_idx += 1

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"Complete Episode - Environment {env_idx}", fontsize=12)
    fig.tight_layout()

    return fig


def plot_episode_grid(eps: Episode, env_idx: int = 0, figsize=(16, 10)):
    """Plot episode data in a grid layout (observations, actions, rewards as columns).

    Args:
        eps: Episode object
        env_idx: Which environment to plot (default: 0)
        figsize: Figure size tuple

    Returns:
        Figure object
    """
    # Create 3-column layout
    fig = plt.figure(figsize=figsize)

    # Plot observations (left column)
    obs_names = []
    for attr in dir(eps):
        if not attr.startswith("_"):
            try:
                obs_enum = getattr(Observables, attr, None)
                if obs_enum is not None and isinstance(obs_enum, Observables):
                    obs_names.append(attr)
            except:
                pass
    n_obs = len(obs_names)

    for idx, obs_name in enumerate(obs_names):
        ax = plt.subplot(max(n_obs, 3), 3, idx * 3 + 1)
        obs_data = getattr(eps, obs_name)[:, env_idx, :]

        if obs_data.shape[1] == 1:
            ax.plot(obs_data[:, 0])
        else:
            for dim in range(obs_data.shape[1]):
                ax.plot(obs_data[:, dim], label=f"[{dim}]", alpha=0.7)
            ax.legend(fontsize=7)

        ax.set_ylabel(obs_name, fontsize=8)
        ax.grid(True, alpha=0.3)
        if idx == n_obs - 1:
            ax.set_xlabel("Timestep", fontsize=8)

    # Plot actions (middle column)
    action_names = []
    for attr in dir(eps):
        if not attr.startswith("_"):
            try:
                action_enum = getattr(Actions, attr, None)
                if action_enum is not None and isinstance(action_enum, Actions):
                    action_names.append(attr)
            except:
                pass
    n_actions = len(action_names)

    for idx, action_name in enumerate(action_names):
        ax = plt.subplot(max(n_actions, 3), 3, idx * 3 + 2)
        action_data = getattr(eps, action_name)[:, env_idx, :]

        if action_data.shape[1] == 1:
            ax.plot(action_data[:, 0], marker="o", markersize=2)
        else:
            for dim in range(action_data.shape[1]):
                ax.plot(action_data[:, dim], marker="o", markersize=2)

        ax.set_ylabel(action_name, fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color="k", linestyle="--", alpha=0.3)
        if idx == n_actions - 1:
            ax.set_xlabel("Timestep", fontsize=8)

    # Plot rewards (right column)
    if hasattr(eps, "reward_components"):
        reward_keys = list(eps.reward_components.keys())
        n_rewards = len(reward_keys)

        for idx, reward_key in enumerate(reward_keys):
            ax = plt.subplot(max(n_rewards, 3), 3, idx * 3 + 3)
            reward_data = eps.reward_components[reward_key][:, env_idx]

            ax.plot(reward_data)
            ax.set_ylabel(reward_key, fontsize=8)
            ax.grid(True, alpha=0.3)
            if idx == n_rewards - 1:
                ax.set_xlabel("Timestep", fontsize=8)

    fig.suptitle(f"Episode Grid - Environment {env_idx}", fontsize=12)
    fig.tight_layout()

    return fig

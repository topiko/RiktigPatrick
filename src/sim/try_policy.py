from typing import Optional

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import torch
from gymnasium.envs.registration import register
from gymnasium.wrappers import (RecordEpisodeStatistics, RecordVideo,
                                TransformObservation)

from sim.rl_parallel_test2 import (REINFORCE, Policy_Network, ValueNet,
                                   dict2tensor)


def run_episode(
    agent: REINFORCE,
    rp_env: gym.Env,
    seed: int = 42,
    nrollouts: int = 1,
) -> np.ndarray:
    sum_rewards = [0.0] * nrollouts
    for rollout in range(nrollouts):
        agent.rollout_index = rollout
        rp_env.reset(seed=seed)
        while True:
            # TODO: Wrap the rpenv into something the flattens the observation.
            obs = torch.Tensor(rp_env.state.get_state_arr())
            action, _, _ = agent.sample_action(obs)

            action = {k: v[0] for k, v in action.items()}

            obs_d, reward, terminated, truncated, _ = rp_env.step(action)

            sum_rewards[rollout] += reward
            agent.reward = reward

            if terminated or truncated:
                break

    return np.array(sum_rewards)


def plot_state_history(
    history: np.ndarray,
    idx_dict: dict[str, np.ndarray],
    keys: Optional[list[str]] = None,
    show_actions: bool = True,
):
    if keys is None:
        keys = list(idx_dict.keys())

    if not show_actions:
        keys = [k for k in keys if not k.startswith("act/")]

    time_idx = idx_dict.pop("time")

    groups = {
        "wheels_lr": ("act/left_wheel", "act/right_wheel"),
        # "head_pt": ("act/head_pitch", "act/head_turn"),
        "sens/head_pt": ("sens/head_pitch", "sens/head_turn"),
    }

    grouped_d = {k: [] for k in groups}
    for key, group in groups.items():
        for gi in group:
            grouped_d[key].append(idx_dict.pop(gi))

    grouped_d = {k: np.concatenate(v) for k, v in grouped_d.items()}
    idx_dict.update(grouped_d)

    n_rows = len(idx_dict)

    _, axarr = plt.subplots(n_rows, 1, sharex=True, figsize=(8, n_rows * 2))

    times = history[:, time_idx]
    for ax, k in zip(axarr, idx_dict):
        data = history[:, idx_dict[k]]
        ax.plot(times, data, "-|", markersize=5, lw=1)
        ax.set_title(f"{k}")
        ax.spines[["right", "top"]].set_visible(False)

    ax.set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs={"record": True},
    )

    from sim.rl_parallel_test2 import OBS_SPACE

    rpenv = gym.make(
        "RiktigPatrick-v0",
        state_keys=OBS_SPACE,
        render_mode="rgb_array",
        disable_env_checker=True,
    )

    rpenv = TransformObservation(rpenv, dict2tensor)
    make_vid = True
    rpenv = RecordVideo(
        rpenv,
        "./video",
        episode_trigger=lambda _: make_vid,
        name_prefix="try_policy_rp",
    )

    rpenv.reset()

    indim = sum(v.shape[0] for v in rpenv.observation_space.values())
    actiondim = sum(v.shape[0] for v in rpenv.action_space.values())

    agent = REINFORCE(indim, actiondim)  # StepAction().ndim)

    run_episode(agent, rpenv)

    history, idx_d = rpenv.state.history

    plot_state_history(history=history, idx_dict=idx_d)

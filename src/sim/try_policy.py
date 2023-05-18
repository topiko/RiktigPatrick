from typing import Optional

import numpy as np

import gymnasium as gym
from gymnasium.envs.registration import register
from gymnasium.wrappers import RecordVideo
from sim.rl_test import REINFORCE
from sim.rl_test import Policy_Network
from sim.rl_test import run_episode

import matplotlib.pyplot as plt


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
        "head_pt": ("act/head_pitch", "act/head_turn"),
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


register(
    id="RiktigPatrick-v0",
    entry_point="sim.envs.rp_env:GymRP",
    max_episode_steps=300,
)

rpenv = gym.make(
    "RiktigPatrick-v0",
    state_keys=[
        "sens/gyro",
        "act/left_wheel",
        "act/right_wheel",
        "filter/rp_pitch",
    ],
    record=True,
    render_mode="rgb_array",
)

make_vid = True
rpenv = RecordVideo(
    rpenv, "./video", episode_trigger=lambda _: make_vid, name_prefix="try_policy_rp"
)

rpenv.reset()

indim = len(rpenv.state.get_state_arr())
agent = REINFORCE(indim, 1)  # StepAction().ndim)

run_episode(agent, rpenv)

history, idx_d = rpenv.state.history

plot_state_history(history=history, idx_dict=idx_d)

import argparse
from dataclasses import dataclass

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import torch
from gymnasium.envs.registration import register
from gymnasium.wrappers import (
    RecordEpisodeStatistics,
    RecordVideo,
    TransformObservation,
)

from sim.algos import REINFORCE
from sim.PIDPolicy import PIDPolicy
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.utils import Tape, register_and_make_env

parser = argparse.ArgumentParser()

parser.add_argument("--policy", type=str, default="REINFORCE")

args = parser.parse_args()
agent_type = args.policy


@dataclass
class PlotGroups:
    pitch: tuple[str, ...] = ("filter/rp_pitch", "simul/rp_pitch")
    wheel_left: tuple[str, ...] = ("act/left_wheel", "sens/left_wheel_vel")
    wheel_right: tuple[str, ...] = ("act/right_wheel", "sens/right_wheel_vel")
    head_pt: tuple[str, ...] = ("sens/head_pitch", "sens/head_turn")

    def __len__(self) -> int:
        return len(self.__dict__)

    def __getitem__(self, item: str) -> tuple[str, ...]:
        return self.__dict__[item]

    def groups(self) -> list[str]:
        return list(self.__dict__.keys())


def run_episode(
    agent: REINFORCE | PIDPolicy,
    rp_env: gym.Env,
    seed: int = 42,
    nrollouts: int = 1,
    tapes: list[Tape] | None = None,
) -> list[Tape] | None:
    for rollout in range(nrollouts):
        agent.rollout_index = rollout
        obs_d, _ = rp_env.reset(seed=seed)

        while True:
            if isinstance(agent, REINFORCE):
                action, probs, values = agent.sample_action(
                    obs_d, dt=ENV_CONFIG["step_time"]
                )
            elif isinstance(agent, PIDPolicy):
                obs = rp_env.state.get_state_dict()
                action = agent.sample_action(obs)

            action = {k: v[0] for k, v in action.items()}

            obs_d, reward, terminated, truncated, _ = rp_env.step(action)

            if tapes is not None:
                tapes[rollout].rewards.append(reward)
                tapes[rollout].probs.append(probs)
                tapes[rollout].values.append(values)

                if terminated:
                    tapes[rollout].build()

            if truncated | terminated:
                break

    return tapes


def plot_state_history(
    history: np.ndarray,
    idx_dict: dict[str, np.ndarray],
    plot_groups: PlotGroups | None = None,
):
    plot_groups = plot_groups or PlotGroups()

    time_idx = idx_dict.pop("time")

    print("Available:")
    for k in idx_dict:
        print(f"\t{k}")

    n_rows = len(plot_groups)

    _, axarr = plt.subplots(n_rows, 1, sharex=True, figsize=(8, n_rows * 2))

    times = history[:, time_idx]
    for ax, k in zip(axarr, plot_groups.groups()):
        for g in plot_groups[k]:
            data = history[:, idx_dict[g]]
            ax.plot(times, data, "-|", markersize=5, lw=1, label=g)
        ax.set_title(f"{k}")
        ax.spines[["right", "top"]].set_visible(False)
        ax.legend(frameon=False)

    ax.set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    ENV_CONFIG["record"] = True
    rpenv = register_and_make_env(ENV_CONFIG, OBS_SPACE)

    rpenv = RecordVideo(
        rpenv,
        "./video",
        episode_trigger=lambda _: True,
        name_prefix="try_policy_rp",
    )

    rpenv.reset()

    indim = sum(
        v.shape[0] for k, v in rpenv.observation_space.items() if k in MODEL_INPUT
    )
    actiondim = sum(v.shape[0] for v in rpenv.action_space.values())

    if agent_type == "REINFORCE":
        agent = REINFORCE(indim, actiondim, MODEL_INPUT)  # StepAction().ndim)
    elif agent_type == "pid":
        agent = PIDPolicy(10, 0.0, 0, ENV_CONFIG["step_time"])
    else:
        raise KeyError("Invalid agent type")

    run_episode(agent, rpenv)

    history, idx_d = rpenv.state.history

    plot_state_history(history=history, idx_dict=idx_d)

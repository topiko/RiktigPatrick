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

from sim.algos import REINFORCE, compute_returns, compute_value_estimates
from sim.custom_policies import NetPolicy, PIDPolicy
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE, RL_CONFIG
from sim.utils import Tape, actiondim, model_indim, register_and_make_env
from sim.nets import ValueNet

parser = argparse.ArgumentParser()

parser.add_argument("--policy", type=str, default="REINFORCE")

args = parser.parse_args()
agent_type = args.policy


class PlotGroups:
    def __init__(self):
        self.pitch = ("filter/rp_pitch", "simul/rp_pitch")
        self.wheel_left = ("act/left_wheel", "sens/left_wheel_vel")
        self.wheel_right = ("act/right_wheel", "sens/right_wheel_vel")
        self.head_pt = ("sens/head_pitch", "sens/head_turn")
        self.reward = ("reward",)

    def __len__(self) -> int:
        return len(self.__dict__)

    def __getitem__(self, item: str) -> tuple[str, ...]:
        return self.__dict__[item]

    def groups(self) -> list[str]:
        return list(self.__dict__.keys())

    def __len__(self) -> int:
        return len(self.__dict__)


def run_episode(
    agent: REINFORCE | PIDPolicy | NetPolicy,
    rp_env: gym.Env,
    seed: int = 42,
    nrollouts: int = 1,
    tapes: list[Tape] | None = None,
) -> list[Tape] | None:
    # Unwrap to get actual env
    env = rp_env
    while hasattr(env, 'env'):
        env = env.env
    
    for rollout in range(nrollouts):
        agent.rollout_index = rollout
        obs_d, _ = rp_env.reset(seed=seed)

        while True:
            if isinstance(agent, REINFORCE):
                result = agent.sample_action(obs_d)
                if len(result) == 4:
                    action, probs, values, entropy = result
                else:
                    action, probs, values = result
                    entropy = torch.zeros_like(probs)
            elif isinstance(agent, PIDPolicy | NetPolicy):
                obs = env.state.get_state_dict()
                action = agent.sample_action(obs)
                probs = torch.zeros(1)
                values = torch.zeros(1)
                entropy = torch.zeros(1)

            action = {k: v[0] for k, v in action.items()}

            obs_d, reward, terminated, truncated, _ = rp_env.step(action)

            if tapes is not None:
                tapes[rollout].rewards.append(reward)
                tapes[rollout].probs.append(probs)
                tapes[rollout].values.append(values)
                tapes[rollout].entropies.append(entropy)

                if truncated | terminated:
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
    ENV_CONFIG["randomize"] = False
    rpenv = register_and_make_env(ENV_CONFIG, OBS_SPACE)

    rpenv = RecordVideo(
        rpenv,
        "./video",
        episode_trigger=lambda _: True,
        name_prefix="try_policy_rp",
    )

    rpenv.reset()

    indim = model_indim(rpenv, MODEL_INPUT)
    actiondim = actiondim(rpenv)

    if agent_type == "REINFORCE":
        agent = REINFORCE(
            indim,
            actiondim,
            MODEL_INPUT,
            use_baseline=False,
            init2zeros=False,
            load_net=True,
        )
    elif agent_type == "pid":
        agent = PIDPolicy(10, 0.0, 0, ENV_CONFIG["step_time"])
    elif agent_type == "net":
        agent = NetPolicy()
    else:
        raise KeyError("Invalid agent type")

    run_episode(agent, rpenv, seed=0)

    rpenv.close()  # Close video recorder

    # Access the underlying env (unwrap RecordVideo if present)
    env = rpenv
    while hasattr(env, 'env'):
        env = env.env
    history, idx_d = env.state.history

    # Compute return using same gamma as training (0.99)
    rewards = history[:, idx_d["reward"][0]]
    returns = compute_returns(rewards, discount=RL_CONFIG["gamma"])
    history = np.column_stack([history, returns])
    idx_d["return"] = np.array([history.shape[1] - 1])

    # Compute value estimates from critic
    value_net = ValueNet(indim)
    try:
        value_net = value_net.load()
    except:
        print("Warning: Could not load value net, using zeros")
        value_net = None
    
    if value_net is not None:
        value_estimates = compute_value_estimates(value_net, history, idx_d, MODEL_INPUT)
    else:
        value_estimates = np.zeros(len(history))
    history = np.column_stack([history, value_estimates])
    idx_d["value_estimate"] = np.array([history.shape[1] - 1])

    # Compute advantage
    advantages = returns - value_estimates
    history = np.column_stack([history, advantages])
    idx_d["advantage"] = np.array([history.shape[1] - 1])

    plot_groups = PlotGroups()
    plot_groups.__dict__['returns'] = ("return", "value_estimate", "advantage")
    plot_groups.__dict__['reward'] = ("reward/step", "reward/pitch", "reward/action", "reward/yaw")

    plot_state_history(history=history, idx_dict=idx_d, plot_groups=plot_groups)
    plt.savefig("plots/episode.png", dpi=100)
    print(f"Episode return: {returns[0]:.2f}")
    print("Plot saved to plots/episode.png")

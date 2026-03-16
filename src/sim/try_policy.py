import argparse

import gymnasium as gym
import matplotlib

matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import torch
from gymnasium.wrappers import (
    RecordVideo,
)

from sim.algos import REINFORCE, compute_returns, compute_value_estimates
from sim.custom_policies import NetPolicy, PIDPolicy
from sim.nets import PolicyNetwork
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE, RL_CONFIG
from sim.utils import Tape, actiondim, model_indim, register_and_make_env

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
    while hasattr(env, "env"):
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
    trim_end_steps: int = 0,
):
    plot_groups = plot_groups or PlotGroups()

    time_idx = idx_dict["env/time"]

    print("Available:")
    for k in idx_dict:
        print(f"\t{k}")

    # Units mapping (display units for plot labels)
    # These are the human-readable units shown in plots
    units = {
        "filter/rp_pitch": "deg",
        "sens/gyro_0": "deg/s",
        "sens/gyro_1": "deg/s",
        "sens/gyro_2": "deg/s",
        "sens/acc_0": "m/s²",
        "sens/acc_1": "m/s²",
        "sens/acc_2": "m/s²",
        "sens/left_wheel_vel": "rev/s",
        "sens/right_wheel_vel": "rev/s",
        "sens/head_pitch": "rad",
        "sens/head_turn": "rad",
        "simul/rp_pitch": "deg",
        "act/left_wheel": "rev/s",
        "act/right_wheel": "rev/s",
        "env/time": "s",
        "reward": "",
        "reward/step": "",
        "reward/pitch": "",
        "reward/action": "",
        "reward/yaw": "",
        "return": "",
        "value_estimate": "",
        "advantage": "",
        "entropy": "",
    }

    # Conversion factors: multiply to convert from SI (sim units) to display units
    # History data is stored in SI units (rad/s), convert for human-readable plots
    conversions = {
        "sens/gyro_0": 180.0 / 3.14159,  # rad/s → deg/s
        "sens/gyro_1": 180.0 / 3.14159,
        "sens/gyro_2": 180.0 / 3.14159,
        "sens/left_wheel_vel": 1.0 / (2 * 3.14159),  # rad/s → rev/s
        "sens/right_wheel_vel": 1.0 / (2 * 3.14159),
        "act/left_wheel": 1.0 / (2 * 3.14159),  # rad/s → rev/s
        "act/right_wheel": 1.0 / (2 * 3.14159),
    }

    n_rows = len(plot_groups)

    _, axarr = plt.subplots(n_rows, 1, sharex=True, figsize=(8, n_rows * 2))

    times = history[:, time_idx]

    # Add vertical red line at trim cutoff
    if trim_end_steps > 0 and len(times) > trim_end_steps:
        trim_time = times[-(trim_end_steps + 1)]

    for ax, k in zip(axarr, plot_groups.groups()):
        for g in plot_groups[k]:
            idx = idx_dict[g]
            if isinstance(idx, np.ndarray):
                idx = idx[0] if len(idx) == 1 else idx
            data = history[:, idx]
            # Apply unit conversion if needed
            if g in conversions:
                data = data * conversions[g]
            unit = units.get(g, "")
            label = f"{g} [{unit}]" if unit else g
            ax.plot(times, data, "-|", markersize=5, lw=1, label=label)
        ax.set_title(f"{k}")
        ax.spines[["right", "top"]].set_visible(False)
        ax.legend(frameon=False)

        # Add vertical red line at trim cutoff
        if trim_end_steps > 0 and len(times) > trim_end_steps:
            ax.axvline(
                x=trim_time,
                color="red",
                linestyle="--",
                linewidth=2,
                label=f"trim cutoff (last {trim_end_steps} steps)",
            )

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

    # Run episode with tapes to capture entropy
    tapes = [Tape(0)]
    run_episode(agent, rpenv, seed=0, tapes=tapes)
    tape = tapes[0]

    rpenv.close()  # Close video recorder

    # Access the underlying env (unwrap RecordVideo if present)
    env = rpenv
    while hasattr(env, "env"):
        env = env.env
    history, idx_d = env.state.history

    # Compute return using same gamma as training (0.99)
    rewards = history[:, idx_d["reward"][0]]
    returns = compute_returns(rewards, discount=RL_CONFIG["gamma"])
    history = np.column_stack([history, returns])
    idx_d["return"] = np.array([history.shape[1] - 1])

    # Get value estimates from policy network's value head
    policy_net = PolicyNetwork(indim, actiondim)
    try:
        policy_net = policy_net.load()
    except:
        print("Warning: Could not load policy net, using zeros")
        policy_net = None

    if policy_net is not None:
        value_estimates = compute_value_estimates(
            policy_net, history, idx_d, MODEL_INPUT
        )
    else:
        value_estimates = np.zeros(len(history))
    history = np.column_stack([history, value_estimates])
    idx_d["value_estimate"] = np.array([history.shape[1] - 1])

    # Compute advantage
    advantages = returns - value_estimates
    history = np.column_stack([history, advantages])
    idx_d["advantage"] = np.array([history.shape[1] - 1])

    # Get entropy from tape (already computed during episode)
    # Entropy shape is (n_steps,) or (n_steps, n_actions)
    if tape.entropies.dim() == 1:
        entropy_estimates = tape.entropies.detach().cpu().numpy()
    else:
        entropy_estimates = tape.entropies.sum(dim=1).detach().cpu().numpy()

    # Match history length (may differ by 1 due to timing)
    # History is recorded by env, entropy by tape - they can differ by 1
    n_history = len(history)
    n_entropy = len(entropy_estimates)

    if n_entropy > n_history:
        # Truncate entropy to match history
        entropy_estimates = entropy_estimates[:n_history]
    elif n_entropy < n_history:
        # Pad entropy with last value (not zeros)
        entropy_estimates = np.pad(
            entropy_estimates,
            (0, n_history - n_entropy),
            constant_values=entropy_estimates[-1] if n_entropy > 0 else 0,
        )

    history = np.column_stack([history, entropy_estimates])
    idx_d["entropy"] = np.array([history.shape[1] - 1])

    plot_groups = PlotGroups()
    plot_groups.__dict__["returns"] = ("return", "value_estimate", "advantage")
    plot_groups.__dict__["entropy"] = ("entropy",)
    plot_groups.__dict__["reward"] = (
        "reward/step",
        "reward/pitch",
        "reward/action",
        "reward/yaw",
    )
    plot_groups.__dict__["sensors"] = (
        "filter/rp_pitch",
        "sens/gyro_0",
        "sens/gyro_1",
        "sens/gyro_2",
    )

    plot_state_history(
        history=history,
        idx_dict=idx_d,
        plot_groups=plot_groups,
        trim_end_steps=RL_CONFIG.get("trim_end_steps", 0),
    )
    import os

    plot_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "plots"
    )
    os.makedirs(plot_dir, exist_ok=True)
    plt.savefig(os.path.join(plot_dir, "episode.png"), dpi=100)
    print(f"Episode return: {returns[0]:.2f}")
    print(f"Plot saved to {plot_dir}/episode.png")

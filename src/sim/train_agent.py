import os
from pathlib import Path

import dotenv
import gymnasium as gym
import hydra
import matplotlib.pyplot as plt
import mlflow
import numpy as np
import torch
from gymnasium.wrappers import RecordVideo
from omegaconf import DictConfig, OmegaConf
from torch import nn

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, Observables
from sim.plot_utils import plot_episode
from sim.utils import Episode, MiscKeys, SingleEnvWrapper, register_and_make_env

dotenv.load_dotenv()  # Load environment variables from .env file


HYDRA_CONFIG_DIR = os.getenv("HYDRA_CONFIG_DIR", "config")

PLOTKS = [
    (Observables.OBS_TIME, (Observables.RP_PITCH, Observables.TRUE_PITCH)),
    (
        Observables.OBS_TIME,
        (Observables.LEFT_WHEEL_VEL, Observables.RIGHT_WHEEL_VEL),
    ),
    (
        Observables.OBS_TIME,
        (Observables.HEAD_PITCH, Observables.HEAD_TURN),
    ),
]


def _flatten_dict(d, parent_key="", sep="_"):
    """Flatten nested dict for MLflow params."""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(_flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def np2tensor(np_dict: dict) -> dict:
    return {k: torch.from_numpy(v).float() for k, v in np_dict.items()}


def tensor2numpy(tensor_dict: dict) -> dict:
    return {k: v.cpu().numpy() for k, v in tensor_dict.items()}


def compute_returns(rewards: np.ndarray, discount: float) -> np.ndarray:
    if rewards.ndim == 2:
        returns = np.zeros_like(rewards, dtype=np.float32)
        for env_idx in range(rewards.shape[0]):
            returns[env_idx] = compute_returns(rewards[env_idx], discount)
        return returns

    if rewards.ndim != 1:
        raise ValueError(f"Expected rewards to be 1D or 2D, got shape {rewards.shape}")

    returns = np.zeros_like(rewards, dtype=np.float32)
    running_return = 0
    for i in reversed(range(len(rewards))):
        running_return = rewards[i] + discount * running_return
        returns[i] = running_return
    return returns


def get_advantages(
    returns: torch.Tensor | np.ndarray, values: torch.Tensor | np.ndarray
) -> torch.Tensor:
    if returns.shape != values.shape:
        raise ValueError(
            f"Returns and values must have the same shape, got {returns.shape} and {values.shape}"
        )
    advantages = returns - values
    return advantages


def rollout(
    rp_env: gym.Env | gym.vector.AsyncVectorEnv,
    agent: nn.Module,
    cfg: DictConfig,
    seed: int = 0,
) -> tuple[
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    list[dict[Observables, np.ndarray]],
    list[dict[Actions, np.ndarray]],
]:
    obs_l = []
    action_l = []
    rewards_l = []
    logps_l = []
    values_l = []
    h = None
    obs_d, _ = rp_env.reset(seed=seed)
    while True:
        obs_d_t = np2tensor(obs_d)

        action, logp, value, h = agent.act(obs_d_t, h)

        obs_l.append(obs_d)
        action_np = tensor2numpy(action)
        action_l.append(action_np)

        obs_d, reward, terminated, truncated, info = rp_env.step(action_np)

        done = terminated | truncated.flatten()

        logps_l.append(logp)
        rewards_l.append(reward)
        values_l.append(value)

        if done.any():
            #obs_d = {k: v[~done] for k, v in obs_d.items()}
            # Batch dim is 1 for h.
            #h = h[:, ~done, :]
            print(info)

            breakpoint()

        if done.all():
            # The last obs contains e.g., termination rewards etc.
            obs_l.append(obs_d)

            break

    logps_t = torch.cat(logps_l, dim=1)
    rewards = np.concat(rewards_l, axis=1)  # Shape: (num_envs, num_steps)
    values_t = torch.cat(values_l, dim=1)

    return logps_t, rewards, values_t, obs_l, action_l


@hydra.main(config_path=HYDRA_CONFIG_DIR, config_name="rlrp", version_base=None)
def main(cfg: DictConfig):
    # Setup MLflow if enabled
    if cfg.logging.mlflow.enabled:
        mlflow.set_tracking_uri(os.getenv("MLFLOW_TRACKING_URI"))
        mlflow.set_experiment(cfg.logging.mlflow.experiment_name)
        mlflow.start_run()

        # Log config parameters
        flat_params = OmegaConf.to_container(cfg, resolve=True)
        mlflow.log_params(
            {str(k): str(v) for k, v in _flatten_dict(flat_params).items()}
        )

    # Create environment
    rp_env = register_and_make_env(cfg)

    if cfg.env.n_parallel == 1:
        rp_env = SingleEnvWrapper(rp_env)

    # video env: Wrap single env with batch dimension handler for rollout compatibility
    rp_video_env = SingleEnvWrapper(
        RecordVideo(
            register_and_make_env(cfg, force_single_env=True),
            "video/",
            episode_trigger=lambda _: True,
            name_prefix="rp",
        )
    )

    # Create agent
    agent = Agent(
        inputs=cfg.policy.inputs,
        actions=cfg.policy.actions,
    )

    optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)

    i = 0
    while True:
        optimizer.zero_grad()

        logps, rewards, values, _, _ = rollout(rp_env, agent, cfg, seed=i)

        G = compute_returns(rewards, discount=cfg.rl.discount)
        G_t = torch.from_numpy(G)

        advantages = get_advantages(G_t, values)

        policy_loss = -torch.mean(logps * advantages)
        values_loss = ((values - G_t) ** 2).mean()

        loss = policy_loss + values_loss

        loss.backward()

        # Gradient clipping
        torch.nn.utils.clip_grad_norm_(agent.parameters(), max_norm=cfg.train.grad_clip)

        optimizer.step()

        mean_return = G.sum(axis=1).mean()
        min_ep_len = rewards.shape[1]

        print(
            f"Step {i:4d}: p_l={policy_loss.item():.4f}, "
            f"v_l={values_loss.item():.4f}, l={loss.item():.4f}, "
            f"ret={mean_return:.2f}, min_ep_len={min_ep_len}"
        )

        # Log metrics to MLflow
        if cfg.logging.mlflow.enabled and i % cfg.logging.mlflow.push_freq == 0:
            mlflow.log_metrics(
                {
                    "policy_loss": policy_loss.item(),
                    "value_loss": values_loss.item(),
                    "total_loss": loss.item(),
                    "mean_return": float(mean_return),
                    "min_episode_length": int(min_ep_len),
                    "max_return": float(G.sum(axis=1).max()),
                    "min_return": float(G.sum(axis=1).min()),
                    "std_return": float(G.sum(axis=1).std()),
                },
                step=i,
            )

        # Logging and visualization
        plot_freq = cfg.logging.plot_freq

        # Generate plots periodically
        if i % plot_freq == 0:
            with torch.no_grad():
                _, rewards, values, obs_l, action_l = rollout(
                    rp_video_env, agent, cfg, seed=i + 10000
                )

            values_np = values.cpu().numpy()
            G = compute_returns(rewards, discount=cfg.rl.discount)

            eps = Episode(
                obs_l,
                action_l,
                value_estimates=values_np,
                returns=G,
                advantages=get_advantages(G, values_np),
            )

            # Generate plot (eps is single episode)
            plot_path = Path(f"./plots/episode_iter_{i:04d}.png")
            fig = plot_episode(
                eps,
                keys=PLOTKS
                + [
                    (Observables.OBS_TIME, (Observables.from_str(o),))
                    for o in cfg.policy.inputs
                    if o
                    not in (Observables.OBS_TIME.value,)
                    + tuple(v_.value for k, v in PLOTKS for v_ in v)
                ]
                + [
                    (Actions.TIME, (Actions.from_str(a),))
                    for a in cfg.policy.actions.keys()
                ]
                + [
                    (
                        Observables.OBS_TIME,
                        tuple(Observables.from_str(r) for r in cfg.reward.keys()),
                    ),
                    (
                        Actions.TIME,
                        (MiscKeys.VALUES_ESTIM, MiscKeys.RETURNS, MiscKeys.ADVANTAGES),
                    ),
                ],
                save_path=plot_path,
            )
            if cfg.logging.mlflow.enabled:
                mlflow.log_artifact(plot_path)
            plt.close(fig)
            print(f"  📊 Saved plot: {plot_path}")

        i += 1


if __name__ == "__main__":
    main()

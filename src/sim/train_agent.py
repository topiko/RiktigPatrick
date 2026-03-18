import os

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
from sim.utils import Episode, SingleEnvWrapper, register_and_make_env

dotenv.load_dotenv()  # Load environment variables from .env file


HYDRA_CONFIG_DIR = os.getenv("HYDRA_CONFIG_DIR", "config")


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
    returns = np.zeros_like(rewards, dtype=np.float32)
    running_return = 0
    for i in reversed(range(len(rewards))):
        running_return = rewards[i] + discount * running_return
        returns[i] = running_return
    return returns


def rollout(
    rp_env: gym.Env | gym.vector.AsyncVectorEnv,
    agent: nn.Module,
    cfg: DictConfig,
    seed: int = 0,
) -> tuple[
    torch.Tensor,
    torch.Tensor,
    list[dict[Observables, np.ndarray]],
    list[dict[Actions, np.ndarray]],
    list[str, np.ndarray],
]:
    obs_d, _ = rp_env.reset(seed=seed)

    max_steps = 1000

    obs_l = []
    action_l = []
    rewards_l = []
    rewards_l2 = []
    logps_l = []

    for _ in range(max_steps):
        obs_d_t = np2tensor(obs_d)
        action, logp = agent.act(obs_d_t)

        obs_l.append(obs_d)
        action_np = tensor2numpy(action)
        action_l.append(action_np)

        obs_d, reward, terminated, truncated, reward_info = rp_env.step(action_np)

        done = terminated | truncated.flatten()
        if done.any():
            break

        logps_l.append(logp)
        rewards_l.append(reward)
        rewards_l2.append(reward_info)

    logps_t = torch.cat(logps_l, dim=1)
    rewards = np.stack(rewards_l, axis=1)  # Shape: (num_envs, num_steps)

    return logps_t, rewards, obs_l, action_l, rewards_l2


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

    # video env: Wrap single env with batch dimension handler for rollout compatibility
    rp_video_env = SingleEnvWrapper(
        RecordVideo(
            register_and_make_env(cfg, force_single_env=True),
            "./video",
            episode_trigger=lambda _: True,
            name_prefix="try_policy_rp",
        )
    )

    # Create agent
    agent = Agent(
        inputs=list(cfg.policy.inputs),
        actions=list(cfg.policy.actions),
    )

    optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)

    i = 0
    while True:
        optimizer.zero_grad()

        logps, rewards, _, _, _ = rollout(rp_env, agent, cfg, seed=i)

        # Compute returns for each environment separately
        G = np.zeros_like(rewards)
        for env_idx in range(rewards.shape[0]):
            G[env_idx] = compute_returns(rewards[env_idx], discount=0.99)

        G_t = torch.from_numpy(G)
        advantages = G_t - G_t.mean(dim=1, keepdim=True)

        policy_loss = -torch.mean(logps * advantages)
        policy_loss.backward()
        optimizer.step()

        mean_return = G.sum(axis=1).mean()
        min_ep_len = rewards.shape[1]

        print(
            f"Step {i:4d}: policy_loss={policy_loss.item():.4f}, "
            f"returns={mean_return:.2f}, min_ep_len={min_ep_len}"
        )

        # Log metrics to MLflow
        if cfg.logging.mlflow.enabled:
            mlflow.log_metrics(
                {
                    "policy_loss": policy_loss.item(),
                    "mean_return": float(mean_return),
                    "min_episode_length": int(min_ep_len),
                    "max_return": float(G.sum(axis=1).max()),
                    "std_return": float(G.sum(axis=1).std()),
                },
                step=i,
            )

        # Logging and visualization
        plot_freq = cfg.logging.plot_freq

        # Generate plots periodically
        if i % plot_freq == 0:
            rp_video_env.name_prefix = f"policy_iter_{i:04d}"
            _, _, obs_l, action_l, reward_l = rollout(
                rp_video_env, agent, cfg, seed=i + 10000
            )
            eps = Episode(obs_l, action_l, reward_l)

            # Generate plot (eps is single episode)
            fig = plot_episode(
                eps,
                keys=[
                    (Observables.OBS_TIME, (Observables.from_str(o),))
                    for o in cfg.policy.inputs
                    if o != Observables.OBS_TIME.value
                ]
                + [
                    (Actions.TIME, (Actions.from_str(list(a.keys())[0]),))
                    for a in cfg.policy.actions
                ],
            )
            plot_path = f"./plots/episode_iter_{i:04d}.png"
            fig.savefig(plot_path, dpi=230, bbox_inches="tight")
            if cfg.logging.mlflow.enabled:
                mlflow.log_artifact(plot_path)
            plt.close(fig)
            print(f"  📊 Saved plot: {plot_path}")

        i += 1


if __name__ == "__main__":
    main()

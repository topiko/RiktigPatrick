import os

import dotenv
import gymnasium as gym
import hydra
import matplotlib
import matplotlib.pyplot as plt
import mlflow
import numpy as np
import torch
from gymnasium.wrappers import RecordVideo
from omegaconf import DictConfig, OmegaConf
from torch import nn

from nn_ctrl.nns import Agent
from sim.plot_utils import plot_episode
from sim.utils import Episode, register_and_make_env

matplotlib.use("Agg")  # Non-interactive backend for headless rendering
dotenv.load_dotenv()

HYDRA_CONFIG_DIR = os.getenv("HYDRA_CONFIG_DIR", "config")


def np2tensor(np_dict: dict) -> dict:
    return {k: torch.from_numpy(v) for k, v in np_dict.items()}


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
    list[dict[str, np.ndarray]],
    list[dict[str, np.ndarray]],
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
    if cfg.mlflow.enabled:
        mlflow.set_tracking_uri(cfg.mlflow.tracking_uri)
        mlflow.set_experiment(cfg.mlflow.experiment_name)
        mlflow.start_run()

        # Log config parameters
        flat_params = OmegaConf.to_container(cfg, resolve=True)
        mlflow.log_params(
            {str(k): str(v) for k, v in _flatten_dict(flat_params).items()}
        )

    # Create environment
    rp_env = register_and_make_env(cfg)

    # Create video recording environment (for periodic video generation)
    video_env = None
    if cfg.mlflow.enabled and cfg.training.get("video_frequency", 0) > 0:
        # Create separate env for video recording
        video_env_config = dict(cfg.env)
        video_env_config.pop("n_parallel")
        video_env_config["record"] = True
        action_keys = [
            list(action_item.keys())[0] for action_item in cfg.policy.actions
        ]
        video_env_config["actions"] = action_keys

        from gymnasium.envs.registration import register

        register(
            id="RiktigPatrick-video-v0",
            entry_point="sim.envs.rp_env:GymRP",
            max_episode_steps=2000,
            kwargs=video_env_config,
        )

        base_video_env = gym.make(
            "RiktigPatrick-video-v0", disable_env_checker=True, **video_env_config
        )
        video_env = RecordVideo(
            base_video_env,
            "./video",
            episode_trigger=lambda ep_id: True,  # Record when we call it
            name_prefix="training",
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

        # Logging and visualization
        plot_freq = cfg.training.get("plot_frequency", 10)
        video_freq = cfg.training.get("video_frequency", 100)

        if i % 10 == 0:
            mean_return = G.sum(axis=1).mean()
            min_ep_len = rewards.shape[1]

            print(
                f"Step {i:4d}: policy_loss={policy_loss.item():.4f}, "
                f"returns={mean_return:.2f}, min_ep_len={min_ep_len}"
            )

            # Log metrics to MLflow
            if cfg.mlflow.enabled:
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

        # Generate plots periodically
        if i % plot_freq == 0 and i > 0:
            _, _, obs_l, action_l, reward_l = rollout(
                rp_env, agent, cfg, seed=i + 10000
            )
            eps = Episode(obs_l, action_l, reward_l)

            # Generate plot
            fig = plot_episode(eps, env_idx=0, figsize=(14, 12))
            if fig and cfg.mlflow.enabled:
                plot_path = f"./plots/episode_iter_{i:04d}.png"
                os.makedirs("./plots", exist_ok=True)
                fig.savefig(plot_path, dpi=100, bbox_inches="tight")
                mlflow.log_artifact(plot_path)
                plt.close(fig)
                print(f"  📊 Saved plot: {plot_path}")

        # Generate video periodically
        if i % video_freq == 0 and i > 0 and video_env is not None:
            print(f"  🎥 Recording video...")

            # Create a temporary single-env wrapper for rollout
            # We need to wrap video_env to work with rollout (expects vectorized or single env)
            class SingleEnvWrapper:
                """Wrapper to make single env compatible with rollout batching."""

                def __init__(self, env):
                    self.env = env

                def reset(self, seed=None):
                    obs_d, info = self.env.reset(seed=seed)
                    # Add batch dimension
                    obs_d_batched = {
                        k: v[np.newaxis, :] if v.ndim == 1 else v
                        for k, v in obs_d.items()
                    }
                    return obs_d_batched, info

                def step(self, action_d):
                    # Remove batch dimension
                    action_single = {k: v[0] for k, v in action_d.items()}
                    obs_d, reward, terminated, truncated, reward_info = self.env.step(
                        action_single
                    )
                    # Add batch dimension
                    obs_d_batched = {
                        k: v[np.newaxis, :] if v.ndim == 1 else v
                        for k, v in obs_d.items()
                    }
                    reward = (
                        np.array([reward])
                        if isinstance(reward, (int, float))
                        else reward[np.newaxis]
                    )
                    terminated = np.array([terminated])
                    truncated = np.array([truncated])
                    return obs_d_batched, reward, terminated, truncated, reward_info

            wrapped_video_env = SingleEnvWrapper(video_env)

            # Use rollout function for video
            _ = rollout(wrapped_video_env, agent, cfg, seed=i + 20000)

            # Videos are automatically saved by RecordVideo wrapper
            # Log video to MLflow
            if cfg.mlflow.enabled:
                # Find the latest video file
                video_files = sorted(
                    [f for f in os.listdir("./video") if f.endswith(".mp4")]
                )
                if video_files:
                    latest_video = os.path.join("./video", video_files[-1])
                    mlflow.log_artifact(latest_video)
                    print(f"  🎥 Logged video: {latest_video}")

        i += 1


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


if __name__ == "__main__":
    main()

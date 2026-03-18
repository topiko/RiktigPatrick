import os

import dotenv
import gymnasium as gym
import hydra
import numpy as np
import torch
from omegaconf import DictConfig
from torch import nn

from nn_ctrl.nns import Agent
from sim.utils import Episode, register_and_make_env

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
    rp_env = register_and_make_env(cfg)

    # Convert config to list format for Agent
    # Actions now include their config inline
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

        if i % 10 == 0:
            print(
                f"Step {i:4d}: policy_loss={policy_loss.item():.4f}, returns={G.sum(axis=1).mean():.2f}, min_ep_len={rewards.shape[1]}"
            )

            _, _, obs_l, action_l, reward_l = rollout(rp_env, agent, cfg, seed=i + 1000)

            eps = Episode(obs_l, action_l, reward_l)

            # Example: Access episode data using enum names
            # print(f"Pitch trajectory: {eps.RP_PITCH.shape}")
            # print(f"Gyro trajectory: {eps.GYRO.shape}")
            # print(f"Actions: {eps.ACC_BOTH_WHEELS.shape}")

        i += 1


if __name__ == "__main__":
    main()

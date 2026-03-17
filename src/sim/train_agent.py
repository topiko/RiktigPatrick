import os

import dotenv
import gymnasium as gym
import hydra
import numpy as np
import torch
from omegaconf import DictConfig
from torch import nn

from nn_ctrl.nns import Agent
from sim.utils import register_and_make_env

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
    rp_env: gym.Env | gym.vector.AsyncVectorEnv, agent: nn.Module, cfg: DictConfig
) -> tuple[torch.Tensor, torch.Tensor]:
    obs_d, _ = rp_env.reset(seed=42)

    max_steps = 1000

    logps = []
    rewards = []

    for _ in range(max_steps):
        obs_d_t = np2tensor(obs_d)
        action, logp = agent.act(obs_d_t)

        action_np = tensor2numpy(action)
        obs_d, reward, terminated, truncated, _ = rp_env.step(action_np)

        done = terminated | truncated.flatten()
        if done.any():
            break

        logps.append(logp)
        rewards.append(reward)

    logps_t = torch.cat(logps, dim=1)
    rewards_t = torch.from_numpy(np.stack(rewards, axis=1))

    return logps_t, rewards_t


@hydra.main(config_path=HYDRA_CONFIG_DIR, config_name="rlrp", version_base=None)
def main(cfg: DictConfig):
    rp_env = register_and_make_env(cfg)
    action_d = {a: cfg.policy.act_map[a] for a in cfg.policy.actions}
    agent = Agent(inputs=cfg.policy.inputs, actions=action_d)

    optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)

    i = 0
    while True:
        optimizer.zero_grad()

        logps, rewards = rollout(rp_env, agent, cfg)

        # Compute returns for each environment separately
        rewards_np = rewards.numpy()
        G = np.zeros_like(rewards_np)
        for env_idx in range(rewards_np.shape[0]):
            G[env_idx] = compute_returns(rewards_np[env_idx], discount=0.99)

        G_t = torch.from_numpy(G)
        advantages = G_t - G_t.mean(dim=1, keepdim=True)

        policy_loss = -torch.mean(logps * advantages)
        policy_loss.backward()
        optimizer.step()

        if i % 10 == 0:
            print(
                f"Step {i:4d}: policy_loss={policy_loss.item():.4f}, returns={G.sum(axis=1).mean():.2f}, min_ep_len={rewards.shape[1]}"
            )

        i += 1


if __name__ == "__main__":
    main()

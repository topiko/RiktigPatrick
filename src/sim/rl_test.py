from __future__ import annotations
import torch
from torch import nn
from torch.distributions.normal import Normal

import numpy as np

import gymnasium as gym
from gymnasium.envs.registration import register

from riktigpatric.patrick import StepAction


class Policy_Network(nn.Module):
    """Parametrized Policy Network."""

    def __init__(self, obs_space_dims: int, action_space_dims: int):
        """Initializes a neural network that estimates the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """
        super().__init__()

        hidden_space1 = 16  # Nothing special with 16, feel free to change
        hidden_space2 = 32  # Nothing special with 32, feel free to change

        # Shared Network
        self.shared_net = nn.Sequential(
            nn.Linear(obs_space_dims, hidden_space1),
            nn.Tanh(),
            nn.Linear(hidden_space1, hidden_space2),
            nn.Tanh(),
        )

        # Policy Mean specific Linear Layer
        self.policy_mean_net = nn.Sequential(
            nn.Linear(hidden_space2, action_space_dims)
        )

        # Policy Std Dev specific Linear Layer
        self.policy_stddev_net = nn.Sequential(
            nn.Linear(hidden_space2, action_space_dims)
        )

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Conditioned on the observation, returns the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            x: Observation from the environment

        Returns:
            action_means: predicted mean of the normal distribution
            action_stddevs: predicted standard deviation of the normal distribution
        """
        shared_features = self.shared_net(x.float())

        action_means = self.policy_mean_net(shared_features)
        action_stddevs = torch.log(
            1 + torch.exp(self.policy_stddev_net(shared_features))
        )

        return action_means, action_stddevs

    def store(self, fname: str = "rpnet.pth"):
        torch.save(self, fname)

    def load(self, fname: str = "rpnet.pth") -> Policy_Network:
        try:
            return torch.load("rpnet.pth")
        except FileNotFoundError:
            print("FAILED to load the network!")
            return self


class REINFORCE:
    """REINFORCE algorithm."""

    def __init__(self, obs_space_dims: int, action_space_dims: int):
        """Initializes an agent that learns a policy via REINFORCE algorithm [1]
        to solve the task at hand (Inverted Pendulum v4).

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """

        # Hyperparameters
        self.learning_rate = 1e-4  # orig = 1e-4 Learning rate for policy optimization
        self.gamma = 0.8  # Discount factor
        self.eps = 1e-6  # small number for mathematical stability

        self._rollout_index = 0
        self.probs = []  # Stores probability values of the sampled action
        self.rewards = []  # Stores the corresponding rewards

        self.net = Policy_Network(obs_space_dims, action_space_dims).load()
        self.optimizer = torch.optim.AdamW(self.net.parameters(), lr=self.learning_rate)

    @property
    def rollout_index(self) -> int:
        return self._rollout_index

    @rollout_index.setter
    def rollout_index(self, idx: int):
        self._rollout_index = idx

    def sample_action(self, obs: torch.Tensor) -> StepAction:
        """Returns an action, conditioned on the policy and observation.

        Args:
            state: Observation from the environment

        Returns:
            action: Action to be performed
        """

        action_means, action_stddevs = self.net(obs)

        # create a normal distribution from the predicted
        #   mean and standard deviation and sample an action
        distrib = Normal(action_means, action_stddevs + self.eps)

        action = distrib.sample()
        prob = distrib.log_prob(action)

        # NASTY HACK for single dim action
        action = torch.concat((action, torch.zeros(3)))

        action = action.numpy()

        try:
            self.probs[self.rollout_index].append(prob)
        except IndexError:
            self.probs.append([prob])

        return StepAction().from_tensor(action)

    @property
    def reward(self) -> list[list[float]]:
        return self.rewards

    @reward.setter
    def reward(self, reward: float):
        try:
            self.rewards[self.rollout_index].append(reward)
        except IndexError:
            self.rewards.append([reward])

    def update(self):
        """Updates the policy network's weights."""
        loss = torch.Tensor([0])

        # TODO: should rewards be normalized somehow?
        for rollout_log_probs, rollout_rewards in zip(self.probs, self.rewards):
            running_g = 0
            gs = []

            # Discounted return (backwards) - [::-1] will return an array in reverse
            for R in rollout_rewards[::-1]:
                running_g = R + self.gamma * running_g
                gs.insert(0, running_g)

            deltas = torch.tensor(gs)

            # minimize -1 * prob * reward obtained
            for log_prob, delta in zip(rollout_log_probs, deltas):
                # log(p1*p2*..) = log(p1) + log(p2) + ...
                loss -= log_prob.sum() * delta


        # Update the policy network
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Empty / zero out all episode-centric/related variables
        self.probs = []
        self.rewards = []


def run_episode(
    agent: REINFORCE,
    rp_env: gym.Env,
    step_time: float = 0.01,
    seed: int = 42,
    show: bool = False,
    fname: str = "",
    nrollouts: int = 1,
) -> np.ndarray:
    obs = rp_env.reset(seed=seed)

    sum_rewards = [0.0] * nrollouts
    for rollout in range(nrollouts):
        agent.rollout_index = rollout
        rp_env.reset()
        while True:
            # TODO: Wrap the rpenv into something the flattens the observation.
            obs = torch.Tensor(rp_env.state.get_state_arr())
            action = agent.sample_action(obs)

            obs_d, reward, terminated, truncated, _ = rp_env.step(action)

            sum_rewards[rollout] += reward
            agent.reward = reward

            if terminated or truncated:
                break

    return np.array(sum_rewards)


if __name__ == "__main__":
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
        render_mode="rgb_array",
    )
    rpenv.reset()

    indim = len(rpenv.state.get_state_arr())
    agent = REINFORCE(indim, 1)  # StepAction().ndim)

    nrollouts = 32

    rewards = []
    for episode in range(10001):
        rewards.append(run_episode(agent, rpenv, nrollouts=nrollouts))

        agent.update()
        if episode % 10 == 0:
            reward = np.array(rewards).mean()
            rewards = []
            agent.net.store()
            print(f"Episode {episode:<6d} ({nrollouts} rollouts) --> {reward:.3f}")

    run_episode(agent, rpenv, show=True)

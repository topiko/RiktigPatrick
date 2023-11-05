from __future__ import annotations

import functools
import logging

import torch
import torch.nn as nn

from sim.envs.rp_env import MAXA, MAXV

log = logging.getLogger(__name__)


def init_weights(m, w: float = 0.0, b: float = 0.01):
    if isinstance(m, nn.Linear):
        # torch.nn.init.xavier_uniform(m.weight)
        m.weight.data.fill_(w)
        m.bias.data.fill_(b)


class ValueNet(nn.Module):
    NETF = "nets/val_net.pth"

    def __init__(self, obs_space_dim: int):
        super().__init__()

        h1 = 32
        h2 = 32

        self.net = nn.Sequential(
            nn.Linear(obs_space_dim, h1),
            nn.LeakyReLU(),
            nn.Linear(h1, h2),
            nn.LeakyReLU(),
            nn.Linear(h2, 1),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)

    def store(self, fname: str = NETF):
        torch.save(self, fname)

    def load(self, fname: str = NETF) -> ValueNet:
        try:
            return torch.load(fname)
        except FileNotFoundError:
            log.warning("Failed to load value network.")
            return self


class PolicyNetwork(nn.Module):
    """Parametrized Policy Network."""

    NETF = "nets/rpnet_p.pth"

    def __init__(
        self, obs_space_dims: int, action_space_dims: int, init2zeros: bool = False
    ):
        """Initializes a neural network that estimates the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """
        super().__init__()

        hidden_space1 = 16  # 32  # Nothing special with 16, feel free to change
        hidden_space2 = 16  # 32  # Nothing special with 32, feel free to change
        # hidden_space3 = 512  # 32  # Nothing special with 32, feel free to change
        # hidden_space4 = 512  # 32  # Nothing special with 32, feel free to change

        # Shared Network
        self.shared_net = nn.Sequential(
            nn.Linear(obs_space_dims, hidden_space1),
            nn.Tanh(),
            nn.Linear(hidden_space1, hidden_space2),
            nn.Tanh(),
            # nn.Linear(hidden_space2, hidden_space3),
            # nn.LeakyReLU(),
            # nn.Linear(hidden_space3, hidden_space4),
            # nn.LeakyReLU(),
        )

        initto0 = functools.partial(init_weights, w=0.01, b=0.01)

        # Policy Mean specific Linear Layer
        self.policy_mean_net = nn.Sequential(
            nn.Linear(hidden_space2, action_space_dims),
            nn.Sigmoid(),
        )

        # Policy Std Dev specific Linear Layer
        self.policy_stddev_net = nn.Sequential(
            nn.Linear(hidden_space2, action_space_dims),
        )

        if init2zeros:
            self.shared_net.apply(initto0)
            self.policy_mean_net.apply(initto0)
            self.policy_stddev_net.apply(initto0)

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Conditioned on the observation, returns the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            x: Observation from the environment

        Returns:
            action_means: predicted mean of the normal distribution
            action_stddevs: predicted standard deviation of the normal distribution
        """

        shared_features = self.shared_net(x)

        action_means = (self.policy_mean_net(shared_features) - 0.5) * 2 * MAXA
        action_stddevs = torch.log(
            1 + torch.exp(self.policy_stddev_net(shared_features))
        )

        return action_means, action_stddevs

    def store(self, fname: str = NETF):
        torch.save(self, fname)

    def load(self, fname: str = NETF) -> PolicyNetwork:
        try:
            return torch.load(fname)
        except FileNotFoundError:
            log.warning("Failed to load policy network.")
            return self

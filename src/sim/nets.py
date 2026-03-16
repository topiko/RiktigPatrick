from __future__ import annotations

import functools
import logging

import torch
import torch.nn as nn

from sim.sim_config import (
    MAX_V,
    MAX_WHEEL_ACC,
    N_ACTIONS,
    OBS_SCALES,
    POLICY_TYPE,
    RAD2DEG,
    RAD2REV,
)

log = logging.getLogger(__name__)


def init_weights(m, w: float = 0.0, b: float = 0.01):
    if isinstance(m, nn.Linear):
        m.weight.data.fill_(w)
        m.bias.data.fill_(b)


class VelocityPolicyNetwork(nn.Module):
    """Continuous Gaussian policy for velocity control."""

    NETF = "src/sim/nets/rpnet_velocity.pth"

    def __init__(
        self, obs_space_dims: int, action_space_dims: int, init2zeros: bool = False
    ):
        super().__init__()

        policy_hidden1 = 32
        policy_hidden2 = 16

        value_hidden1 = 64
        value_hidden2 = 32
        value_hidden3 = 16

        initto0 = functools.partial(init_weights, w=0.01, b=0.01)

        self.policy_encoder = nn.Sequential(
            nn.Linear(obs_space_dims, policy_hidden1),
            nn.Tanh(),
            nn.Linear(policy_hidden1, policy_hidden2),
            nn.Tanh(),
        )

        self.policy_mean_net = nn.Sequential(
            nn.Linear(policy_hidden2, action_space_dims),
            nn.Sigmoid(),
        )

        self.policy_stddev_net = nn.Sequential(
            nn.Linear(policy_hidden2, action_space_dims),
        )

        self.value_encoder = nn.Sequential(
            nn.Linear(obs_space_dims, value_hidden1),
            nn.Tanh(),
            nn.Linear(value_hidden1, value_hidden2),
            nn.Tanh(),
            nn.Linear(value_hidden2, value_hidden3),
            nn.Tanh(),
        )

        self.value_head = nn.Sequential(
            nn.Linear(value_hidden3 + 1, 1),
        )

        with torch.no_grad():
            self.value_head[0].weight[0, -1] = 1.0

        if init2zeros:
            self.policy_encoder.apply(initto0)
            self.policy_mean_net.apply(initto0)
            self.policy_stddev_net.apply(initto0)
            self.value_encoder.apply(initto0)
            self.value_head.apply(initto0)

        self.return_mean = 0.0
        self.return_std = 1.0

    def forward(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        x = x.clone()

        x_norm = torch.zeros_like(x)
        x_norm[:, 0] = x[:, 0] / OBS_SCALES["filter/rp_pitch"]
        x_norm[:, 1:4] = x[:, 1:4] * RAD2DEG / OBS_SCALES["sens/gyro"]
        x_norm[:, 4] = x[:, 4] * RAD2REV / OBS_SCALES["sens/left_wheel_vel"]
        x_norm[:, 5] = x[:, 5] * RAD2REV / OBS_SCALES["sens/right_wheel_vel"]
        x_norm[:, 6] = x[:, 6] / (x[:, 6] + OBS_SCALES["env/action_time"])

        policy_features = self.policy_encoder(x_norm)
        value_features = self.value_encoder(x_norm)

        time_feature = x_norm[:, 6:7]
        value_input = torch.cat([value_features, time_feature], dim=1)

        action_means = (self.policy_mean_net(policy_features) - 0.5) * 2 * MAX_V
        action_stddevs = torch.log(
            1 + torch.exp(self.policy_stddev_net(policy_features))
        )
        if (abs(action_means) > MAX_V).any():
            raise ValueError("Invalid action mean value(s).")

        value = self.value_head(value_input)

        return action_means, action_stddevs, value

    def store(self, fname: str = NETF):
        import os

        os.makedirs(os.path.dirname(fname), exist_ok=True)
        torch.save(self, fname)

    def load(self, fname: str = NETF) -> "VelocityPolicyNetwork":
        try:
            return torch.load(fname, weights_only=False)
        except FileNotFoundError:
            log.warning("Failed to load policy network.")
            return self

    @classmethod
    def from_file(cls, fname: str = NETF) -> "VelocityPolicyNetwork":
        return torch.load(fname, weights_only=False)


class AccelPolicyNetwork(nn.Module):
    """Discrete Categorical policy for acceleration control."""

    NETF = "src/sim/nets/rpnet_accel.pth"

    def __init__(self, obs_space_dims: int, n_actions: int, init2zeros: bool = False):
        super().__init__()

        self.n_actions = n_actions

        policy_hidden1 = 32
        policy_hidden2 = 16

        value_hidden1 = 64
        value_hidden2 = 32
        value_hidden3 = 16

        initto0 = functools.partial(init_weights, w=0.01, b=0.01)

        self.policy_encoder = nn.Sequential(
            nn.Linear(obs_space_dims, policy_hidden1),
            nn.Tanh(),
            nn.Linear(policy_hidden1, policy_hidden2),
            nn.Tanh(),
        )

        self.left_wheel_logits = nn.Linear(policy_hidden2, n_actions)
        self.right_wheel_logits = nn.Linear(policy_hidden2, n_actions)

        self.value_encoder = nn.Sequential(
            nn.Linear(obs_space_dims, value_hidden1),
            nn.Tanh(),
            nn.Linear(value_hidden1, value_hidden2),
            nn.Tanh(),
            nn.Linear(value_hidden2, value_hidden3),
            nn.Tanh(),
        )

        self.value_head = nn.Sequential(
            nn.Linear(value_hidden3 + 1, 1),
        )

        with torch.no_grad():
            self.value_head[0].weight[0, -1] = 1.0

        if init2zeros:
            self.policy_encoder.apply(initto0)
            self.left_wheel_logits.apply(initto0)
            self.right_wheel_logits.apply(initto0)
            self.value_encoder.apply(initto0)
            self.value_head.apply(initto0)

        self.return_mean = 0.0
        self.return_std = 1.0

    def forward(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        x = x.clone()

        x_norm = torch.zeros_like(x)
        x_norm[:, 0] = x[:, 0] / OBS_SCALES["filter/rp_pitch"]
        x_norm[:, 1:4] = x[:, 1:4] * RAD2DEG / OBS_SCALES["sens/gyro"]
        x_norm[:, 4] = x[:, 4] * RAD2REV / OBS_SCALES["sens/left_wheel_vel"]
        x_norm[:, 5] = x[:, 5] * RAD2REV / OBS_SCALES["sens/right_wheel_vel"]
        x_norm[:, 6] = x[:, 6] / (x[:, 6] + OBS_SCALES["env/action_time"])

        policy_features = self.policy_encoder(x_norm)
        value_features = self.value_encoder(x_norm)

        time_feature = x_norm[:, 6:7]
        value_input = torch.cat([value_features, time_feature], dim=1)

        left_logits = self.left_wheel_logits(policy_features)
        right_logits = self.right_wheel_logits(policy_features)

        value = self.value_head(value_input)

        return left_logits, right_logits, value

    @staticmethod
    def action_idx_to_acc(idx: int, n_actions: int, max_acc: float) -> float:
        """Map discrete index to acceleration value.

        N=5 example: idx {0,1,2,3,4} → {-MAX, -MAX/2, 0, +MAX/2, +MAX}
        """
        return (2 * idx / (n_actions - 1) - 1) * max_acc

    def store(self, fname: str = NETF):
        import os

        os.makedirs(os.path.dirname(fname), exist_ok=True)
        torch.save(self, fname)

    def load(self, fname: str = NETF) -> "AccelPolicyNetwork":
        try:
            return torch.load(fname, weights_only=False)
        except FileNotFoundError:
            log.warning("Failed to load policy network.")
            return self

    @classmethod
    def from_file(cls, fname: str = NETF) -> "AccelPolicyNetwork":
        return torch.load(fname, weights_only=False)


# Backward compatibility alias
PolicyNetwork = VelocityPolicyNetwork

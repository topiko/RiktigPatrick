from __future__ import annotations

import functools
import logging

import torch
import torch.nn as nn

from sim.sim_config import MAX_V, OBS_SCALES, RAD2DEG, RAD2REV

log = logging.getLogger(__name__)


def init_weights(m, w: float = 0.0, b: float = 0.01):
    if isinstance(m, nn.Linear):
        # torch.nn.init.xavier_uniform(m.weight)
        m.weight.data.fill_(w)
        m.bias.data.fill_(b)


class PolicyNetwork(nn.Module):
    """Parametrized Policy Network with separate encoders for policy and value."""

    NETF = "src/sim/nets/rpnet_p.pth"

    def __init__(
        self, obs_space_dims: int, action_space_dims: int, init2zeros: bool = False
    ):
        """Initializes a neural network with separate policy and value encoders.

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """
        super().__init__()

        policy_hidden1 = 32
        policy_hidden2 = 16

        value_hidden1 = 64
        value_hidden2 = 32
        value_hidden3 = 16

        initto0 = functools.partial(init_weights, w=0.01, b=0.01)

        # Policy encoder (separate from value)
        self.policy_encoder = nn.Sequential(
            nn.Linear(obs_space_dims, policy_hidden1),
            nn.Tanh(),
            nn.Linear(policy_hidden1, policy_hidden2),
            nn.Tanh(),
        )

        # Policy Mean head
        self.policy_mean_net = nn.Sequential(
            nn.Linear(policy_hidden2, action_space_dims),
            nn.Sigmoid(),
        )

        # Policy Std Dev head
        self.policy_stddev_net = nn.Sequential(
            nn.Linear(policy_hidden2, action_space_dims),
        )

        # Value encoder (separate from policy)
        self.value_encoder = nn.Sequential(
            nn.Linear(obs_space_dims, value_hidden1),
            nn.Tanh(),
            nn.Linear(value_hidden1, value_hidden2),
            nn.Tanh(),
            nn.Linear(value_hidden2, value_hidden3),
            nn.Tanh(),
        )

        # Value head with time skip connection
        # value_hidden3 + 1 (normalized time) = 17 inputs
        self.value_head = nn.Sequential(
            nn.Linear(value_hidden3 + 1, 1),
        )

        # Initialize time feature weight to larger value
        # This helps the value network differentiate based on time
        with torch.no_grad():
            self.value_head[0].weight[0, -1] = 1.0

        if init2zeros:
            self.policy_encoder.apply(initto0)
            self.policy_mean_net.apply(initto0)
            self.policy_stddev_net.apply(initto0)
            self.value_encoder.apply(initto0)
            self.value_head.apply(initto0)

        # Return normalization stats (set by agent during training)
        self.return_mean = 0.0
        self.return_std = 1.0

    def forward(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Forward pass returns policy means, stds, and value estimate.

        Unit Conversion Flow:
            Input (from env):     rad/s, rad/s, rad/s, deg, s
                                 (gyro)  (wheel) (wheel) (pitch) (time)
            ↓
            Convert to display:   deg/s, rev/s, rev/s, deg, normalized
            ↓
            Normalize:            ÷600   ÷5     ÷5     ÷20   t/(t+10)
            ↓
            Network processing (separate encoders for policy and value)
            ↓
            Output (to env):      rad/s (actions)

        Args:
            x: Observation from environment in SI units (rad/s, deg, s)

        Returns:
            action_means: predicted mean in rad/s
            action_stddevs: predicted stddev in rad/s
            value: state value estimate
        """
        x = x.clone()

        # Convert SI units to human-friendly units before normalizing
        # Input order: pitch[deg], gyro[rad/s](3), left_vel[rad/s], right_vel[rad/s], time[s]
        x_norm = torch.zeros_like(x)
        x_norm[:, 0] = (
            x[:, 0] / OBS_SCALES["filter/rp_pitch"]
        )  # pitch: deg → normalized
        x_norm[:, 1:4] = (
            x[:, 1:4] * RAD2DEG / OBS_SCALES["sens/gyro"]
        )  # gyro: rad/s → deg/s → normalized
        x_norm[:, 4] = (
            x[:, 4] * RAD2REV / OBS_SCALES["sens/left_wheel_vel"]
        )  # wheel: rad/s → rev/s → normalized
        x_norm[:, 5] = (
            x[:, 5] * RAD2REV / OBS_SCALES["sens/right_wheel_vel"]
        )  # wheel: rad/s → rev/s → normalized
        x_norm[:, 6] = x[:, 6] / (
            x[:, 6] + OBS_SCALES["env/time"]
        )  # time: s → normalized (asymptotic)

        # Separate encoders for policy and value
        policy_features = self.policy_encoder(x_norm)
        value_features = self.value_encoder(x_norm)

        # Skip connection: pass normalized time directly to value head
        time_feature = x_norm[:, 6:7]  # Normalized time
        value_input = torch.cat([value_features, time_feature], dim=1)

        # Output actions in rad/s (SI units for environment)
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

    def load(self, fname: str = NETF) -> PolicyNetwork:
        try:
            return torch.load(fname, weights_only=False)
        except FileNotFoundError:
            log.warning("Failed to load policy network.")
            return self

    @classmethod
    def from_file(cls, fname: str = NETF) -> PolicyNetwork:
        return torch.load(fname, weights_only=False)

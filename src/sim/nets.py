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
    """Parametrized Policy Network with shared encoder and separate policy/value heads."""

    NETF = "src/sim/nets/rpnet_p.pth"

    def __init__(
        self, obs_space_dims: int, action_space_dims: int, init2zeros: bool = False
    ):
        """Initializes a neural network with shared encoder and policy/value heads.

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """
        super().__init__()

        hidden_space1 = 32
        hidden_space2 = 32
        hidden_space3 = 16
        hidden_space4 = 8

        # Shared Encoder
        self.shared_net = nn.Sequential(
            nn.Linear(obs_space_dims, hidden_space1),
            nn.Tanh(),
            nn.Linear(hidden_space1, hidden_space2),
            nn.Tanh(),
            nn.Linear(hidden_space2, hidden_space3),
            nn.Tanh(),
            nn.Linear(hidden_space3, hidden_space4),
            nn.Tanh(),
        )

        initto0 = functools.partial(init_weights, w=0.01, b=0.01)

        # Policy Mean head
        self.policy_mean_net = nn.Sequential(
            nn.Linear(hidden_space4, action_space_dims),
            nn.Sigmoid(),
        )

        # Policy Std Dev head
        self.policy_stddev_net = nn.Sequential(
            nn.Linear(hidden_space4, action_space_dims),
        )

        # Value head (critic)
        self.value_net = nn.Sequential(
            nn.Linear(hidden_space4, 1),
        )

        if init2zeros:
            self.shared_net.apply(initto0)
            self.policy_mean_net.apply(initto0)
            self.policy_stddev_net.apply(initto0)
            self.value_net.apply(initto0)

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
            Network processing
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
        x[:, 0] = x[:, 0] / OBS_SCALES["filter/rp_pitch"]  # pitch: deg → normalized
        x[:, 1:4] = (
            x[:, 1:4] * RAD2DEG / OBS_SCALES["sens/gyro"]
        )  # gyro: rad/s → deg/s → normalized
        x[:, 4] = (
            x[:, 4] * RAD2REV / OBS_SCALES["sens/left_wheel_vel"]
        )  # wheel: rad/s → rev/s → normalized
        x[:, 5] = (
            x[:, 5] * RAD2REV / OBS_SCALES["sens/right_wheel_vel"]
        )  # wheel: rad/s → rev/s → normalized
        x[:, 6] = x[:, 6] / (
            x[:, 6] + OBS_SCALES["env/time"]
        )  # time: s → normalized (asymptotic)

        shared_features = self.shared_net(x)

        # Output actions in rad/s (SI units for environment)
        action_means = (self.policy_mean_net(shared_features) - 0.5) * 2 * MAX_V
        action_stddevs = torch.log(
            1 + torch.exp(self.policy_stddev_net(shared_features))
        )
        if (abs(action_means) > MAX_V).any():
            raise ValueError("Invalid action mean value(s).")

        value = self.value_net(shared_features)

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

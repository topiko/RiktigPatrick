"""
Neural Network Agent for RiktigPatrick

UNIT SYSTEM:
All inputs and outputs use SI units (International System of Units):
- Observations: rad, rad/s (wheel velocities, gyro, etc.)
- Actions: rad/s² (wheel accelerations)
- See rp_env.py for detailed unit documentation
"""

import torch
from torch import nn

from riktigpatric.patrick import Actions, Observables


def idx2value(
    action: Actions, idx: torch.tensor, nbins: int, max_acc: float
) -> torch.tensor:
    """Convert discrete action index to continuous acceleration value.

    Args:
        action: Action type
        idx: Discrete action index (0 to nbins-1)
        nbins: Number of discrete action bins
        max_acc: Maximum acceleration in rad/s² (SI units)

    Returns:
        Continuous action value in rad/s² (SI units)
    """
    if action == Actions.ACC_BOTH_WHEELS:
        return (-1.0 + 2.0 * idx / (nbins - 1)) * max_acc  # rad/s²

    raise ValueError(f"Unknown action: {action}")


class Agent(nn.Module):
    """Neural network policy for robot control.

    Inputs: Observations in SI units (rad, rad/s, etc.)
    Outputs: Actions in SI units (rad/s²)
    """

    def __init__(
        self,
        inputs: tuple[Observables],
        actions: dict[Actions, int],
        max_wheel_acc: float,
    ):
        """Initialize agent.

        Args:
            inputs: List of observation keys to use as input
            actions: Dict mapping action types to number of discrete bins
            max_wheel_acc: Maximum wheel acceleration in rad/s² (SI units)
        """
        super().__init__()
        self.inputs = inputs
        self.actions = actions
        self.max_wheel_acc = max_wheel_acc  # rad/s² (SI units)
        self.model = torch.nn.Sequential(
            torch.nn.Linear(len(inputs), 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 32),
            torch.nn.ReLU(),
        )

        self.action_heads = torch.nn.ModuleDict(
            {a: torch.nn.Linear(32, n) for a, n in actions.items()}
        )

    def forward(
        self, x: dict[Observables, torch.Tensor]
    ) -> dict[Actions, torch.Tensor]:
        input_tensor = torch.cat([x[obs] for obs in self.inputs], dim=-1)

        action_logits = {
            action: head(self.model(input_tensor))
            for action, head in self.action_heads.items()
        }

        return action_logits

    def act(
        self, x: dict[Observables, torch.Tensor]
    ) -> tuple[dict[Actions, torch.Tensor], torch.Tensor]:
        action_logits = self.forward(x)

        actions: dict[Actions, torch.Tensor] = {}
        logp_l = []
        for k, v in action_logits.items():
            if k == Actions.ACC_BOTH_WHEELS:
                dist = torch.distributions.Categorical(logits=v)
                action_idx = dist.sample()
                logp = dist.log_prob(action_idx)
                action = idx2value(k, action_idx, self.actions[k], self.max_wheel_acc)

            else:
                raise ValueError(f"Unknown action: {k}")

            actions[k] = action.unsqueeze(-1)
            logp_l.append(logp.unsqueeze(-1))

        logp = torch.cat(logp_l, dim=1).sum(dim=1, keepdim=True)

        return actions, logp

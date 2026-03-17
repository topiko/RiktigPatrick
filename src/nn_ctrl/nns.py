import torch
from torch import nn

from riktigpatric.patrick import Actions, Observables
from sim.envs.rp_env import MAX_WHEEL_ACC


def idx2value(action: Actions, idx: torch.tensor, nbins: int) -> torch.tensor:
    if action == Actions.ACC_BOTH_WHEELS:
        return (-1.0 + 2.0 * idx / (nbins - 1)) * MAX_WHEEL_ACC

    raise ValueError(f"Unknown action: {action}")


class Agent(nn.Module):
    def __init__(self, inputs: tuple[Observables], actions: dict[Actions, int]):
        super().__init__()
        self.inputs = inputs
        self.actions = actions
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
                action = idx2value(k, action_idx, self.actions[k])

            else:
                raise ValueError(f"Unknown action: {k}")

            actions[k] = action.unsqueeze(-1)
            logp_l.append(logp.unsqueeze(-1))

        logp = torch.cat(logp_l, dim=1).sum(dim=1, keepdim=True)

        return actions, logp

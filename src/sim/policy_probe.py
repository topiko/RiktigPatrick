"""Cheap conditional policy-KL diagnostics on sampled rollout observations/states."""

from dataclasses import dataclass

import numpy as np
import torch
from torch.distributions import Categorical, Normal, kl_divergence

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import Actions, StateVarKey


@dataclass
class PolicySample:
    observations: dict[StateVarKey, torch.Tensor]
    hidden: torch.Tensor
    parameters: dict[Actions, torch.Tensor]


class PolicyProbe:
    """Compare before/after distributions conditional on the same cached GRU state.

    Samples stay on CPU without autograd history. This measures local policy
    changes, not a full replay with the updated policy's recurrent history.
    """

    def __init__(self, agent: Agent, every: int):
        if type(every) is not int or every < 1:
            raise ValueError("kl_probe_every must be a positive integer")
        self.every = every
        self.actions = tuple(
            a for a in agent.action_configs if a not in agent.inactive_actions
        )
        self.old_stds = {
            a: head.std.detach().cpu().clone()
            for a in self.actions
            if isinstance(head := agent.action_heads[a.value], ContinuousHead)
        }
        self.samples: list[PolicySample] = []

    @torch.no_grad()
    def capture(self, agent: Agent, observations: dict, hidden, active: np.ndarray):
        parameters, _, _ = agent.forward(observations, hidden)
        keep = torch.as_tensor(active, device=agent.device)
        if hidden is None:
            hidden = torch.zeros(
                agent.rnn.num_layers, len(active), agent.rnn.hidden_size,
                device=agent.device, dtype=next(agent.parameters()).dtype,
            )
        self.samples.append(PolicySample(
            {k: v[keep].detach().cpu().clone() for k, v in observations.items()},
            hidden[:, keep].detach().cpu().clone(),
            {a: parameters[a][keep].detach().cpu().clone() for a in self.actions},
        ))

    @torch.no_grad()
    def metrics(self, agent: Agent) -> dict[str, float]:
        if not self.samples or not self.actions:
            return {}
        observations = {
            key: torch.cat([s.observations[key] for s in self.samples]).to(agent.device)
            for key in self.samples[0].observations
        }
        hidden = torch.cat([s.hidden for s in self.samples], dim=1).to(agent.device)
        current, _, _ = agent.forward(observations, hidden)
        joint_kl = torch.zeros(hidden.shape[1], device=agent.device)
        for action in self.actions:
            previous = torch.cat([
                s.parameters[action] for s in self.samples
            ]).to(agent.device)
            head = agent.action_heads[action.value]
            if isinstance(head, ContinuousHead):
                # KL is invariant under the common invertible tanh/scale transform.
                old = Normal(previous, self.old_stds[action].to(agent.device))
                new = Normal(current[action], head.std)
                joint_kl += kl_divergence(old, new).sum(dim=-1)
            else:
                joint_kl += kl_divergence(
                    Categorical(logits=previous), Categorical(logits=current[action])
                )
        joint_kl = joint_kl.clamp_min(0)  # Suppress tiny negative round-off at zero KL.
        return {
            "optimization/policy_kl_mean": float(joint_kl.mean()),
            "optimization/policy_kl_max": float(joint_kl.max()),
            "optimization/policy_probe_samples": float(joint_kl.numel()),
        }

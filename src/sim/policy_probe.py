"""Conditional probes and full-history recurrent policy-KL replay."""

from dataclasses import dataclass

import numpy as np
import torch
from torch.distributions import Categorical, Normal, kl_divergence

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import Actions, StateVar, StateVarKey
from sim.utils import EpisodeBuffer


def _policy_kl(agent, previous, current, old_stds):
    terms = []
    for action, parameters in previous.items():
        head = agent.action_heads[action.value]
        if isinstance(head, ContinuousHead):
            # The common invertible tanh/scale transform leaves KL unchanged.
            old = Normal(parameters, old_stds[action].to(agent.device))
            new = Normal(current[action], head.std)
            terms.append(kl_divergence(old, new).sum(dim=-1))
        else:
            terms.append(kl_divergence(
                Categorical(logits=parameters), Categorical(logits=current[action])
            ))
    return torch.stack(terms).sum(dim=0).clamp_min(0)


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
        previous = {
            action: torch.cat([
                s.parameters[action] for s in self.samples
            ]).to(agent.device)
            for action in self.actions
        }
        joint_kl = _policy_kl(agent, previous, current, self.old_stds)
        return {
            "optimization/policy_kl_mean": float(joint_kl.mean()),
            "optimization/policy_kl_max": float(joint_kl.max()),
            "optimization/policy_probe_samples": float(joint_kl.numel()),
        }


class RecurrentPolicyReplay:
    """Compare policies on old observation histories, rebuilding each GRU state.

    All valid decision-time observations are retained. Each replay starts from a
    fresh hidden state; padding and final observations do not contribute to KL.
    No actions are sampled and no simulator transitions are executed.
    """

    def __init__(self, agent: Agent, buffers: list[EpisodeBuffer]):
        lengths = [buffer.seq_len for buffer in buffers]
        if not lengths or min(lengths) < 1:
            raise ValueError("Recurrent KL replay requires nonempty finished episodes")
        self.steps = max(lengths)
        self.samples = sum(lengths)
        self.lengths = lengths
        self.episode_metrics: list[dict[str, float]] = []
        self.actions = tuple(
            action for action in agent.action_configs
            if action not in agent.inactive_actions
        )
        self.valid = (
            torch.arange(self.steps, device=agent.device)[None, :]
            < torch.tensor(lengths, device=agent.device)[:, None]
        )
        self.observations = {}
        for name in agent.inputs:
            key = StateVar.from_str(name)
            shape = np.asarray(buffers[0].obs_l[0][key]).shape
            values = np.zeros((len(buffers), self.steps, *shape), dtype=np.float32)
            for row, buffer in enumerate(buffers):
                values[row, :lengths[row]] = np.stack([
                    observations[key] for observations in buffer.obs_l[:-1]
                ])
            self.observations[key] = torch.from_numpy(values).to(agent.device)
        self.old_stds = {
            action: head.std.detach().clone()
            for action in self.actions
            if isinstance(head := agent.action_heads[action.value], ContinuousHead)
        }
        self.previous = self._replay(agent)

    @torch.no_grad()
    def _replay(self, agent):
        if not self.actions:
            return {}
        was_training = agent.training
        agent.eval()
        try:
            hidden = None
            parameters = {action: [] for action in self.actions}
            for step in range(self.steps):
                observations = {key: values[:, step]
                                for key, values in self.observations.items()}
                current, _, hidden = agent.forward(observations, hidden)
                for action in self.actions:
                    parameters[action].append(current[action])
            return {
                action: torch.stack(values, dim=1)[self.valid]
                for action, values in parameters.items()
            }
        finally:
            agent.train(was_training)

    @torch.no_grad()
    def metrics(self, agent: Agent) -> dict[str, float]:
        joint = (
            _policy_kl(agent, self.previous, self._replay(agent), self.old_stds)
            if self.actions else torch.zeros(self.samples, device=agent.device)
        ).cpu()
        self.episode_metrics = [
            {"index": index, "min": float(values.min()),
             "mean": float(values.mean()), "max": float(values.max()),
             "steps": length}
            for index, (length, values) in enumerate(zip(
                self.lengths, joint.split(self.lengths), strict=True
            ))
        ]
        means = np.array([episode["mean"] for episode in self.episode_metrics])
        return {
            "min": float(joint.min()), "mean": float(joint.mean()),
            "max": float(joint.max()),
            "samples": float(self.samples),
            "episode_mean_max": float(means.max()),
            "episode_mean_p95": float(np.quantile(means, 0.95)),
            "worst_episode": float(means.argmax()),
        }

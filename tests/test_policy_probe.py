"""Conditional KL diagnostics must detect updates without changing rollout sampling."""

import unittest

import numpy as np
import torch

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import Actions, Observable, StateVarKey
from sim.policy_probe import PolicyProbe


def agent():
    return Agent([Observable.OBS_TIME.value], {
        Actions.ACC_BOTH_WHEELS.value: {
            "type": "continuous", "limit": 150, "initial_std": 0.1,
            "min_std": 0.02, "max_std": 0.5,
        },
        Actions.VEL_WHEEL_DIFF.value: {"type": "discrete", "bins": [-1, 0, 1]},
    }, hsize=4)


class PolicyProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_kl_matches_known_gaussian_shift_and_ignores_inactive_heads(self):
        policy = agent()
        policy.inactive_actions = (Actions.VEL_WHEEL_DIFF,)
        probe = PolicyProbe(policy, 1)
        obs = {Observable.OBS_TIME: torch.zeros(2, 1)}
        rng = torch.get_rng_state().clone()
        probe.capture(policy, obs, None, np.array([True, False]))
        self.assertAlmostEqual(probe.metrics(policy)["optimization/policy_kl_mean"], 0)
        wheel = policy.action_heads[Actions.ACC_BOTH_WHEELS.value]
        assert isinstance(wheel, ContinuousHead) and wheel.bias is not None
        yaw = policy.action_heads[Actions.VEL_WHEEL_DIFF.value]
        assert isinstance(yaw, torch.nn.Linear) and yaw.bias is not None
        with torch.no_grad():
            wheel.bias.add_(0.1)
            yaw.bias[0].add_(10)
        metrics = probe.metrics(policy)
        self.assertAlmostEqual(metrics["optimization/policy_kl_mean"], 0.5, places=5)
        self.assertAlmostEqual(metrics["optimization/policy_kl_max"], 0.5, places=5)
        self.assertEqual(metrics["optimization/policy_probe_samples"], 1)
        torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)
        self.assertTrue(all(p.grad is None for p in policy.parameters()))

    def test_probe_owns_detached_states_and_detects_shared_recurrent_changes(self):
        policy = agent()
        head = policy.action_heads[Actions.ACC_BOTH_WHEELS.value]
        assert isinstance(head, ContinuousHead)
        with torch.no_grad():
            head.weight[0, 0] = 1
        obs: dict[StateVarKey, torch.Tensor] = {Observable.OBS_TIME: torch.ones(2, 1)}
        _, _, hidden = policy.forward(obs)
        assert hidden is not None
        probe = PolicyProbe(policy, 1)
        probe.capture(policy, obs, hidden, np.array([True, True]))
        cached = probe.samples[0]
        expected_hidden = cached.hidden.clone()
        obs[Observable.OBS_TIME].fill_(99)
        with torch.no_grad():
            hidden.fill_(99)
            policy.rnn.bias_ih_l0[0].add_(1)
        torch.testing.assert_close(cached.hidden, expected_hidden)
        torch.testing.assert_close(
            cached.observations[Observable.OBS_TIME], torch.ones(2, 1)
        )
        for tensor in (cached.hidden, *cached.parameters.values()):
            self.assertIsNone(tensor.grad_fn)
            self.assertEqual(tensor.device.type, "cpu")
        self.assertGreater(probe.metrics(policy)["optimization/policy_kl_mean"], 0)


if __name__ == "__main__":
    unittest.main()

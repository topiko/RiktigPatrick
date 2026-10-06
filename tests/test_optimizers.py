"""Separate Adam rates must survive checkpoint resume and guard recovery."""

import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import torch
from test_critic_isolation import agent_and_inputs

from sim.checkpoints import PolicyGuard, capture_state, load_checkpoint, save_checkpoint
from sim.optimizers import make_optimizer


def prime(agent, optimizer):
    sum(p.square().sum() for p in agent.parameters()).backward()
    optimizer.step()
    optimizer.zero_grad(set_to_none=True)


class OptimizerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_groups_cover_model_once_and_apply_independent_rates(self):
        agent, _ = agent_and_inputs()
        optimizer = make_optimizer(agent, 0.0001, 0.002)
        grouped = [p for group in optimizer.param_groups for p in group["params"]]
        self.assertEqual(len(grouped), len(list(agent.parameters())))
        self.assertEqual({id(p) for p in grouped}, {id(p) for p in agent.parameters()})
        self.assertEqual(
            {id(p) for p in optimizer.param_groups[1]["params"]},
            {id(p) for p in agent.value_head.parameters()},
        )
        before = deepcopy(agent.state_dict())
        for parameter in agent.parameters():
            parameter.grad = torch.ones_like(parameter)
        optimizer.step()
        for name, parameter in agent.named_parameters():
            rate = 0.002 if name.startswith("value_head.") else 0.0001
            torch.testing.assert_close(parameter, before[name] - rate)

    def test_resume_rates_can_be_overridden_independently_without_losing_moments(self):
        agent, _ = agent_and_inputs()
        optimizer = make_optimizer(agent, 0.0001, 0.002)
        prime(agent, optimizer)
        snapshot = capture_state(agent, optimizer, 8)
        with tempfile.TemporaryDirectory() as directory:
            path = save_checkpoint(Path(directory) / "model.pt", snapshot, {})
            for policy, critic, expected in (
                (None, None, [0.0001, 0.002]),
                (0.0003, None, [0.0003, 0.002]),
                (None, 0.004, [0.0001, 0.004]),
                (0.0003, 0.004, [0.0003, 0.004]),
            ):
                self.assertEqual(load_checkpoint(
                    path, agent, optimizer, learning_rate=policy,
                    critic_learning_rate=critic,
                ), 8)
                self.assertEqual([g["lr"] for g in optimizer.param_groups], expected)
                torch.testing.assert_close(optimizer.state_dict()["state"],
                                           snapshot["optimizer"]["state"])

    def test_legacy_single_group_preserves_moments_when_split(self):
        agent, _ = agent_and_inputs()
        legacy = torch.optim.Adam(agent.parameters(), lr=0.003,
                                  betas=(0.8, 0.95), amsgrad=True)
        prime(agent, legacy)
        snapshot = capture_state(agent, legacy, 8)
        optimizer = make_optimizer(agent, 0.0001, 0.002)
        with tempfile.TemporaryDirectory() as directory:
            path = save_checkpoint(Path(directory) / "legacy.pt", snapshot, {})
            load_checkpoint(path, agent, optimizer)
        self.assertEqual(
            [g["name"] for g in optimizer.param_groups], ["policy", "critic"]
        )
        self.assertEqual([g["lr"] for g in optimizer.param_groups], [0.003, 0.003])
        for group in optimizer.param_groups:
            self.assertEqual(group["betas"], (0.8, 0.95))
            self.assertTrue(group["amsgrad"])
        for parameter in agent.parameters():
            torch.testing.assert_close(
                optimizer.state[parameter], legacy.state[parameter]
            )

    def test_guard_clears_both_groups_and_preserves_distinct_reduced_rates(self):
        agent, _ = agent_and_inputs()
        optimizer = make_optimizer(agent, 0.0002, 0.002)
        prime(agent, optimizer)
        guard = PolicyGuard(agent, optimizer, patience=1, lr_factor=0.5)
        guard.observe(100, 0)
        prime(agent, optimizer)
        self.assertEqual(guard.observe(0, 1), "rollback")
        self.assertEqual(len(optimizer.state), 0)
        self.assertEqual([g["lr"] for g in optimizer.param_groups], [0.0001, 0.001])
        prime(agent, optimizer)
        self.assertEqual(guard.observe(0, 2), "recovery_rejected")
        self.assertEqual(len(optimizer.state), 0)
        self.assertEqual([g["lr"] for g in optimizer.param_groups], [0.0001, 0.001])

    def test_invalid_group_learning_rates_are_rejected(self):
        agent, _ = agent_and_inputs()
        for rate in (0, -1, float("nan"), float("inf")):
            with self.assertRaisesRegex(ValueError, "policy learning rate"):
                make_optimizer(agent, rate, 0.001)
            with self.assertRaisesRegex(ValueError, "critic learning rate"):
                make_optimizer(agent, 0.001, rate)


if __name__ == "__main__":
    unittest.main()

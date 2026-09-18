"""Frozen-checkpoint diagnostics must match real updates and keep scoring fixed."""

import random
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import numpy as np
import torch
from omegaconf import OmegaConf
from test_continuous_policy import config as continuous_config
from test_training_safety import OneStepEnv, agent_and_optimizer, config

from sim.checkpoints import capture_state, restore_state, save_checkpoint
from sim.curriculum import Curriculum
from sim.diagnose_updates import (
    apply_gradients,
    collect_gradients,
    compare_updates,
    mean_gradients,
)
from sim.train_agent import make_agent, training_update
from sim.utils import SingleEnvWrapper


class UpdateDiagnosticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_joint_diagnostic_reproduces_real_adam_update(self):
        cfg = config()
        agent, optimizer = agent_and_optimizer()
        before = capture_state(agent, optimizer, 0)
        env = SingleEnvWrapper(OneStepEnv())
        seed = cfg.seed + 3
        gradients, _ = collect_gradients(cfg, env, agent, None, seed, "joint")
        torch.testing.assert_close(agent.state_dict(), before["model"], rtol=0, atol=0)
        apply_gradients(agent, optimizer, gradients, cfg.train.grad_clip)
        candidate = capture_state(agent, optimizer, 4)

        restore_state(agent, optimizer, before)
        torch.manual_seed(seed)
        np.random.seed(seed)
        random.seed(seed)
        training_update(cfg, env, agent, optimizer, 3)
        torch.testing.assert_close(agent.state_dict(), candidate["model"])
        torch.testing.assert_close(optimizer.state_dict(), candidate["optimizer"])

    def test_actor_diagnostics_leave_critic_weights_and_adam_moments_untouched(self):
        agent, optimizer = agent_and_optimizer()
        cfg = config()
        env = SingleEnvWrapper(OneStepEnv())
        before = capture_state(agent, optimizer, 0)
        first, metrics = collect_gradients(cfg, env, agent, None, 123, "actor")
        again, repeated = collect_gradients(cfg, env, agent, None, 123, "actor")
        torch.testing.assert_close(first, again, rtol=0, atol=0)
        self.assertEqual(metrics, repeated)
        critic = [p for name, p in agent.named_parameters() if name.startswith(
            "value_head."
        )]
        states = [{k: v.clone() for k, v in optimizer.state[p].items()} for p in critic]
        apply_gradients(agent, optimizer, mean_gradients([first, again]), 1.0)
        for name, parameter in agent.named_parameters():
            if name.startswith("value_head."):
                self.assertIsNone(parameter.grad)
                torch.testing.assert_close(parameter, before["model"][name])
        for parameter, state in zip(critic, states):
            torch.testing.assert_close(optimizer.state[parameter], state)

    def test_reward_ablation_keeps_validation_and_source_checkpoint_fixed(self):
        cfg = continuous_config()
        curriculum = Curriculum(cfg)
        agent = make_agent(cfg)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        state = capture_state(agent, optimizer, 10, curriculum)
        with tempfile.TemporaryDirectory() as folder, redirect_stdout(StringIO()):
            cfg.checkpoints.dir = folder
            saved_config = OmegaConf.to_container(cfg, resolve=True)
            assert isinstance(saved_config, dict)
            path = save_checkpoint(Path(folder) / "best.pt", state, saved_config)
            original = path.read_bytes()
            normal = compare_updates(cfg, path, batches=2, validation_seeds=[1042])
            no_speed = compare_updates(cfg, path, batches=2, validation_seeds=[1042],
                                       hold_velocity_weight=0.0)
            repeated = compare_updates(cfg, path, batches=2, validation_seeds=[1042])
            self.assertEqual(normal, repeated)
            self.assertEqual(normal["baseline"], no_speed["baseline"])
            self.assertEqual(set(normal["candidates"]), {"batch_0", "batch_1", "mean"})
            self.assertEqual(no_speed["training_hold_velocity_weight"], 0.0)
            self.assertEqual(no_speed["validation_hold_velocity_weight"], 0.125)
            self.assertEqual(cfg.curriculum.hold_velocity_weight, 0.125)
            self.assertEqual(path.read_bytes(), original)
            for old, new in zip(normal["batches"], no_speed["batches"]):
                self.assertGreater(new["return_mean"], old["return_mean"])


if __name__ == "__main__":
    unittest.main()

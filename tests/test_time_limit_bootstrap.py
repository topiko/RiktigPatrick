"""Time limits bootstrap the final recurrent state; falls and padding do not."""

import csv
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import gymnasium as gym
import numpy as np
import torch
from gymnasium.vector import AutoresetMode, SyncVectorEnv
from omegaconf import OmegaConf
from test_training_safety import config

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, Observable
from sim.checkpoints import capture_state
from sim.diagnose_updates import apply_gradients, collect_gradients
from sim.episode_io import save_episode_csv
from sim.train_agent import evaluation_rollout, rollout, training_update
from sim.utils import (
    actor_critic_losses,
    ebufs2batchd,
    ebufs2bootstrap_values,
    ebufs2step_times,
    get_returns,
)


class EndingEnv(gym.Env):
    """A next-step-autoreset environment with distinguishable final observations."""

    def __init__(self, length, ending):
        self.length, self.ending = length, ending
        self.observation_space = gym.spaces.Dict({
            Observable.OBS_TIME: gym.spaces.Box(0, 100, (1,), dtype=np.float32),
        })
        self.action_space = gym.spaces.Dict({
            Actions.ACC_BOTH_WHEELS: gym.spaces.Box(-1, 1, (1,), dtype=np.float32),
        })
        self.time = 0

    def observation(self):
        return {Observable.OBS_TIME: np.array([self.time], dtype=np.float32)}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.time = 0
        return self.observation(), {}

    def step(self, action):
        self.time += 1
        done = self.time == self.length
        return (self.observation(), 1.0, done and self.ending in ("fall", "both"),
                done and self.ending in ("limit", "both"), {"step_time": 1.0})


class MemoryAgent(Agent):
    """Analytic critic: accumulate (observation time + 1) in recurrent memory."""

    def __init__(self):
        super().__init__([Observable.OBS_TIME.value], {
            Actions.ACC_BOTH_WHEELS.value: {"type": "discrete", "bins": [-1., 1.]},
        }, hsize=4)
        self.memory_scale = torch.nn.Parameter(torch.tensor(1.0))

    def forward(self, x, h=None):
        increment = self.memory_scale * (x[Observable.OBS_TIME] + 1).unsqueeze(0)
        increment = increment.expand(1, -1, self.rnn.hidden_size)
        h = increment if h is None else h + increment
        values = h[0, :, :1]
        logits = torch.cat((-values, values), dim=-1) * 0.05
        return {Actions.ACC_BOTH_WHEELS: logits}, values, h


class TimeLimitBootstrapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def env(self, endings=((2, "fall"), (3, "limit"), (2, "both"), (4, "limit"))):
        env = SyncVectorEnv([
            lambda length=length, ending=ending: EndingEnv(length, ending)
            for length, ending in endings
        ], autoreset_mode=AutoresetMode.NEXT_STEP)
        self.addCleanup(env.close)
        return env

    def test_final_state_and_memory_are_used_before_reset_without_sampling(self):
        agent = MemoryAgent()
        rng = torch.get_rng_state().clone()
        baseline = rollout(self.env(), agent, tbptt_steps=2)
        after = torch.get_rng_state().clone()
        torch.set_rng_state(rng)
        with patch.object(agent, "forward", wraps=agent.forward) as forward:
            bootstrapped = rollout(self.env(), agent, tbptt_steps=2,
                                   bootstrap_time_limits=True)
        # Four actions plus two final-state value estimates.
        self.assertEqual(forward.call_count, 6)
        torch.testing.assert_close(torch.get_rng_state(), after, rtol=0, atol=0)
        # Final values sum 1..(T+1), including the final observation. Resetting
        # hidden state or evaluating an autoreset observation gives different values.
        self.assertEqual([b.bootstrap_value for b in bootstrapped], [0, 10, 0, 15])
        self.assertEqual(
            [b.terminated for b in bootstrapped], [True, False, True, False]
        )
        self.assertEqual([b.truncated for b in bootstrapped], [False, True, True, True])
        for plain, boot in zip(baseline, bootstrapped):
            self.assertEqual(plain.bootstrap_value, 0)
            np.testing.assert_array_equal(plain.get_action(Actions.ACC_BOTH_WHEELS),
                                          boot.get_action(Actions.ACC_BOTH_WHEELS))
            torch.testing.assert_close(
                plain.get_logps(), boot.get_logps(), rtol=0, atol=0
            )
            torch.testing.assert_close(
                plain.get_values(), boot.get_values(), rtol=0, atol=0
            )
            self.assertEqual(plain.rewards_l, boot.rewards_l)

    def test_bootstrap_is_discounted_once_at_unequal_ends_and_detached(self):
        rewards = torch.tensor([[1., 2., 999., 999.], [3., 4., 5., 6.]])
        mask = torch.tensor([[1., 1., 0., 0.], [1., 1., 1., 1.]])
        bootstrap = torch.tensor([10., 0.], requires_grad=True)
        times = torch.tensor([[0.02, 0.01, 0., 0.], [0.01, 0.01, 0.01, 0.01]])
        for step_times, first in ((None, 4.5), (times, 2.75)):
            targets = get_returns(rewards, 0.5, bootstrap_values=bootstrap,
                                  valid_mask=mask, step_times=step_times)
            torch.testing.assert_close(targets, torch.tensor([
                [first, 7., 0., 0.], [7., 8., 8., 6.],
            ]))
            self.assertFalse(targets.requires_grad)
        torch.testing.assert_close(
            get_returns(torch.tensor([1., 2.]), 0.5,
                        bootstrap_values=torch.tensor(-6.)),
            torch.tensor([0.5, -1.]),
        )

    def test_flag_changes_learning_targets_but_not_measured_returns(self):
        cfg = config()
        cfg.env.step_time = 1.0
        cfg.rl.discount = 0.5
        for enabled, expected_mse in ((False, 2.125), (True, 2.5)):
            cfg.rl.bootstrap_time_limits = enabled
            agent = MemoryAgent()
            optimizer = torch.optim.Adam(agent.parameters())
            metrics = training_update(
                cfg, self.env(((2, "limit"),)), agent, optimizer, 0,
            )
            self.assertAlmostEqual(metrics["losses/value"], expected_mse)
            self.assertEqual(metrics["returns/mean"], 2.0)

    def test_final_estimate_is_not_a_second_critic_gradient_and_is_exported(self):
        agent = MemoryAgent()
        buffers = rollout(self.env(((2, "limit"),)), agent, bootstrap_time_limits=True)
        logps, rewards, values, _, mask = ebufs2batchd(buffers)
        targets = get_returns(
            rewards, 0.5, bootstrap_values=ebufs2bootstrap_values(buffers),
            valid_mask=mask, step_times=ebufs2step_times(buffers),
            reference_step_time=1,
        )
        torch.testing.assert_close(targets, torch.tensor([[3., 4.]]))
        _, value_loss = actor_critic_losses(logps, targets, values, mask)
        gradient, = torch.autograd.grad(value_loss, agent.memory_scale)
        self.assertEqual(gradient.item(), -5.0)
        with tempfile.TemporaryDirectory() as folder:
            path = save_episode_csv(buffers[0], Path(folder) / "episode.csv",
                                    returns=targets[0].numpy(),
                                    advantages=(targets - values.detach())[0].numpy())
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[-1]["episode/terminated"], "0.0")
            self.assertEqual(rows[-1]["episode/truncated"], "1.0")
            self.assertEqual(rows[-1]["episode/bootstrap_value"], "6.0")
            self.assertEqual(rows[-1]["policy/return"], "")
            self.assertEqual(rows[0]["episode/bootstrap_value"], "")

    def test_evaluation_bootstrap_preserves_policy_mode_and_training_rng(self):
        agent = MemoryAgent()
        before = torch.get_rng_state().clone()
        buffers = evaluation_rollout(self.env(((2, "limit"),)), agent, 123,
                                     bootstrap_time_limits=True)
        self.assertTrue(agent.training)
        self.assertEqual(buffers[0].bootstrap_value, 6.0)
        torch.testing.assert_close(torch.get_rng_state(), before, rtol=0, atol=0)

    def test_gradient_diagnostics_use_the_same_bootstrapped_targets_as_training(self):
        cfg = config()
        cfg.rl.bootstrap_time_limits = True
        cfg.env.step_time = 1.0
        cfg.train.kl_probe_every = 0
        agent = MemoryAgent()
        before = {k: v.clone() for k, v in agent.state_dict().items()}
        optimizer = torch.optim.Adam(agent.parameters())
        gradients, _ = collect_gradients(
            cfg, self.env(((2, "limit"),)), agent, None, cfg.seed, "joint",
        )
        apply_gradients(agent, optimizer, gradients, cfg.train.grad_clip)
        expected = {k: v.clone() for k, v in agent.state_dict().items()}
        agent.load_state_dict(before)
        optimizer = torch.optim.Adam(agent.parameters())
        torch.manual_seed(cfg.seed)
        training_update(cfg, self.env(((2, "limit"),)), agent, optimizer, 0)
        torch.testing.assert_close(agent.state_dict(), expected)

    @unittest.skipUnless(torch.cuda.is_available(), "CUDA device unavailable")
    def test_cuda_final_values_and_return_targets(self):
        agent = MemoryAgent().to("cuda")
        buffers = rollout(self.env(), agent, bootstrap_time_limits=True)
        _, rewards, _, _, mask = ebufs2batchd(buffers)
        self.assertEqual(rewards.device.type, "cuda")
        values = ebufs2bootstrap_values(buffers, device=agent.device)
        torch.testing.assert_close(values.cpu(), torch.tensor([0., 10., 0., 15.]))
        targets = get_returns(rewards, .5, bootstrap_values=values, valid_mask=mask)
        self.assertFalse(targets.requires_grad)
        torch.testing.assert_close(targets.cpu(), torch.tensor([
            [1.5, 1., 0., 0.], [3., 4., 6., 0.],
            [1.5, 1., 0., 0.], [2.8125, 3.625, 5.25, 8.5],
        ]))

    def test_nonfinite_final_value_restores_the_training_transaction(self):
        cfg = config()
        cfg.rl.bootstrap_time_limits = True
        agent = MemoryAgent()
        optimizer = torch.optim.Adam(agent.parameters())
        before = capture_state(agent, optimizer, 0)
        forward = agent.forward

        def invalid_final_value(*args, **kwargs):
            logits, values, h = forward(*args, **kwargs)
            # At time 2 the rollout has finished, so only the bootstrap visits it.
            if (args[0][Observable.OBS_TIME] == 2).any():
                values = values * float("nan")
            return logits, values, h

        cfg.train.kl_probe_every = 0
        with patch.object(agent, "forward", side_effect=invalid_final_value):
            with self.assertRaisesRegex(FloatingPointError, "bootstrap"):
                training_update(cfg, self.env(((2, "limit"),)), agent, optimizer, 0)
        after = capture_state(agent, optimizer, 0)
        for key in ("model", "optimizer", "torch_rng"):
            torch.testing.assert_close(before[key], after[key], rtol=0, atol=0)

    def test_invalid_bootstrap_shapes_and_configuration_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "per episode"):
            get_returns(torch.ones(2, 3), .9, bootstrap_values=torch.ones(1))
        with self.assertRaisesRegex(ValueError, "finite"):
            get_returns(torch.ones(2), .9, bootstrap_values=torch.tensor(float("nan")))
        with self.assertRaisesRegex(ValueError, "valid_mask"):
            get_returns(torch.ones(2), .9, valid_mask=torch.ones(3))
        cfg = OmegaConf.create({"bootstrap_time_limits": "yes"})
        with self.assertRaisesRegex(ValueError, "boolean"):
            rollout(self.env(), MemoryAgent(),
                    bootstrap_time_limits=cfg.bootstrap_time_limits)


if __name__ == "__main__":
    unittest.main()

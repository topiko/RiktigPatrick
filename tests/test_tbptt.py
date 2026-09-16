"""Periodic detachment preserves recurrent memory and truncates gradient history."""

import unittest
from pathlib import Path
from typing import cast
from unittest.mock import patch

import numpy as np
import torch
from gymnasium.vector import SyncVectorEnv
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, Observable
from sim.train_agent import rollout, training_update
from sim.utils import ebufs2batchd, get_advantages, get_returns


class ScriptedEnv:
    """Unequal episodes with nonconstant rewards, including a terminal bonus."""

    def __init__(self, lengths=(1, 3, 7)):
        self.lengths = np.array(lengths)
        self.num_envs = len(lengths)
        self.time = 0

    def observations(self):
        time = np.full((self.num_envs, 1), self.time * 0.1, dtype=np.float32)
        return {
            Observable.OBS_TIME: time,
            Observable.GYRO: np.concatenate((time, time + 0.2, time - 0.3), axis=1),
        }

    def reset(self, seed=None):
        self.time = 0
        return self.observations(), {}

    def step(self, actions):
        self.time += 1
        done = self.time >= self.lengths
        reward = self.time * 0.2 + 7 * (self.time == self.lengths)
        return self.observations(), reward, done, np.zeros_like(done), {}


def scripted_env(lengths=(1, 3, 7)) -> SyncVectorEnv:
    # The test double implements only the vector-env methods used by rollout.
    return cast(SyncVectorEnv, ScriptedEnv(lengths))


def make_agent():
    return Agent(
        [Observable.OBS_TIME.value, Observable.GYRO.value],
        {Actions.ACC_BOTH_WHEELS.value: {"type": "discrete", "bins": [-1., 1.]}},
        hsize=8, n_rnnlayers=2,
    )


def config():
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    cfg.train.tbptt_steps = 2
    return cfg


class TBPTTTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def setUp(self):
        torch.manual_seed(123)

    def test_hidden_values_carry_forward_but_gradients_stop_at_boundaries(self):
        for size in (1, 2, 4, 8):
            with self.subTest(size=size):
                agent = make_agent()
                hidden_states = []
                act = agent.act

                def record(obs, h, **kwargs):
                    if hidden_states:
                        torch.testing.assert_close(h, hidden_states[-1], rtol=0, atol=0)
                    result = act(obs, h, **kwargs)
                    hidden_states.append(result[3])
                    return result

                with patch.object(agent, "act", side_effect=record):
                    buffer, = rollout(scripted_env((7,)), agent, tbptt_steps=size)
                for time in range(1, 7):
                    gradients = torch.autograd.grad(
                        buffer.values_l[time], hidden_states[:time],
                        allow_unused=True, retain_graph=True,
                    )
                    for earlier, grad in enumerate(gradients):
                        if earlier // size == time // size:
                            self.assertIsNotNone(grad)
                            self.assertGreater(grad.abs().sum().item(), 0)
                        else:
                            self.assertIsNone(grad)

    def test_detaching_preserves_actions_returns_and_losses_in_unequal_episodes(self):
        agent = make_agent()
        rng = torch.get_rng_state().clone()
        baseline = None
        full_gradients = None
        for size in (None, 8, 2):
            torch.set_rng_state(rng)
            agent.zero_grad(set_to_none=True)
            buffers = rollout(scripted_env(), agent, tbptt_steps=size)
            logps, rewards, values, lengths, mask = ebufs2batchd(buffers)
            np.testing.assert_array_equal(lengths, [1, 3, 7])
            returns = get_returns(rewards, 0.9)
            self.assertAlmostEqual(returns[0, 0].item(), 7.2, places=5)
            self.assertAlmostEqual(returns[2, 6].item(), 8.4, places=5)
            loss = (
                (-logps * get_advantages(returns, values) + (values - returns).square())
                * mask
            ).sum() / mask.sum()
            actual = [logps, values, returns, loss, *[
                torch.from_numpy(buffer.get_action(Actions.ACC_BOTH_WHEELS))
                for buffer in buffers
            ]]
            if baseline is None:
                baseline = [tensor.detach().clone() for tensor in actual]
            else:
                for tensor, expected in zip(actual, baseline):
                    torch.testing.assert_close(tensor, expected, rtol=0, atol=0)
            loss.backward()
            gradients = []
            for parameter in agent.parameters():
                self.assertIsNotNone(parameter.grad)
                assert parameter.grad is not None
                self.assertTrue(torch.isfinite(parameter.grad).all())
                gradients.append(parameter.grad.flatten())
            gradients = torch.cat(gradients)
            if full_gradients is None:
                full_gradients = gradients
            elif size == 8:
                torch.testing.assert_close(gradients, full_gradients)
            else:
                self.assertFalse(torch.allclose(gradients, full_gradients))

    def test_training_uses_one_rollout_and_one_backward_and_optimizer_step(self):
        agent = make_agent()
        optimizer = torch.optim.Adam(agent.parameters())
        original_backward = torch.Tensor.backward
        backward_calls = []

        def backward(tensor, *args, **kwargs):
            backward_calls.append(tensor.detach())
            return original_backward(tensor, *args, **kwargs)

        with (
            patch("sim.train_agent.rollout", wraps=rollout) as collect,
            patch.object(agent, "act", wraps=agent.act) as act,
            patch.object(torch.Tensor, "backward", new=backward),
            patch.object(optimizer, "step", wraps=optimizer.step) as step,
        ):
            metrics = training_update(config(), scripted_env(), agent, optimizer, 0)
        collect.assert_called_once()
        self.assertEqual(collect.call_args.kwargs["tbptt_steps"], 2)
        self.assertEqual(act.call_count, 7)
        self.assertEqual(len(backward_calls), 1)
        step.assert_called_once()
        self.assertTrue(all(np.isfinite(value) for value in metrics.values()))
        self.assertAlmostEqual(metrics["returns/mean"], (7.2 + 8.2 + 12.6) / 3,
                               places=5)

    def test_invalid_chunk_sizes_fail_before_collecting(self):
        agent = make_agent()
        optimizer = torch.optim.Adam(agent.parameters())
        cfg = config()
        for size in (0, -1, 1.5, True, None):
            cfg.train.tbptt_steps = size
            with patch("sim.train_agent.rollout") as collect:
                with self.assertRaisesRegex(ValueError, "positive integer"):
                    training_update(cfg, scripted_env(), agent, optimizer, 0)
                collect.assert_not_called()


if __name__ == "__main__":
    unittest.main()

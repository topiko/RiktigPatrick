"""Value regression must not change the policy or its clipping scale."""

import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import torch
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, Observable
from sim.checkpoints import capture_state, load_checkpoint, save_checkpoint
from sim.train_agent import make_agent
from sim.utils import clip_agent_gradients


def agent_and_inputs(*, detach_critic=True):
    torch.manual_seed(17)
    agent = Agent(
        [Observable.OBS_TIME.value, Observable.GYRO.value],
        {Actions.ACC_BOTH_WHEELS.value: {"type": "discrete", "bins": [-1, 0, 1]}},
        hsize=8, critic_hidden_sizes=(12, 8), detach_critic=detach_critic,
    )
    observations = {
        Observable.OBS_TIME: torch.randn(4, 1),
        Observable.GYRO: torch.randn(4, 3),
    }
    return agent, observations


class CriticIsolationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_critic_only_step_leaves_policy_unchanged_even_with_adam_momentum(self):
        agent, observations = agent_and_inputs()
        optimizer = torch.optim.Adam(agent.parameters(), lr=0.001)
        sum(p.square().sum() for p in agent.parameters()).backward()
        optimizer.step()
        optimizer.zero_grad(set_to_none=True)
        before = deepcopy(agent.state_dict())
        _, values, _ = agent.forward(observations)
        (values - 10).square().mean().backward()
        for name, parameter in agent.named_parameters():
            if not name.startswith("value_head."):
                self.assertIsNone(parameter.grad, name)
        clip_agent_gradients(agent, 1.0)
        optimizer.step()
        changed = []
        for name, parameter in agent.named_parameters():
            if name.startswith("value_head."):
                changed.append(not torch.equal(parameter, before[name]))
            else:
                torch.testing.assert_close(parameter, before[name], rtol=0, atol=0)
        self.assertTrue(any(changed))

    def test_actor_still_trains_shared_features_and_not_critic(self):
        agent, observations = agent_and_inputs()
        _, logp, _, _ = agent.act(observations)
        (-logp.mean()).backward()
        for prefix in ("encoders.", "rnn.", "layernorm.", "action_heads."):
            norm = sum(
                float(p.grad.abs().sum()) for name, p in agent.named_parameters()
                if name.startswith(prefix) and p.grad is not None
            )
            self.assertGreater(norm, 0, prefix)
        self.assertTrue(all(p.grad is None for p in agent.value_head.parameters()))

    def test_large_critic_gradients_do_not_change_policy_clipping(self):
        agent, _ = agent_and_inputs()
        clipped = []
        for critic_scale in (1.0, 1e8):
            for name, parameter in agent.named_parameters():
                scale = critic_scale if name.startswith("value_head.") else 1.0
                parameter.grad = torch.full_like(parameter, scale)
            metrics = clip_agent_gradients(agent, 1.0)
            policy = torch.cat([
                p.grad.flatten().clone() for name, p in agent.named_parameters()
                if not name.startswith("value_head.") and p.grad is not None
            ])
            self.assertLessEqual(float(policy.norm()), 1.000001)
            self.assertIn("optimization/critic_gradient_norm", metrics)
            clipped.append(policy)
        torch.testing.assert_close(clipped[0], clipped[1], rtol=0, atol=0)

    def test_shared_critic_can_be_enabled_for_controlled_comparison(self):
        agent, observations = agent_and_inputs(detach_critic=False)
        _, values, _ = agent.forward(observations)
        (values - 10).square().mean().backward()
        gradient = agent.rnn.weight_hh_l0.grad
        self.assertIsNotNone(gradient)
        input_gradient = agent.rnn.weight_ih_l0.grad
        assert input_gradient is not None
        self.assertGreater(float(input_gradient.abs().sum()), 0)

    def test_configured_critic_round_trip_and_incompatible_shape_rejection(self):
        cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
        assert isinstance(cfg, DictConfig)
        cfg.train.device = "cpu"
        cfg.policy.critic_hidden_sizes = [12, 8]
        agent = make_agent(cfg)
        self.assertEqual([layer.out_features for layer in agent.value_head
                          if isinstance(layer, torch.nn.Linear)], [12, 8, 1])
        optimizer = torch.optim.Adam(agent.parameters())
        state = capture_state(agent, optimizer, 7)
        with tempfile.TemporaryDirectory() as directory:
            path = save_checkpoint(Path(directory) / "model.pt", state, {})
            self.assertEqual(load_checkpoint(path, agent, optimizer), 7)
            cfg.policy.critic_hidden_sizes = [16]
            other = make_agent(cfg)
            before = deepcopy(other.state_dict())
            with self.assertRaisesRegex(ValueError, "critic_hidden_sizes"):
                load_checkpoint(path, other, torch.optim.Adam(other.parameters()))
            torch.testing.assert_close(other.state_dict(), before, rtol=0, atol=0)

    def test_legacy_config_retains_critic_shape_and_allows_explicit_detachment(self):
        cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
        assert isinstance(cfg, DictConfig)
        cfg.train.device = "cpu"
        del cfg.policy.critic_hidden_sizes
        del cfg.policy.detach_critic
        legacy = make_agent(cfg)
        self.assertFalse(legacy.detach_critic)
        self.assertEqual(legacy.value_head[0].out_features, 16)
        state = capture_state(legacy, torch.optim.Adam(legacy.parameters()), 7)
        cfg.policy.critic_hidden_sizes = [16]
        cfg.policy.detach_critic = True
        current = make_agent(cfg)
        with tempfile.TemporaryDirectory() as directory:
            path = save_checkpoint(Path(directory) / "legacy.pt", state, {})
            load_checkpoint(path, current, torch.optim.Adam(current.parameters()))
        self.assertTrue(current.detach_critic)
        torch.testing.assert_close(current.state_dict(), legacy.state_dict())


if __name__ == "__main__":
    unittest.main()

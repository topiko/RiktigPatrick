"""Bounded Gaussian actions must preserve score-function gradients and recovery."""

import math
import tempfile
import unittest
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import torch
from hydra import compose, initialize_config_dir
from torch.distributions import (
    AffineTransform,
    Normal,
    TanhTransform,
    TransformedDistribution,
)

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import Actions, Observable, StateVarKey
from sim.checkpoints import capture_state, load_checkpoint, save_checkpoint
from sim.curriculum import STAGES, Curriculum
from sim.train_agent import (
    make_agent,
    make_validation_env,
    training_update,
    validate_policy,
)
from sim.utils import register_and_make_env


def action_config(limit=150.0):
    return {"type": "continuous", "limit": limit, "initial_std": 0.1,
            "min_std": 0.02, "max_std": 0.5}


def config(device="cpu"):
    folder = Path(__file__).resolve().parents[1] / "config"
    with initialize_config_dir(config_dir=str(folder), version_base=None):
        cfg = compose(config_name="continuous")
    cfg.train.device = device
    cfg.policy.hsize = 8
    cfg.env.n_parallel = 3
    cfg.env.max_episode_steps = 4
    cfg.train.tbptt_steps = 2
    return cfg


class ContinuousPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def setUp(self):
        torch.manual_seed(123)

    def test_initial_samples_are_local_and_log_density_matches_transformed_normal(self):
        head = ContinuousHead(4, action_config())
        mean = head(torch.randn(8192, 4))
        action, logp = head.sample(mean)
        self.assertFalse(action.requires_grad)
        self.assertTrue((action.abs() <= 150).all())
        self.assertLess(action.abs().max().item(), 75)
        self.assertAlmostEqual(action.std().item(), 15, delta=0.5)
        reference = TransformedDistribution(
            Normal(mean, head.std),
            [TanhTransform(), AffineTransform(loc=0, scale=head.limit)],
        )
        torch.testing.assert_close(
            logp, reference.log_prob(action), atol=2e-5, rtol=1e-5
        )

    def test_mean_and_std_receive_the_correct_score_function_gradient(self):
        head = ContinuousHead(2, action_config()).double()
        mean = torch.tensor([[0.1], [-0.2]], dtype=torch.float64, requires_grad=True)
        latent = torch.tensor([[0.3], [-0.1]], dtype=torch.float64)
        with patch.object(Normal, "sample", return_value=latent):
            _, logp = head.sample(mean)
        logp.sum().backward()
        sigma = head.std.detach()
        expected_mean = (latent - mean.detach()) / sigma.square()
        torch.testing.assert_close(mean.grad, expected_mean)
        fraction = head.std_logit.detach().sigmoid()
        d_log_std = (head.log_std_max - head.log_std_min) * fraction * (1 - fraction)
        expected_std = (
            ((latent - mean.detach()) / sigma).square() - 1
        ).sum() * d_log_std
        torch.testing.assert_close(head.std_logit.grad, expected_std)
        assert mean.grad is not None
        self.assertGreater(mean.grad.abs().sum().item(), 1)

    def test_log_density_stays_finite_when_tanh_rounds_to_its_limits(self):
        head = ContinuousHead(2, action_config())
        mean = torch.zeros(2, 1, requires_grad=True)
        with patch.object(Normal, "sample", return_value=torch.tensor([[-80.], [80.]])):
            action, logp = head.sample(mean)
        torch.testing.assert_close(action, torch.tensor([[-150.], [150.]]))
        self.assertTrue(torch.isfinite(logp).all())
        logp.sum().backward()
        assert mean.grad is not None and head.std_logit.grad is not None
        self.assertTrue(torch.isfinite(mean.grad).all())
        self.assertTrue(torch.isfinite(head.std_logit.grad).all())

    def test_std_bounds_are_smooth_and_bad_settings_are_rejected(self):
        head = ContinuousHead(2, action_config())
        for raw in (-100, -5, 0, 5, 100):
            with torch.no_grad():
                head.std_logit.fill_(raw)
            self.assertGreaterEqual(head.std.item(), 0.02 - 1e-7)
            self.assertLessEqual(head.std.item(), 0.5 + 1e-7)
            if abs(raw) == 5:
                grad, = torch.autograd.grad(head.std, head.std_logit)
                self.assertGreater(grad.item(), 0)
        for key, value in (
            ("limit", 0), ("limit", float("inf")), ("initial_std", float("nan")),
            ("min_std", 0), ("min_std", 0.1), ("max_std", 0.1),
        ):
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                ContinuousHead(2, {**action_config(), key: value})

    def test_mixed_head_log_probabilities_and_deterministic_rng_isolation(self):
        agent = Agent([Observable.OBS_TIME.value], {
            Actions.ACC_BOTH_WHEELS.value: action_config(),
            Actions.VEL_WHEEL_DIFF.value: {"type": "discrete", "bins": [-4., 0., 4.]},
        }, hsize=4)
        obs: dict[StateVarKey, torch.Tensor] = {Observable.OBS_TIME: torch.ones(3, 1)}
        parameters, _, _ = agent.forward(obs)
        actions, logp, values, h = agent.act(obs)
        head = agent.action_heads[Actions.ACC_BOTH_WHEELS.value]
        assert isinstance(head, ContinuousHead)
        reference = TransformedDistribution(
            Normal(parameters[Actions.ACC_BOTH_WHEELS], head.std),
            [TanhTransform(), AffineTransform(loc=0, scale=head.limit)],
        )
        bins = getattr(agent, agent.action_configs[Actions.VEL_WHEEL_DIFF]["bins_name"])
        indices = (actions[Actions.VEL_WHEEL_DIFF] == bins).to(torch.int64).argmax(-1)
        discrete = torch.distributions.Categorical(
            logits=parameters[Actions.VEL_WHEEL_DIFF]
        )
        expected = reference.log_prob(actions[Actions.ACC_BOTH_WHEELS])
        expected += discrete.log_prob(indices).unsqueeze(1)
        torch.testing.assert_close(logp, expected)
        self.assertEqual(tuple(values.shape), (3, 1))
        assert h is not None
        self.assertEqual(tuple(h.shape), (1, 3, 4))
        rng = torch.get_rng_state().clone()
        deterministic, _, _, _ = agent.act(obs, deterministic=True)
        torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)
        torch.testing.assert_close(
            deterministic[Actions.ACC_BOTH_WHEELS], torch.zeros(3, 1)
        )
        torch.testing.assert_close(
            deterministic[Actions.VEL_WHEEL_DIFF][:, 0],
            bins[parameters[Actions.VEL_WHEEL_DIFF].argmax(-1)],
        )
        agent.inactive_actions = tuple(agent.action_configs)
        neutral, neutral_logp, _, _ = agent.act(obs)
        torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)
        for action in agent.action_configs:
            torch.testing.assert_close(neutral[action], torch.zeros(3, 1))
        torch.testing.assert_close(neutral_logp, torch.zeros(3, 1))

    def test_curriculum_activation_resets_mean_std_and_optimizer_moments(self):
        cfg = config()
        agent = make_agent(cfg)
        curriculum = Curriculum(cfg)
        optimizer = torch.optim.Adam(agent.parameters())
        yaw = agent.action_heads[Actions.VEL_WHEEL_DIFF.value]
        assert isinstance(yaw, ContinuousHead)
        with torch.no_grad():
            yaw.weight.fill_(1)
            yaw.bias.fill_(2)
            yaw.std_logit.fill_(4)
        optimizer.state[yaw.std_logit] = {
            "step": torch.tensor(1.), "exp_avg": torch.ones_like(yaw.std_logit),
            "exp_avg_sq": torch.ones_like(yaw.std_logit),
        }
        wheel = agent.action_heads[Actions.ACC_BOTH_WHEELS.value]
        wheel_before = {k: v.clone() for k, v in wheel.state_dict().items()}
        curriculum.advance(agent, optimizer)  # Balance -> stop enables no new heads.
        curriculum.advance(agent, optimizer)
        self.assertAlmostEqual(yaw.std.item(), 0.1, places=6)
        torch.testing.assert_close(yaw.weight, torch.zeros_like(yaw.weight))
        torch.testing.assert_close(yaw.bias, torch.zeros_like(yaw.bias))
        self.assertNotIn(yaw.std_logit, optimizer.state)
        torch.testing.assert_close(wheel.state_dict(), wheel_before)
        self.assertNotIn(Actions.VEL_WHEEL_DIFF, agent.inactive_actions)

    def test_mlflow_restore_rejects_changed_action_type_or_limits(self):
        cfg = config()
        agent = make_agent(cfg)
        for key, value in (("type", "discrete"), ("limit", 300.0)):
            expected = deepcopy(cfg)
            expected.policy.restore_id = "test-model"
            expected.policy.actions[Actions.ACC_BOTH_WHEELS.value][key] = value
            with (
                patch("sim.train_agent.mlflow.get_logged_model",
                      return_value=SimpleNamespace(model_uri="unused")),
                patch("sim.train_agent.mlflow.pytorch",
                      SimpleNamespace(load_model=Mock(return_value=agent))),
            ):
                with self.assertRaisesRegex(ValueError, "do not match"):
                    make_agent(expected)

    def exercise_training_and_checkpoint(self, device):
        cfg = config(device)
        agent = make_agent(cfg)
        curriculum = Curriculum(cfg)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        env = register_and_make_env(cfg)
        self.addCleanup(env.close)
        for stage in STAGES:
            self.assertEqual(curriculum.stage, stage)
            metrics = training_update(cfg, env, agent, optimizer, 0, curriculum)
            self.assertTrue(all(math.isfinite(value) for value in metrics.values()))
            for action, head in agent.action_heads.items():
                assert isinstance(head, ContinuousHead)
                self.assertIn(f"policy/std/{action}", metrics)
                if Actions.from_str(action) in curriculum.inactive_actions:
                    self.assertTrue(all(p.grad is None for p in head.parameters()))
                else:
                    self.assertIsNotNone(head.std_logit.grad)
                    assert head.std_logit.grad is not None
                    self.assertTrue(torch.isfinite(head.std_logit.grad).all())
                    self.assertGreater(head.std_logit.grad.abs().item(), 0)
            if stage != "full_control":
                curriculum.advance(agent, optimizer)
        before = capture_state(agent, optimizer, 12, curriculum)
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "latest.pt", before, {})
            with torch.no_grad():
                for head in agent.action_heads.values():
                    assert isinstance(head, ContinuousHead)
                    head.std_logit.add_(1)
            self.assertEqual(load_checkpoint(path, agent, optimizer, curriculum), 12)
        for key, tensor in agent.state_dict().items():
            torch.testing.assert_close(
                tensor.cpu(), before["model"][key], rtol=0, atol=0
            )
        self.assertEqual(agent.inactive_actions, ())

    def test_cpu_training_all_stages_and_checkpoint_round_trip(self):
        self.exercise_training_and_checkpoint("cpu")

    @unittest.skipUnless(torch.cuda.is_available(), "CUDA device unavailable")
    def test_cuda_training_all_stages_and_checkpoint_round_trip(self):
        self.exercise_training_and_checkpoint("cuda")

    def test_paired_validation_is_repeatable_and_gates_use_stochastic_metrics(self):
        cfg = config()
        agent = make_agent(cfg)
        curriculum = Curriculum(cfg)
        optimizer = torch.optim.Adam(agent.parameters())
        curriculum.apply_policy(agent)
        before = capture_state(agent, optimizer, 0, curriculum)
        with ExitStack() as resources:
            env = make_validation_env(cfg, resources, curriculum)
            first = validate_policy(cfg, env, agent, curriculum)
            second = validate_policy(cfg, env, agent, curriculum)
        self.assertEqual(first, second)
        self.assertIn("validation_deterministic/returns/mean", first)
        self.assertIn("validation_deterministic/velocity_mae", first)
        after = capture_state(agent, optimizer, 0, curriculum)
        torch.testing.assert_close(
            after["torch_rng"], before["torch_rng"], rtol=0, atol=0
        )
        self.assertEqual(after["numpy_rng"], before["numpy_rng"])
        self.assertEqual(after["curriculum"], before["curriculum"])
        curriculum.success_streak = cfg.curriculum.consecutive_passes - 1
        first.update({"validation/survival_fraction": 0,
                      "validation_deterministic/survival_fraction": 1})
        self.assertFalse(curriculum.observe(first))
        self.assertEqual(curriculum.success_streak, 0)


if __name__ == "__main__":
    unittest.main()

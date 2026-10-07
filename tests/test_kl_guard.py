"""Full-history KL and backtracking must not replay optimizer momentum or physics."""

import random
import unittest
from copy import deepcopy
from unittest.mock import patch

import numpy as np
import torch
from omegaconf import OmegaConf
from test_policy_probe import agent as mixed_agent
from test_tbptt import make_agent, scripted_env
from test_training_safety import config

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import Actions, Observable, StateVarKey
from sim.checkpoints import capture_state, restore_state
from sim.kl_guard import KLGuardSettings, guarded_optimizer_step
from sim.optimizers import make_optimizer
from sim.policy_probe import PolicyProbe, RecurrentPolicyReplay
from sim.train_agent import rollout, training_rollout, training_update
from sim.utils import EpisodeBuffer


def buffers(lengths=(1, 3, 5)):
    episodes = []
    for length in lengths:
        episode = EpisodeBuffer()
        for step in range(length):
            obs: dict[StateVarKey, np.ndarray] = {
                Observable.OBS_TIME: np.array([step + 1.0], dtype=np.float32)
            }
            episode.add_step(obs, {Actions.TIME: obs[Observable.OBS_TIME]},
                             1.0, torch.tensor(0.0), torch.tensor(0.0))
        episode.finish({Observable.OBS_TIME: np.array([1e6], dtype=np.float32)})
        episodes.append(episode)
    return episodes


class HistoryAgent(Agent):
    """A parameter changes future hidden states but not logits at a cached state."""

    def __init__(self):
        super().__init__([Observable.OBS_TIME.value], {
            Actions.ACC_BOTH_WHEELS.value: {"type": "discrete", "bins": [-1, 1]},
        }, hsize=1)
        self.memory_gain = torch.nn.Parameter(torch.tensor(0.1))

    def forward(self, x, h=None):
        previous = torch.zeros_like(x[Observable.OBS_TIME]) if h is None else h[0]
        hidden = (previous + self.memory_gain * x[Observable.OBS_TIME]).unsqueeze(0)
        logits = torch.cat((previous, -previous), dim=-1)
        return {Actions.ACC_BOTH_WHEELS: logits}, previous, hidden


class KLGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def assert_optimizer_equal(self, actual, expected):
        self.assertEqual(actual["param_groups"], expected["param_groups"])
        torch.testing.assert_close(actual["state"], expected["state"], rtol=0, atol=0)

    def test_replay_detects_hidden_history_change_missed_by_conditional_probe(self):
        agent = HistoryAgent()
        episodes = buffers((4,))
        probe = PolicyProbe(agent, 1)
        hidden = None
        for obs in episodes[0].obs_l[:-1]:
            inputs = {key: torch.tensor(value[None]) for key, value in obs.items()}
            probe.capture(agent, inputs, hidden, np.array([True]))
            _, _, hidden = agent.forward(inputs, hidden)
        replay = RecurrentPolicyReplay(agent, episodes)
        self.assertEqual(replay.metrics(agent)["mean"], 0)
        with torch.no_grad():
            agent.memory_gain.fill_(0.6)
        self.assertEqual(probe.metrics(agent)["optimization/policy_kl_mean"], 0)
        self.assertGreater(replay.metrics(agent)["mean"], 0.1)
        self.assertTrue(agent.training)
        self.assertTrue(all(p.grad is None for p in agent.parameters()))

    def test_replayed_old_distributions_match_rollout_across_tbptt_boundaries(self):
        agent = make_agent()
        recorded = []
        forward = agent.forward

        def capture(observations, hidden=None):
            result = forward(observations, hidden)
            recorded.append(result[0][Actions.ACC_BOTH_WHEELS].detach().clone())
            return result

        with patch.object(agent, "forward", side_effect=capture):
            episodes = rollout(scripted_env(), agent, tbptt_steps=2)
        replay = RecurrentPolicyReplay(agent, episodes)
        torch.testing.assert_close(replay.previous[Actions.ACC_BOTH_WHEELS],
                                   torch.stack(recorded, dim=1)[replay.valid])

    def test_replay_masks_padding_and_final_observations_and_owns_inputs(self):
        agent = HistoryAgent()
        episodes = buffers()
        replay = RecurrentPolicyReplay(agent, episodes)
        with torch.no_grad():
            agent.memory_gain.fill_(0.4)
        expected = []
        for length in (1, 3, 5):
            for step in range(length):
                memory = step * (step + 1) / 2
                old = torch.distributions.Categorical(
                    logits=torch.tensor([0.1 * memory, -0.1 * memory])
                )
                new = torch.distributions.Categorical(
                    logits=torch.tensor([0.4 * memory, -0.4 * memory])
                )
                expected.append(torch.distributions.kl_divergence(old, new))
        for episode in episodes:
            for obs in episode.obs_l:
                obs[Observable.OBS_TIME].fill(999)
        actual = replay.metrics(agent)
        self.assertEqual(actual["samples"], 9)
        self.assertEqual(len(replay.episode_metrics), 3)
        self.assertEqual([row["steps"] for row in replay.episode_metrics], [1, 3, 5])
        self.assertEqual(actual["worst_episode"], 2)
        self.assertEqual(replay.episode_metrics[0]["mean"], 0)
        self.assertAlmostEqual(actual["episode_mean_max"],
                               replay.episode_metrics[2]["mean"])
        self.assertAlmostEqual(
            actual["mean"], float(torch.stack(expected).mean()), places=6
        )
        self.assertAlmostEqual(
            actual["max"], float(torch.stack(expected).max()), places=6
        )
        self.assertTrue(all(value.grad_fn is None
                            for value in replay.previous.values()))

    def test_gaussian_replay_ignores_inactive_heads_and_preserves_rng(self):
        agent = mixed_agent()
        agent.inactive_actions = (Actions.VEL_WHEEL_DIFF,)
        agent.eval()
        torch_rng = torch.get_rng_state().clone()
        numpy_rng = np.random.get_state()
        python_rng = random.getstate()
        replay = RecurrentPolicyReplay(agent, buffers())
        head = agent.action_heads[Actions.ACC_BOTH_WHEELS.value]
        assert isinstance(head, ContinuousHead) and head.bias is not None
        with torch.no_grad():
            head.bias.add_(0.1)
            for parameter in agent.action_heads[Actions.VEL_WHEEL_DIFF].parameters():
                parameter.add_(10)
            for parameter in agent.value_head.parameters():
                parameter.add_(10)
        metrics = replay.metrics(agent)
        self.assertAlmostEqual(metrics["mean"], 0.5, places=5)
        self.assertAlmostEqual(metrics["max"], 0.5, places=5)
        torch.testing.assert_close(torch.get_rng_state(), torch_rng, rtol=0, atol=0)
        np.testing.assert_equal(np.random.get_state(), numpy_rng)
        self.assertEqual(random.getstate(), python_rng)
        self.assertFalse(agent.training)

    def test_real_gaussian_update_backtracks_to_kl_budget(self):
        agent = mixed_agent()
        agent.inactive_actions = (Actions.VEL_WHEEL_DIFF,)
        optimizer = make_optimizer(agent, 0.03, 0.002)
        before = capture_state(agent, optimizer, 0)
        head = agent.action_heads[Actions.ACC_BOTH_WHEELS]
        assert isinstance(head, ContinuousHead) and head.bias is not None
        head.bias.grad = -torch.ones_like(head.bias)
        metrics = guarded_optimizer_step(
            agent, optimizer, before, buffers(), KLGuardSettings(enabled=True),
        )
        self.assertEqual(metrics["kl_guard/attempts"], 2)
        self.assertEqual(metrics["kl_guard/accepted"], 1)
        self.assertAlmostEqual(metrics["kl_guard/kl_mean"], 0.01125, places=5)
        self.assertAlmostEqual(float(head.bias.detach()), 0.015, places=6)
        self.assertEqual([g["lr"] for g in optimizer.param_groups], [0.015, 0.002])
        self.assertEqual(optimizer.state[head.bias]["step"], 1)
        self.assertAlmostEqual(metrics["kl_guard/initial_kl_mean"], 0.045, places=5)

    def test_mixed_heads_include_std_and_categorical_distribution_changes(self):
        agent = mixed_agent()
        wheel = agent.action_heads[Actions.ACC_BOTH_WHEELS]
        yaw = agent.action_heads[Actions.VEL_WHEEL_DIFF]
        assert isinstance(wheel, ContinuousHead)
        assert isinstance(yaw, torch.nn.Linear) and yaw.bias is not None
        with torch.no_grad():
            yaw.weight.zero_()
            yaw.bias.zero_()
        replay = RecurrentPolicyReplay(agent, buffers())
        old_std = wheel.std.detach().clone()
        with torch.no_grad():
            wheel.std_logit.add_(1)
            yaw.bias[1] = np.log(2)
        expected = torch.distributions.kl_divergence(
            torch.distributions.Normal(0, old_std),
            torch.distributions.Normal(0, wheel.std),
        ) + torch.distributions.kl_divergence(
            torch.distributions.Categorical(probs=torch.tensor([1/3, 1/3, 1/3])),
            torch.distributions.Categorical(logits=yaw.bias),
        )
        self.assertAlmostEqual(replay.metrics(agent)["mean"],
                               float(expected.detach()), places=6)

    def test_inactive_policy_has_zero_kl(self):
        agent = mixed_agent()
        agent.inactive_actions = tuple(agent.action_configs)
        replay = RecurrentPolicyReplay(agent, buffers())
        with torch.no_grad():
            for parameter in agent.parameters():
                parameter.add_(10)
        metrics = replay.metrics(agent)
        self.assertEqual(metrics["mean"], 0)
        self.assertEqual(metrics["max"], 0)
        self.assertEqual(metrics["samples"], 9)
        self.assertEqual(metrics["episode_mean_max"], 0)

    def test_retry_matches_one_smaller_step_and_updates_critic_only_once(self):
        agent = mixed_agent()
        optimizer = make_optimizer(agent, 0.01, 0.002)
        sum(p.square().sum() for p in agent.parameters()).backward()
        optimizer.step()
        optimizer.zero_grad(set_to_none=True)
        before = capture_state(agent, optimizer, 12)
        expected_agent = deepcopy(agent)
        expected_optimizer = make_optimizer(expected_agent, 0.01, 0.002)
        restore_state(expected_agent, expected_optimizer, before, restore_rng=False)
        expected_optimizer.param_groups[0]["lr"] = 0.005
        for model in (agent, expected_agent):
            for parameter in model.parameters():
                parameter.grad = torch.full_like(parameter, 0.3)
        expected_optimizer.step()
        torch.rand(3)
        after_rollout_rng = torch.get_rng_state().clone()
        with patch("sim.kl_guard.RecurrentPolicyReplay") as replay:
            replay.return_value.metrics.side_effect = [
                {"mean": 0.1, "max": 0.2, "samples": 9},
                {"mean": 0.01, "max": 0.04, "samples": 9},
            ]
            metrics = guarded_optimizer_step(
                agent, optimizer, before, buffers(), KLGuardSettings(enabled=True),
            )
            self.assertEqual(replay.call_count, 1)
        self.assertEqual(metrics["kl_guard/backtracks"], 1)
        torch.testing.assert_close(agent.state_dict(), expected_agent.state_dict(),
                                   rtol=0, atol=0)
        self.assert_optimizer_equal(
            optimizer.state_dict(), expected_optimizer.state_dict()
        )
        torch.testing.assert_close(
            torch.get_rng_state(), after_rollout_rng, rtol=0, atol=0
        )

    def test_exhaustion_restores_weights_and_moments_but_keeps_lower_policy_lr(self):
        agent = mixed_agent()
        optimizer = make_optimizer(agent, 0.01, 0.002)
        sum(p.square().sum() for p in agent.parameters()).backward()
        optimizer.step()
        before = capture_state(agent, optimizer, 9)
        for parameter in agent.parameters():
            parameter.grad = torch.ones_like(parameter)
        with (
            patch("sim.kl_guard.RecurrentPolicyReplay") as replay,
            self.assertLogs("sim.kl_guard", level="WARNING"),
        ):
            replay.return_value.metrics.return_value = {
                "mean": 1., "max": 2., "samples": 9
            }
            metrics = guarded_optimizer_step(
                agent, optimizer, before, buffers(),
                KLGuardSettings(enabled=True, max_backtracks=2),
            )
        expected = deepcopy(before["optimizer"])
        expected["param_groups"][0]["lr"] = 0.0025
        torch.testing.assert_close(agent.state_dict(), before["model"], rtol=0, atol=0)
        self.assert_optimizer_equal(optimizer.state_dict(), expected)
        self.assertEqual(metrics["kl_guard/attempts"], 3)
        self.assertEqual(metrics["kl_guard/skipped_update"], 1)
        self.assertEqual(metrics["kl_guard/kl_mean"], 0)
        self.assertEqual(metrics["kl_guard/last_candidate_kl_mean"], 1)

    def test_lr_floor_stops_retries_and_never_increases_existing_rate(self):
        for rate, expected, attempts in ((0.01, 0.003, 3), (0.001, 0.001, 1)):
            agent = mixed_agent()
            optimizer = make_optimizer(agent, rate, 0.002)
            before = capture_state(agent, optimizer, 0)
            for parameter in agent.parameters():
                parameter.grad = torch.ones_like(parameter)
            with (
                patch("sim.kl_guard.RecurrentPolicyReplay") as replay,
                self.assertLogs("sim.kl_guard", level="WARNING"),
            ):
                replay.return_value.metrics.return_value = {
                    "mean": 1., "max": 2., "samples": 9
                }
                metrics = guarded_optimizer_step(
                    agent, optimizer, before, buffers(),
                    KLGuardSettings(enabled=True, min_lr=0.003),
                )
            self.assertEqual(metrics["kl_guard/attempts"], attempts)
            self.assertEqual(optimizer.param_groups[0]["lr"], expected)

    def test_training_collects_and_backpropagates_once_despite_retries(self):
        agent = make_agent()
        optimizer = make_optimizer(agent, 0.01, 0.002)
        cfg = config()
        cfg.kl_guard = OmegaConf.structured(KLGuardSettings(enabled=True))
        backward = torch.Tensor.backward
        backward_calls = []

        def counted_backward(tensor, *args, **kwargs):
            backward_calls.append(1)
            return backward(tensor, *args, **kwargs)

        with (
            patch(
                "sim.train_agent.training_rollout", wraps=training_rollout
            ) as collect,
            patch.object(torch.Tensor, "backward", new=counted_backward),
            patch.object(optimizer, "step", wraps=optimizer.step) as step,
            patch.object(RecurrentPolicyReplay, "metrics", side_effect=[
                {"mean": 0.1, "max": 0.2, "samples": 11},
                {"mean": 0.01, "max": 0.03, "samples": 11},
            ]),
        ):
            metrics = training_update(cfg, scripted_env(), agent, optimizer, 0)
        collect.assert_called_once()
        self.assertEqual(len(backward_calls), 1)
        self.assertEqual(step.call_count, 2)
        self.assertEqual(metrics["kl_guard/accepted"], 1)

    def test_invalid_settings_are_rejected(self):
        for kwargs in (
            {"enabled": 1}, {"max_backtracks": -1}, {"max_backtracks": True},
            {"max_kl": 0}, {"max_kl": float("nan")}, {"min_lr": float("inf")},
            {"min_lr": 0}, {"lr_factor": 1}, {"lr_factor": 0},
            {"diagnostics": 1}, {"loo_top_k": -1}, {"loo_top_k": True},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                KLGuardSettings(**kwargs)  # ty: ignore[invalid-argument-type]

    def test_nonfinite_replay_restores_the_training_transaction(self):
        agent = make_agent()
        optimizer = make_optimizer(agent, 0.01, 0.002)
        sum(p.square().sum() for p in agent.parameters()).backward()
        optimizer.step()
        cfg = config()
        cfg.kl_guard = OmegaConf.structured(KLGuardSettings(enabled=True))
        before = capture_state(agent, optimizer, 0)
        with (
            patch.object(RecurrentPolicyReplay, "metrics", return_value={
                "mean": float("nan"), "max": float("nan"), "samples": 11,
            }),
            self.assertRaisesRegex(FloatingPointError, "replayed policy KL"),
        ):
            training_update(cfg, scripted_env(), agent, optimizer, 0)
        torch.testing.assert_close(agent.state_dict(), before["model"], rtol=0, atol=0)
        self.assert_optimizer_equal(optimizer.state_dict(), before["optimizer"])
        torch.testing.assert_close(torch.get_rng_state(), before["torch_rng"])

    @unittest.skipUnless(torch.cuda.is_available(), "CUDA unavailable")
    def test_replay_works_on_cuda(self):
        agent = mixed_agent().cuda()
        replay = RecurrentPolicyReplay(agent, buffers())
        self.assertEqual(replay.metrics(agent)["mean"], 0)
        self.assertTrue(all(value.device.type == "cuda"
                            for value in replay.observations.values()))


if __name__ == "__main__":
    unittest.main()

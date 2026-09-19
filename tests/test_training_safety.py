"""Training recovery must preserve weights, Adam state, RNG, and episode alignment."""

import json
import random
import tempfile
import unittest
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch
from gymnasium import Env
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, Observable
from sim.checkpoints import (
    PolicyGuard,
    capture_state,
    load_checkpoint,
    restore_state,
    save_checkpoint,
)
from sim.envs.rp_env import GymRP
from sim.train_agent import (
    get_policy_inputs,
    make_validation_env,
    run_training_loop,
    training_update,
    validate_policy,
    write_checkpoint,
)
from sim.utils import SingleEnvWrapper


class OneStepEnv(Env):
    def __init__(self, reward=1.0):
        self.reward = reward

    def reset(self, seed=None, options=None):
        return {Observable.OBS_TIME: np.array([0.0])}, {}

    def step(self, action):
        return {Observable.OBS_TIME: np.array([1.0])}, self.reward, True, False, {}


def agent_and_optimizer():
    agent = Agent([Observable.OBS_TIME.value], {
        Actions.ACC_BOTH_WHEELS.value: {"type": "discrete", "bins": [-1.0, 0.0, 1.0]}
    }, hsize=4)
    optimizer = torch.optim.Adam(agent.parameters(), lr=0.01)
    # Initialize Adam's moving averages so restoring only weights cannot pass.
    torch.stack([p.square().sum() for p in agent.parameters()]).sum().backward()
    optimizer.step()
    optimizer.zero_grad(set_to_none=True)
    return agent, optimizer


def config(folder="unused"):
    return OmegaConf.create({
        "seed": 42, "rl": {"discount": 0.99, "value_loss_coef": 0.1},
        "env": {"step_time": 0.01},
        "train": {
            "grad_clip": 1.0, "max_iterations": 1, "tbptt_steps": 32,
            "kl_probe_every": 32,
            "target_positions": None, "target_velocities": None,
            "target_yaw_rates": None,
            "target_trajectories": None,
            "head_targets": None, "head_trajectories": None,
        },
        "logging": {"mlflow": {"enabled": False}, "plot_freq": 0},
        "checkpoints": {"dir": folder, "every": 100},
        "guard": {"seed": 1042, "every": 25},
    })


class TrainingSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def assert_nested_equal(self, actual, expected):
        if isinstance(expected, torch.Tensor):
            torch.testing.assert_close(actual, expected, rtol=0, atol=0)
        elif isinstance(expected, dict):
            self.assertEqual(actual.keys(), expected.keys())
            for key in expected:
                self.assert_nested_equal(actual[key], expected[key])
        elif isinstance(expected, (list, tuple)):
            self.assertEqual(len(actual), len(expected))
            for a, e in zip(actual, expected):
                self.assert_nested_equal(a, e)
        else:
            self.assertEqual(actual, expected)

    def test_checkpoint_round_trip_restores_optimizer_progress_and_rng(self):
        agent, optimizer = agent_and_optimizer()
        state = capture_state(agent, optimizer, next_iteration=123)
        expected_torch = torch.rand(4)
        expected_numpy = np.random.rand(4)
        expected_python = random.random()
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "latest.pt", state, {"seed": 42})
            # Verify the file contains data, not a pickled Python agent instance.
            self.assertEqual(torch.load(path, weights_only=True)["format_version"], 1)
            with torch.no_grad():
                for parameter in agent.parameters():
                    parameter.add_(100)
            optimizer.param_groups[0]["lr"] = 0.2
            self.assertEqual(load_checkpoint(path, agent, optimizer), 123)
        self.assert_nested_equal(agent.state_dict(), state["model"])
        self.assert_nested_equal(optimizer.state_dict(), state["optimizer"])
        torch.testing.assert_close(torch.rand(4), expected_torch, rtol=0, atol=0)
        np.testing.assert_array_equal(np.random.rand(4), expected_numpy)
        self.assertEqual(random.random(), expected_python)

    def test_failed_write_preserves_previous_checkpoint(self):
        agent, optimizer = agent_and_optimizer()
        state = capture_state(agent, optimizer, 10)
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "best.pt", state, {})

            def failed_save(_payload, stream):
                stream.write(b"incomplete checkpoint")
                raise OSError("simulated full disk")

            with patch("sim.checkpoints.torch.save", side_effect=failed_save):
                with self.assertRaises(OSError):
                    save_checkpoint(path, state, {})
            self.assertEqual(load_checkpoint(path, agent, optimizer), 10)
            self.assertFalse(path.with_suffix(".pt.tmp").exists())

    def test_checkpoint_rejects_different_policy_before_loading(self):
        agent, optimizer = agent_and_optimizer()
        state = capture_state(agent, optimizer, 0)
        state["policy"]["inputs"] = [Observable.RP_PITCH.value]
        with self.assertRaises(ValueError):
            restore_state(agent, optimizer, state)

    def test_guard_needs_sustained_drop_and_restores_adam_with_lower_lr(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, recovery_attempts=0)
        self.assertEqual(guard.observe(100, 10), "best")
        assert guard.best_state is not None
        best = deepcopy(guard.best_state)
        with torch.no_grad():
            next(agent.parameters()).add_(1)
        optimizer.state[next(agent.parameters())]["exp_avg"].add_(2)
        self.assertEqual(guard.observe(20, 11), "keep")
        self.assertEqual(guard.observe(80, 12), "keep")  # breaks the bad streak
        self.assertEqual(guard.observe(20, 13), "keep")
        rng = torch.get_rng_state().clone()
        self.assertEqual(guard.observe(19, 14), "rollback")
        self.assert_nested_equal(agent.state_dict(), best["model"])
        expected_optimizer = deepcopy(best["optimizer"])
        expected_optimizer["param_groups"][0]["lr"] = 0.005
        self.assert_nested_equal(optimizer.state_dict(), expected_optimizer)
        torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)
        self.assertEqual(guard.best_score, 100)
        self.assertEqual(guard.rollbacks, 1)
        self.assertEqual(guard.observe(0, 15), "keep")
        self.assertEqual(guard.observe(0, 16), "rollback")
        self.assertEqual(optimizer.param_groups[0]["lr"], 0.0025)

    def test_recovery_rejects_material_drops_without_further_lr_decay(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, patience=1, recovery_attempts=2)
        guard.observe(100, 0)
        guard.observe(0, 1)
        self.assertTrue(guard.recovering)
        best = capture_state(agent, optimizer, 1)
        for score, decision in ((49, "recovery_rejected"), (0, "recovery_stop")):
            with torch.no_grad():
                next(agent.parameters()).add_(1)
            optimizer.state[next(agent.parameters())]["exp_avg"].add_(5)
            torch.rand(3)
            rng = torch.get_rng_state().clone()
            self.assertEqual(guard.observe(score, 2), decision)
            self.assert_nested_equal(agent.state_dict(), best["model"])
            self.assert_nested_equal(optimizer.state_dict(), best["optimizer"])
            torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)
        self.assertTrue(guard.stop_requested)
        self.assertEqual(optimizer.param_groups[0]["lr"], 0.005)

    def test_recovery_accepts_threshold_without_replacing_best_or_rewinding_state(self):
        # Exercise both the relative-drop and absolute-drop branches of the floor.
        for best_score, absolute_drop, threshold in ((1000, 100, 700), (100, 50, 50)):
            with self.subTest(best=best_score):
                agent, optimizer = agent_and_optimizer()
                guard = PolicyGuard(agent, optimizer, patience=1,
                                    drop_fraction=0.3, absolute_drop=absolute_drop)
                self.assertIsNone(guard.rejection_threshold)
                guard.observe(best_score, 0)
                self.assertEqual(guard.rejection_threshold, threshold)
                guard.observe(threshold - 1, 1)
                restored = capture_state(agent, optimizer, 1)
                self.assertEqual(guard.observe(threshold - 1, 2), "recovery_rejected")
                self.assertEqual(guard.recovery_failures, 1)
                for score in (best_score - 1, (best_score + threshold) / 2,
                              threshold, best_score):
                    with torch.no_grad():
                        next(agent.parameters()).add_(1)
                    optimizer.state[next(agent.parameters())]["exp_avg"].add_(5)
                    torch.rand(3)
                    candidate = capture_state(agent, optimizer, 3)
                    self.assertEqual(guard.observe(score, 3), "recovery_accepted")
                    self.assert_nested_equal(capture_state(agent, optimizer, 3),
                                             candidate)
                    self.assertEqual(guard.recovery_failures, 0)
                    self.assertEqual(guard.best_score, best_score)
                    self.assertEqual(guard.rejection_threshold, threshold)
                    self.assertTrue(guard.should_evaluate(3, 10))
                # Several accepted decreases cannot shift the floor downward.
                self.assertEqual(guard.observe(threshold - 1, 4), "recovery_rejected")
                self.assert_nested_equal(agent.state_dict(), restored["model"])
                self.assert_nested_equal(optimizer.state_dict(), restored["optimizer"])

    def test_recovery_accepts_improvements_and_new_stage_resets_monitoring(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, patience=1, recovery_attempts=2)
        guard.observe(100, 0)
        guard.observe(0, 1)
        self.assertEqual(guard.observe(49, 2), "recovery_rejected")
        with torch.no_grad():
            next(agent.parameters()).add_(1)
        improved = capture_state(agent, optimizer, 3)
        self.assertEqual(guard.observe(101, 3), "best")
        self.assertTrue(guard.recovering)  # A pass must not restart unchecked updates.
        self.assertEqual(guard.recovery_failures, 0)
        self.assertEqual(guard.rejection_threshold, 50.5)
        self.assertEqual(guard.observe(50, 4), "recovery_rejected")
        self.assert_nested_equal(agent.state_dict(), improved["model"])
        guard.reset_baseline()
        self.assertFalse(guard.recovering)
        self.assertFalse(guard.stop_requested)
        self.assertEqual(guard.recovery_failures, 0)
        self.assertEqual(guard.observe(5, 4), "best")

    def test_recovery_checks_each_update_and_saves_resumable_clean_stop(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, patience=1)
        before = capture_state(agent, optimizer, 0)
        env = SingleEnvWrapper(OneStepEnv())
        with tempfile.TemporaryDirectory() as folder:
            cfg = config(folder)
            cfg.train.max_iterations = 100
            cfg.guard.every = 4
            scores = [100, 0, 49, 48, 47]
            with (
                patch("sim.train_agent.validate_policy", side_effect=[
                    {"validation/returns/mean": score} for score in scores
                ]) as validate,
                patch("sim.train_agent.training_update",
                      wraps=training_update) as update,
                self.assertLogs("sim.train_agent", level="WARNING"),
            ):
                run_training_loop(cfg, env, None, object(), agent, optimizer, guard, 0)
            self.assertEqual(validate.call_count, 5)
            self.assertEqual([c.args[4] for c in update.call_args_list], list(range(7)))
            latest = torch.load(Path(folder) / "latest.pt", weights_only=True)["state"]
            self.assertEqual(latest["next_iteration"], 7)
            self.assert_nested_equal(latest["model"], before["model"])
            expected = deepcopy(before["optimizer"])
            expected["param_groups"][0]["lr"] = 0.005
            self.assert_nested_equal(latest["optimizer"], expected)
            report = json.loads((Path(folder) / "training_stop.json").read_text())
            self.assertEqual(report["reason"], "recovery_exhausted")
            self.assertEqual(report["best_return"], 100)
            self.assertEqual(report["rejected_return"], 47)
            self.assertEqual(report["rejection_threshold"], 50)
            self.assertEqual(report["recovery_failures"], 3)
            self.assertFalse((Path(folder) / "training_failure.txt").exists())
            self.assertEqual(load_checkpoint(Path(folder) / "latest.pt", agent,
                                             optimizer), 7)

    def test_recovery_keeps_near_best_updates_and_saves_their_progress(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, patience=1,
                            drop_fraction=0.3, absolute_drop=100)
        before = capture_state(agent, optimizer, 0)
        env = SingleEnvWrapper(OneStepEnv())
        with tempfile.TemporaryDirectory() as folder:
            cfg = config(folder)
            cfg.train.max_iterations = 7
            cfg.guard.every = 4
            with (
                patch("sim.train_agent.validate_policy", side_effect=[
                    {"validation/returns/mean": score}
                    for score in (1185.33, 105.08, 1171.17, 1177.07, 1170.11)
                ]),
                self.assertLogs("sim.train_agent", level="INFO"),
            ):
                run_training_loop(cfg, env, None, object(), agent, optimizer, guard, 0)
            self.assertEqual(guard.rollbacks, 1)
            self.assertEqual(guard.recovery_failures, 0)
            self.assertFalse(guard.stop_requested)
            self.assertEqual(guard.best_score, 1185.33)
            latest = torch.load(Path(folder) / "latest.pt", weights_only=True)["state"]
            best = torch.load(Path(folder) / "best.pt", weights_only=True)["state"]
            self.assertEqual(latest["next_iteration"], 7)
            self.assertEqual(best["next_iteration"], 0)
            self.assert_nested_equal(best["model"], before["model"])
            self.assertTrue(any(
                not torch.equal(value, before["model"][key])
                for key, value in latest["model"].items()
            ))
            self.assert_nested_equal(latest["model"], agent.state_dict())
            self.assert_nested_equal(latest["optimizer"], optimizer.state_dict())
            self.assertFalse((Path(folder) / "training_stop.json").exists())

    def test_invalid_recovery_attempt_counts_are_rejected(self):
        agent, optimizer = agent_and_optimizer()
        for attempts in (-1, 1.5, True):
            with self.assertRaisesRegex(ValueError, "recovery_attempts"):
                PolicyGuard(
                    agent, optimizer,
                    recovery_attempts=attempts,  # ty: ignore[invalid-argument-type]
                )

    def test_guard_does_not_lock_initial_learning_and_keeps_improvements(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer)
        self.assertEqual(guard.observe(-10, 0), "best")
        self.assertEqual(guard.observe(-100, 1), "keep")
        self.assertEqual(guard.observe(-100, 2), "keep")
        self.assertEqual(guard.observe(120, 3), "best")
        assert guard.best_state is not None
        self.assertEqual(guard.best_state["next_iteration"], 3)
        with self.assertRaises(FloatingPointError):
            guard.observe(float("nan"), 4)

    def test_disabled_rollback_still_tracks_best_without_restoring_bad_updates(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, patience=1)
        self.assertEqual(guard.observe(100, 0, allow_rollback=False), "best")
        with torch.no_grad():
            next(agent.parameters()).add_(1)
        current = capture_state(agent, optimizer, 1)
        self.assertEqual(guard.observe(-100, 1, allow_rollback=False), "disabled")
        self.assert_nested_equal(capture_state(agent, optimizer, 1), current)
        self.assertEqual(guard.best_score, 100)
        self.assertEqual(guard.rollbacks, 0)
        self.assertEqual(guard.observe(101, 2, allow_rollback=False), "best")
        self.assertEqual(guard.best_score, 101)

    def test_best_recording_skips_duplicate_periodic_recording(self):
        agent, optimizer = agent_and_optimizer()
        guard = PolicyGuard(agent, optimizer, patience=1)
        env = SingleEnvWrapper(OneStepEnv())
        with tempfile.TemporaryDirectory() as folder:
            cfg = config(folder)
            cfg.train.max_iterations = 3
            cfg.guard.every = 1
            cfg.guard.enabled = False
            cfg.logging.plot_freq = 1
            recordings = []

            def record(*args, **kwargs):
                saved = torch.load(Path(folder) / "best.pt", weights_only=True)["state"]
                if kwargs.get("artifact_name"):
                    torch.testing.assert_close(agent.state_dict(), saved["model"])
                recordings.append((args[3], kwargs.get("artifact_name")))

            with (
                patch("sim.train_agent.validate_policy", side_effect=[
                    {"validation/returns/mean": score} for score in (100, 101, 100, 102)
                ]),
                patch("sim.train_agent.evaluate_and_plot", side_effect=record),
            ):
                run_training_loop(
                    cfg, env, object(), object(), agent, optimizer, guard, 0,
                )
            self.assertEqual(recordings, [
                (0, "best_iter_000000"), (1, "best_iter_000001"),
                (1, None),  # Periodic evaluation of the non-best update at iteration 1.
                (3, "best_iter_000003"),
            ])

    def test_best_recording_switch_disables_event_recordings(self):
        agent, optimizer = agent_and_optimizer()
        with tempfile.TemporaryDirectory() as folder:
            cfg = config(folder)
            cfg.logging.record_best = False
            cfg.train.max_iterations = 0
            with (
                patch("sim.train_agent.validate_policy", return_value={
                    "validation/returns/mean": 100.0,
                }),
                patch("sim.train_agent.evaluate_and_plot") as record,
            ):
                run_training_loop(cfg, None, object(), object(), agent, optimizer,
                                  PolicyGuard(agent, optimizer), 0)
            record.assert_not_called()
            self.assertTrue((Path(folder) / "best.pt").is_file())

    def test_nonfinite_reward_gradient_and_optimizer_updates_are_rolled_back(self):
        for failure in ("reward", "gradient", "optimizer"):
            with self.subTest(failure=failure):
                agent, optimizer = agent_and_optimizer()
                reward = float("nan") if failure == "reward" else 1.0
                env = SingleEnvWrapper(OneStepEnv(reward))
                before = capture_state(agent, optimizer, 0)
                if failure == "gradient":
                    hook = next(agent.parameters()).register_hook(
                        lambda grad: grad * float("nan")
                    )
                    self.addCleanup(hook.remove)
                original_step = optimizer.step

                def bad_step(*args, **kwargs):
                    result = original_step(*args, **kwargs)
                    if failure == "optimizer":
                        with torch.no_grad():
                            next(agent.parameters()).fill_(float("inf"))
                    return result

                with patch.object(optimizer, "step", side_effect=bad_step):
                    with self.assertRaises((FloatingPointError, RuntimeError)):
                        training_update(config(), env, agent, optimizer, 0)
                self.assert_nested_equal(capture_state(agent, optimizer, 0), before)
                self.assertTrue(all(p.grad is None for p in agent.parameters()))

    def test_failed_training_writes_resumable_state_and_failure_report(self):
        agent, optimizer = agent_and_optimizer()
        env = SingleEnvWrapper(OneStepEnv(float("nan")))
        with tempfile.TemporaryDirectory() as folder:
            cfg = config(folder)
            with self.assertLogs("sim.train_agent", level="ERROR"):
                with self.assertRaises(FloatingPointError):
                    run_training_loop(cfg, env, None, None, agent, optimizer,
                                      PolicyGuard(agent, optimizer), 0)
            self.assertEqual(
                load_checkpoint(Path(folder) / "latest.pt", agent, optimizer), 0
            )
            report = (Path(folder) / "training_failure.txt").read_text()
            self.assertIn("Non-finite reward", report)

    def test_validation_is_repeatable_and_does_not_consume_training_rng(self):
        cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
        assert isinstance(cfg, DictConfig)
        cfg.env.max_episode_steps = 5
        cfg.env.randomize = True
        agent = Agent(get_policy_inputs(cfg), cfg.policy.actions, hsize=4)
        with ExitStack() as resources:
            env = make_validation_env(cfg, resources)
            self.assertEqual(env.num_envs, cfg.guard.episodes)
            rng = torch.get_rng_state().clone()
            first = validate_policy(cfg, env, agent)
            second = validate_policy(cfg, env, agent)
        self.assertEqual(first, second)
        self.assertTrue(agent.training)
        torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)

    def test_interrupt_saves_a_checkpoint_and_identifies_the_stop_reason(self):
        agent, optimizer = agent_and_optimizer()
        raw_env = OneStepEnv()
        env = SingleEnvWrapper(raw_env)
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(raw_env, "step", side_effect=KeyboardInterrupt):
                with self.assertLogs("sim.train_agent", level="ERROR"):
                    with self.assertRaises(KeyboardInterrupt):
                        run_training_loop(
                            config(folder), env, None, None, agent, optimizer,
                            PolicyGuard(agent, optimizer), 0,
                        )
            self.assertEqual(
                load_checkpoint(Path(folder) / "latest.pt", agent, optimizer), 0
            )
            report = (Path(folder) / "training_failure.txt").read_text()
            self.assertIn("KeyboardInterrupt", report)

    def test_plane_collisions_extend_beyond_visual_arena(self):
        env = GymRP(actions=[Actions.ACC_BOTH_WHEELS], arena_half_size=4)
        self.addCleanup(env.close)
        env.reset(seed=1)
        physics = env.dm_env
        floor = physics.model.name2id("floor", "geom")
        physics.data.qpos[0] = 100.0
        physics.forward()
        contacts = [(int(c.geom1), int(c.geom2)) for c in physics.data.contact]
        self.assertEqual(sum(floor in pair for pair in contacts), 2)

    def test_terminal_observations_are_checked_even_with_finite_reward(self):
        agent, optimizer = agent_and_optimizer()
        env = OneStepEnv()
        invalid_transition = (
            {Observable.OBS_TIME: np.array([float("nan")])}, 1.0, True, False, {}
        )
        before = capture_state(agent, optimizer, 0)
        with patch.object(env, "step", return_value=invalid_transition):
            with self.assertRaisesRegex(FloatingPointError, "Non-finite observation"):
                training_update(config(), SingleEnvWrapper(env), agent, optimizer, 0)
        self.assert_nested_equal(capture_state(agent, optimizer, 0), before)

    def test_metric_transfer_failure_restores_the_completed_optimizer_update(self):
        agent, optimizer = agent_and_optimizer()
        before = capture_state(agent, optimizer, 0)
        original_step = optimizer.step
        original_numpy = torch.Tensor.numpy
        fail_transfer = False

        def step(*args, **kwargs):
            nonlocal fail_transfer
            result = original_step(*args, **kwargs)
            fail_transfer = True
            return result

        def numpy(tensor, *args, **kwargs):
            nonlocal fail_transfer
            if fail_transfer:
                fail_transfer = False
                raise RuntimeError("metric transfer failed")
            return original_numpy(tensor, *args, **kwargs)

        with (
            patch.object(optimizer, "step", side_effect=step),
            patch.object(torch.Tensor, "numpy", new=numpy),
        ):
            with self.assertRaisesRegex(RuntimeError, "metric transfer failed"):
                training_update(config(), SingleEnvWrapper(OneStepEnv()),
                                agent, optimizer, 0)
        self.assert_nested_equal(capture_state(agent, optimizer, 0), before)

    def test_failed_kl_measurement_restores_the_completed_optimizer_update(self):
        agent, optimizer = agent_and_optimizer()
        before = capture_state(agent, optimizer, 0)
        with (
            patch("sim.train_agent.PolicyProbe.metrics",
                  side_effect=RuntimeError("KL probe")),
            patch.object(optimizer, "step", wraps=optimizer.step) as step,
        ):
            with self.assertRaisesRegex(RuntimeError, "KL probe"):
                training_update(config(), SingleEnvWrapper(OneStepEnv()),
                                agent, optimizer, 0)
        step.assert_called_once()
        self.assert_nested_equal(capture_state(agent, optimizer, 0), before)

    def test_value_loss_coefficient_scales_critic_gradient(self):
        agent, _ = agent_and_optimizer()
        models = [deepcopy(agent), deepcopy(agent)]
        rng = torch.get_rng_state().clone()
        gradients = []
        for model, coefficient in zip(models, (1.0, 0.1)):
            torch.set_rng_state(rng)
            cfg = config()
            cfg.rl.value_loss_coef = coefficient
            cfg.train.grad_clip = 1e9  # Compare unclipped critic gradients.
            optimizer = torch.optim.Adam(model.parameters())
            metrics = training_update(cfg, SingleEnvWrapper(OneStepEnv()),
                                      model, optimizer, 0)
            head = model.value_head[-1]
            assert isinstance(head, torch.nn.Linear) and head.bias is not None
            assert head.bias.grad is not None
            gradients.append(head.bias.grad.clone())
            self.assertAlmostEqual(
                metrics["losses/total"],
                metrics["losses/policy"] + metrics["losses/value_weighted"], places=5,
            )
        self.assertGreater(gradients[0].abs().sum(), 0)
        torch.testing.assert_close(gradients[1], 0.1 * gradients[0])

    def test_final_checkpoint_failure_also_writes_a_failure_report(self):
        agent, optimizer = agent_and_optimizer()
        calls = 0

        def save(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 2:  # Initial checkpoint succeeds; final checkpoint fails.
                raise OSError("final upload failed")
            return write_checkpoint(*args, **kwargs)

        with tempfile.TemporaryDirectory() as folder:
            cfg = config(folder)
            cfg.train.max_iterations = 0
            with (
                patch("sim.train_agent.write_checkpoint", side_effect=save),
                self.assertLogs("sim.train_agent", level="ERROR"),
            ):
                with self.assertRaisesRegex(OSError, "final upload failed"):
                    run_training_loop(
                        cfg, SingleEnvWrapper(OneStepEnv()), None, None,
                        agent, optimizer, PolicyGuard(agent, optimizer), 0,
                    )
            self.assertEqual(calls, 3)
            self.assertEqual(
                load_checkpoint(Path(folder) / "latest.pt", agent, optimizer), 0
            )
            report = (Path(folder) / "training_failure.txt").read_text()
            self.assertIn("final upload failed", report)

    def test_host_backup_is_saved_even_if_in_memory_restore_fails(self):
        agent, optimizer = agent_and_optimizer()
        before = capture_state(agent, optimizer, 0)
        env = SingleEnvWrapper(OneStepEnv())
        original_step = optimizer.step

        def corrupt_step():
            original_step()
            with torch.no_grad():
                next(agent.parameters()).fill_(float("inf"))

        with tempfile.TemporaryDirectory() as folder:
            with (
                patch.object(optimizer, "step", side_effect=corrupt_step),
                patch("sim.train_agent.restore_state",
                      side_effect=RuntimeError("device lost")),
                self.assertLogs("sim.train_agent", level="ERROR"),
            ):
                with self.assertRaises(FloatingPointError):
                    run_training_loop(
                        config(folder), env, None, None, agent, optimizer,
                        PolicyGuard(agent, optimizer), 0,
                    )
            checkpoint = torch.load(Path(folder) / "latest.pt", weights_only=True)
            self.assert_nested_equal(checkpoint["state"], before)
            self.assertTrue(torch.isinf(next(agent.parameters())).all())


if __name__ == "__main__":
    unittest.main()

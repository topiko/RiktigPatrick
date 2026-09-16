"""Curriculum masks, physical promotion criteria, recovery and stage-run ownership."""

import tempfile
import unittest
from contextlib import ExitStack, contextmanager
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import mlflow.pytorch as mlflow_pytorch
import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf

from riktigpatric.patrick import Actions, DerivedObs, Observable, Target
from sim.checkpoints import (
    PolicyGuard,
    capture_state,
    load_checkpoint,
    restore_state,
    save_checkpoint,
)
from sim.curriculum import HEAD_ACTIONS, HEAD_REWARDS, STAGES, Curriculum
from sim.train_agent import (
    check_policy_guard,
    make_agent,
    make_validation_env,
    run_training_loop,
    start_stage_run,
    training_update,
    validate_policy,
)
from sim.utils import EpisodeBuffer, npd2tensord, register_and_make_env


def config():
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    cfg.train.device = "cpu"
    cfg.policy.hsize = 8
    cfg.env.n_parallel = 3
    cfg.env.max_episode_steps = 4
    cfg.train.tbptt_steps = 2
    cfg.logging.plot_freq = 0
    return cfg


def passing_metrics(score=100.0):
    return {
        "validation/returns/mean": score,
        "validation/survival_fraction": 0.9,
        "validation/position_mae": 0.02,
        "validation/velocity_mae": 0.04,
        "validation/yaw_rate_mae": 0.1,
    }


def metric_buffer(times, velocity_errors, *, terminated=False):
    observations = [{
        Observable.OBS_TIME: np.array([time]),
        DerivedObs.CURRENT_VEL: np.array([error]),
        Target.TARGET_VEL: np.array([0.0]),
        DerivedObs.CURRENT_POS: np.array([0.1]),
        Target.TARGET_POS: np.array([0.0]),
        DerivedObs.YAW_RATE: np.array([0.2]),
        Target.YAW_RATE: np.array([0.0]),
    } for time, error in zip(times, velocity_errors)]
    buffer = EpisodeBuffer()
    for obs in observations[:-1]:
        buffer.add_step(obs, {Actions.TIME: obs[Observable.OBS_TIME]},
                        0.0, torch.tensor(0.0), torch.tensor(0.0))
    buffer.finish(observations[-1], terminated=terminated)
    return buffer


class CurriculumTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def setUp(self):
        torch.manual_seed(123)
        np.random.seed(123)

    def training(self):
        cfg = config()
        curriculum = Curriculum(cfg)
        agent = make_agent(cfg)
        curriculum.apply_policy(agent)
        optimizer = torch.optim.Adam(agent.parameters(), lr=0.001)
        return cfg, curriculum, agent, optimizer

    def env(self, cfg):
        env = register_and_make_env(cfg)
        self.addCleanup(env.close)
        return env

    def assert_state_equal(self, actual, expected):
        for key in ("model", "optimizer", "torch_rng"):
            torch.testing.assert_close(actual[key], expected[key], rtol=0, atol=0)
        for key in ("curriculum", "numpy_rng", "python_rng", "next_iteration"):
            self.assertEqual(actual[key], expected[key])

    def test_inactive_heads_neither_sample_nor_contribute_log_probability(self):
        cfg, curriculum, agent, _ = self.training()
        env = self.env(cfg)
        obs, _ = env.reset(seed=1)
        obs = npd2tensord(obs)
        sample = torch.distributions.Categorical.sample
        sampled = []

        def counted_sample(dist, *args, **kwargs):
            sampled.append(dist)
            return sample(dist, *args, **kwargs)

        with patch.object(
            torch.distributions.Categorical, "sample", new=counted_sample
        ):
            actions, logp, values, _ = agent.act(obs)
        self.assertEqual(len(sampled), 1)
        for action in curriculum.inactive_actions:
            torch.testing.assert_close(actions[action], torch.zeros((3, 1)))
        key = Actions.ACC_BOTH_WHEELS
        bins = getattr(agent, agent.action_configs[key]["bins_name"])
        indices = (actions[key] == bins).to(torch.int64).argmax(dim=-1)
        torch.testing.assert_close(logp[:, 0], sampled[0].log_prob(indices))
        (values.square().mean() - logp.mean()).backward()
        for action in curriculum.inactive_actions:
            self.assertTrue(all(
                p.grad is None for p in agent.action_heads[action].parameters()
            ))
        self.assertGreater(agent.rnn.weight_ih_l0.grad.abs().sum().item(), 0)

    def test_stages_mask_rewards_and_leave_inactive_head_weights_alone(self):
        cfg, curriculum, agent, optimizer = self.training()
        env = self.env(cfg)
        for stage in STAGES:
            self.assertEqual(curriculum.stage, stage)
            before = deepcopy(agent.state_dict())
            metrics = training_update(cfg, env, agent, optimizer, 0, curriculum)
            self.assertTrue(all(np.isfinite(value) for value in metrics.values()))
            for action in curriculum.inactive_actions:
                for name, parameter in agent.action_heads[action].named_parameters():
                    torch.testing.assert_close(
                        parameter, before[f"action_heads.{action.value}.{name}"],
                        rtol=0, atol=0,
                    )
                    self.assertIsNone(parameter.grad)
                    self.assertNotIn(parameter, optimizer.state)
            raw = env.envs[0].unwrapped
            state = raw.state
            state.derived_obs[DerivedObs.CURRENT_VEL][:] = 0.2
            state.derived_obs[DerivedObs.YAW_RATE][:] = 0.7
            state.obs.set_observable(Observable.TRUE_CAMERA_PITCH, np.array([0.3]))
            state.obs.set_observable(Observable.HEAD_TURN, np.array([0.4]))
            rewards = raw._calculate_rewards(state, terminated=False)
            for key in (
                Observable.REWARD_VEL, Observable.REWARD_YAW_RATE, *HEAD_REWARDS
            ):
                if key in curriculum.disabled_rewards:
                    self.assertEqual(rewards[key], 0)
            if stage != "balance":
                self.assertLess(rewards[Observable.REWARD_VEL], 0)
            self.assertEqual(rewards[Observable.REWARD_WHEEL_VEL], 0)
            if stage in ("locomotion", "full_control"):
                self.assertLess(rewards[Observable.REWARD_YAW_RATE], 0)
            if stage == "full_control":
                self.assertLess(rewards[Observable.REWARD_CAMERA_PITCH], 0)
                self.assertLess(rewards[Observable.REWARD_HEAD_YAW], 0)
            self.assertAlmostEqual(
                rewards[Observable.REWARD_TOTAL],
                sum(v for k, v in rewards.items() if k != Observable.REWARD_TOTAL),
            )
            self.assertEqual(raw.reward_scales[Observable.REWARD_CAMERA_PITCH], -0.5)
            if stage != "full_control":
                curriculum.advance(agent, optimizer)

    def test_activation_prefers_zero_and_preserves_existing_heads_and_adam(self):
        cfg, curriculum, agent, optimizer = self.training()
        env = self.env(cfg)
        training_update(cfg, env, agent, optimizer, 0, curriculum)
        for newly_active in ((), (Actions.VEL_WHEEL_DIFF,), HEAD_ACTIONS):
            before = deepcopy(agent.state_dict())
            adam = deepcopy(optimizer.state_dict())
            curriculum.advance(agent, optimizer)
            torch.testing.assert_close(optimizer.state_dict(), adam)
            for name, tensor in agent.state_dict().items():
                if not any(
                    name.startswith(f"action_heads.{key.value}.")
                    for key in newly_active
                ):
                    torch.testing.assert_close(tensor, before[name], rtol=0, atol=0)
            obs, _ = env.reset(seed=2)
            logits, _, _ = agent.forward(npd2tensord(obs))
            for action in newly_active:
                bins = getattr(agent, agent.action_configs[action]["bins_name"])
                probs = logits[action].softmax(dim=-1)
                torch.testing.assert_close(probs[:, bins == 0], torch.full((3, 1), 0.9))
                self.assertNotIn(action, agent.inactive_actions)

    def test_commands_cover_pairs_and_evaluation_does_not_consume_training_rng(self):
        cfg, curriculum, agent, optimizer = self.training()
        dummy = SimpleNamespace(num_envs=200, set_attr=Mock())
        for stage in ("balance", "hold_position"):
            self.assertEqual(curriculum.stage, stage)
            commands = curriculum.prepare_rollout(dummy, agent)
            np.testing.assert_array_equal(commands["target_velocities"], 0)
            np.testing.assert_array_equal(commands["target_yaw_rates"], 0)
            curriculum.advance(agent, optimizer)
        commands = curriculum.prepare_rollout(dummy, agent)
        actual = set(zip(commands["target_velocities"], commands["target_yaw_rates"]))
        self.assertEqual(actual, set(map(tuple, curriculum.command_pairs)))
        before = np.random.get_state()
        first = curriculum.prepare_rollout(dummy, agent, evaluation=True)
        second = curriculum.prepare_rollout(dummy, agent, evaluation=True)
        np.testing.assert_array_equal(
            first["target_yaw_rates"], second["target_yaw_rates"]
        )
        after = np.random.get_state()
        assert isinstance(after, tuple) and isinstance(before, tuple)
        np.testing.assert_array_equal(after[1], before[1])
        with ExitStack() as resources:
            env = make_validation_env(cfg, resources, curriculum)
            rng = capture_state(agent, optimizer, 0, curriculum)
            self.assertEqual(validate_policy(cfg, env, agent, curriculum),
                             validate_policy(cfg, env, agent, curriculum))
            self.assert_state_equal(capture_state(agent, optimizer, 0, curriculum), rng)
            self.assertTrue(all(e.unwrapped._randomize for e in env.envs))

    def test_physical_metrics_include_short_failures_and_distinguish_time_limit(self):
        curriculum = Curriculum(config())
        long = metric_buffer([0, 0.5, 1, 10], [0, 99, 0.02, 0.04])
        short = metric_buffer([0, 0.5], [0, 2], terminated=True)
        metrics = curriculum.validation_metrics([long, short])
        self.assertEqual(metrics["validation/survival_fraction"], 0.5)
        self.assertAlmostEqual(metrics["validation/velocity_mae"], (0.03 + 2) / 2)
        self.assertAlmostEqual(metrics["validation/position_mae"], 0.1)
        self.assertAlmostEqual(metrics["validation/yaw_rate_mae"], 0.2)
        long.terminated = True
        self.assertEqual(
            curriculum.validation_metrics([long])["validation/survival_fraction"], 0
        )
        late_fall = metric_buffer([0, 11], [0, 0], terminated=True)
        self.assertEqual(
            curriculum.validation_metrics([late_fall])["validation/survival_fraction"],
            1,
        )
        for stage in ("hold_position", "locomotion"):
            curriculum.stage = stage
            self.assertEqual(
                curriculum.validation_metrics([late_fall])["validation/survival_fraction"],
                0,
            )

    def test_promotion_needs_consecutive_passes_and_stage_specific_metrics(self):
        _, curriculum, agent, optimizer = self.training()
        good = passing_metrics()
        good["validation/yaw_rate_mae"] = 100
        good["validation/velocity_mae"] = 100  # Both errors are ignored in balance.
        self.assertFalse(curriculum.observe(good))
        self.assertFalse(curriculum.observe(good))
        self.assertFalse(curriculum.observe({
            **good, "validation/survival_fraction": 0.8
        }))
        self.assertEqual(curriculum.success_streak, 0)
        for expected in (False, False, True):
            self.assertEqual(curriculum.observe(good), expected)
        curriculum.advance(agent, optimizer)
        self.assertEqual(curriculum.stage, "hold_position")
        self.assertEqual(curriculum.success_streak, 0)
        self.assertFalse(curriculum.observe(good))  # Forward velocity now matters.
        good["validation/velocity_mae"] = 0.04
        self.assertFalse(curriculum.observe({**good, "validation/position_mae": 1.0}))
        for expected in (False, False, True):
            self.assertEqual(curriculum.observe(good), expected)  # Yaw still ignored.
        curriculum.advance(agent, optimizer)
        self.assertEqual(curriculum.stage, "locomotion")
        self.assertFalse(curriculum.observe(good))  # Yaw now matters too.
        for expected in (False, False, True):
            self.assertEqual(curriculum.observe(passing_metrics()), expected)
        curriculum.advance(agent, optimizer)
        self.assertFalse(curriculum.observe(passing_metrics()))

    def test_checkpoint_restores_stage_streak_weights_adam_rng_and_neutral_mask(self):
        cfg, curriculum, agent, optimizer = self.training()
        curriculum.advance(agent, optimizer)
        curriculum.advance(agent, optimizer)
        training_update(cfg, self.env(cfg), agent, optimizer, 0, curriculum)
        curriculum.success_streak = 2
        before = capture_state(agent, optimizer, 17, curriculum)
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "latest.pt", before, {})
            curriculum.advance(agent, optimizer)
            torch.rand(5)
            np.random.rand(5)
            self.assertEqual(load_checkpoint(path, agent, optimizer, curriculum), 17)
        self.assert_state_equal(capture_state(agent, optimizer, 17, curriculum), before)
        self.assertEqual(agent.inactive_actions, HEAD_ACTIONS)
        changed = deepcopy(cfg)
        changed.guard.seed += 1
        restarted = Curriculum(changed)
        restarted.load_state_dict(before["curriculum"])
        self.assertEqual(restarted.stage, "locomotion")
        self.assertEqual(restarted.success_streak, 0)
        with self.assertRaisesRegex(ValueError, "curriculum.enabled"):
            restore_state(agent, optimizer, before)
        legacy = capture_state(agent, optimizer, 17)
        del legacy["curriculum"]
        with self.assertRaisesRegex(ValueError, "curriculum.enabled"):
            restore_state(agent, optimizer, legacy, curriculum=curriculum)

    def test_position_hold_penalizes_drift_with_gentle_velocity_damping(self):
        cfg, curriculum, agent, optimizer = self.training()
        env = self.env(cfg)
        curriculum.advance(agent, optimizer)
        self.assertEqual(curriculum.stage, "hold_position")
        curriculum.prepare_rollout(env, agent)
        env.reset(seed=1)
        raw = env.envs[0].unwrapped
        state = raw.state
        self.assertEqual(state.target_pos, 0)
        self.assertIn(Target.TARGET_POS.value, agent.inputs)
        self.assertIn(DerivedObs.CURRENT_POS.value, agent.inputs)
        state.derived_obs[DerivedObs.CURRENT_VEL][:] = 0.2
        for sensor in (Observable.LEFT_WHEEL_VEL, Observable.RIGHT_WHEEL_VEL):
            state.obs.set_observable(sensor, np.array([4.0]))
        for position in (-0.8, -0.4, 0.4, 0.8):
            state.derived_obs[DerivedObs.CURRENT_POS][:] = position
            rewards = raw._calculate_rewards(state, terminated=False)
            self.assertAlmostEqual(rewards[Observable.REWARD_POS], -abs(position))
            self.assertAlmostEqual(rewards[Observable.REWARD_VEL], -0.1)
            self.assertEqual(rewards[Observable.REWARD_WHEEL_VEL], 0)
        curriculum.advance(agent, optimizer)
        curriculum.prepare_rollout(env, agent)
        rewards = raw._calculate_rewards(state, terminated=False)
        self.assertEqual(rewards[Observable.REWARD_POS], 0)
        self.assertAlmostEqual(rewards[Observable.REWARD_VEL], -0.8)

    def test_guard_rollbacks_are_stage_local_and_clear_promotion_streak(self):
        _, curriculum, agent, optimizer = self.training()
        guard = PolicyGuard(agent, optimizer, curriculum=curriculum, patience=1)
        guard.observe(100, 0)
        curriculum.success_streak = 2
        self.assertEqual(guard.observe(0, 1), "rollback")
        self.assertEqual(curriculum.success_streak, 0)
        self.assertEqual(curriculum.stage, "balance")
        curriculum.advance(agent, optimizer)
        with self.assertRaisesRegex(ValueError, "across curriculum stages"):
            guard.observe(0, 2)
        guard.reset_baseline()
        self.assertEqual(guard.observe(10, 2), "best")
        assert guard.best_state is not None
        self.assertEqual(guard.best_state["curriculum"]["stage"], "hold_position")

    def test_legacy_checkpoints_keep_their_objective_and_reset_promotion_progress(self):
        cfg, curriculum, agent, optimizer = self.training()
        curriculum.advance(agent, optimizer)  # The old balance objective was stop.
        training_update(cfg, self.env(cfg), agent, optimizer, 0, curriculum)
        before = capture_state(agent, optimizer, 400, curriculum)
        for old, new in (("balance", "hold_position"), ("locomotion", "locomotion"),
                         ("full_control", "full_control")):
            with self.subTest(stage=old), tempfile.TemporaryDirectory() as folder:
                legacy = deepcopy(before)
                progress = legacy["curriculum"]
                del progress["version"]
                del progress["settings"]["hold_seconds"]
                progress.update(stage=old, success_streak=2)
                path = save_checkpoint(Path(folder) / "legacy.pt", legacy, {})
                self.assertEqual(
                    load_checkpoint(path, agent, optimizer, curriculum), 400
                )
                self.assertEqual(curriculum.stage, new)
                self.assertEqual(curriculum.success_streak, 0)
                self.assertEqual(agent.inactive_actions, curriculum.inactive_actions)
                migrated = capture_state(agent, optimizer, 400, curriculum)
                for key in ("model", "optimizer", "torch_rng"):
                    torch.testing.assert_close(
                        migrated[key], before[key], rtol=0, atol=0
                    )
                self.assertEqual(migrated["curriculum"]["version"], Curriculum.VERSION)
                # Saving and loading again must not reinterpret the stage a second time.
                path = save_checkpoint(Path(folder) / "migrated.pt", migrated, {})
                load_checkpoint(path, agent, optimizer, curriculum)
                self.assertEqual(curriculum.stage, new)

    def test_checkpoint_versions_are_validated_before_restoring_weights(self):
        _, curriculum, agent, optimizer = self.training()
        before = capture_state(agent, optimizer, 0, curriculum)
        for version, stage in ((4, "balance"), (True, "balance"), (1, "stop")):
            invalid = deepcopy(before)
            invalid["curriculum"].update(version=version, stage=stage)
            with (
                self.subTest(version=version, stage=stage),
                self.assertRaises(ValueError),
            ):
                restore_state(agent, optimizer, invalid, curriculum=curriculum)
            self.assert_state_equal(
                capture_state(agent, optimizer, 0, curriculum), before
            )
        legacy = deepcopy(before)
        del legacy["curriculum"]["version"]
        with self.assertRaisesRegex(ValueError, "across curriculum stages"):
            restore_state(agent, optimizer, legacy, curriculum=curriculum,
                          restore_curriculum=False)

    def test_version_two_stop_maps_to_position_hold_but_balance_stays_balance(self):
        curriculum = Curriculum(config())
        for old, new in (("balance", "balance"), ("stop", "hold_position")):
            state = curriculum.state_dict()
            state.update(version=2, stage=old, success_streak=2)
            curriculum.load_state_dict(state)
            self.assertEqual(curriculum.stage, new)
            self.assertEqual(curriculum.success_streak, 0)

    def test_failed_update_restores_sampled_commands_rng_and_curriculum_progress(self):
        cfg, curriculum, agent, optimizer = self.training()
        env = self.env(cfg)
        curriculum.advance(agent, optimizer)
        curriculum.advance(agent, optimizer)
        curriculum.success_streak = 2
        before = capture_state(agent, optimizer, 0, curriculum)
        with patch.object(env, "step", side_effect=RuntimeError("broken rollout")):
            with self.assertRaisesRegex(RuntimeError, "broken rollout"):
                training_update(cfg, env, agent, optimizer, 0, curriculum)
        self.assert_state_equal(capture_state(agent, optimizer, 0, curriculum), before)
        self.assertTrue(all(p.grad is None for p in agent.parameters()))

    def test_failed_activation_restores_partially_initialized_heads_and_stage(self):
        cfg, curriculum, agent, optimizer = self.training()
        curriculum.advance(agent, optimizer)
        curriculum.advance(agent, optimizer)
        curriculum.success_streak = 2
        guard = PolicyGuard(agent, optimizer, curriculum=curriculum)
        initialize = agent.initialize_neutral_head

        def fail_second(action, probability):
            initialize(action, probability)
            if action == Actions.VEL_HEAD_TURN:
                raise RuntimeError("head initialization failed")

        with (
            patch.object(agent, "initialize_neutral_head", side_effect=fail_second),
            patch("sim.train_agent.validate_policy", return_value=passing_metrics()),
            patch("sim.train_agent.write_checkpoint"),
        ):
            with self.assertRaisesRegex(RuntimeError, "initialization failed") as error:
                check_policy_guard(cfg, None, agent, guard, 100, curriculum)
        before = getattr(error.exception, "training_state")
        self.assert_state_equal(
            capture_state(agent, optimizer, 100, curriculum), before
        )
        self.assertEqual(curriculum.stage, "locomotion")
        self.assertEqual(agent.inactive_actions, HEAD_ACTIONS)

    def test_stage_runs_own_metrics_artifacts_and_resume_starts_a_fresh_child(self):
        cfg, curriculum, agent, optimizer = self.training()
        cfg.curriculum.consecutive_passes = 1
        cfg.guard.every = 1
        cfg.train.max_iterations = len(STAGES)
        cfg.logging.mlflow.enabled = True
        cfg.logging.mlflow.push_freq = 1
        cfg.logging.save_freq = 1
        guard = PolicyGuard(agent, optimizer, curriculum=curriculum)
        active = ["parent"]
        children, logged_metrics, artifact_stages, tags = [], [], [], []

        @contextmanager
        def start_child(*, run_name, nested):
            self.assertTrue(nested)
            self.assertEqual(active, ["parent"])
            active.append(run_name)
            children.append(run_name)
            try:
                yield
            finally:
                active.pop()

        def log_artifact(path, **kwargs):
            state = torch.load(path, weights_only=True)["state"]
            self.assertEqual(state["curriculum"]["stage"], active[-1])
            artifact_stages.append(active[-1])

        def validate(*args):
            return passing_metrics({
                "balance": 100, "hold_position": 75, "locomotion": 50,
                "full_control": 25,
            }[curriculum.stage])

        def log_metrics(metrics, **kwargs):
            logged_metrics.append((active[-1], metrics.copy()))

        update_metrics = {"losses/policy": 1., "losses/value": 2., "returns/mean": 3.,
                          "episodes/length/mean": 4.}
        with (
            tempfile.TemporaryDirectory() as folder,
            patch("sim.train_agent.mlflow.start_run", side_effect=start_child),
            patch("sim.train_agent.mlflow.log_params"),
            patch("sim.train_agent.mlflow.set_tags", side_effect=tags.append),
            patch("sim.train_agent.mlflow.log_metrics", side_effect=log_metrics),
            patch("sim.train_agent.mlflow.log_artifact", side_effect=log_artifact),
            patch.object(mlflow_pytorch, "log_model") as log_model,
            patch("sim.train_agent.validate_policy", side_effect=validate),
            patch("sim.train_agent.training_update", return_value=update_metrics),
            ExitStack() as stages,
        ):
            cfg.checkpoints.dir = folder
            run_training_loop(cfg, None, None, object(), agent, optimizer, guard, 0,
                              curriculum, stages)
            self.assertEqual(children, list(STAGES))
            self.assertEqual(set(artifact_stages), set(STAGES))
            training_stages = [
                stage for stage, m in logged_metrics if "losses/policy" in m
            ]
            self.assertEqual(training_stages, list(STAGES))
            self.assertEqual(log_model.call_count, len(STAGES))
            self.assertEqual(guard.best_score, 25)
            latest = torch.load(Path(folder) / "latest.pt", weights_only=True)["state"]
            self.assertEqual(latest["curriculum"]["stage"], "full_control")
            cfg.train.resume_from = str(Path(folder) / "latest.pt")
            start_stage_run(cfg, agent, curriculum, len(STAGES), stages)
            self.assertEqual(children[-2:], ["full_control", "full_control"])
            self.assertEqual(tags[-1]["training.start_iteration"], len(STAGES))
            self.assertEqual(tags[-1]["training.resume_from"], cfg.train.resume_from)
        self.assertEqual(active, ["parent"])

    def test_promotion_retains_cpu_backup_if_restoration_also_fails(self):
        cfg, curriculum, agent, optimizer = self.training()
        curriculum.success_streak = 2
        guard = PolicyGuard(agent, optimizer, curriculum=curriculum)
        with (
            patch.object(curriculum, "advance", side_effect=RuntimeError("activation")),
            patch("sim.train_agent.restore_state",
                  side_effect=RuntimeError("device lost")),
            patch("sim.train_agent.validate_policy", return_value=passing_metrics()),
            patch("sim.train_agent.write_checkpoint"),
            self.assertLogs("sim.train_agent", level="ERROR"),
        ):
            with self.assertRaisesRegex(RuntimeError, "activation") as error:
                check_policy_guard(cfg, None, agent, guard, 100, curriculum)
        backup = getattr(error.exception, "training_state")
        self.assertEqual(backup["curriculum"]["stage"], "balance")
        self.assertTrue(all(
            value.device.type == "cpu" for value in backup["model"].values()
        ))

    def test_initial_or_resumed_evaluation_does_not_count_toward_promotion(self):
        cfg, curriculum, agent, optimizer = self.training()
        curriculum.success_streak = 2
        guard = PolicyGuard(agent, optimizer, curriculum=curriculum)
        with (
            patch("sim.train_agent.validate_policy", return_value=passing_metrics()),
            patch("sim.train_agent.write_checkpoint"),
        ):
            check_policy_guard(cfg, None, agent, guard, 100, curriculum, advance=False)
        self.assertEqual(curriculum.stage, "balance")
        self.assertEqual(curriculum.success_streak, 2)

    def test_invalid_curriculum_configuration_is_rejected(self):
        for key, value in (
            ("curriculum.neutral_probability", 1.0),
            ("curriculum.consecutive_passes", 0),
            ("curriculum.hold_seconds", 0),
            ("curriculum.forward_velocities", []),
            ("curriculum.yaw_rates", [float("nan")]),
            ("guard.episodes", 1),
            ("env.tracking_mode", "position"),
            ("env.head_tracking", False),
            ("train.target_yaw_rates", [0, 0, 0]),
        ):
            cfg = config()
            OmegaConf.update(cfg, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                Curriculum(cfg)


if __name__ == "__main__":
    unittest.main()

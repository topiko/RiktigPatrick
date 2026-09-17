"""Variable control intervals must preserve SI timing, returns and reproducibility."""

import csv
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch
from gymnasium import Env
from omegaconf import DictConfig, OmegaConf

from riktigpatric.patrick import Actions, Observable
from sim.checkpoints import capture_state, restore_state
from sim.curriculum import Curriculum
from sim.envs.rp_env import GymRP
from sim.episode_io import resample_video_frames, save_episode_csv
from sim.timing import PHYSICS_STEP, ControlTiming
from sim.train_agent import make_agent, rollout, training_update
from sim.utils import ebufs2batchd, ebufs2step_times, get_returns, register_and_make_env


def config():
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    cfg.train.device = "cpu"
    cfg.policy.hsize = 8
    cfg.env.n_parallel = 2
    cfg.env.step_time_std = 0.002
    cfg.env.max_episode_time = 0.04
    cfg.train.tbptt_steps = 2
    return cfg


class TimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_sampling_is_seeded_quantized_and_fixed_timing_consumes_no_rng(self):
        timing = ControlTiming(0.01, 0.002, 0.007)
        rng = np.random.default_rng(12)
        first = [timing.sample_steps(rng) for _ in range(100)]
        rng = np.random.default_rng(12)
        self.assertEqual(first, [timing.sample_steps(rng) for _ in range(100)])
        self.assertGreater(len(set(first)), 1)
        self.assertGreaterEqual(min(first), 4)  # 7 ms lower bound becomes 8 ms.
        fixed = ControlTiming(0.01, 0, 0.007)
        before = rng.bit_generator.state
        self.assertEqual([fixed.sample_steps(rng) for _ in range(10)], [5] * 10)
        self.assertEqual(before, rng.bit_generator.state)

    def test_invalid_timing_is_rejected(self):
        for mean, std, minimum in (
            (0, 0, 0.007), (0.003, 0, 0.007), (0.01, -1, 0.007),
            (0.01, float("nan"), 0.007), (0.01, 0.002, 0), (0.01, 0.002, 0.02),
        ):
            with self.subTest(mean=mean, std=std, minimum=minimum):
                with self.assertRaises(ValueError):
                    ControlTiming(mean, std, minimum)

    def test_nominal_commands_and_elapsed_time_reward_scaling(self):
        env = GymRP(
            actions=[Actions.ACC_BOTH_WHEELS], step_time_std=0.002,
            reward_scales={Observable.REWARD_STEP: 1., Observable.REWARD_FELL: -7.,
                           Observable.REWARD_TOTAL: 1.},
        )
        self.addCleanup(env.close)
        env.reset(seed=1)
        with patch.object(env.timing, "sample_steps", return_value=10):
            obs, reward, terminated, _, info = env.step({
                Actions.ACC_BOTH_WHEELS: np.array([40.0])
            })
        self.assertFalse(terminated)
        self.assertEqual(info["step_time"], 0.02)
        self.assertAlmostEqual(obs[Observable.OBS_TIME][0], 0.02)
        self.assertEqual(reward, 2.0)
        for actuator in (env.left_wheel_act, env.right_wheel_act):
            self.assertAlmostEqual(float(env.dm_env.bind(actuator).ctrl), 0.4)
        rewards = env._calculate_rewards(env.state, terminated=True, time_fraction=2)
        self.assertEqual(rewards[Observable.REWARD_FELL], -7)
        self.assertEqual(rewards[Observable.REWARD_TOTAL], -5)
        env.reset(seed=1)
        self.assertEqual(env.dm_env.model.opt.timestep, PHYSICS_STEP)

    def test_duration_limit_allows_more_decisions_and_shortens_the_last_interval(self):
        cfg = config()
        cfg.env.n_parallel = 1
        cfg.env.max_episode_steps = 4  # Legacy nominal budget: 4 * 10 ms = 40 ms.
        for key in cfg.reward:
            cfg.reward[key] = 0.0
        cfg.reward[Observable.REWARD_STEP.value] = 1.0
        cfg.reward[Observable.REWARD_TOTAL.value] = 1.0
        env = register_and_make_env(cfg)
        assert isinstance(env, Env)
        self.addCleanup(env.close)
        raw = env.unwrapped
        assert isinstance(raw, GymRP)
        for substeps, expected in ((4, [0.008] * 5), (6, [0.012] * 3 + [0.004])):
            env.reset(seed=1)
            durations, total = [], 0.0
            with patch.object(raw.timing, "sample_steps", return_value=substeps):
                truncated = False
                while not truncated:
                    obs, reward, terminated, truncated, info = env.step({
                        Actions.ACC_BOTH_WHEELS: np.array([0.0])
                    })
                    self.assertFalse(terminated)
                    durations.append(info["step_time"])
                    total += float(reward)
            np.testing.assert_allclose(durations, expected)
            self.assertAlmostEqual(obs[Observable.OBS_TIME][0], 0.04)
            self.assertAlmostEqual(total, 4.0)

    def test_vector_intervals_are_independent_and_repeat_after_seeded_reset(self):
        cfg = config()
        cfg.env.n_parallel = 3
        cfg.env.max_episode_time = 0.1
        env = register_and_make_env(cfg)
        self.addCleanup(env.close)
        actions = {
            key: np.zeros((3, 1), dtype=np.float32) for key in cfg.policy.actions
        }
        sequences = []
        for _ in range(2):
            env.reset(seed=42)
            durations = []
            for _ in range(6):
                _, _, _, _, info = env.step(actions)
                durations.append(info["step_time"].copy())
            sequences.append(np.array(durations))
        np.testing.assert_array_equal(sequences[0], sequences[1])
        self.assertFalse(np.array_equal(sequences[0][:, 0], sequences[0][:, 1]))

    def test_variable_discounts_preserve_reward_alignment_and_padding(self):
        rewards = torch.tensor([[1., 2., 3.], [4., 5., 0.]])
        intervals = torch.tensor([[0.02, 0.01, 0.01], [0.005, 0.02, 0.]])
        result = get_returns(
            rewards, 0.5, step_times=intervals, reference_step_time=0.01
        )
        torch.testing.assert_close(result, torch.tensor([
            [1.875, 3.5, 3.], [4 + math.sqrt(0.5) * 5, 5., 0.],
        ]))
        torch.testing.assert_close(
            get_returns(rewards[0], 0.5, step_times=intervals[0]), result[0]
        )
        torch.testing.assert_close(
            get_returns(rewards, 0.99, step_times=torch.full_like(rewards, 0.01)),
            get_returns(rewards, 0.99), rtol=0, atol=0,
        )
        torch.testing.assert_close(
            get_returns(rewards, 0, step_times=intervals), rewards
        )

    def test_jitter_durations_are_exported_and_updates_replay_after_restore(self):
        cfg = config()
        env = register_and_make_env(cfg)
        assert not isinstance(env, Env)  # This fixture uses a vector environment.
        self.addCleanup(env.close)
        agent = make_agent(cfg)
        curriculum = Curriculum(cfg)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        curriculum.apply_policy(agent)
        before = capture_state(agent, optimizer, 0, curriculum)
        first = training_update(cfg, env, agent, optimizer, 0, curriculum)
        restore_state(agent, optimizer, before, curriculum=curriculum)
        self.assertEqual(
            first, training_update(cfg, env, agent, optimizer, 0, curriculum)
        )
        self.assertAlmostEqual(first["episodes/duration/mean"], 0.04, places=6)
        self.assertAlmostEqual(first["returns/mean"], 4.0, places=6)
        with torch.no_grad():
            buffers = rollout(env, agent, seed=3, curriculum=curriculum)
        _, rewards, values, lengths, mask = ebufs2batchd(buffers)
        intervals = ebufs2step_times(buffers)
        self.assertEqual(intervals.shape, mask.shape)
        self.assertTrue((intervals[mask == 0] == 0).all())
        returns = get_returns(rewards, cfg.rl.discount, step_times=intervals)
        with tempfile.TemporaryDirectory() as folder:
            path = save_episode_csv(
                buffers[0], Path(folder) / "trace.csv",
                returns=returns[0, :int(lengths[0])].numpy(),
                advantages=(returns - values)[0, :int(lengths[0])].numpy(),
            )
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            np.testing.assert_allclose(
                [float(row["transition/duration"]) for row in rows[:-1]],
                buffers[0].get_step_times(),
            )
            self.assertEqual(rows[-1]["transition/duration"], "")

    def test_video_resampling_uses_elapsed_time_and_keeps_the_terminal_image(self):
        self.assertEqual(
            resample_video_frames([0, 1, 2, 3], np.array([.008, .014, .008]), 100),
            [0, 1, 3],
        )
        self.assertEqual(
            resample_video_frames([0, 1, 2], np.array([.02, .008]), 100), [0, 0, 2]
        )
        with self.assertRaises(ValueError):
            resample_video_frames([0, 1], np.array([.01, .01]), 100)


if __name__ == "__main__":
    unittest.main()

"""Free leaning in every stage, quadratic shaping and the independent fall limit."""

import math
import unittest
from pathlib import Path

import numpy as np
import torch
from gymnasium import Env
from omegaconf import DictConfig, OmegaConf

from riktigpatric.patrick import Actions, Observable
from sim.curriculum import STAGES, Curriculum
from sim.envs.rp_env import GymRP
from sim.rewards import FALL_PITCH_LIMIT, pitch_reward, validate_pitch_deadband
from sim.train_agent import make_agent
from sim.utils import SingleEnvWrapper, register_and_make_env


def config():
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    cfg.train.device = "cpu"
    cfg.policy.hsize = 4
    return cfg


class PitchRewardTests(unittest.TestCase):
    def test_free_region_is_symmetric_and_quadratic_cost_matches_at_fall_limit(self):
        deadband = math.radians(5)
        scale = -2 / FALL_PITCH_LIMIT
        for angle, expected in ((0, 0), (2, 0), (5, 0), (10, -2 / 9), (20, -2)):
            for sign in (-1, 1):
                self.assertAlmostEqual(
                    pitch_reward(sign * math.radians(angle), scale, deadband), expected
                )
        # null retains linear shaping; zero selects quadratic shaping from zero.
        self.assertAlmostEqual(pitch_reward(math.radians(10), scale, None), -1)
        self.assertAlmostEqual(pitch_reward(math.radians(10), scale, 0), -0.5)
        self.assertTrue(math.isnan(pitch_reward(math.nan, scale, deadband)))

    def test_invalid_deadbands_fail_without_replacing_valid_configuration(self):
        env = GymRP(actions=[Actions.ACC_BOTH_WHEELS], pitch_deadband=0.05)
        self.addCleanup(env.close)
        cfg = config()
        for value in (-0.1, float("nan"), float("inf"), FALL_PITCH_LIMIT, 1.0):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    env.pitch_deadband = value
                self.assertEqual(env.pitch_deadband, 0.05)
                cfg.curriculum.pitch_deadband = value
                with self.assertRaises(ValueError):
                    Curriculum(cfg)
        validate_pitch_deadband(None)
        validate_pitch_deadband(0)

    def test_stage_deadbands_and_independent_true_pitch_termination(self):
        for count in (1, 2):
            cfg = config()
            cfg.env.n_parallel = count
            curriculum = Curriculum(cfg)
            initial_state = curriculum.state_dict()
            agent = make_agent(cfg)
            optimizer = torch.optim.Adam(agent.parameters())
            env = register_and_make_env(cfg)
            self.addCleanup(env.close)
            if isinstance(env, Env):
                raw_envs = [env.unwrapped]
                wrapped = SingleEnvWrapper(env)
            else:
                raw_envs = [item.unwrapped for item in env.envs]
                wrapped = env
            for stage in STAGES:
                curriculum.prepare_rollout(wrapped, agent, evaluation=True)
                wrapped.reset(seed=1)
                for raw in raw_envs:
                    assert isinstance(raw, GymRP)
                    self.assertEqual(raw.pitch_deadband, cfg.curriculum.pitch_deadband)
                    raw.state.obs.set_observable(
                        Observable.RP_PITCH, np.array([math.radians(3)])
                    )
                    rewards = raw._calculate_rewards(raw.state, terminated=False)
                    self.assertEqual(rewards[Observable.REWARD_RP_PITCH], 0)
                    self.assertAlmostEqual(
                        rewards[Observable.REWARD_TOTAL],
                        sum(v for k, v in rewards.items()
                            if k != Observable.REWARD_TOTAL),
                    )
                    for sign in (-1, 1):
                        for angle, fallen in ((19, False), (21, True)):
                            pitch = sign * math.radians(angle)
                            raw.dm_env.data.qpos[3:7] = [
                                math.cos(pitch / 2), 0, math.sin(pitch / 2), 0
                            ]
                            raw.dm_env.forward()
                            self.assertEqual(raw.terminated, fallen)
                if stage != "full_control":
                    curriculum.advance(agent, optimizer)
            # Restoring position hold keeps the same free-leaning allowance.
            curriculum.load_state_dict(initial_state)
            curriculum.prepare_rollout(wrapped, agent, evaluation=True)
            for raw in raw_envs:
                assert isinstance(raw, GymRP)
                self.assertEqual(raw.pitch_deadband, cfg.curriculum.pitch_deadband)
            # Explicit null still restores legacy linear shaping in reused environments.
            cfg.curriculum.pitch_deadband = None
            Curriculum(cfg).prepare_rollout(wrapped, agent, evaluation=True)
            for raw in raw_envs:
                assert isinstance(raw, GymRP)
                self.assertIsNone(raw.pitch_deadband)


if __name__ == "__main__":
    unittest.main()

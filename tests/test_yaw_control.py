"""Differential-wheel steering, body yaw-rate references, and policy integration."""

import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch
from gymnasium import Env
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, DerivedObs, Observable, State, Target
from riktigpatric.wheel_control import wheel_velocity_targets
from sim.envs.rp_env import GymRP
from sim.episode_io import save_episode_csv
from sim.train_agent import (
    add_targets,
    add_yaw_targets,
    get_plot_keys,
    get_policy_inputs,
    rollout,
    training_update,
)
from sim.utils import SingleEnvWrapper, get_returns, npd2tensord, register_and_make_env

WHEEL_ACTIONS: list[str] = [Actions.ACC_BOTH_WHEELS, Actions.VEL_WHEEL_DIFF]


def config():
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    cfg.env.n_parallel = 3
    cfg.env.max_episode_steps = 4
    cfg.train.tbptt_steps = 2
    return cfg


class YawControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_mixing_preserves_mean_and_prioritizes_balance_at_speed_limits(self):
        def mix(measured, acceleration, difference):
            return wheel_velocity_targets(
                np.array(measured), acceleration, 0.01, 10.0, 50.0, difference
            )

        # Acceleration is clipped to 50; steering replaces the measured difference.
        np.testing.assert_allclose(mix([2, 4], 100, 4), [1.5, 5.5])
        np.testing.assert_allclose(mix([2, 4], 100, 0), [3.5, 3.5])
        np.testing.assert_allclose(mix([2, 4], 100, None), [2.5, 4.5])
        for sign in (-1, 1):
            result = mix([sign * 9, sign * 9], 0, 8)
            self.assertAlmostEqual(result.mean(), sign * 9)
            self.assertAlmostEqual(result[1] - result[0], 2)
            self.assertTrue((abs(result) <= 10).all())
            np.testing.assert_allclose(
                mix([sign * 12, sign * 12], 0, 8), [sign * 10, sign * 10]
            )
        # An absolute velocity difference must not accumulate on every step.
        result = mix([0, 0], 0, 4)
        np.testing.assert_allclose(mix(result, 0, 4), result)
        np.testing.assert_allclose(mix([0, 0], 0, -4), -result)

    def test_action_order_does_not_change_mixing_or_head_commands(self):
        env = GymRP(actions=[*WHEEL_ACTIONS, Actions.VEL_HEAD_TURN])
        self.addCleanup(env.close)
        env.reset(seed=1)
        env.state.obs.set_observable(Observable.LEFT_WHEEL_VEL, np.array([1.0]))
        env.state.obs.set_observable(Observable.RIGHT_WHEEL_VEL, np.array([3.0]))
        commands = {
            Actions.ACC_BOTH_WHEELS: np.array([20.0]),
            Actions.VEL_WHEEL_DIFF: np.array([4.0]),
            Actions.VEL_HEAD_TURN: np.array([-0.3]),
        }
        for actions in (commands, dict(reversed(list(commands.items())))):
            env._apply_actions(actions)
            self.assertAlmostEqual(float(env.dm_env.bind(env.left_wheel_act).ctrl), 0.2)
            self.assertAlmostEqual(
                float(env.dm_env.bind(env.right_wheel_act).ctrl), 4.2
            )
            self.assertAlmostEqual(float(env.dm_env.bind(env.head_turn_act).ctrl), -0.3)
        # Merely changing the target cannot command the wheels.
        before = env.dm_env.data.ctrl.copy()
        env.target_yaw_rate = 1.0
        np.testing.assert_array_equal(env.dm_env.data.ctrl, before)

    def test_conflicting_wheel_commands_are_rejected_before_actuation(self):
        with self.assertRaisesRegex(ValueError, "requires accelerate_both_wheels"):
            GymRP(actions=[Actions.VEL_WHEEL_DIFF])
        with self.assertRaisesRegex(ValueError, "requires the wheel_vel_diff"):
            GymRP(actions=[Actions.ACC_BOTH_WHEELS], yaw_tracking=True)
        env = GymRP(actions=WHEEL_ACTIONS)
        self.addCleanup(env.close)
        env.reset(seed=1)
        before = env.dm_env.data.ctrl.copy()
        for conflicting in (
            Actions.ACC_LEFT_WHEEL, Actions.ACC_RIGHT_WHEEL,
            Actions.VEL_LEFT_WHEEL, Actions.VEL_RIGHT_WHEEL, Actions.ACC_YAW_TURN,
        ):
            with self.subTest(action=conflicting):
                with self.assertRaisesRegex(ValueError, "cannot be combined"):
                    env._apply_actions({
                        Actions.ACC_BOTH_WHEELS: np.array([1.0]),
                        Actions.VEL_WHEEL_DIFF: np.array([2.0]),
                        conflicting: np.array([3.0]),
                    })
                np.testing.assert_array_equal(env.dm_env.data.ctrl, before)

    def test_positive_difference_turns_left_and_reward_uses_post_step_gyro(self):
        env = GymRP(
            actions=WHEEL_ACTIONS, yaw_tracking=True, target_yaw_rate=0.4,
            reward_scales={Observable.REWARD_YAW_RATE: -2.0,
                           Observable.REWARD_TOTAL: 1.0},
        )
        self.addCleanup(env.close)
        for sign in (-1, 1):
            env.reset(seed=1)
            for _ in range(30):
                obs, reward, terminated, _, _ = env.step({
                    Actions.ACC_BOTH_WHEELS: np.array([0.0]),
                    Actions.VEL_WHEEL_DIFF: np.array([sign * 2.0]),
                })
                rate = obs[DerivedObs.YAW_RATE][0]
                self.assertAlmostEqual(rate, obs[Observable.GYRO][2])
                self.assertAlmostEqual(reward, -2 * abs(rate - 0.4), places=5)
                self.assertTrue(env.observation_space.contains(obs))
                self.assertFalse(terminated)
            self.assertGreater(sign * obs[DerivedObs.YAW_RATE][0], 0.01)

    def test_shared_sensor_state_uses_gyro_not_wheel_slip_and_preserves_target(self):
        state = State(wheel_radius=0.05)
        state.target_yaw_rate = -0.5
        packet = {
            Observable.OBS_TIME: np.array([0.0]),
            Observable.ACC: np.array([0.0, 0.0, 9.81]),
            Observable.GYRO: np.array([0.1, 0.2, 0.3]),
            Observable.LEFT_WHEEL_VEL: np.array([-8.0]),
            Observable.RIGHT_WHEEL_VEL: np.array([8.0]),
        }
        state.reset(packet)
        self.assertAlmostEqual(state.snapshot()[DerivedObs.YAW_RATE][0], 0.3)
        packet[Observable.OBS_TIME][:] = 0.01
        packet[Observable.GYRO][2] = -0.2
        state.update(packet)
        packet[Observable.GYRO][2] = 999
        self.assertAlmostEqual(state.snapshot()[DerivedObs.YAW_RATE][0], -0.2)
        state.reset()
        self.assertEqual(state.snapshot()[DerivedObs.YAW_RATE][0], 0.0)
        self.assertEqual(state.target_yaw_rate, -0.5)
        for invalid in (float("nan"), float("inf"), 1e100):
            with self.assertRaises(ValueError):
                state.target_yaw_rate = invalid
            self.assertEqual(state.target_yaw_rate, -0.5)

    def test_batched_targets_reach_policy_and_can_change_live_or_survive_reset(self):
        cfg = config()
        for count in (1, 3):
            cfg.env.n_parallel = count
            env = register_and_make_env(cfg)
            if isinstance(env, Env):
                env = SingleEnvWrapper(env)
            self.addCleanup(env.close)
            agent = Agent(get_policy_inputs(cfg), cfg.policy.actions)
            self.assertEqual(len(agent.action_heads), 4)
            obs, _ = env.reset(seed=1)
            obs = add_targets(obs, env, target_velocities=[0.1] * count)
            targets = np.linspace(-0.5, 0.5, count)
            seen = []
            hook = agent.encoders[Target.YAW_RATE.value].register_forward_pre_hook(
                lambda _module, args: seen.append(args[0].clone())
            )
            self.addCleanup(hook.remove)
            for values in (targets, -targets):
                time = obs[Observable.OBS_TIME].copy()
                obs = add_yaw_targets(obs, env, values)
                np.testing.assert_array_equal(obs[Observable.OBS_TIME], time)
                np.testing.assert_allclose(obs[Target.TARGET_VEL], 0.1)
                with torch.no_grad():
                    actions, _, _, _ = agent.act(npd2tensord(obs))
                self.assertEqual(
                    tuple(actions[Actions.VEL_WHEEL_DIFF].shape), (count, 1)
                )
                np.testing.assert_allclose(seen[-1].numpy()[:, 0], values)
            for invalid in ([np.nan] * count, [1e100] * count, [0.] * (count + 1)):
                with self.assertRaises(ValueError):
                    add_yaw_targets(obs, env, invalid)
            obs, _ = env.reset(seed=2)
            np.testing.assert_allclose(obs[Target.YAW_RATE][:, 0], -targets)

    def test_rollout_csv_and_training_include_yaw_actions_targets_and_gradients(self):
        cfg = config()
        env = register_and_make_env(cfg)
        if isinstance(env, Env):
            env = SingleEnvWrapper(env)
        self.addCleanup(env.close)
        agent = Agent(get_policy_inputs(cfg), cfg.policy.actions)
        targets = [-0.5, 0.0, 0.5]
        with torch.no_grad():
            buffers = rollout(env, agent, target_yaw_rates=targets)
        for buffer, target in zip(buffers, targets):
            np.testing.assert_allclose(buffer.get_observable(Target.YAW_RATE), target)
            np.testing.assert_allclose(
                buffer.get_observable(Observable.REWARD_YAW_RATE)[1:, 0],
                -abs(buffer.get_observable(DerivedObs.YAW_RATE)[1:, 0] - target),
                atol=1e-7,
            )
        with tempfile.TemporaryDirectory() as folder:
            returns = get_returns(buffers[0].get_rewards(), cfg.rl.discount)
            path = save_episode_csv(
                buffers[0], Path(folder) / "yaw.csv", returns=returns.numpy(),
                advantages=(returns - buffers[0].get_values()).numpy(),
            )
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertIn(Actions.VEL_WHEEL_DIFF.value, rows[0])
            self.assertEqual(float(rows[0][Target.YAW_RATE.value]), targets[0])
            self.assertEqual(rows[-1][Actions.VEL_WHEEL_DIFF.value], "")
        cfg.train.target_yaw_rates = targets
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        metrics = training_update(cfg, env, agent, optimizer, 0)
        head = agent.action_heads[Actions.VEL_WHEEL_DIFF.value]
        assert isinstance(head, torch.nn.Linear)
        grad = head.weight.grad
        self.assertIsNotNone(grad)
        assert grad is not None
        self.assertTrue(torch.isfinite(grad).all())
        self.assertGreater(grad.abs().sum().item(), 0)
        self.assertTrue(all(np.isfinite(value) for value in metrics.values()))

    def test_yaw_tracking_can_be_disabled_independently(self):
        cfg = config()
        for enabled in (False, True):
            cfg.env.yaw_tracking = enabled
            inputs = get_policy_inputs(cfg)
            agent = Agent(inputs, cfg.policy.actions)
            plotted = {key for _, row in get_plot_keys(cfg, agent) for key in row}
            self.assertEqual(Target.YAW_RATE.value in inputs, enabled)
            self.assertEqual(Observable.REWARD_YAW_RATE in plotted, enabled)
            env = GymRP(
                actions=WHEEL_ACTIONS, yaw_tracking=enabled, target_yaw_rate=0.5,
                reward_scales={Observable.REWARD_YAW_RATE: -2.0},
            )
            self.addCleanup(env.close)
            env.reset(seed=1)
            rewards = env._calculate_rewards(env.state, terminated=False)
            self.assertAlmostEqual(
                rewards[Observable.REWARD_YAW_RATE], -1. if enabled else 0.
            )


if __name__ == "__main__":
    unittest.main()

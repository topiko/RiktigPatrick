"""Regression checks for the maintained MuJoCo training path."""

import unittest
from functools import partial
from pathlib import Path

import numpy as np
import torch
from gymnasium import Env
from gymnasium.vector import SyncVectorEnv
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, DerivedObs, Observable, State, Target
from riktigpatric.trajectory import PositionTrajectory
from sim.envs.rp_env import GymRP
from sim.train_agent import add_targets, get_policy_inputs, rollout
from sim.utils import (
    EpisodeBuffer,
    SingleEnvWrapper,
    ebufs2batchd,
    get_advantages,
    get_returns,
    npd2tensord,
    register_and_make_env,
)


def config() -> DictConfig:
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    return cfg


def single_env(steps: int, tracking_mode: str = "velocity") -> Env:
    cfg = config()
    cfg.env.max_episode_steps = steps
    cfg.env.tracking_mode = tracking_mode
    env = register_and_make_env(cfg, force_single_env=True)
    assert isinstance(env, Env)
    return env


class ActiveSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_odometry_is_in_meters_and_filter_is_in_radians(self):
        state = State(wheel_radius=0.05)
        state.reset()
        state.targets[Target.TARGET_POS] = np.array([0.5])
        state.update({
            Observable.OBS_TIME: np.array([0.1]),
            Observable.ACC: np.array([0.0, 0.0, 9.81]),
            Observable.GYRO: np.array([0.0, 0.4, 0.0]),
            Observable.LEFT_WHEEL_VEL: np.array([2.0]),
            Observable.RIGHT_WHEEL_VEL: np.array([4.0]),
        })
        self.assertAlmostEqual(state.derived_obs[DerivedObs.CURRENT_POS][0], 0.015)
        self.assertAlmostEqual(state.derived_obs[DerivedObs.CURRENT_VEL][0], 0.15)
        self.assertAlmostEqual(state.euler[1], 0.04, places=4)
        np.testing.assert_allclose(
            state.obs.get_observable(Observable.RP_PITCH), [state.euler[1]]
        )
        np.testing.assert_array_equal(state.targets[Target.TARGET_POS], [0.5])
        state.reset()
        np.testing.assert_array_equal(state.derived_obs[DerivedObs.CURRENT_POS], [0.0])
        np.testing.assert_array_equal(state.targets[Target.TARGET_POS], [0.5])

    def test_environment_observations_rewards_and_history_are_current(self):
        env = GymRP(
            actions=[Actions.ACC_BOTH_WHEELS], record=True, target_pos=0.5,
            tracking_mode="position",
            reward_scales={
                Observable.REWARD_STEP: 1.0,
                Observable.REWARD_RP_PITCH: -2.0,
                Observable.REWARD_WHEEL_VEL: -0.2,
                Observable.REWARD_POS: -1.0,
                Observable.REWARD_TOTAL: 1.0,
            },
        )
        self.addCleanup(env.close)
        initial, _ = env.reset(seed=1)
        self.assertTrue(env.observation_space.contains(initial))
        for i in range(10):
            obs, reward, terminated, truncated, _ = env.step(
                {Actions.ACC_BOTH_WHEELS: np.array([50.0])}
            )
            self.assertTrue(env.observation_space.contains(obs))
            self.assertIsInstance(terminated, bool)
            self.assertIsInstance(truncated, bool)
            self.assertAlmostEqual(obs[Observable.OBS_TIME][0], (i + 1) * 0.01)
            self.assertAlmostEqual(
                obs[Observable.RP_PITCH][0], env.state.euler[1]
            )
            self.assertAlmostEqual(
                obs[Observable.REWARD_RP_PITCH][0],
                -2.0 * abs(obs[Observable.RP_PITCH][0]),
            )
            expected = (
                1.0 - 2.0 * abs(obs[Observable.RP_PITCH][0])
                - 0.1 * (
                    abs(obs[Observable.LEFT_WHEEL_VEL][0])
                    + abs(obs[Observable.RIGHT_WHEEL_VEL][0])
                )
                - abs(obs[DerivedObs.CURRENT_POS][0] - 0.5)
            )
            self.assertAlmostEqual(reward, expected, places=6)
        np.testing.assert_array_equal(initial[DerivedObs.CURRENT_POS], [0.0])
        history, indices = env.state.history
        self.assertEqual(history.shape[0], 10)
        self.assertAlmostEqual(history[-1, indices[Observable.OBS_TIME]], 0.1)
        self.assertAlmostEqual(history[-1, indices[Observable.REWARD_TOTAL]], reward)

    def test_true_pitch_uses_radians_and_fall_limit_is_twenty_degrees(self):
        env = GymRP(actions=[Actions.ACC_BOTH_WHEELS])
        self.addCleanup(env.close)
        env.reset(seed=1)
        for angle, fallen in [(0.2, False), (0.4, True)]:
            env.dm_env.data.qpos[3:7] = [np.cos(angle / 2), 0, np.sin(angle / 2), 0]
            env.dm_env.forward()
            measurements = env._read_sensors()
            self.assertAlmostEqual(measurements[Observable.TRUE_PITCH][0], angle)
            self.assertEqual(env.terminated, fallen)

    def test_distinct_targets_survive_steps_and_resets(self):
        for count in (1, 3):
            with self.subTest(count=count):
                cfg = config()
                cfg.env.n_parallel = count
                cfg.env.tracking_mode = "position"
                cfg.env.max_episode_steps = 4
                env = register_and_make_env(cfg)
                if isinstance(env, Env):
                    env = SingleEnvWrapper(env)
                self.addCleanup(env.close)
                obs, _ = env.reset(seed=1)
                targets = np.arange(count, dtype=np.float32) * 0.2 + 0.1
                obs = add_targets(obs, env, targets)
                self.assertEqual(obs[Target.TARGET_POS].shape, (count, 1))
                actions = {
                    Actions.ACC_BOTH_WHEELS: np.zeros((count, 1), dtype=np.float32),
                    Actions.VEL_HEAD_PITCH: np.zeros((count, 1), dtype=np.float32),
                }
                obs, _, _, _, _ = env.step(actions)
                np.testing.assert_allclose(obs[Target.TARGET_POS][:, 0], targets)
                np.testing.assert_allclose(
                    obs[Observable.REWARD_POS][:, 0],
                    -abs(obs[DerivedObs.CURRENT_POS][:, 0] - targets),
                )
                obs, _ = env.reset(seed=2)
                np.testing.assert_allclose(obs[Target.TARGET_POS][:, 0], targets)
                for invalid in ([0.0] * (count + 1), [np.nan] * count):
                    with self.assertRaises(ValueError):
                        add_targets(obs, env, invalid)

    def test_rollout_handles_different_episode_lengths_and_backpropagation(self):
        env = SyncVectorEnv([partial(single_env, 1), partial(single_env, 4)])
        self.addCleanup(env.close)
        cfg = config()
        agent = Agent(get_policy_inputs(cfg), cfg.policy.actions)
        buffers = rollout(env, agent, seed=1, target_velocities=[0.1, -0.2])
        logps, rewards, values, lengths, mask = ebufs2batchd(buffers)
        np.testing.assert_array_equal(lengths, [1, 4])
        for buffer, target in zip(buffers, [0.1, -0.2]):
            np.testing.assert_allclose(buffer.get_observable(Target.TARGET_VEL), target)
            np.testing.assert_allclose(
                buffer.get_rewards().numpy(),
                buffer.get_observable(Observable.REWARD_TOTAL)[1:, 0],
                rtol=1e-6,
            )
        returns = get_returns(rewards, discount=cfg.rl.discount)
        loss = (
            (-logps * get_advantages(returns, values) + (values - returns).square())
            * mask
        ).sum() / mask.sum()
        loss.backward()
        for parameter in agent.parameters():
            self.assertIsNotNone(parameter.grad)
            assert parameter.grad is not None
            self.assertTrue(torch.isfinite(parameter.grad).all())

    def test_policy_loss_does_not_train_the_value_baseline(self):
        values = torch.tensor([[2.0, 3.0]], requires_grad=True)
        logps = torch.tensor([[-0.4, -0.5]], requires_grad=True)
        returns = torch.tensor([[5.0, 7.0]])
        advantages = get_advantages(returns, values)
        (-logps * advantages).sum().backward()
        self.assertIsNone(values.grad)
        torch.testing.assert_close(logps.grad, torch.tensor([[-3.0, -4.0]]))

    def test_returns_include_first_reward_and_terminal_reward_exactly_once(self):
        buffer = EpisodeBuffer()
        for i, reward in enumerate([1.0, 2.0, 4.0]):
            buffer.add_step(
                {Observable.OBS_TIME: np.array([float(i)])},
                {Actions.TIME: np.array([float(i)])},
                reward, torch.tensor(0.0), torch.tensor(0.0),
            )
        buffer.finish({Observable.OBS_TIME: np.array([3.0])})
        torch.testing.assert_close(buffer.get_rewards(), torch.tensor([1.0, 2.0, 4.0]))
        torch.testing.assert_close(
            get_returns(buffer.get_rewards(), discount=1.0),
            torch.tensor([7.0, 6.0, 4.0]),
        )

    def test_trajectory_interpolation_holds_endpoints_and_copies_input(self):
        points = [[0.0, 0.1], [0.2, 0.3], [1.0, -0.1]]
        trajectory = PositionTrajectory(points)
        points[0][1] = 99.0
        for time, position in [(0, 0.1), (0.1, 0.2), (0.6, 0.1), (2.0, -0.1)]:
            self.assertAlmostEqual(trajectory.position_at(time), position)
        self.assertEqual(PositionTrajectory([[0, 0.5]]).position_at(100), 0.5)

    def test_invalid_trajectories_are_rejected(self):
        for points in (
            [], [[0]], [[1, 0]], [[0, 0], [0, 1]], [[0, 0], [-1, 1]],
            [[0, np.nan]], [[0, 0], [np.inf, 1]], [[0, 1e100]],
        ):
            with self.subTest(points=points), self.assertRaises(ValueError):
                PositionTrajectory(points)
        trajectory = PositionTrajectory([[0, 0]])
        for time in (-1, np.nan, np.inf):
            with self.subTest(time=time), self.assertRaises(ValueError):
                trajectory.position_at(time)

    def test_shared_state_restarts_trajectory_and_fixed_target_replaces_it(self):
        state = State(wheel_radius=0.05)
        state.target_trajectory = PositionTrajectory([[0, 0.2], [0.1, -0.2]])
        measurements = {
            Observable.OBS_TIME: np.array([0.1]),
            Observable.ACC: np.array([0.0, 0.0, 9.81]),
            Observable.GYRO: np.zeros(3),
            Observable.LEFT_WHEEL_VEL: np.zeros(1),
            Observable.RIGHT_WHEEL_VEL: np.zeros(1),
        }
        state.update(measurements)
        self.assertAlmostEqual(state.target_pos, -0.2)
        state.reset()
        self.assertAlmostEqual(state.target_pos, 0.2)
        state.target_pos = 0.6
        self.assertIsNone(state.target_trajectory)
        state.update(measurements)
        state.reset()
        self.assertAlmostEqual(state.target_pos, 0.6)
        with self.assertRaises(ValueError):
            state.target_pos = 1e100
        self.assertAlmostEqual(state.target_pos, 0.6)

    def test_batched_trajectories_align_policy_targets_and_rewards(self):
        env = SyncVectorEnv([
            partial(single_env, 2, "position"), partial(single_env, 5, "position")
        ])
        self.addCleanup(env.close)
        cfg = config()
        cfg.env.tracking_mode = "position"
        agent = Agent(get_policy_inputs(cfg), cfg.policy.actions)
        trajectories = [
            [[0, 0.1], [0.03, 0.4]],
            [[0, -0.2], [0.04, 0.2]],
        ]
        buffers = rollout(env, agent, seed=2, target_trajectories=trajectories)
        self.assertEqual([buffer.seq_len for buffer in buffers], [2, 5])
        for buffer, start, end in zip(buffers, [0.1, -0.2], [0.4, 0.2]):
            times = buffer.get_observable(Observable.OBS_TIME)[:, 0]
            targets = np.minimum(start + 10 * times, end)
            np.testing.assert_allclose(
                buffer.get_observable(Target.TARGET_POS)[:, 0], targets, atol=1e-7
            )
            positions = buffer.get_observable(DerivedObs.CURRENT_POS)[:, 0]
            np.testing.assert_allclose(
                buffer.get_observable(Observable.REWARD_POS)[1:, 0],
                -np.abs(positions[1:] - targets[1:]), atol=1e-7,
            )
        obs, _ = env.reset(seed=2)
        np.testing.assert_allclose(obs[Target.TARGET_POS][:, 0], [0.1, -0.2])
        with self.assertRaises(ValueError):
            add_targets(obs, env, target_trajectories=[trajectories[0], [[1, 0]]])
        with self.assertRaises(ValueError):
            add_targets(obs, env, [0, 0], trajectories)
        with self.assertRaises(ValueError):
            add_targets(obs, env, target_trajectories=[trajectories[0]])
        obs, _ = env.reset(seed=2)
        np.testing.assert_allclose(obs[Target.TARGET_POS][:, 0], [0.1, -0.2])

    def test_single_env_trajectory_can_be_replaced_mid_episode(self):
        env = SingleEnvWrapper(single_env(10, "position"))
        self.addCleanup(env.close)
        obs, _ = env.reset(seed=1)
        actions = {
            Actions.ACC_BOTH_WHEELS: np.zeros((1, 1), dtype=np.float32),
            Actions.VEL_HEAD_PITCH: np.zeros((1, 1), dtype=np.float32),
        }
        for _ in range(2):
            obs, _, _, _, _ = env.step(actions)
        obs = add_targets(obs, env, target_trajectories=[[[0, 0], [0.1, 1]]])
        np.testing.assert_allclose(obs[Target.TARGET_POS], [[0.2]])
        previous_target = obs[Target.TARGET_POS]
        obs, _, _, _, _ = env.step(actions)
        np.testing.assert_allclose(obs[Target.TARGET_POS], [[0.3]])
        np.testing.assert_allclose(previous_target, [[0.2]])
        obs = add_targets(obs, env, target_positions=[-0.5])
        obs, _, _, _, _ = env.step(actions)
        np.testing.assert_allclose(obs[Target.TARGET_POS], [[-0.5]])
        obs, _ = env.reset(seed=1)
        np.testing.assert_allclose(obs[Target.TARGET_POS], [[-0.5]])

    def test_sensor_reads_and_reward_calculation_do_not_update_state(self):
        env = GymRP(
            actions=[Actions.ACC_BOTH_WHEELS], record=True,
            tracking_mode="position",
            reward_scales={Observable.REWARD_POS: -1.0, Observable.REWARD_TOTAL: 1.0},
        )
        self.addCleanup(env.close)
        initial, _ = env.reset(seed=1)
        state_before = env.state.snapshot()
        measurements = env._read_sensors()
        self.assertNotIn(Observable.RP_PITCH, measurements)
        self.assertNotIn(Observable.REWARD_TOTAL, measurements)

        other_state = State(wheel_radius=0.05)
        other_state.reset(measurements)
        other_state.target_pos = 2.0
        rewards = env._calculate_rewards(other_state, terminated=False)
        self.assertEqual(rewards[Observable.REWARD_POS], -2.0)
        self.assertEqual(other_state.target_pos, 2.0)
        for key, values in state_before.items():
            np.testing.assert_array_equal(env.state.snapshot()[key], values)
        self.assertNotIn(Observable.REWARD_TOTAL, env.state.snapshot())
        self.assertEqual(initial[Observable.REWARD_TOTAL][0], 0.0)
        with self.assertRaises(KeyError):
            _ = env.state.history  # Neither reset, acquisition nor rewards record.

    def test_shared_state_can_replay_sensor_packets_without_simulation_fields(self):
        env = GymRP(
            actions=[Actions.ACC_BOTH_WHEELS],
            target_trajectory=[[0, 0.1], [0.1, -0.1]],
        )
        self.addCleanup(env.close)
        env.reset(seed=1)
        replay = State(wheel_radius=0.05)
        replay.target_trajectory = env.target_trajectory

        def hardware_packet():
            packet = env._read_sensors()
            del packet[Observable.TRUE_PITCH]
            return packet

        replay.reset(hardware_packet())
        for _ in range(10):
            obs, _, _, _, _ = env.step({Actions.ACC_BOTH_WHEELS: np.array([50.0])})
            packet = hardware_packet()
            replay.update(packet)
            snapshot = replay.snapshot()
            self.assertNotIn(Observable.TRUE_PITCH, snapshot)
            self.assertNotIn(Observable.REWARD_TOTAL, snapshot)
            for key, values in snapshot.items():
                np.testing.assert_allclose(values, obs[key], rtol=0, atol=1e-7)
            packet[Observable.ACC][:] = 123.0  # A backend may reuse its buffers.
            np.testing.assert_array_equal(
                replay.snapshot()[Observable.ACC], obs[Observable.ACC]
            )

    def test_recording_is_explicit_and_does_not_mutate_actions(self):
        state = State(wheel_radius=0.05, record=True)
        state.update({
            Observable.OBS_TIME: np.array([0.1]),
            Observable.ACC: np.array([0.0, 0.0, 9.81]),
            Observable.GYRO: np.zeros(3),
            Observable.LEFT_WHEEL_VEL: np.zeros(1),
            Observable.RIGHT_WHEEL_VEL: np.zeros(1),
        })
        with self.assertRaises(KeyError):
            _ = state.history
        action = {Actions.ACC_BOTH_WHEELS: np.array([10.0])}
        state.record_transition(0.0, action, {Observable.REWARD_TOTAL: 2.0})
        self.assertNotIn(Actions.TIME, action)
        self.assertNotIn(Observable.REWARD_TOTAL, state.snapshot())
        action[Actions.ACC_BOTH_WHEELS][:] = 999.0
        history, indices = state.history
        self.assertEqual(history.shape[0], 1)
        self.assertEqual(history[0, indices[Actions.ACC_BOTH_WHEELS]], 10.0)
        self.assertEqual(history[0, indices[Actions.TIME]], 0.0)
        self.assertEqual(history[0, indices[Observable.OBS_TIME]], 0.1)
        self.assertEqual(history[0, indices[Observable.REWARD_TOTAL]], 2.0)
        state.reset()
        with self.assertRaises(KeyError):
            _ = state.history
        self.assertNotIn(Observable.ACC, state.snapshot())

    def test_repeated_measurement_time_does_not_partially_update_state(self):
        state = State(wheel_radius=0.05)
        packet = {
            Observable.OBS_TIME: np.array([0.1]),
            Observable.ACC: np.array([0.0, 0.0, 9.81]),
            Observable.GYRO: np.zeros(3),
            Observable.LEFT_WHEEL_VEL: np.ones(1),
            Observable.RIGHT_WHEEL_VEL: np.ones(1),
        }
        state.update(packet)
        before = state.snapshot()
        for time in (0.1, 0.0, np.nan):
            packet[Observable.OBS_TIME][:] = time
            with self.assertRaises(ValueError):
                state.update(packet)
            for key, values in before.items():
                np.testing.assert_array_equal(state.snapshot()[key], values)

    def test_zero_velocity_is_default_and_policy_inputs_follow_the_task(self):
        cfg = config()
        self.assertEqual(cfg.env.tracking_mode, "velocity")
        self.assertEqual(cfg.env.target_vel, 0.0)
        self.assertIsNone(cfg.train.target_velocities)
        self.assertIsNone(cfg.train.target_positions)
        self.assertIsNone(cfg.train.target_trajectories)
        self.assertIsNone(cfg.env.target_trajectory)
        for mode, expected in (
            ("velocity", {Target.TARGET_VEL.value, DerivedObs.CURRENT_VEL.value}),
            ("position", {Target.TARGET_POS.value, DerivedObs.CURRENT_POS.value}),
            ("none", set()),
        ):
            cfg.env.tracking_mode = mode
            inputs = get_policy_inputs(cfg)
            self.assertEqual(set(inputs) - set(cfg.policy.inputs), expected)

    def test_tracking_rewards_are_exclusive_and_velocity_does_not_penalize_motion(self):
        state = State(wheel_radius=0.05)
        state.reset({
            Observable.OBS_TIME: np.array([0.0]),
            Observable.LEFT_WHEEL_VEL: np.array([2.0]),
            Observable.RIGHT_WHEEL_VEL: np.array([2.0]),
            Observable.HEAD_PITCH: np.array([0.0]),
        })
        state.derived_obs[DerivedObs.CURRENT_POS][:] = 5.0
        state.target_vel = 0.1
        for mode, expected_pos, expected_wheel in (
            ("velocity", 0.0, 0.0), ("position", -5.0, -0.4), ("none", 0.0, -0.4)
        ):
            env = GymRP(
                actions=[Actions.ACC_BOTH_WHEELS], tracking_mode=mode,
                reward_scales={
                    Observable.REWARD_STEP: 1.0, Observable.REWARD_POS: -1.0,
                    Observable.REWARD_VEL: -4.0, Observable.REWARD_WHEEL_VEL: -0.2,
                    Observable.REWARD_TOTAL: 1.0,
                },
            )
            self.addCleanup(env.close)
            rewards = env._calculate_rewards(state, terminated=False)
            self.assertAlmostEqual(rewards[Observable.REWARD_POS], expected_pos)
            self.assertAlmostEqual(rewards[Observable.REWARD_WHEEL_VEL], expected_wheel)
            self.assertAlmostEqual(rewards[Observable.REWARD_VEL], 0.0, places=7)
            self.assertAlmostEqual(
                rewards[Observable.REWARD_TOTAL], 1.0 + expected_pos + expected_wheel
            )
            if mode == "velocity":
                state.target_vel = -0.1
                rewards = env._calculate_rewards(state, terminated=False)
                self.assertAlmostEqual(rewards[Observable.REWARD_VEL], -0.8, places=7)
                state.target_vel = 0.1

    def test_batched_velocity_setpoints_can_change_without_resetting(self):
        for count in (1, 3):
            cfg = config()
            cfg.env.n_parallel = count
            cfg.env.max_episode_steps = 5
            env = register_and_make_env(cfg)
            if isinstance(env, Env):
                env = SingleEnvWrapper(env)
            self.addCleanup(env.close)
            obs, _ = env.reset(seed=1)
            targets = np.linspace(-0.1, 0.1, count)
            obs = add_targets(obs, env, target_velocities=targets)
            self.assertEqual(obs[Target.TARGET_VEL].shape, (count, 1))
            actions = {
                Actions.ACC_BOTH_WHEELS: np.zeros((count, 1), dtype=np.float32),
                Actions.VEL_HEAD_PITCH: np.zeros((count, 1), dtype=np.float32),
            }
            obs, _, _, _, _ = env.step(actions)
            np.testing.assert_allclose(
                obs[Observable.REWARD_VEL][:, 0],
                -4 * abs(obs[DerivedObs.CURRENT_VEL][:, 0] - targets), atol=1e-7,
            )
            np.testing.assert_array_equal(obs[Observable.REWARD_POS], 0.0)
            np.testing.assert_array_equal(obs[Observable.REWARD_WHEEL_VEL], 0.0)
            time = obs[Observable.OBS_TIME].copy()
            previous = obs[Target.TARGET_VEL]
            updated = targets + 0.2
            obs = add_targets(obs, env, target_velocities=updated)
            np.testing.assert_array_equal(obs[Observable.OBS_TIME], time)
            np.testing.assert_allclose(previous[:, 0], targets)
            obs, _ = env.reset(seed=2)
            np.testing.assert_allclose(obs[Target.TARGET_VEL][:, 0], updated)
            for invalid in ([np.nan] * count, [1e100] * count, [0.0] * (count + 1)):
                with self.assertRaises(ValueError):
                    add_targets(obs, env, target_velocities=invalid)
            with self.assertRaises(ValueError):
                add_targets(
                    obs, env, target_positions=targets, target_velocities=targets
                )

    def test_none_mode_ignores_position_and_velocity_targets(self):
        cfg = config()
        cfg.env.n_parallel = 1
        cfg.env.tracking_mode = "none"
        env = register_and_make_env(cfg)
        assert isinstance(env, Env)
        self.addCleanup(env.close)
        obs, _ = env.reset(seed=1)
        self.assertTrue(env.observation_space.contains(obs))
        base_env = env.unwrapped
        assert isinstance(base_env, GymRP)
        state = base_env.state
        state.target_pos = 10.0
        state.target_vel = -0.2
        obs, _, _, _, _ = env.step({Actions.ACC_BOTH_WHEELS: np.array([0.0])})
        self.assertEqual(obs[Observable.REWARD_POS][0], 0.0)
        self.assertEqual(obs[Observable.REWARD_VEL][0], 0.0)
        self.assertTrue(env.observation_space.contains(obs))
        with self.assertRaises(ValueError):
            GymRP(actions=[Actions.ACC_BOTH_WHEELS], tracking_mode="unknown")

    def test_live_velocity_commands_reach_the_policy_encoder(self):
        cfg = config()
        env = SingleEnvWrapper(single_env(5))
        self.addCleanup(env.close)
        agent = Agent(get_policy_inputs(cfg), cfg.policy.actions)
        self.assertNotIn(Target.TARGET_POS.value, agent.encoders)
        seen = []
        hook = agent.encoders[Target.TARGET_VEL.value].register_forward_pre_hook(
            lambda _module, args: seen.append(args[0].clone())
        )
        self.addCleanup(hook.remove)
        obs, _ = env.reset(seed=1)
        for command in (0.2, -0.1):
            obs = add_targets(obs, env, target_velocities=[command])
            with torch.no_grad():
                agent.act(npd2tensord(obs))
        torch.testing.assert_close(seen[0], torch.tensor([[0.2]]))
        torch.testing.assert_close(seen[1], torch.tensor([[-0.1]]))


if __name__ == "__main__":
    unittest.main()

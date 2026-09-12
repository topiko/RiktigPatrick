"""CAD geometry, camera kinematics and shared head-control contracts."""

import itertools
import unittest
from pathlib import Path

import numpy as np
import torch
from gymnasium import Env
from gymnasium.vector import SyncVectorEnv
from gymnasium.wrappers import TimeLimit
from omegaconf import DictConfig, OmegaConf

from filters.qutils import as_rotation_matrix, eul2q
from nn_ctrl.nns import Agent
from riktigpatric.geometry import (
    CAMERA_TIP,
    HEAD_OFFSET,
    HEAD_PITCH_LIMITS,
    HEAD_YAW_LIMITS,
    NECK_PIVOT,
    camera_elevation,
    camera_rotation,
)
from riktigpatric.patrick import Actions, DerivedObs, Observable, State, Target
from riktigpatric.trajectory import HeadTrajectory, PositionTrajectory
from sim.envs.rp_env import GymRP
from sim.train_agent import (
    add_head_targets,
    add_targets,
    get_plot_keys,
    get_policy_inputs,
    rollout,
)
from sim.utils import (
    Episode,
    EpisodeBuffer,
    SingleEnvWrapper,
    npd2tensord,
    register_and_make_env,
)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.env = GymRP(actions=[Actions.ACC_BOTH_WHEELS])
        self.env.reset(seed=1)
        self.addCleanup(self.env.close)
        self.physics = self.env.dm_env

    def test_camera_mount_wheels_and_head_limits(self):
        physics = self.physics
        site = physics.model.name2id("frame/head_camera_site", "site")
        expected = (
            np.array(NECK_PIVOT) + HEAD_OFFSET + CAMERA_TIP
            + np.array([0, 0, physics.data.qpos[2]])
        )
        np.testing.assert_allclose(physics.data.site_xpos[site], expected, atol=1e-12)
        rotation = physics.data.site_xmat[site].reshape(3, 3)
        np.testing.assert_allclose(-rotation[:, 2], [1, 0, 0], atol=1e-12)
        for wheel in ("rightwheel", "leftwheel"):
            geom = physics.model.name2id(f"frame/{wheel}_cyl", "geom")
            self.assertAlmostEqual(physics.model.geom_size[geom, 0] * 2, 0.100)
        for name, limits in (
            ("headpitch", HEAD_PITCH_LIMITS), ("headturn", HEAD_YAW_LIMITS)
        ):
            joint = physics.model.name2id(f"frame/{name}_joint", "joint")
            actuator = physics.model.name2id(f"frame/{name}_actuator", "actuator")
            self.assertTrue(physics.model.jnt_limited[joint])
            np.testing.assert_allclose(physics.model.jnt_range[joint], limits)
            np.testing.assert_allclose(
                physics.model.actuator_actrange[actuator], limits
            )
        self.assertAlmostEqual(physics.model.body_mass.sum(), 0.64)

    def test_camera_rotation_matches_mujoco_for_combined_body_and_neck_rotations(self):
        physics = self.physics
        site = physics.model.name2id("frame/head_camera_site", "site")
        pitch_joint = physics.model.name2id("frame/headpitch_joint", "joint")
        yaw_joint = physics.model.name2id("frame/headturn_joint", "joint")
        for roll, pitch, yaw, head_pitch, head_yaw in itertools.product(
            [-0.1, 0.1], [-0.25, 0.25], [-1.0, 1.0], [-0.35, 0.4], [-0.6, 0.5]
        ):
            body = eul2q(roll, pitch, yaw)
            physics.data.qpos[3:7] = body.as_array()
            physics.data.qpos[physics.model.jnt_qposadr[pitch_joint]] = head_pitch
            physics.data.qpos[physics.model.jnt_qposadr[yaw_joint]] = head_yaw
            physics.forward()
            expected = camera_rotation(as_rotation_matrix(body), head_pitch, head_yaw)
            actual = physics.data.site_xmat[site].reshape(3, 3)
            np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_planar_level_gaze_and_tilted_neck_yaw_coupling(self):
        body = as_rotation_matrix(eul2q(0, 0.2, 0))  # body pitches nose down
        self.assertAlmostEqual(camera_elevation(camera_rotation(body, 0.2, 0)), 0)
        elevation = camera_elevation(camera_rotation(np.eye(3), 0.3, 0.5))
        self.assertAlmostEqual(elevation, np.arcsin(np.sin(0.3) * np.cos(0.5)))
        self.assertNotAlmostEqual(elevation, 0.3)

    def test_body_proxy_matches_cad_reference_bounds(self):
        physics = self.physics
        geom = physics.model.name2id("frame/body", "geom")
        mesh = physics.model.geom_dataid[geom]
        start = physics.model.mesh_vertadr[mesh]
        count = physics.model.mesh_vertnum[mesh]
        vertices = physics.model.mesh_vert[start:start + count]
        world = (
            vertices @ physics.data.geom_xmat[geom].reshape(3, 3).T
            + physics.data.geom_xpos[geom]
        )
        root_height = physics.data.qpos[2]
        np.testing.assert_allclose(
            world.min(axis=0), [-0.020, -0.050, root_height - 0.025], atol=1e-8
        )
        np.testing.assert_allclose(
            world.max(axis=0), [0.039, 0.050, root_height + 0.181], atol=1e-8
        )


class HeadTrackingTests(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(1)
        cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
        assert isinstance(cfg, DictConfig)
        self.cfg = cfg

    def make_env(self):
        env = GymRP(
            actions=list(self.cfg.policy.actions), head_tracking=True,
            reward_scales={
                Observable.REWARD_STEP: 1.0, Observable.REWARD_TOTAL: 1.0,
                Observable.REWARD_CAMERA_PITCH: -1.0, Observable.REWARD_HEAD_YAW: -1.0,
                Observable.REWARD_HEAD_PITCH: -10.0,
            },
        )
        self.addCleanup(env.close)
        env.reset(seed=1)
        return env

    def test_estimated_camera_pose_needs_no_simulation_truth(self):
        state = State(wheel_radius=0.05)
        measurements = {
            Observable.OBS_TIME: np.array([0.1]),
            Observable.ACC: np.array([0.0, 0.0, 9.81]),
            Observable.GYRO: np.zeros(3),
            Observable.LEFT_WHEEL_VEL: np.zeros(1),
            Observable.RIGHT_WHEEL_VEL: np.zeros(1),
            Observable.HEAD_PITCH: np.array([0.3]),
            Observable.HEAD_TURN: np.array([0.5]),
        }
        state.update(measurements)
        expected = np.arcsin(np.sin(0.3) * np.cos(0.5))
        self.assertAlmostEqual(
            state.snapshot()[DerivedObs.CAMERA_PITCH_WORLD][0], expected
        )
        measurements[Observable.OBS_TIME][:] = 0.2
        measurements[Observable.TRUE_CAMERA_PITCH] = np.array([-1.4])
        state.update(measurements)
        self.assertAlmostEqual(
            state.snapshot()[DerivedObs.CAMERA_PITCH_WORLD][0], expected
        )

    def test_reward_scores_actual_pose_and_disables_neutral_pitch_cost(self):
        env = self.make_env()
        physics = env.dm_env
        physics.data.qpos[3:7] = eul2q(0, 0.2, 0).as_array()
        joint = physics.model.name2id("frame/headpitch_joint", "joint")
        physics.data.qpos[physics.model.jnt_qposadr[joint]] = 0.2
        physics.forward()
        env.state.reset(env._read_sensors())
        # Filter starts at identity: its estimate differs from actual level gaze.
        self.assertAlmostEqual(
            env.state.snapshot()[DerivedObs.CAMERA_PITCH_WORLD][0], 0.2
        )
        rewards = env._calculate_rewards(env.state, terminated=False)
        self.assertAlmostEqual(rewards[Observable.REWARD_CAMERA_PITCH], 0.0)
        self.assertEqual(rewards[Observable.REWARD_HEAD_PITCH], 0.0)
        env.head_target = [0.1, 0.2]
        rewards = env._calculate_rewards(env.state, terminated=False)
        self.assertAlmostEqual(rewards[Observable.REWARD_CAMERA_PITCH], -0.1)
        self.assertAlmostEqual(rewards[Observable.REWARD_HEAD_YAW], -0.2)
        env.head_tracking = False
        rewards = env._calculate_rewards(env.state, terminated=False)
        self.assertAlmostEqual(rewards[Observable.REWARD_HEAD_PITCH], -2.0)
        self.assertEqual(rewards[Observable.REWARD_CAMERA_PITCH], 0.0)
        self.assertEqual(rewards[Observable.REWARD_HEAD_YAW], 0.0)

    def test_targets_do_not_generate_or_compensate_actuator_commands(self):
        env = self.make_env()
        physics = env.dm_env
        physics.data.qpos[3:7] = eul2q(0, 0.2, 0).as_array()
        physics.forward()
        controls = physics.data.ctrl.copy()
        env.head_target = [-0.1, 0.2]
        np.testing.assert_array_equal(physics.data.ctrl, controls)
        for pitch_rate, yaw_rate in ((0.0, 0.0), (0.3, -0.2)):
            env.step({
                Actions.ACC_BOTH_WHEELS: np.array([0.0]),
                Actions.VEL_HEAD_PITCH: np.array([pitch_rate]),
                Actions.VEL_HEAD_TURN: np.array([yaw_rate]),
            })
            self.assertEqual(float(physics.bind(env.head_pitch_act).ctrl), pitch_rate)
            self.assertEqual(float(physics.bind(env.head_turn_act).ctrl), yaw_rate)

    def test_head_and_position_trajectories_are_independent_and_restart(self):
        trajectory = HeadTrajectory([[0, 0, 0], [2, 0.2, -0.4]])
        np.testing.assert_allclose(trajectory.angles_at(1), [0.1, -0.2])
        np.testing.assert_allclose(trajectory.angles_at(3), [0.2, -0.4])
        state = State(wheel_radius=0.05)
        state.target_trajectory = PositionTrajectory([[0, 0], [2, 1]])
        state.head_trajectory = trajectory
        state.reset({
            Observable.OBS_TIME: np.array([1.0]),
            Observable.LEFT_WHEEL_VEL: np.zeros(1),
            Observable.RIGHT_WHEEL_VEL: np.zeros(1),
        })
        np.testing.assert_allclose(state.head_target, [0.1, -0.2])
        self.assertAlmostEqual(state.target_pos, 0.5)
        with self.assertRaises(ValueError):
            state.head_target = [0, 0.8]
        self.assertIs(state.head_trajectory, trajectory)
        state.reset()
        np.testing.assert_array_equal(state.head_target, [0, 0])
        state.head_target = [0.1, 0.2]
        self.assertIsNone(state.head_trajectory)
        self.assertIsNotNone(state.target_trajectory)
        state.reset()
        np.testing.assert_allclose(state.head_target, [0.1, 0.2])
        invalid = ([[0, 0]], [[0, 2, 0]], [[0, 0, 0.8]], [[0, 0, 0], [0, 0.1, 0]])
        for points in invalid:
            with self.assertRaises(ValueError):
                HeadTrajectory(points)

    def test_batched_head_references_coexist_with_velocity_and_reach_the_nn(self):
        for count in (1, 3):
            self.cfg.env.n_parallel = count
            env = register_and_make_env(self.cfg)
            if isinstance(env, Env):
                env = SingleEnvWrapper(env)
            self.addCleanup(env.close)
            obs, _ = env.reset(seed=1)
            angles = [[0.05 * (i + 1), -0.05 * i] for i in range(count)]
            obs = add_targets(obs, env, target_velocities=[0.1] * count)
            obs = add_head_targets(obs, env, head_targets=angles)
            np.testing.assert_allclose(obs[Target.TARGET_VEL], 0.1)
            inputs = get_policy_inputs(self.cfg)
            self.assertNotIn(Observable.TRUE_CAMERA_PITCH.value, inputs)
            agent = Agent(inputs, self.cfg.policy.actions)
            head_keys = (Target.CAMERA_PITCH_WORLD, Target.HEAD_YAW_NECK)
            for column, key in enumerate(head_keys):
                seen = []
                hook = agent.encoders[key.value].register_forward_pre_hook(
                    lambda _module, args: seen.append(args[0].clone())
                )
                with torch.no_grad():
                    agent.act(npd2tensord(obs))
                hook.remove()
                np.testing.assert_allclose(
                    seen[0].numpy()[:, 0], np.asarray(angles)[:, column]
                )
            with self.assertRaises(ValueError):
                add_head_targets(obs, env, head_targets=angles[:-1])
            invalid = [*angles[:-1], [0.0, np.nan]]
            with self.assertRaises(ValueError):
                add_head_targets(obs, env, head_targets=invalid)
            obs, _ = env.reset(seed=2)
            np.testing.assert_allclose(
                obs[Target.CAMERA_PITCH_WORLD][:, 0], np.asarray(angles)[:, 0]
            )

    def test_head_trajectory_reward_and_terminal_timing_in_a_batch(self):
        env = SyncVectorEnv([
            lambda: TimeLimit(self.make_env(), max_episode_steps=2),
            lambda: TimeLimit(self.make_env(), max_episode_steps=4),
        ])
        self.addCleanup(env.close)
        agent = Agent(get_policy_inputs(self.cfg), self.cfg.policy.actions)
        trajectories = [
            [[0, 0, 0], [0.04, 0.2, 0.3]],
            [[0, 0.1, 0], [0.04, 0, -0.2]],
        ]
        buffers = rollout(
            env, agent, seed=1, target_velocities=[0.1, -0.1],
            head_trajectories=trajectories,
        )
        self.assertEqual([buf.seq_len for buf in buffers], [2, 4])
        for buf, points in zip(buffers, trajectories):
            time = buf.get_observable(Observable.OBS_TIME)[:, 0]
            expected = np.interp(time, [0, 0.04], [points[0][1], points[1][1]])
            np.testing.assert_allclose(
                buf.get_observable(Target.CAMERA_PITCH_WORLD)[:, 0], expected, atol=1e-7
            )
            true_pitch = buf.get_observable(Observable.TRUE_CAMERA_PITCH)[1:, 0]
            np.testing.assert_allclose(
                buf.get_observable(Observable.REWARD_CAMERA_PITCH)[1:, 0],
                -abs(true_pitch - expected[1:]), atol=1e-7,
            )
        obs, _ = env.reset(seed=2)
        np.testing.assert_allclose(obs[Target.CAMERA_PITCH_WORLD][:, 0], [0, 0.1])

    def test_head_tracking_can_be_disabled_independently_of_locomotion(self):
        for enabled in (False, True):
            self.cfg.env.head_tracking = enabled
            self.cfg.env.tracking_mode = "none"
            inputs = get_policy_inputs(self.cfg)
            self.assertEqual(Target.CAMERA_PITCH_WORLD.value in inputs, enabled)
            self.assertEqual(Target.HEAD_YAW_NECK.value in inputs, enabled)
            agent = Agent(inputs, self.cfg.policy.actions)
            plotted = {key for _, row in get_plot_keys(self.cfg, agent) for key in row}
            self.assertEqual(Target.CAMERA_PITCH_WORLD in plotted, enabled)
            self.assertEqual(Observable.REWARD_CAMERA_PITCH in plotted, enabled)
            self.assertEqual(Observable.REWARD_HEAD_PITCH in plotted, not enabled)

    def test_boundary_targets_survive_float32_round_trip(self):
        state = State(wheel_radius=0.05)
        for sign in (-1, 1):
            state.head_target = [sign * np.pi / 2, sign * HEAD_YAW_LIMITS[1]]
            expected = state.head_target.copy()
            state.head_target = expected
            np.testing.assert_array_equal(state.head_target, expected)

    def test_episode_keeps_target_and_estimated_camera_pitch_distinct(self):
        buffer = EpisodeBuffer()
        buffer.add_step(
            {
                Target.CAMERA_PITCH_WORLD: np.array([0.1]),
                DerivedObs.CAMERA_PITCH_WORLD: np.array([0.2]),
            },
            {Actions.TIME: np.array([0.0])},
            1.0, torch.tensor(0.0), torch.tensor(0.0),
        )
        buffer.finish({
            Target.CAMERA_PITCH_WORLD: np.array([0.1]),
            DerivedObs.CAMERA_PITCH_WORLD: np.array([0.3]),
        })
        episode = Episode(buffer)
        np.testing.assert_allclose(
            episode.get_observable(Target.CAMERA_PITCH_WORLD)[:, 0], [0.1, 0.1]
        )
        np.testing.assert_allclose(
            episode.get_observable(DerivedObs.CAMERA_PITCH_WORLD)[:, 0], [0.2, 0.3]
        )


if __name__ == "__main__":
    unittest.main()

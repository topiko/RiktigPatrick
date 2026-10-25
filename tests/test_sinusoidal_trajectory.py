"""Analytic paired references, including reversals and shared-state reset semantics."""

import unittest

import numpy as np

from riktigpatric.patrick import DerivedObs, Observable, State
from riktigpatric.trajectory import SinusoidalPositionTrajectory


class SinusoidalTrajectoryTests(unittest.TestCase):
    def test_velocity_is_position_derivative_and_reverses_at_extrema(self):
        for amplitude in (-0.1, 0.1):
            trajectory = SinusoidalPositionTrajectory(amplitude, 0.6)
            for time in np.linspace(0.01, 20, 50):
                derivative = (trajectory.position_at(time + 1e-5)
                              - trajectory.position_at(time - 1e-5)) / 2e-5
                self.assertAlmostEqual(trajectory.velocity_at(time), derivative,
                                       places=9)
            turning_time = np.pi / (2 * trajectory.omega)
            self.assertAlmostEqual(trajectory.position_at(turning_time), amplitude)
            self.assertAlmostEqual(trajectory.velocity_at(turning_time), 0)
            self.assertLess(trajectory.velocity_at(turning_time - 0.1)
                            * trajectory.velocity_at(turning_time + 0.1), 0)

    def test_shared_state_updates_both_targets_at_sensor_time_and_restarts(self):
        state = State(wheel_radius=0.05)
        state.target_trajectory = SinusoidalPositionTrajectory(0.1, 0.6)
        for time in (0.007, 0.024, 3.0, 8.0, 20.0):
            state.update({
                Observable.OBS_TIME: np.array([time]),
                Observable.ACC: np.array([0.0, 0.0, 9.81]),
                Observable.GYRO: np.zeros(3),
                Observable.LEFT_WHEEL_VEL: np.zeros(1),
                Observable.RIGHT_WHEEL_VEL: np.zeros(1),
            })
            self.assertAlmostEqual(state.target_pos, 0.1 * np.sin(0.6 * time))
            self.assertAlmostEqual(state.target_vel, 0.06 * np.cos(0.6 * time))
            snapshot = state.snapshot()
            np.testing.assert_allclose(snapshot[DerivedObs.POSITION_ERROR],
                                       np.tanh(-state.target_pos/state.position_error_scale))
            np.testing.assert_allclose(snapshot[DerivedObs.VELOCITY_ERROR],
                                       np.tanh(-state.target_vel/state.velocity_error_scale))
        state.reset()
        self.assertEqual(state.target_pos, 0)
        self.assertAlmostEqual(state.target_vel, 0.06)

    def test_invalid_parameters_and_time_are_rejected(self):
        for amplitude, omega in ((np.nan, 1), (np.inf, 1), (1, 0), (1, -1),
                                 (1, np.inf), (1, np.nan), (1e30, 1e30)):
            with self.subTest(amplitude=amplitude, omega=omega):
                with self.assertRaises(ValueError):
                    SinusoidalPositionTrajectory(amplitude, omega)
        trajectory = SinusoidalPositionTrajectory(0.1, 0.6)
        for time in (-1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                trajectory.position_at(time)
            with self.assertRaises(ValueError):
                trajectory.velocity_at(time)

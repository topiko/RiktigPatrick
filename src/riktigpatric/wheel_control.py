"""Shared SI wheel-command mixing, independent of simulator and wire protocol."""

import numpy as np


def wheel_velocity_targets(
    measured: np.ndarray,
    acceleration: float,
    step_time: float,
    max_velocity: float,
    max_acceleration: float,
    velocity_difference: float | None = None,
) -> np.ndarray:
    """Return [left, right] velocity targets; positive difference turns left.

    With steering, acceleration changes the mean wheel speed and the difference
    is right minus left, in wheel rad/s. Saturation prioritizes the mean (balance)
    command. Without steering, retain each wheel's measured-speed reference.
    """
    increment = np.clip(acceleration, -max_acceleration, max_acceleration) * step_time
    if velocity_difference is None:
        return np.clip(measured + increment, -max_velocity, max_velocity)
    mean = np.clip(np.mean(measured) + increment, -max_velocity, max_velocity)
    headroom = max_velocity - abs(mean)
    half_difference = np.clip(velocity_difference / 2, -headroom, headroom)
    return np.array([mean - half_difference, mean + half_difference])

"""Pitch reward shaping and its shared fall-angle reference, in radians."""

import math

FALL_PITCH_LIMIT = math.radians(20)


def validate_pitch_deadband(deadband: float | None) -> None:
    if deadband is not None and (
        not math.isfinite(deadband) or not 0 <= deadband < FALL_PITCH_LIMIT
    ):
        raise ValueError("pitch_deadband must be null or in [0, 20°), in radians")


def pitch_reward(pitch: float, scale: float, deadband: float | None) -> float:
    """Linear shaping, or a free region followed by a quadratic penalty.

    The quadratic curve matches the linear penalty at the fall-angle reference,
    preserving the meaning of the configured reward scale at that boundary.
    Deadbands are validated when configured, rather than at every control step.
    """
    magnitude = abs(pitch)
    if deadband is not None:
        excess = max(magnitude - deadband, 0.0)
        magnitude = FALL_PITCH_LIMIT * (excess / (FALL_PITCH_LIMIT - deadband)) ** 2
    return scale * magnitude

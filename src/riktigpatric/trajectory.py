"""Backend-independent position and head targets, evaluated using episode time."""

from collections.abc import Sequence

import numpy as np

from riktigpatric.geometry import HEAD_YAW_LIMITS


def validate_head_target(angles: Sequence[float] | np.ndarray) -> np.ndarray:
    """Validate [world camera elevation, neck-relative yaw], in radians."""
    values = np.asarray(angles, dtype=np.float64)
    if values.shape != (2,) or not np.isfinite(values).all():
        raise ValueError("Head target must contain two finite angles")
    lower = np.array([-np.pi / 2, HEAD_YAW_LIMITS[0]])
    upper = np.array([np.pi / 2, HEAD_YAW_LIMITS[1]])
    # Keep validation idempotent after a target travels through float32 observations.
    tolerance = np.finfo(np.float32).eps
    if np.any(values < lower - tolerance) or np.any(values > upper + tolerance):
        raise ValueError("Head target exceeds elevation +/-90 or yaw +/-40 degrees")
    return np.clip(values, lower, upper).astype(np.float32)


class _WaypointTrajectory:
    """Shared linear interpolation, holding the final target after the last time."""

    def __init__(self, waypoints: Sequence[Sequence[float]], dimensions: int):
        points = np.array(waypoints, dtype=np.float64, copy=True)
        if points.ndim != 2 or points.shape[1] != dimensions + 1 or len(points) == 0:
            raise ValueError(f"Expected [time, {dimensions} target values] waypoints")
        if not np.isfinite(points).all():
            raise ValueError("Trajectory waypoints must be finite")
        if points[0, 0] != 0 or np.any(np.diff(points[:, 0]) <= 0):
            raise ValueError(
                "Trajectory times must start at zero and increase strictly"
            )
        if np.any(np.abs(points[:, 1:]) > np.finfo(np.float32).max):
            raise ValueError("Trajectory targets must fit in float32 observations")
        points.setflags(write=False)
        self._points = points

    def _sample(self, time: float) -> np.ndarray:
        if not np.isfinite(time) or time < 0:
            raise ValueError("Trajectory time must be finite and nonnegative")
        return np.array([
            np.interp(time, self._points[:, 0], channel)
            for channel in self._points[:, 1:].T
        ])


class PositionTrajectory(_WaypointTrajectory):
    """[time (s), position (m)] waypoints, starting at zero and holding the end."""

    def __init__(self, waypoints: Sequence[Sequence[float]]):
        super().__init__(waypoints, dimensions=1)

    def position_at(self, time: float) -> float:
        return float(self._sample(time)[0])


class HeadTrajectory(_WaypointTrajectory):
    """[time (s), camera elevation (rad), neck yaw (rad)] waypoints.

    Neck yaw is a bounded joint angle, so interpolation does not wrap at +/-pi.
    """

    def __init__(self, waypoints: Sequence[Sequence[float]]):
        super().__init__(waypoints, dimensions=2)
        for angles in self._points[:, 1:]:
            validate_head_target(angles)

    def angles_at(self, time: float) -> np.ndarray:
        return self._sample(time).astype(np.float32)

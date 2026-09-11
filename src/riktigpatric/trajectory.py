"""Backend-independent position targets, evaluated using episode time."""

from collections.abc import Sequence

import numpy as np


class PositionTrajectory:
    """Linearly interpolate (time in seconds, position in meters) waypoints.

    The first waypoint must be at time zero. Hold the final position after the
    last waypoint. A single waypoint represents a fixed target.
    """

    def __init__(self, waypoints: Sequence[Sequence[float]]):
        points = np.array(waypoints, dtype=np.float64, copy=True)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
            raise ValueError("Expected one or more [time, position] waypoints")
        if not np.isfinite(points).all():
            raise ValueError("Trajectory waypoints must be finite")
        if points[0, 0] != 0 or np.any(np.diff(points[:, 0]) <= 0):
            raise ValueError(
                "Trajectory times must start at zero and increase strictly"
            )
        if np.any(np.abs(points[:, 1]) > np.finfo(np.float32).max):
            raise ValueError("Trajectory positions must fit in float32 observations")
        points.setflags(write=False)
        self._points = points

    def position_at(self, time: float) -> float:
        """Return the target at an episode-relative timestamp."""
        if not np.isfinite(time) or time < 0:
            raise ValueError("Trajectory time must be finite and nonnegative")
        return float(np.interp(time, self._points[:, 0], self._points[:, 1]))

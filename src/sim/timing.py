"""Control scheduling jitter on a fixed MuJoCo integration grid."""

import math

import numpy as np

PHYSICS_STEP = 0.002


def physics_steps(seconds: float, name: str) -> int:
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError(f"{name} must be positive and finite")
    steps = round(seconds / PHYSICS_STEP)
    if steps < 1 or not math.isclose(steps * PHYSICS_STEP, seconds, rel_tol=1e-9):
        raise ValueError(f"{name} must be a positive multiple of {PHYSICS_STEP} s")
    return steps


class ControlTiming:
    def __init__(self, mean: float, std: float, minimum: float):
        self.nominal_steps = physics_steps(mean, "step_time")
        if not math.isfinite(std) or std < 0:
            raise ValueError("step_time_std must be finite and nonnegative")
        if not math.isfinite(minimum) or minimum <= 0:
            raise ValueError("min_step_time must be positive and finite")
        if std > 0 and minimum > mean:
            raise ValueError("With jitter, min_step_time must not exceed step_time")
        self.mean = mean
        self.std = std
        self.minimum = minimum
        self.minimum_steps = max(1, math.ceil(minimum / PHYSICS_STEP))

    def sample_steps(self, rng: np.random.Generator) -> int:
        if self.std == 0:
            return self.nominal_steps
        duration = max(self.minimum, float(rng.normal(self.mean, self.std)))
        return max(self.minimum_steps, round(duration / PHYSICS_STEP))

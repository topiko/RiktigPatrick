"""
Configuration loader for RL training.

Unit Convention:
- Simulation (MuJoCo) uses SI units: rad/s, rad, s
- Network uses human-friendly units: deg/s, rev/s, deg
- Conversions happen at boundaries (network input, plotting)

Conversion factors:
- RAD2DEG: rad/s → deg/s (180/π)
- RAD2REV: rad/s → rev/s (1/(2π))
"""

import numpy as np

MAX_WHEEL_VEL = 10
MAX_WHEEL_ACC = 10


# Unit conversion factors (multiply to convert from SI to display units)
RAD2REV = 1.0 / (2 * np.pi)  # rad/s → rev/s
RAD2DEG = 180.0 / np.pi  # rad/s → deg/s

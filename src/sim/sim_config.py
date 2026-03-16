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

import os
import yaml

import numpy as np

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")

with open(CONFIG_PATH, "r") as f:
    _config = yaml.safe_load(f)

OBS_SPACE = [
    "filter/rp_pitch",
    "sens/gyro",
    "sens/head_pitch",
    "sens/head_turn",
    "sens/left_wheel_vel",
    "sens/right_wheel_vel",
    "simul/rp_pitch",
    "env/time",
]

MODEL_INPUT = _config["model"]["model_input"]
OBS_SCALES = _config["model"]["obs_scales"]

ENV_CONFIG = _config["env"]
RL_CONFIG = _config["rl"]
TRAIN_CONFIG = _config["training"]
REWARD_CONFIG = _config["reward"]
ACTION_CONFIG = _config["action"]

# MAX_V: max wheel velocity in rev/s (from config), converted to rad/s for internal use
# The network outputs actions in rad/s, but config specifies limits in human-friendly rev/s
MAX_V = ACTION_CONFIG["max_wheel_vel"] * 2 * np.pi  # rev/s → rad/s

# Unit conversion factors (multiply to convert from SI to display units)
RAD2REV = 1.0 / (2 * np.pi)  # rad/s → rev/s
RAD2DEG = 180.0 / np.pi  # rad/s → deg/s

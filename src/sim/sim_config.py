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
    "env/action_time",
    "env/obs_time",
]

MODEL_INPUT = _config["model"]["model_input"]
OBS_SCALES = _config["model"]["obs_scales"]

ENV_CONFIG = _config["env"]
RL_CONFIG = _config["rl"]
TRAIN_CONFIG = _config["training"]
REWARD_CONFIG = _config["reward"]
POLICY_CONFIG = _config["policy"]

# Policy type: "velocity" or "acceleration"
POLICY_TYPE = POLICY_CONFIG["type"]

# Velocity mode limits
if POLICY_TYPE == "velocity":
    MAX_WHEEL_VEL = (
        POLICY_CONFIG["velocity"]["max_wheel_vel"] * 2 * np.pi
    )  # rev/s → rad/s
    N_ACTIONS = None
    MAX_WHEEL_ACC = None

# Acceleration mode limits
elif POLICY_TYPE == "acceleration":
    N_ACTIONS = POLICY_CONFIG["acceleration"]["n_actions"]
    MAX_WHEEL_ACC = (
        POLICY_CONFIG["acceleration"]["max_wheel_acc"] * 2 * np.pi
    )  # rev/s² → rad/s²
    MAX_WHEEL_VEL = (
        POLICY_CONFIG["velocity"]["max_wheel_vel"] * 2 * np.pi
    )  # rev/s → rad/s (for clipping)
else:
    raise ValueError(f"Invalid policy type: {POLICY_TYPE}")

# Backward compatibility aliases
MAX_V = MAX_WHEEL_VEL

# Unit conversion factors (multiply to convert from SI to display units)
RAD2REV = 1.0 / (2 * np.pi)  # rad/s → rev/s
RAD2DEG = 180.0 / np.pi  # rad/s → deg/s

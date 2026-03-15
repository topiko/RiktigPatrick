import os
import yaml

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
]

MODEL_INPUT = _config["model"]["model_input"]

ENV_CONFIG = _config["env"]
RL_CONFIG = _config["rl"]
TRAIN_CONFIG = _config["training"]
REWARD_CONFIG = _config["reward"]

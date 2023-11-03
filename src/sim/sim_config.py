OBS_SPACE = [
    "filter/rp_pitch",
    "sens/gyro",
    "sens/head_pitch",
    "sens/head_turn",
    "sens/left_wheel_vel",
    "sens/right_wheel_vel",
    "simul/rp_pitch",
]
MODEL_INPUT = [
    "filter/rp_pitch",
    "sens/gyro",
    "sens/left_wheel_vel",
    "sens/right_wheel_vel",
]

CTRL_MODE = "vel"
ENV_CONFIG = {"ctrl_mode": CTRL_MODE, "lock_head": True, "step_time": 0.01}

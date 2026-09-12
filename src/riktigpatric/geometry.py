"""SI geometry from archive/scad/{usedims,frame,head,electronics}.scad.

Robot axes: +X forward, +Y left, +Z up. CAD +Y maps to robot +X, CAD +X
to robot -Y. Body origin: nominal motor-mount midpoint. Shape profiles
approximate the rounded CAD shells; mass properties remain estimates.
"""

import numpy as np

FRAME_HEIGHT = 0.193
BODY_BOTTOM_WIDTH = 0.100
BODY_DEPTH = 0.055
BODY_TAPER = np.deg2rad(4.5)
MOTOR_HEIGHT = 0.023
MOTOR_FORWARD = 0.009  # frame.scad: -Rtop + 20 mm
MOTOR_HALF_SPACING = BODY_BOTTOM_WIDTH / 2 - MOTOR_HEIGHT * np.tan(BODY_TAPER)

WHEEL_D = 0.100  # confirmed independently of body depth
WHEEL_WIDTH = 0.020  # retained approximation
WHEEL_CLEARANCE = 0.001

NECK_PIVOT = (-MOTOR_FORWARD, 0.0, FRAME_HEIGHT - MOTOR_HEIGHT)
HEAD_OFFSET = (0.0, 0.0, 0.0114)  # Rtop + Hneck
HEAD_HEIGHT = 0.115
HEAD_DEPTH = 0.042
HEAD_BACK = -0.0285  # -Lhead + mountT/2 + 2.5 mm
HEAD_FRONT = HEAD_BACK + HEAD_DEPTH
HEAD_SLOPE = np.deg2rad(6.9)  # shell slope, NOT camera mounting pitch
HEAD_BOTTOM_WIDTH = (
    BODY_BOTTOM_WIDTH - 2 * (FRAME_HEIGHT + 0.0134) * np.tan(BODY_TAPER)
)

# Positive neck pitch looks up; positive neck-relative yaw turns left.
HEAD_PITCH_AXIS = (0.0, -1.0, 0.0)
HEAD_YAW_AXIS = (0.0, 0.0, 1.0)
HEAD_PITCH_LIMITS = tuple(np.deg2rad([-28.0, 50.0]))
HEAD_YAW_LIMITS = tuple(np.deg2rad([-40.0, 40.0]))

CAMERA_RADIUS = 0.016
CAMERA_LENGTH = 0.035
CAMERA_BASE = (-0.0215, 0.0, 0.090)
CAMERA_TIP = (CAMERA_BASE[0] + CAMERA_LENGTH, 0.0, CAMERA_BASE[2])

# Columns: camera right, up, backward in the neutral head frame.
# MuJoCo cameras look down local -Z with +Y up.
CAMERA_MOUNT_ROTATION = np.array([
    [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
])


def camera_rotation(body_rotation: np.ndarray, neck_pitch: float, neck_yaw: float):
    """Forward kinematics only: body -> pitched neck -> yawed head -> camera."""
    cp, sp = np.cos(neck_pitch), np.sin(neck_pitch)
    cy, sy = np.cos(neck_yaw), np.sin(neck_yaw)
    pitch = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
    yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return body_rotation @ pitch @ yaw @ CAMERA_MOUNT_ROTATION


def camera_elevation(rotation: np.ndarray) -> float:
    """Optical-axis elevation above the world horizon, positive upward, radians."""
    forward = -rotation[:, 2]
    return float(np.arctan2(forward[2], np.hypot(forward[0], forward[1])))

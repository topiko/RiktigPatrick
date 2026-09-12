"""

MuJoCo Robot Environment for RiktigPatrick

UNIT SYSTEM:
All units follow the International System of Units (SI):
- Angles: radians (rad)
- Angular velocities: radians per second (rad/s)
- Angular accelerations: radians per second squared (rad/s²)
- Time: seconds (s)
- Distances: meters (m)
- Mass: kilograms (kg)

MuJoCo natively uses SI units throughout.
Observations from sensors are in SI units (e.g., jointvel sensor returns rad/s).
Actions (wheel accelerations) are in rad/s².
"""

import warnings
from collections.abc import Sequence
from typing import Any, Optional

import gymnasium
import gymnasium.spaces
import matplotlib.pyplot as plt
import numpy as np
from dm_control import mjcf
from matplotlib import animation

from filters.qutils import as_rotation_matrix, q2eul
from riktigpatric.geometry import (
    BODY_BOTTOM_WIDTH,
    BODY_DEPTH,
    BODY_TAPER,
    CAMERA_BASE,
    CAMERA_MOUNT_ROTATION,
    CAMERA_RADIUS,
    CAMERA_TIP,
    FRAME_HEIGHT,
    HEAD_BACK,
    HEAD_BOTTOM_WIDTH,
    HEAD_FRONT,
    HEAD_HEIGHT,
    HEAD_OFFSET,
    HEAD_PITCH_AXIS,
    HEAD_PITCH_LIMITS,
    HEAD_SLOPE,
    HEAD_YAW_AXIS,
    HEAD_YAW_LIMITS,
    MOTOR_FORWARD,
    MOTOR_HALF_SPACING,
    MOTOR_HEIGHT,
    NECK_PIVOT,
    WHEEL_CLEARANCE,
    WHEEL_D,
    WHEEL_WIDTH,
    camera_elevation,
)
from riktigpatric.patrick import (
    Actions,
    DerivedObs,
    Observable,
    State,
    StateVarKey,
    Target,
)
from riktigpatric.trajectory import HeadTrajectory, PositionTrajectory

BODY_M = 0.4
HEAD_M = 0.2

FORCERANGE = 15

# Unit conversion constants (for display/debugging only - not used in main code path)
RAD2REV = 1.0 / (2 * np.pi)  # rad/s → rev/s (revolutions per second)
RAD2DEG = 180.0 / np.pi  # rad/s → deg/s (degrees per second)


def _profile_mesh(model, name, profile, width, origin=(0.0, 0.0)):
    """Convex tapered proxy from a CAD forward/height profile, in meters."""
    vertices = [
        (forward - origin[0],
         side * (width / 2 - max(height, 0) * np.tan(BODY_TAPER)),
         height - origin[1])
        for forward, height in profile for side in (-1, 1)
    ]
    return model.asset.add("mesh", name=name, vertex=np.asarray(vertices).ravel())


class MujocoRP:
    def __init__(
        self,
        rgba: list[float] = [0.20269912, 0.4307427, 0.33218761, 1.0],
        wheel_markers: bool = True,
        seed: Optional[int] = None,
        randomize: bool = False,
        random_scale: float = 0.02,
        max_wheel_vel: float = 10.0,  # rad/s (SI units)
        max_wheel_acc: float = 50.0,  # rad/s² (SI units)
        max_head_vel: float = 1.0,  # rad/s; hardware speed remains to be measured
    ):
        """
        Initialize MuJoCo robot model.

        All physical parameters use SI units (m, kg, s, rad).

        Args:
            rgba: Robot body color
            wheel_markers: Whether to add visual markers to wheels
            seed: Random seed for physical parameter randomization
            max_wheel_vel: Maximum wheel velocity in rad/s (SI units)
            max_wheel_acc: Maximum wheel acceleration in rad/s² (SI units)

        """
        rng = np.random.default_rng(seed)

        # Store limits (SI units: rad/s, rad/s²)
        self.max_wheel_vel = max_wheel_vel
        self.max_wheel_acc = max_wheel_acc

        self.model = mjcf.RootElement("frame")

        # Body:
        frame = self.model.worldbody.add("body", name="torso")

        body_m = rng.normal(BODY_M, BODY_M * random_scale) if randomize else BODY_M
        body_mesh = _profile_mesh(
            self.model, "body_mesh",
            [(-0.011, -0.002), (0.026, -0.002), (BODY_DEPTH - 0.007, 0.030),
             (0.011, FRAME_HEIGHT), (0.0, FRAME_HEIGHT + 0.011),
             (-0.011, FRAME_HEIGHT)],
            BODY_BOTTOM_WIDTH, origin=(MOTOR_FORWARD, MOTOR_HEIGHT),
        )
        frame.add(
            "geom",
            name="body",
            type="mesh",
            mesh=body_mesh,
            rgba=rgba,
            mass=body_m,
        )

        # Wheels
        wheel_d = rng.normal(WHEEL_D, WHEEL_D * random_scale) if randomize else WHEEL_D
        self.spawn_height = (
            wheel_d / 2 * np.cos(BODY_TAPER) - WHEEL_CLEARANCE * np.sin(BODY_TAPER)
        )
        kp_wheel = 20.0  # was 1.2
        for diry, key in zip([-1, 1], ["rightwheel", "leftwheel"]):
            outward = np.array([0.0, diry * np.cos(BODY_TAPER), np.sin(BODY_TAPER)])
            center = np.array([0.0, diry * MOTOR_HALF_SPACING, 0.0])
            center += outward * (WHEEL_CLEARANCE + WHEEL_WIDTH / 2)
            # Wheel
            wheel = frame.add("body", name=key, pos=center)
            wheel.add(
                "geom",
                type="cylinder",
                name=key + "_cyl",
                zaxis=outward,
                friction=(2, 0.005, 0.0001),
                size=[wheel_d / 2, WHEEL_WIDTH / 2],
                mass=0.020,  # kg
            )
            if wheel_markers:
                wheel.add(
                    "geom",
                    type="cylinder",
                    name=key + "_marker",
                    zaxis=outward,
                    pos=[0, -diry * np.sin(BODY_TAPER) * wheel_d / 4,
                         np.cos(BODY_TAPER) * wheel_d / 4],
                    mass=0, contype=0, conaffinity=0,
                    rgba=[0, 0, 0, 1],
                    size=[wheel_d / 12, WHEEL_WIDTH / 2 + 0.0005],
                )

            # Wheel joint
            wheel = wheel.add(
                "joint", name=key + "_joint", axis=outward * diry, damping=0.1
            )

            # Wheel actuator
            self.model.actuator.add(
                "intvelocity",
                name=key + "_actuator",
                joint=wheel,
                gear=(1,),
                ctrllimited=True,
                ctrlrange=[-self.max_wheel_vel, self.max_wheel_vel],
                # forcelimited=True,
                # forcerange=[-FORCERANGE, FORCERANGE],
                actrange=[-(10**6), 10**6],
                kp=kp_wheel,  # <-- intvelocity feedback gain
            )
            self.model.sensor.add(
                "jointvel", name=f"{key}_vel_sensor", joint=f"{key}_joint"
            )

        # Head:
        kp_servo = 1.500

        neck = frame.add("body", name="neck", pos=NECK_PIVOT)
        head = neck.add("body", name="head", pos=HEAD_OFFSET)

        # Joints
        head_pitch = neck.add(
            "joint", name="headpitch_joint", axis=HEAD_PITCH_AXIS, damping=0.1,
            limited=True, range=np.rad2deg(HEAD_PITCH_LIMITS),
        )
        self.model.sensor.add(
            "jointpos", name="headpitch_sensor", joint="headpitch_joint"
        )
        self.model.sensor.add(
            "jointvel", name="headpitch_vel_sensor", joint="headpitch_joint"
        )

        head_lr = head.add(
            "joint", name="headturn_joint", axis=HEAD_YAW_AXIS, damping=0.1,
            limited=True, range=np.rad2deg(HEAD_YAW_LIMITS),
        )
        self.model.sensor.add(
            "jointpos", name="headturn_sensor", joint="headturn_joint"
        )
        self.model.sensor.add(
            "jointvel", name="headturn_vel_sensor", joint="headturn_joint"
        )

        head_m = rng.normal(HEAD_M, HEAD_M * random_scale) if randomize else HEAD_M
        # Split the combined head/neck mass budget; the visual camera adds none.
        neck.add(
            "geom", name="neckgeom", type="cylinder", size=[0.011],
            fromto=[0, 0, 0, *HEAD_OFFSET], mass=head_m * 0.05, rgba=rgba,
        )
        head_mesh = _profile_mesh(
            self.model, "head_mesh",
            [(HEAD_BACK, 0), (HEAD_FRONT, 0),
             (HEAD_FRONT - HEAD_HEIGHT * np.tan(HEAD_SLOPE), HEAD_HEIGHT),
             (HEAD_BACK, HEAD_HEIGHT)], HEAD_BOTTOM_WIDTH,
        )
        head.add(
            "geom",
            type="mesh", mesh=head_mesh,
            name="headgeom",
            rgba=rgba,
            mass=head_m * 0.95,
        )
        head.add(
            "geom", name="camera_cylinder", type="cylinder",
            fromto=[*CAMERA_BASE, *CAMERA_TIP], size=[CAMERA_RADIUS],
            rgba=[0.08, 0.08, 0.08, 1], mass=0, contype=0, conaffinity=0,
        )
        camera_axes = np.concatenate((CAMERA_MOUNT_ROTATION[:, 0],
                                      CAMERA_MOUNT_ROTATION[:, 1]))
        camera_site = head.add(
            "site", name="head_camera_site", pos=CAMERA_TIP, xyaxes=camera_axes,
            size=[0.001], rgba=[0, 0, 0, 0],
        )
        head.add(
            "camera", name="head_camera", pos=CAMERA_TIP, xyaxes=camera_axes, fovy=60
        )
        self.model.sensor.add(
            "framequat", name="camera_framequat_sensor",
            objtype="site", objname=camera_site,
        )

        self.model.actuator.add(
            "intvelocity",
            name="headpitch_actuator",
            joint=head_pitch,
            kp=kp_servo,
            ctrllimited=True,
            ctrlrange=[-max_head_vel, max_head_vel],
            actrange=HEAD_PITCH_LIMITS,  # rad; joint ranges above use MJCF degrees
        )
        self.model.actuator.add(
            "intvelocity",
            name="headturn_actuator",
            joint=head_lr,
            kp=kp_servo,
            ctrllimited=True,
            ctrlrange=[-max_head_vel, max_head_vel],
            actrange=HEAD_YAW_LIMITS,
        )

        # Sensors:
        imu_pos = [0, 0, FRAME_HEIGHT / 2 - MOTOR_HEIGHT]  # approximate IMU location
        imu_site = frame.add(
            "site",
            name="imu_site",
            pos=imu_pos,
        )

        self.body_quat = self.model.sensor.add(
            "framequat",
            objtype="site",
            objname="imu_site",
            name="framequat_sensor",
        )
        self.gyro = self.model.sensor.add("gyro", site=imu_site, name="gyro")
        self.acc = self.model.sensor.add(
            "accelerometer", site=imu_site, name="accelerometer", cutoff=9.81 * 10
        )


def make_arena() -> mjcf.RootElement:
    arena = mjcf.RootElement("arena")
    chequered = arena.asset.add(
        "texture",
        type="2d",
        builtin="checker",
        name="grid_texture",
        width=100,
        height=100,
        rgb1=[0.2, 0.3, 0.4],
        rgb2=[0.3, 0.4, 0.5],
    )
    grid = arena.asset.add(
        "material",
        name="grid",
        texture=chequered,
        texrepeat=[80, 80],
        reflectance=0.1,
    )
    arena.worldbody.add(
        "geom", name="floor", type="plane", size=[4, 4, 0.1], material=grid
    )
    for x in [-2, 2]:
        arena.worldbody.add(
            "light", name="light_{}".format(x), pos=[x, -1, 3], dir=[-x, 1, -2]
        )

    # Close three-quarter view of the robot, with world-up as the image vertical.
    arena.worldbody.add(
        "camera", name="overview", pos=[0.45, -0.85, 0.4],
        xyaxes=[0.85, 0.45, 0, -0.1035, 0.1955, 0.925],
    )

    return arena


def _get_action_space(
    actions: list[str], max_wheel_vel: float, max_wheel_acc: float,
    max_head_vel: float = 1.0,
) -> gymnasium.spaces.Dict:
    """Define action space for the robot.

    Args:
        actions: List of action keys to include
        max_wheel_vel: Maximum wheel velocity in rad/s
        max_wheel_acc: Maximum wheel acceleration in rad/s²

    Returns:
        Dict space with action ranges (all SI units)
    """
    d_: dict[str, gymnasium.Space] = {}
    for k in actions:
        # Head velocity commands (rad/s), independently of joint angle limits.
        if k in [Actions.VEL_HEAD_PITCH, Actions.VEL_HEAD_TURN]:
            d_[k] = gymnasium.spaces.Box(
                low=-max_head_vel,
                high=max_head_vel,
                shape=(1,),
                dtype=np.float32,  # rad/s
            )
        elif k in [Actions.VEL_LEFT_WHEEL, Actions.VEL_RIGHT_WHEEL]:
            d_[k] = gymnasium.spaces.Box(
                low=-max_wheel_vel, high=max_wheel_vel, shape=(1,), dtype=np.float32
            )
        elif k in [
            Actions.ACC_LEFT_WHEEL,
            Actions.ACC_RIGHT_WHEEL,
            Actions.ACC_BOTH_WHEELS,
            Actions.ACC_YAW_TURN,
        ]:
            d_[k] = gymnasium.spaces.Box(
                low=-max_wheel_acc, high=max_wheel_acc, shape=(1,), dtype=np.float32
            )
        else:
            raise ValueError(f"Invalid action key: {k}")
    return gymnasium.spaces.Dict(d_)


def _get_observation_space() -> gymnasium.spaces.Dict:
    spaces: dict[str, gymnasium.Space] = {
        key: gymnasium.spaces.Box(
            low=-np.inf, high=np.inf, shape=(key.dim(),), dtype=np.float32
        )
        for key in (*Observable, *DerivedObs, *Target)
    }
    return gymnasium.spaces.Dict(spaces)


class GymRP(gymnasium.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 100}

    def __init__(
        self,
        actions: list[str],
        record: bool = False,
        step_time: float = 0.01,
        randomize: bool = False,
        max_wheel_vel: float = 10.0,  # rad/s
        max_wheel_acc: float = 50.0,  # rad/s²
        reward_scales: dict[Observable, float] | None = None,
        random_scale: float = 0.02,
        target_pos: float = 0.0,
        target_trajectory: Sequence[Sequence[float]] | None = None,
        target_vel: float = 0.0,
        tracking_mode: str = "velocity",
        head_tracking: bool = False,
        head_target: Sequence[float] = (0.0, 0.0),
        head_trajectory: Sequence[Sequence[float]] | None = None,
        max_head_vel: float = 1.0,
        camera_view: str = "external",
    ):
        if tracking_mode not in ("position", "velocity", "none"):
            raise ValueError("tracking_mode must be position, velocity or none")
        self.tracking_mode = tracking_mode
        if camera_view not in ("external", "head", "both"):
            raise ValueError("camera_view must be external, head or both")
        if not np.isfinite(max_head_vel) or max_head_vel <= 0:
            raise ValueError("max_head_vel must be positive and finite")
        if head_tracking and not {
            Actions.VEL_HEAD_PITCH, Actions.VEL_HEAD_TURN
        } <= set(actions):
            raise ValueError("Head tracking requires pitch and yaw velocity actions")
        self.head_tracking = head_tracking
        self.camera_view = camera_view
        self.max_head_vel = max_head_vel
        self._randomize = randomize
        self._init_pitch_scale = 2.0
        self.max_wheel_vel = max_wheel_vel
        self.max_wheel_acc = max_wheel_acc
        self.random_scale = random_scale
        self.dm_env = self._reset_env()
        assert self.dm_env is not None

        self.simul_timestep = 0.002
        self.dm_env.model.opt.timestep = self.simul_timestep

        self.state = State(wheel_radius=WHEEL_D / 2, record=record)
        self.target_pos = target_pos
        self.target_trajectory = target_trajectory
        self.target_vel = target_vel
        self.head_target = head_target
        self.head_trajectory = head_trajectory

        self.step_time = step_time
        self._substeps = round(step_time / self.simul_timestep)
        if self._substeps < 1 or not np.isclose(
            self._substeps * self.simul_timestep, step_time
        ):
            raise ValueError("step_time must be a positive multiple of 0.002 seconds")
        self.render_mode = "rgb_array"
        self.metadata["render_fps"] = int(1 / self.step_time)

        self.action_space = _get_action_space(
            actions, self.max_wheel_vel, self.max_wheel_acc, self.max_head_vel
        )
        self.observation_space = _get_observation_space()
        self.reward_scales = {
            obs: 0.0 for obs in Observable if obs.value.startswith("reward/")
        }
        self.reward_scales.update(reward_scales or {})

    @property
    def target_pos(self) -> float:
        """Current fore/aft target in meters."""
        return self.state.target_pos

    @target_pos.setter
    def target_pos(self, value: float):
        self.state.target_pos = value

    @property
    def target_vel(self) -> float:
        return self.state.target_vel

    @target_vel.setter
    def target_vel(self, value: float):
        self.state.target_vel = value

    @property
    def target_trajectory(self) -> PositionTrajectory | None:
        return self.state.target_trajectory

    @target_trajectory.setter
    def target_trajectory(
        self, value: PositionTrajectory | Sequence[Sequence[float]] | None
    ):
        if value is not None and not isinstance(value, PositionTrajectory):
            value = PositionTrajectory(value)
        self.state.target_trajectory = value

    @property
    def head_target(self) -> np.ndarray:
        return self.state.head_target

    @head_target.setter
    def head_target(self, value: Sequence[float]):
        self.state.head_target = value

    @property
    def head_trajectory(self) -> HeadTrajectory | None:
        return self.state.head_trajectory

    @head_trajectory.setter
    def head_trajectory(self, value: HeadTrajectory | Sequence[Sequence[float]] | None):
        if value is not None and not isinstance(value, HeadTrajectory):
            value = HeadTrajectory(value)
        self.state.head_trajectory = value

    def _reset_env(self, seed: int | None = 42) -> mjcf.Physics:
        prng = np.random.default_rng(seed)

        # Make rp:
        rp = MujocoRP(
            max_wheel_vel=self.max_wheel_vel,
            max_wheel_acc=self.max_wheel_acc,
            seed=seed,
            randomize=self._randomize,
            random_scale=self.random_scale,
            max_head_vel=self.max_head_vel,
        )

        # Make arena:
        arena = make_arena()

        init_pitch = prng.normal(0, self._init_pitch_scale) if self._randomize else 0.0

        # Spawn rp at arena:
        xpos, ypos, zpos = 0.0, 0.0, rp.spawn_height
        spawn_site = arena.worldbody.add(
            "site",
            name="rp_site",
            pos=[xpos, ypos, zpos],
            axisangle=[0, 1, 0, init_pitch],
            group=3,
        )
        spawn_site.attach(rp.model).add("freejoint")
        overview = arena.find("camera", "overview")
        overview.mode = "targetbodycom"
        overview.target = rp.model.find("body", "torso")

        # Actuators:
        self.left_wheel_act = rp.model.find("actuator", "leftwheel_actuator")
        self.right_wheel_act = rp.model.find("actuator", "rightwheel_actuator")
        self.head_pitch_act = rp.model.find("actuator", "headpitch_actuator")
        self.head_turn_act = rp.model.find("actuator", "headturn_actuator")

        # Sensors:
        self.gyro_sens = rp.model.find("sensor", "gyro")
        self.acc_sens = rp.model.find("sensor", "accelerometer")
        self.head_pitch_sens = rp.model.find("sensor", "headpitch_sensor")
        self.head_turn_sens = rp.model.find("sensor", "headturn_sensor")
        self.head_pitch_vel_sens = rp.model.find("sensor", "headpitch_vel_sensor")
        self.head_turn_vel_sens = rp.model.find("sensor", "headturn_vel_sensor")
        self.camera_quat_sens = rp.model.find("sensor", "camera_framequat_sensor")
        self.head_camera_name = rp.model.find("camera", "head_camera").full_identifier
        self.body_quat = rp.model.find("sensor", "framequat_sensor")
        self.left_wheel_vel_sens = rp.model.find("sensor", "leftwheel_vel_sensor")
        self.right_wheel_vel_sens = rp.model.find("sensor", "rightwheel_vel_sensor")

        # Make environment:
        physics = mjcf.Physics.from_mjcf_model(arena)

        return physics

    def _get_obs(
        self, rewards: dict[Observable, float]
    ) -> dict[StateVarKey, np.ndarray]:
        """Export shared state plus reward diagnostics; no state updates."""
        return {
            **self.state.snapshot(),
            **{
                key: np.array([value], dtype=np.float32)
                for key, value in rewards.items()
            },
        }

    @property
    def simul_time(self) -> float:
        return self.dm_env.data.time

    def _read_sensors(self) -> dict[Observable, np.ndarray]:
        """Read a timestamped MuJoCo sensor packet without touching shared state.

        All observations are in SI units:
        - ACC: m/s² (accelerometer)
        - GYRO: rad/s (gyroscope)
        - HEAD_PITCH, HEAD_TURN: rad (joint angles)
        - LEFT_WHEEL_VEL, RIGHT_WHEEL_VEL: rad/s (from jointvel sensors)
        - TRUE_PITCH: rad (optional simulation diagnostic for shared State)
        """

        def _get_sens(sens):
            return self.dm_env.bind(sens).sensordata.copy()

        obs_d = {
            Observable.OBS_TIME: self.simul_time,  # seconds
            Observable.ACC: _get_sens(self.acc_sens),  # m/s²
            Observable.GYRO: _get_sens(self.gyro_sens),  # rad/s
            Observable.HEAD_PITCH: _get_sens(self.head_pitch_sens)[0],  # rad
            Observable.HEAD_TURN: _get_sens(self.head_turn_sens)[0],  # rad
            Observable.HEAD_PITCH_VEL: _get_sens(self.head_pitch_vel_sens)[0],
            Observable.HEAD_TURN_VEL: _get_sens(self.head_turn_vel_sens)[0],
            Observable.LEFT_WHEEL_VEL: _get_sens(self.left_wheel_vel_sens)[0],  # rad/s
            Observable.RIGHT_WHEEL_VEL: _get_sens(self.right_wheel_vel_sens)[
                0
            ],  # rad/s
            Observable.TRUE_PITCH: q2eul(
                self.dm_env.bind(self.body_quat).sensordata.copy()
            )[1],  # rad
            Observable.TRUE_CAMERA_PITCH: camera_elevation(
                as_rotation_matrix(_get_sens(self.camera_quat_sens))
            ),
        }

        return {key: np.atleast_1d(value) for key, value in obs_d.items()}

    def _calculate_rewards(
        self, state: State, *, terminated: bool
    ) -> dict[Observable, float]:
        """Calculate rewards from an explicitly supplied, already updated state."""

        step_reward = self.reward_scales[Observable.REWARD_STEP]

        fell_cost = (
            self.reward_scales[Observable.REWARD_FELL] if terminated else 0.0
        )
        pitch_reward = self.reward_scales[Observable.REWARD_RP_PITCH] * abs(
            state.obs.get_observable(Observable.RP_PITCH)[0]
        )

        wheel_vel_reward = (
            self.reward_scales[Observable.REWARD_WHEEL_VEL]
            * (
                abs(state.obs.get_observable(Observable.LEFT_WHEEL_VEL)[0])
                + abs(state.obs.get_observable(Observable.RIGHT_WHEEL_VEL)[0])
            )
            / 2
        )
        # An absolute-speed penalty would oppose nonzero velocity commands.
        if self.tracking_mode == "velocity":
            wheel_vel_reward = 0.0

        position_reward = 0.0
        velocity_reward = 0.0
        if self.tracking_mode == "position":
            position_reward = self.reward_scales[Observable.REWARD_POS] * abs(
                state.derived_obs[DerivedObs.CURRENT_POS][0] - state.target_pos
            )
        elif self.tracking_mode == "velocity":
            velocity_reward = self.reward_scales[Observable.REWARD_VEL] * abs(
                state.derived_obs[DerivedObs.CURRENT_VEL][0] - state.target_vel
            )

        components = {
            Observable.REWARD_STEP: step_reward,
            Observable.REWARD_RP_PITCH: pitch_reward,
            Observable.REWARD_WHEEL_VEL: wheel_vel_reward,
            Observable.REWARD_POS: position_reward,
            Observable.REWARD_VEL: velocity_reward,
            Observable.REWARD_FELL: fell_cost,
            **self._head_rewards(state),
        }
        components[Observable.REWARD_TOTAL] = (
            sum(components.values()) * self.reward_scales[Observable.REWARD_TOTAL]
        )
        return components

    def _head_rewards(self, state: State) -> dict[Observable, float]:
        neutral = camera = yaw = 0.0
        if self.head_tracking:
            # Score actual camera pose, never an estimator's belief about its pose.
            camera = self.reward_scales[Observable.REWARD_CAMERA_PITCH] * abs(
                state.obs.get_observable(Observable.TRUE_CAMERA_PITCH)[0]
                - state.targets[Target.CAMERA_PITCH_WORLD][0]
            )
            yaw = self.reward_scales[Observable.REWARD_HEAD_YAW] * abs(
                state.obs.get_observable(Observable.HEAD_TURN)[0]
                - state.targets[Target.HEAD_YAW_NECK][0]
            )
        else:
            neutral = self.reward_scales[Observable.REWARD_HEAD_PITCH] * abs(
                state.obs.get_observable(Observable.HEAD_PITCH)[0]
            )
        return {
            Observable.REWARD_HEAD_PITCH: neutral,
            Observable.REWARD_CAMERA_PITCH: camera,
            Observable.REWARD_HEAD_YAW: yaw,
        }

    def reset(
        self, options: Optional[Any] = None, seed: int | None = None
    ) -> tuple[dict, dict]:
        super().reset(seed=seed)
        self.dm_env = self._reset_env(seed)
        self.state.reset(self._read_sensors())
        # Reset has an initial observation, but no action or transition reward.
        initial_rewards = dict.fromkeys(self.reward_scales, 0.0)
        return self._get_obs(initial_rewards), {}

    @property
    def terminated(self) -> bool:
        true_pitch = q2eul(self.dm_env.bind(self.body_quat).sensordata.copy())[1]
        return bool(abs(true_pitch) > np.deg2rad(20))

    @property
    def truncated(self) -> bool:
        return self.simul_time >= 20.0

    def render(self):
        if self.camera_view == "both":
            # Two small views keep frame storage below the old 640x480 video size.
            return np.concatenate([
                self.dm_env.render(camera_id=0, height=240, width=320),
                self.dm_env.render(
                    camera_id=self.head_camera_name, height=240, width=320
                ),
            ], axis=1)
        camera = self.head_camera_name if self.camera_view == "head" else 0
        return self.dm_env.render(camera_id=camera, height=480, width=640)

    def _apply_action(self, action: Actions, value: np.ndarray):
        if action == Actions.TIME:
            return
        velocity_actuators = {
            Actions.VEL_HEAD_PITCH: self.head_pitch_act,
            Actions.VEL_HEAD_TURN: self.head_turn_act,
            Actions.VEL_LEFT_WHEEL: self.left_wheel_act,
            Actions.VEL_RIGHT_WHEEL: self.right_wheel_act,
        }
        if action in velocity_actuators:
            self.dm_env.bind(velocity_actuators[action]).ctrl = value[0]
            return
        if action == Actions.ACC_YAW_TURN:
            raise NotImplementedError("ACC_YAW_TURN not implemented yet")
        if action not in {
            Actions.ACC_LEFT_WHEEL,
            Actions.ACC_RIGHT_WHEEL,
            Actions.ACC_BOTH_WHEELS,
        }:
            raise ValueError(f"Invalid action key: {action}")

        acceleration = np.clip(value[0], -self.max_wheel_acc, self.max_wheel_acc)
        for wheel_action, sensor, actuator in (
            (Actions.ACC_LEFT_WHEEL, Observable.LEFT_WHEEL_VEL, self.left_wheel_act),
            (Actions.ACC_RIGHT_WHEEL, Observable.RIGHT_WHEEL_VEL, self.right_wheel_act),
        ):
            if action in (wheel_action, Actions.ACC_BOTH_WHEELS):
                velocity = self.state.obs.get_observable(sensor)[0]
                self.dm_env.bind(actuator).ctrl = np.clip(
                    velocity + acceleration * self.step_time,
                    -self.max_wheel_vel,
                    self.max_wheel_vel,
                )

    def step(
        self, action: dict[Actions, np.ndarray]
    ) -> tuple[dict, float, bool, bool, dict]:
        """Execute one environment step.

        All actions are in SI units:
        - VEL_*_WHEEL: rad/s (wheel target velocities)
        - ACC_*_WHEEL: rad/s² (wheel accelerations)
        - VEL_HEAD_PITCH, VEL_HEAD_TURN: rad/s (head velocities)
        - TIME: seconds (observation time for sync check, not applied as action)
        """
        # Check time synchronization if TIME action is present
        try:
            action_time = action[Actions.TIME]
        except KeyError:
            action_time = None

        if action_time is not None:
            # Check if times are roughly aligned (within half a timestep)
            time_diff = np.abs(action_time - self.simul_time)
            max_diff = self.step_time * 0.5  # Half timestep tolerance

            if time_diff > max_diff:
                warnings.warn(
                    f"Action time desync detected! "
                    f"Max diff: {time_diff.max():.6f}s (allowed: {max_diff:.6f}s)"
                )

        # Apply the actions at time t (all in SI units)
        for a, val in action.items():
            self._apply_action(a, val)

        # Step the MuJoCo environment t -> t + self.step_time.
        t0 = self.dm_env.data.time
        for _ in range(self._substeps):
            self.dm_env.step()

        # Keep acquisition, shared processing, reward and recording explicit.
        measurements = self._read_sensors()
        self.state.update(measurements)
        terminated = self.terminated
        rewards = self._calculate_rewards(self.state, terminated=terminated)
        self.state.record_transition(t0, action, rewards)

        reward = float(rewards[Observable.REWARD_TOTAL])
        return self._get_obs(rewards), reward, terminated, self.truncated, {}


def display_video(frames, framerate=30, fname: str = ""):
    height, width, _ = frames[0].shape
    dpi = 120
    fig, ax = plt.subplots(1, 1, figsize=(width / dpi, height / dpi), dpi=dpi)
    ax.set_axis_off()
    ax.set_aspect("equal")
    ax.set_position((0, 0, 1, 1))
    im = ax.imshow(frames[0])

    def update(frame):
        im.set_data(frame)
        return [im]

    interval = 1000 / framerate
    anim = animation.FuncAnimation(
        fig=fig, func=update, frames=frames, interval=interval, blit=True, repeat=False
    )

    if fname:
        anim.save(fname)
        plt.close()
    else:
        plt.show()


if __name__ == "__main__":
    # Make rp:
    rp = MujocoRP()

    # Make arena:
    arena = make_arena()

    # Spawn rp at arena:
    xpos, ypos, zpos = 0.0, 0.0, rp.spawn_height
    spawn_site = arena.worldbody.add(
        "site", name="rp_site", pos=[xpos, ypos, zpos], group=3
    )
    spawn_site.attach(rp.model).add("freejoint")  # "freejoint"

    # print(arena.to_xml_string())
    print(arena.to_xml_string())
    with open("rp_env.xml", "w") as text_file:
        text_file.write(arena.to_xml_string())

    dm_env = mjcf.Physics.from_mjcf_model(arena)

    frames = []
    FRAMERATE = 30
    for _ in range(2000):
        dm_env.step()
        if dm_env.data.time > (1.0 / FRAMERATE) * len(frames):
            frames.append(dm_env.render(camera_id=0, height=480, width=640))

    display_video(frames, FRAMERATE)

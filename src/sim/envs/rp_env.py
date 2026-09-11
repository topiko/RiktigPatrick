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

from filters.qutils import q2eul
from riktigpatric.patrick import (
    Actions,
    DerivedObs,
    Observable,
    State,
    StateVarKey,
    StepAction,
    Target,
)
from riktigpatric.trajectory import PositionTrajectory

BODY_D = 0.05
BODY_H = 0.25
BODY_W = 0.1
BODY_M = 0.4

WHEEL_D = BODY_D * 2

HEAD_D = 0.03
HEAD_H = 0.1
HEAD_W = 0.1
HEAD_M = 0.2

FORCERANGE = 15

# Unit conversion constants (for display/debugging only - not used in main code path)
RAD2REV = 1.0 / (2 * np.pi)  # rad/s → rev/s (revolutions per second)
RAD2DEG = 180.0 / np.pi  # rad/s → deg/s (degrees per second)


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
        frame.add(
            "geom",
            name="body",
            type="box",
            size=[BODY_D / 2, BODY_W / 2, BODY_H / 2],
            pos=[0, 0, BODY_H / 2],
            rgba=rgba,
            mass=body_m,
        )

        # Wheels
        wheel_d = rng.normal(WHEEL_D, WHEEL_D * random_scale) if randomize else WHEEL_D
        kp_wheel = 20.0  # was 1.2
        for diry, key in zip([-1, 1], ["rightwheel", "leftwheel"]):
            y = diry * (BODY_W / 2 + 0.001)
            # Wheel
            wheel = frame.add("body", name=key, pos=[-0.001, y, 0])
            wheel.add(
                "geom",
                type="cylinder",
                name=key + "_cyl",
                fromto=[0, 0, 0, 0, diry * 0.02, 0],
                friction=(2, 0.005, 0.0001),
                size=[wheel_d / 2],
                mass=0.020,  # kg
            )
            if wheel_markers:
                wheel.add(
                    "geom",
                    type="cylinder",
                    name=key + "_marker",
                    fromto=[0, 0, wheel_d / 4, 0, diry * 0.021, wheel_d / 4],
                    friction=(2, 0.005, 0.0001),
                    rgba=[0, 0, 0, 1],
                    size=[wheel_d / 12],
                )

            # Wheel joint
            wheel = wheel.add("joint", name=key + "_joint", axis=[0, 1, 0], damping=0.1)

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

        head = self.model.worldbody.add(
            "body", name="head", pos=[BODY_D / 2, 0, BODY_H]
        )

        # Joints
        head_pitch = head.add(
            "joint", name="headpitch_joint", axis=[0, 1, 0], damping=0.1
        )
        self.model.sensor.add(
            "jointpos", name="headpitch_sensor", joint="headpitch_joint"
        )

        head_lr = head.add("joint", name="headturn_joint", axis=[0, 0, 1], damping=0.1)
        self.model.sensor.add(
            "jointpos", name="headturn_sensor", joint="headturn_joint"
        )

        head_m = rng.normal(HEAD_M, HEAD_M * random_scale) if randomize else HEAD_M
        head.add(
            "geom",
            type="box",
            name="headgeom",
            size=[HEAD_D / 2, HEAD_W / 2, HEAD_H / 2],
            pos=[0, 0, HEAD_H / 2],
            rgba=rgba,
            mass=head_m,
        )

        self.model.actuator.add(
            "intvelocity",
            name="headpitch_actuator",
            joint=head_pitch,
            kp=kp_servo,
            ctrllimited=True,
            ctrlrange=[-1, 1],  # rad/s - commanded velocity
            actrange=[-1, 1],  # rad - integrated position target
        )
        self.model.actuator.add(
            "intvelocity",
            name="headturn_actuator",
            joint=head_lr,
            kp=kp_servo,
            ctrllimited=True,
            ctrlrange=[-1, 1],  # rad/s - commanded velocity
            actrange=[-1, 1],  # rad - integrated position target
        )

        # Sensors:
        imu_pos = [0, 0, BODY_H / 2]  # IMU at center of body
        imu_site = self.model.worldbody.add(
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

    # TODO: use quat for better camera angle
    camera_site = arena.worldbody.add(
        "site", name="camerasite", pos=[0.1, -3, 2.5], euler=[50, 0, 0]
    )
    camera = mjcf.RootElement("camera")
    camera.worldbody.add("camera", name="camera", mode="trackcom")
    camera_site.attach(camera)

    return arena


def _get_action_space(
    actions: list[str], max_wheel_vel: float, max_wheel_acc: float
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
        # Head velocity control (hardcoded ±1 rad/s)
        if k in [Actions.VEL_HEAD_PITCH, Actions.VEL_HEAD_TURN]:
            d_[k] = gymnasium.spaces.Box(
                low=-1.0,
                high=1.0,
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
    d_: dict[str, gymnasium.Space] = {}

    for obs in Observable:
        if obs in [
            Observable.OBS_TIME,
            Observable.HEAD_PITCH,
            Observable.HEAD_TURN,
            Observable.LEFT_WHEEL_VEL,
            Observable.RIGHT_WHEEL_VEL,
            Observable.RP_PITCH,
            Observable.TRUE_PITCH,
            Observable.REWARD_STEP,
            Observable.REWARD_FELL,
            Observable.REWARD_RP_PITCH,
            Observable.REWARD_WHEEL_VEL,
            Observable.REWARD_HEAD_PITCH,
            Observable.REWARD_POS,
            Observable.REWARD_TOTAL,
        ]:
            d_[obs] = gymnasium.spaces.Box(
                low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32
            )
        elif obs in [Observable.ACC, Observable.GYRO]:
            d_[obs] = gymnasium.spaces.Box(
                low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32
            )
        else:
            raise ValueError(f"Invalid observation key: {obs}")

    for derived_obs in DerivedObs:
        if derived_obs == DerivedObs.CURRENT_POS:
            d_[derived_obs] = gymnasium.spaces.Box(
                low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32
            )
        else:
            raise ValueError(f"Invalid derived observation key: {derived_obs}")

    for target in Target:
        if target == Target.TARGET_POS:
            d_[target] = gymnasium.spaces.Box(
                low=-np.inf, high=np.inf, shape=(1,), dtype=np.float32
            )
        else:
            raise ValueError(f"Invalid target key: {target}")

    return gymnasium.spaces.Dict(d_)


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
    ):
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

        self.step_time = step_time
        self._substeps = round(step_time / self.simul_timestep)
        if self._substeps < 1 or not np.isclose(
            self._substeps * self.simul_timestep, step_time
        ):
            raise ValueError("step_time must be a positive multiple of 0.002 seconds")
        self.render_mode = "rgb_array"
        self.metadata["render_fps"] = int(1 / self.step_time)

        self.action_space = _get_action_space(
            actions, self.max_wheel_vel, self.max_wheel_acc
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
    def target_trajectory(self) -> PositionTrajectory | None:
        return self.state.target_trajectory

    @target_trajectory.setter
    def target_trajectory(
        self, value: PositionTrajectory | Sequence[Sequence[float]] | None
    ):
        if value is not None and not isinstance(value, PositionTrajectory):
            value = PositionTrajectory(value)
        self.state.target_trajectory = value

    def _reset_env(self, seed: int | None = 42) -> mjcf.Physics:
        prng = np.random.default_rng(seed)

        # Make rp:
        rp = MujocoRP(
            max_wheel_vel=self.max_wheel_vel,
            max_wheel_acc=self.max_wheel_acc,
            seed=seed,
            randomize=self._randomize,
            random_scale=self.random_scale,
        )

        # Make arena:
        arena = make_arena()

        init_pitch = prng.normal(0, self._init_pitch_scale) if self._randomize else 0.0

        # Spawn rp at arena:
        xpos, ypos, zpos = 0.0, 0.0, WHEEL_D / 2
        spawn_site = arena.worldbody.add(
            "site",
            name="rp_site",
            pos=[xpos, ypos, zpos],
            axisangle=[0, 1, 0, init_pitch],
            group=3,
        )
        spawn_site.attach(rp.model).add("freejoint")

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
        self.body_quat = rp.model.find("sensor", "framequat_sensor")
        self.left_wheel_vel_sens = rp.model.find("sensor", "leftwheel_vel_sensor")
        self.right_wheel_vel_sens = rp.model.find("sensor", "rightwheel_vel_sensor")

        # Make environment:
        physics = mjcf.Physics.from_mjcf_model(arena)

        return physics

    def _get_obs(self) -> dict[StateVarKey, np.ndarray]:
        d: dict[StateVarKey, np.ndarray] = {
            **self.state.obs.to_dict(),
            **self.state.derived_obs,
            **self.state.targets,
        }

        return {key: value.astype(np.float32, copy=True) for key, value in d.items()}

    @property
    def simul_time(self) -> float:
        return self.dm_env.data.time

    def _update_obs(self, first: bool = False):
        """Update observations from MuJoCo sensors.

        All observations are in SI units:
        - ACC: m/s² (accelerometer)
        - GYRO: rad/s (gyroscope)
        - HEAD_PITCH, HEAD_TURN: rad (joint angles)
        - LEFT_WHEEL_VEL, RIGHT_WHEEL_VEL: rad/s (from jointvel sensors)
        - TRUE_PITCH, RP_PITCH: rad
        """

        def _get_sens(sens):
            return self.dm_env.bind(sens).sensordata.copy()

        obs_d = {
            Observable.OBS_TIME: self.simul_time,  # seconds
            Observable.ACC: _get_sens(self.acc_sens),  # m/s²
            Observable.GYRO: _get_sens(self.gyro_sens),  # rad/s
            Observable.HEAD_PITCH: _get_sens(self.head_pitch_sens)[0],  # rad
            Observable.HEAD_TURN: _get_sens(self.head_turn_sens)[0],  # rad
            Observable.LEFT_WHEEL_VEL: _get_sens(self.left_wheel_vel_sens)[0],  # rad/s
            Observable.RIGHT_WHEEL_VEL: _get_sens(self.right_wheel_vel_sens)[
                0
            ],  # rad/s
            Observable.TRUE_PITCH: q2eul(
                self.dm_env.bind(self.body_quat).sensordata.copy()
            )[1],  # rad
            Observable.RP_PITCH: self.state.euler[1],  # rad; updated below
        }

        self.state.update_obs(
            {key: np.atleast_1d(value) for key, value in obs_d.items()}
        )
        if not first:
            self.state.step()

        # Rewards and filtered pitch must describe the newly observed state.
        rew_d = self._get_reward(first=first)
        reward_arrays = {key: np.array([value]) for key, value in rew_d.items()}
        self.state.obs.set_observables(reward_arrays)
        if not first:
            self.state.update_rewards(reward_arrays)

    def _get_reward(self, first: bool = False) -> dict[Observable, float]:
        if first:
            return dict.fromkeys(self.reward_scales, 0.0)

        step_reward = self.reward_scales[Observable.REWARD_STEP]

        fell_cost = (
            self.reward_scales[Observable.REWARD_FELL] if self.terminated else 0.0
        )
        pitch_reward = self.reward_scales[Observable.REWARD_RP_PITCH] * abs(
            self.state.obs.get_observable(Observable.RP_PITCH)[0]
        )

        wheel_vel_reward = (
            self.reward_scales[Observable.REWARD_WHEEL_VEL]
            * (
                abs(self.state.obs.get_observable(Observable.LEFT_WHEEL_VEL)[0])
                + abs(self.state.obs.get_observable(Observable.RIGHT_WHEEL_VEL)[0])
            )
            / 2
        )

        head_pitch_reward = self.reward_scales[Observable.REWARD_HEAD_PITCH] * abs(
            self.state.obs.get_observable(Observable.HEAD_PITCH)[0]
        )
        position_reward = self.reward_scales[Observable.REWARD_POS] * abs(
            self.state.derived_obs[DerivedObs.CURRENT_POS][0] - self.target_pos
        )

        total = (
            step_reward
            + fell_cost
            + pitch_reward
            + wheel_vel_reward
            + head_pitch_reward
            + position_reward
        )

        # We need to list all obrservable rewards here...
        return {
            Observable.REWARD_STEP: step_reward,
            Observable.REWARD_RP_PITCH: pitch_reward,
            Observable.REWARD_TOTAL: total
            * self.reward_scales[Observable.REWARD_TOTAL],
            Observable.REWARD_WHEEL_VEL: wheel_vel_reward,
            Observable.REWARD_HEAD_PITCH: head_pitch_reward,
            Observable.REWARD_POS: position_reward,
            Observable.REWARD_FELL: fell_cost,
        }

    def reset(
        self, options: Optional[Any] = None, seed: int | None = None
    ) -> tuple[dict, dict]:
        super().reset(seed=seed)
        self.dm_env = self._reset_env(seed)
        self.state.reset()

        self._update_obs(first=True)

        return self._get_obs(), {}

    @property
    def terminated(self) -> bool:
        true_pitch = q2eul(self.dm_env.bind(self.body_quat).sensordata.copy())[1]
        return bool(abs(true_pitch) > np.deg2rad(20))

    @property
    def truncated(self) -> bool:
        return self.simul_time >= 20.0

    def render(self):
        if self.render_mode == "rgb_array":
            return self.dm_env.render(camera_id=0, height=480, width=640)

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

        self._prev_action = StepAction(action)

        # Step the MuJoCo environment t -> t + self.step_time.
        t0 = self.dm_env.data.time
        for _ in range(self._substeps):
            self.dm_env.step()

        # The action was taken at t0
        self.state.update_action(t0, action)

        # Read sensors, update filter/odometry, then compute the reward at t + dt.
        self._update_obs()

        # The reward is received at time t
        reward = self.state.obs.get_observable(Observable.REWARD_TOTAL)[0]

        return self._get_obs(), reward, self.terminated, self.truncated, {}


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
    xpos, ypos, zpos = 0.0, 0.0, WHEEL_D / 2
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

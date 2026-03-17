from typing import Any, Optional

import gymnasium
import gymnasium.spaces
import matplotlib.pyplot as plt
import numpy as np
from dm_control import mjcf

from filters.qutils import q2eul
from riktigpatric.patrick import Actions, Observables, State, StepAction

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

MAX_WHEEL_VEL = 10
MAX_WHEEL_ACC = 100

RAD2REV = 1.0 / (2 * np.pi)  # rad/s → rev/s
RAD2DEG = 180.0 / np.pi  # rad/s → deg/s


class MujocoRP:
    def __init__(
        self,
        rgba: list[float] = [0.20269912, 0.4307427, 0.33218761, 1.0],
        wheel_markers: bool = True,
        seed: Optional[int] = None,
    ):
        """

        Args:
            rgba:
            markers:
            seed: random seed to introduce variance None for deterministic env. NOT IMPLEMENTED

        """
        # TODO: seed to introduce variance to RP

        self.model = mjcf.RootElement("frame")

        # Body:
        frame = self.model.worldbody.add("body", name="torso")

        body = frame.add(
            "geom",
            name="body",
            type="box",
            size=[BODY_D / 2, BODY_W / 2, BODY_H / 2],
            pos=[0, 0, BODY_H / 2],
            rgba=rgba,
            mass=BODY_M,
        )

        # Wheels
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
                size=[WHEEL_D / 2],
                mass=0.020,  # kg
            )
            if wheel_markers:
                wheel.add(
                    "geom",
                    type="cylinder",
                    name=key + "_marker",
                    fromto=[0, 0, WHEEL_D / 4, 0, diry * 0.021, WHEEL_D / 4],
                    friction=(2, 0.005, 0.0001),
                    rgba=[0, 0, 0, 1],
                    size=[WHEEL_D / 12],
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
                ctrlrange=[-MAX_WHEEL_VEL, MAX_WHEEL_VEL],
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

        head.add(
            "geom",
            type="box",
            name="headgeom",
            size=[HEAD_D / 2, HEAD_W / 2, HEAD_H / 2],
            pos=[0, 0, HEAD_H / 2],
            rgba=rgba,
            mass=HEAD_M,
        )

        self.model.actuator.add(
            "intvelocity",
            name="headpitch_actuator",
            joint=head_pitch,
            kp=kp_servo,
            actrange=[-1, 1],  # <--- range in radians
        )
        self.model.actuator.add(
            "intvelocity",
            name="headturn_actuator",
            joint=head_lr,
            kp=kp_servo,
            actrange=[-1, 1],  # <--- range in radians
        )

        # Sensors:
        imu_site = self.model.worldbody.add(
            "site",
            name="imu_site",
            pos=[0, 0, 0],
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


def _get_action_space(actions: list[str]) -> gymnasium.spaces.Dict:
    d_ = {}
    for k in actions:
        if k in [Actions.HEAD_PITCH, Actions.HEAD_TURN]:
            d_[k] = gymnasium.spaces.Box(
                low=-1.0, high=1.0, shape=(1,), dtype=np.float32
            )
        elif k in [Actions.VEL_LEFT_WHEEL, Actions.VEL_RIGHT_WHEEL]:
            d_[k] = gymnasium.spaces.Box(
                low=-MAX_WHEEL_VEL, high=MAX_WHEEL_VEL, shape=(1,), dtype=np.float32
            )
        elif k in [
            Actions.ACC_LEFT_WHEEL,
            Actions.ACC_RIGHT_WHEEL,
            Actions.ACC_BOTH_WHEELS,
            Actions.ACC_YAW_TURN,
        ]:
            d_[k] = gymnasium.spaces.Box(
                low=-MAX_WHEEL_ACC, high=MAX_WHEEL_ACC, shape=(1,), dtype=np.float32
            )
        else:
            raise ValueError(f"Invalid action key: {k}")
    return gymnasium.spaces.Dict(d_)


def _get_observation_space() -> gymnasium.spaces.Dict:
    d_ = {}
    for obs in Observables:
        if obs in [
            Observables.OBS_TIME,
            Observables.HEAD_PITCH,
            Observables.HEAD_TURN,
            Observables.LEFT_WHEEL_VEL,
            Observables.RIGHT_WHEEL_VEL,
            Observables.RP_PITCH,
        ]:
            d_[obs] = gymnasium.spaces.Box(
                low=-100.0, high=100.0, shape=(1,), dtype=np.float32
            )
        elif obs in [Observables.ACC, Observables.GYRO]:
            d_[obs] = gymnasium.spaces.Box(
                low=-100.0, high=100.0, shape=(3,), dtype=np.float32
            )
        elif obs == Observables.TRUE_PITCH:
            d_[obs] = gymnasium.spaces.Box(
                low=-180.0, high=180.0, shape=(1,), dtype=np.float32
            )
        else:
            raise ValueError(f"Invalid observation key: {obs}")

    return gymnasium.spaces.Dict(d_)


class GymRP(gymnasium.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 100}

    def __init__(
        self,
        record: bool = False,
        step_time: float = 0.01,
        randomize: bool = False,
        actions: list[str] = None,
    ):
        self._randomize = randomize
        self._init_pitch_scale = 2.0
        self.dm_env = self._reset_env()
        assert self.dm_env is not None

        self.simul_timestep = 0.002
        self.dm_env.model.opt.timestep = self.simul_timestep

        self.state = State(record=record)

        self.step_time = step_time
        self.render_mode = "rgb_array"
        self.metadata["render_fps"] = int(1 / self.step_time)

        self.action_space = _get_action_space(actions)
        self.observation_space = _get_observation_space()

    def _reset_env(self, seed: int | None = 42) -> mjcf.Physics:
        prng = np.random.default_rng(seed)

        # Make rp:
        rp = MujocoRP()

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

    def _get_obs(self) -> dict[str, np.ndarray]:
        return self.state.obs.to_dict()

    @property
    def simul_time(self) -> float:
        return self.dm_env.data.time

    def _update_obs(self):
        def _get_sens(sens):
            return self.dm_env.bind(sens).sensordata.copy()

        self.state.update_obs(
            {
                Observables.OBS_TIME: self.simul_time,
                Observables.ACC: _get_sens(self.acc_sens),
                Observables.GYRO: _get_sens(self.gyro_sens),
                Observables.HEAD_PITCH: _get_sens(self.head_pitch_sens)[0],
                Observables.HEAD_TURN: _get_sens(self.head_turn_sens)[0],
                Observables.LEFT_WHEEL_VEL: _get_sens(self.left_wheel_vel_sens)[0],
                Observables.RIGHT_WHEEL_VEL: _get_sens(self.right_wheel_vel_sens)[0],
                Observables.TRUE_PITCH: q2eul(
                    self.dm_env.bind(self.body_quat).sensordata.copy()
                )[1]
                / np.pi
                * 180,
                Observables.RP_PITCH: self.state.euler[1],
            }
        )

    def _get_reward(self) -> tuple[float, dict]:
        step_reward = 1

        return step_reward, {"step_reward": step_reward}

    def _get_info(self) -> dict:
        return {}

    def reset(
        self, options: Optional[Any] = None, seed: int | None = None
    ) -> tuple[dict, dict]:
        self.dm_env = self._reset_env(seed)
        self.state.reset()

        self._update_obs()

        d, i = self._get_obs(), self._get_info()
        return d, i

    @property
    def terminated(self) -> bool:
        return abs(self.state.euler[1]) > 20

    @property
    def truncated(self) -> bool:
        return self.state.obs.get_observable(Observables.OBS_TIME) > 20

    def render(self):
        if self.render_mode == "rgb_array":
            return self.dm_env.render(camera_id=0, height=480, width=640)

    def step(
        self, action_d: dict[Actions, np.ndarray]
    ) -> tuple[dict, float, bool, bool, dict]:
        rvel = self.state.obs.get_observable(Observables.RIGHT_WHEEL_VEL)[0]
        lvel = self.state.obs.get_observable(Observables.LEFT_WHEEL_VEL)[0]
        # Apply the actions at time t.
        for a, val in action_d.items():
            if a == Actions.HEAD_PITCH:
                self.dm_env.bind(self.head_pitch_act).ctrl = val[0]
            elif a == Actions.HEAD_TURN:
                self.dm_env.bind(self.head_turn_act).ctrl = val[0]
            elif a == Actions.VEL_LEFT_WHEEL:
                self.dm_env.bind(self.left_wheel_act).ctrl = val[0]

            elif a == Actions.VEL_RIGHT_WHEEL:
                self.dm_env.bind(self.right_wheel_act).ctrl = val[0]

            elif a == Actions.ACC_LEFT_WHEEL:
                left_vel = (
                    self.state.obs.get_observable(Observables.LEFT_WHEEL_VEL)[0]
                    + val[0] * self.step_time
                )
                left_vel = np.clip(left_vel, -MAX_WHEEL_VEL, MAX_WHEEL_VEL)
                self.dm_env.bind(self.left_wheel_act).ctrl = left_vel
            elif a == Actions.ACC_RIGHT_WHEEL:
                right_vel = rvel + val[0] * self.step_time
                right_vel = np.clip(right_vel, -MAX_WHEEL_VEL, MAX_WHEEL_VEL)
                self.dm_env.bind(self.right_wheel_act).ctrl = right_vel

            elif a == Actions.ACC_BOTH_WHEELS:
                rvel = np.clip(
                    rvel + val[0] * self.step_time, -MAX_WHEEL_VEL, MAX_WHEEL_VEL
                )
                lvel = np.clip(
                    lvel + val[0] * self.step_time, -MAX_WHEEL_VEL, MAX_WHEEL_VEL
                )
                self.dm_env.bind(self.right_wheel_act).ctrl = rvel
                self.dm_env.bind(self.left_wheel_act).ctrl = lvel

            elif a == Actions.ACC_YAW_TURN:
                raise NotImplementedError("ACC_YAW_TURN not implemented yet")

            else:
                raise ValueError(f"Invalid action key: {a}")

        self._prev_action = StepAction(action_d)

        # Step the MuJoCo environment t -> t + self.step_time.
        t0 = self.dm_env.data.time
        t = t0
        while t < t0 + self.step_time:
            self.dm_env.step()
            t = self.dm_env.data.time

        # Read the observation at time t + self.step_time.
        self._update_obs()

        # The action was taken at t0
        self.state.update_action(t0, action_d)

        # The reward is received at time t
        reward, reward_info = self._get_reward()
        self.state.update_rewards(reward_info)

        # The state needs a step as ewll
        self.state.step()

        info = self._get_info()

        return (
            self._get_obs(),
            reward,
            self.terminated,
            self.truncated,
            info,
        )


def display_video(frames, framerate=30, fname: str = ""):
    height, width, _ = frames[0].shape
    dpi = 120
    fig, ax = plt.subplots(1, 1, figsize=(width / dpi, height / dpi), dpi=dpi)
    ax.set_axis_off()
    ax.set_aspect("equal")
    ax.set_position([0, 0, 1, 1])
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

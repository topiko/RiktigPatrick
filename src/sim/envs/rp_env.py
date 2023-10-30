from typing import Any, Optional, Union

import gymnasium
import matplotlib.pyplot as plt
import numpy as np
import torch
from dm_control import mjcf
from filters.qutils import q2eul
from gymnasium import spaces
from riktigpatric.patrick import State, StepAction, StepReturn

BODY_D = 0.05
BODY_H = 0.25  # 0.25
BODY_W = 0.1
BODY_M = 0.4  # 0.400

WHEEL_D = BODY_D * 2

HEAD_D = 0.03
HEAD_H = 0.1
HEAD_W = 0.1
HEAD_M = 0.2  # 0.200

FORCERANGE = 15
MAXV = 20
MAXA = MAXV / 0.5


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
                ctrlrange=[-MAXV, MAXV],
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


class GymRP(gymnasium.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 100}

    def __init__(
        self,
        state_keys: list[str],
        render_mode="rgb_array",
        record: bool = False,
        lock_head: bool = True,
        ctrl_mode: str = "vel",
        step_time: float = 0.01,
    ):
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

        # Make environment:
        self.simul_timestep = 0.002  # MuJoCo default 0.002
        self.dm_env = mjcf.Physics.from_mjcf_model(arena)
        assert self.dm_env is not None

        self.dm_env.model.opt.timestep = self.simul_timestep

        # Actuators:
        self.left_wheel_act = rp.model.find("actuator", "leftwheel_actuator")
        self.right_wheel_act = rp.model.find("actuator", "rightwheel_actuator")

        # Sensors:
        self.gyro_sens = rp.model.find("sensor", "gyro")
        self.acc_sens = rp.model.find("sensor", "accelerometer")
        self.head_pitch_sens = rp.model.find("sensor", "headpitch_sensor")
        self.head_turn_sens = rp.model.find("sensor", "headturn_sensor")
        self.body_quat = rp.model.find("sensor", "framequat_sensor")
        self.left_wheel_vel_sens = rp.model.find("sensor", "leftwheel_vel_sensor")
        self.right_wheel_vel_sens = rp.model.find("sensor", "rightwheel_vel_sensor")

        self.state = State(keys=state_keys, record=record)

        self.observation_space = self.state.to_obs_space()

        # TODO: import these from somwehere
        max_w_wheel = np.pi * 2 * 5
        max_w_head = np.pi * 2

        action_space = {
            "act/left_wheel": spaces.Box(
                -max_w_wheel, max_w_wheel, shape=(1,), dtype=float
            ),
            "act/right_wheel": spaces.Box(
                -max_w_wheel, max_w_wheel, shape=(1,), dtype=float
            ),
        }

        if not lock_head:
            self.head_pitch_act = rp.model.find("actuator", "headpitch_actuator")
            self.head_turn_act = rp.model.find("actuator", "headturn_actuator")
            action_space.update(
                {
                    "act/head_pitch": spaces.Box(
                        -max_w_head, max_w_head, shape=(1,), dtype=float
                    ),
                    "act/head_turn": spaces.Box(
                        -max_w_head, max_w_head, shape=(1,), dtype=float
                    ),
                }
            )

        self.lock_head = lock_head
        self.action_space = spaces.Dict(action_space)
        self.render_mode = render_mode
        self.step_time = step_time  # s
        self.metadata["render_fps"] = int(1 / self.step_time)
        self.ctrl_mode = ctrl_mode
        self._prev_action = StepAction()

    def _update_state(self):
        body_quat = self.dm_env.bind(self.body_quat).sensordata.copy()
        pitch = q2eul(body_quat)[1] / np.pi * 180

        self.state.update(
            t=self.dm_env.data.time,
            acc=self.dm_env.bind(self.acc_sens).sensordata.copy(),
            gyro=self.dm_env.bind(self.gyro_sens).sensordata.copy(),
            head_pitch=self.dm_env.bind(self.head_pitch_sens).sensordata.copy()[0],
            head_turn=self.dm_env.bind(self.head_turn_sens).sensordata.copy()[0],
            left_wheel_vel=self.dm_env.bind(self.left_wheel_vel_sens).sensordata.copy()[
                0
            ],
            right_wheel_vel=self.dm_env.bind(
                self.right_wheel_vel_sens
            ).sensordata.copy()[0],
            true_pitch=pitch,
            action=self._prev_action,
            reward=self._get_reward(),
        )

    def _get_obs(self) -> dict:
        d = self.state.get_state_dict(keys="all")
        return d

    def _get_reward(self) -> float:
        rew = 1
        dir_rew = (
            -((self.state.obs.left_wheel_vel - self.state.obs.right_wheel_vel)[0] ** 2)
            / (2 * MAXV) ** 2
            / 100
        )

        return rew + dir_rew

    def _get_info(self) -> dict:
        return {}

    def reset(
        self, options: Optional[Any] = None, seed: Optional[int] = None
    ) -> tuple[dict, dict]:
        self.dm_env.reset()
        self.state.reset()

        d, i = self._get_obs(), self._get_info()
        return d, i

    @property
    def terminated(self) -> bool:
        return abs(self.state.euler[1]) > 10

    @property
    def truncated(self) -> bool:
        return False

    def render(self):
        if self.render_mode == "rgb_array":
            return self.dm_env.render(camera_id=0, height=480, width=640)

    def step(
        self, action: Optional[Union[dict[str, np.ndarray], StepAction]] = None
    ) -> tuple[dict, float, bool, bool, dict]:
        if action is not None:
            if isinstance(action, dict):
                action = StepAction().from_dict(action)

            self._prev_action = action

            if self.ctrl_mode == "acc":
                mul_ = self.step_time
                vl = self.state.obs.left_wheel_vel
                vr = self.state.obs.right_wheel_vel
            elif self.ctrl_mode == "vel":
                mul_ = 1
                vl = 0
                vr = 0
            else:
                raise KeyError(f"Invalid ctrl_mode {self.ctrl_mode}")

            self.dm_env.bind(self.left_wheel_act).ctrl = action.left_wheel * mul_ + vl
            self.dm_env.bind(self.right_wheel_act).ctrl = (
                action.right_wheel * mul_ + vr
            )  # rad/s
            if not self.lock_head:
                self.dm_env.bind(self.head_pitch_act).ctrl = action.head_pitch  # rad/s
                self.dm_env.bind(self.head_turn_act).ctrl = action.head_turn  # rad/s

        if self.step_time is None:
            step_time = self.dm_env.timestep()
        else:
            step_time = self.step_time

        t0 = self.dm_env.data.time
        t = t0
        while t < t0 + step_time:
            self.dm_env.step()

            t = self.dm_env.data.time

        self._update_state()

        return (
            self._get_obs(),
            self._get_reward(),
            self.terminated,
            self.truncated,
            self._get_info(),
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

import torch
import gymnasium
from gymnasium import spaces

from typing import Optional
from typing import Any

import numpy as np
import matplotlib.pyplot as plt

from dm_control import mjcf

from riktigpatric.patrick import State
from riktigpatric.patrick import StepReturn
from riktigpatric.patrick import StepAction

from utils import display_video

BODY_D = 0.05
BODY_H = 0.25
BODY_W = 0.1
BODY_M = 400

WHEEL_D = BODY_D * 2

HEAD_D = 0.03
HEAD_H = 0.1
HEAD_W = 0.1
HEAD_M = 200


class MujocoRP:
    def __init__(
        self,
        rgba: list[float] = [0.20269912, 0.4307427, 0.33218761, 1.0],
        markers: bool = True,
    ):
        self.model = mjcf.RootElement("frame")

        # Body:
        frame = self.model.worldbody.add("body", name="torso")

        frame.add(
            "geom",
            name="body",
            type="box",
            size=[BODY_D / 2, BODY_W / 2, BODY_H / 2],
            pos=[0, 0, BODY_H / 2],
            rgba=rgba,
            mass=BODY_M,
        )

        # Wheels
        kv_wheel = 3200
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
                mass=20,  # g
            )
            if markers:
                wheel.add(
                    "geom",
                    type="cylinder",
                    name=key + "_marker",
                    fromto=[0, 0, WHEEL_D / 4, 0, diry * 0.021, WHEEL_D / 4],
                    friction=(2, 0.005, 0.0001),
                    rgba=[0, 0, 0, 1],
                    size=[WHEEL_D / 12],
                )

            # Weel joint
            wheel = wheel.add("joint", name=key + "_joint", axis=[0, 1, 0])

            # Wheel actuator
            self.model.actuator.add(
                "intvelocity",
                name=key + "_actuator",
                joint=wheel,
                gear=(1,),
                actrange=[-50, 50],
                kp=kv_wheel,  # <-- velocity feedback gain
            )

        # Head:
        kp_servo = 2500

        head = self.model.worldbody.add(
            "body", name="head", pos=[BODY_D / 2, 0, BODY_H]
        )

        # Joints
        head_pitch = head.add("joint", name="headpitch_joint", axis=[0, 1, 0])
        self.model.sensor.add(
            "jointpos", name="headpitch_sensor", joint="headpitch_joint"
        )

        head_lr = head.add("joint", name="headturn_joint", axis=[0, 0, 1])
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
            actrange=[-50, 50],
        )
        self.model.actuator.add(
            "intvelocity",
            name="headturn_actuator",
            joint=head_lr,
            kp=kp_servo,
            actrange=[-50, 50],
        )

        # Sensors:
        imu_site = self.model.worldbody.add(
            "site",
            name="imu_site",
            pos=[0, 0, 0],
        )

        self.gyro = self.model.sensor.add("gyro", site=imu_site, name="gyro")
        self.acc = self.model.sensor.add(
            "accelerometer", site=imu_site, name="accelerometer"
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
    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(self, render_mode="rgb_array"):
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
        self.head_pitch_act = rp.model.find("actuator", "headpitch_actuator")
        self.head_turn_act = rp.model.find("actuator", "headturn_actuator")

        # Sensors:
        self.gyro_sens = rp.model.find("sensor", "gyro")
        self.acc_sens = rp.model.find("sensor", "accelerometer")
        self.head_picth_sens = rp.model.find("sensor", "headpitch_sensor")
        self.head_turn_sens = rp.model.find("sensor", "headturn_sensor")

        self.state = State()

        self.observation_space = self.state.to_obs_space()

        # TODO: import these from somwehere
        max_w_wheel = np.pi * 2 * 5
        max_w_head = np.pi * 2
        self.action_space = spaces.Dict(
            {
                "left_wheel": spaces.Box(
                    -max_w_wheel, max_w_wheel, shape=(1,), dtype=float
                ),
                "right_wheel": spaces.Box(
                    -max_w_wheel, max_w_wheel, shape=(1,), dtype=float
                ),
                "head_pitch_v": spaces.Box(
                    -max_w_head, max_w_head, shape=(1,), dtype=float
                ),
                "head_turn_v": spaces.Box(
                    -max_w_head, max_w_head, shape=(1,), dtype=float
                ),
            }
        )

        self.render_mode = render_mode
        self.step_time = 0.01  # s

    def _get_obs(self) -> dict:
        self.state.obs.update_t = self.dm_env.data.time
        self.state.obs.update_acc = self.dm_env.bind(self.acc_sens).sensordata.copy()
        self.state.obs.update_gyro = self.dm_env.bind(self.gyro_sens).sensordata.copy()
        self.state.obs.update_head_pitch = self.dm_env.bind(
            self.head_picth_sens
        ).sensordata.copy()[0]
        self.state.obs.update_head_turn = self.dm_env.bind(
            self.head_turn_sens
        ).sensordata.copy()[0]
        self.state.update()

        return self.state.get_state_dict()

    def _get_reward(self, type: str = "time", obs: Optional[State] = None) -> float:
        """Function of obs and ?"""
        # TODO: device a proper reward.
        return 1.0

    def _get_theta(self) -> float:
        # return self.state.pitch
        # TODO: state.get_theta()
        return 0.0

    def _get_info(self) -> dict:
        return {}

    def reset(
        self, options: Optional[Any] = None, seed: Optional[int] = None
    ) -> tuple[dict, dict]:
        self.dm_env.reset()

        return self._get_obs(), self._get_info()

    @property
    def terminated(self) -> bool:
        return self._get_theta() > (10 / 180 * np.pi)

    @property
    def truncated(self) -> bool:
        return False

    def render(self):
        if self.render_mode == "rgb_array":
            return self.dm_env.render(camera_id=0, height=480, width=640)

    def step(
        self, action: Optional[StepAction] = None
    ) -> tuple[dict, float, bool, bool, dict]:
        if action is not None:
            self.dm_env.bind(self.left_wheel_act).ctrl = action.left_wheel
            self.dm_env.bind(self.right_wheel_act).ctrl = action.right_wheel  # rad/s
            self.dm_env.bind(self.head_pitch_act).ctrl = action.head_pitch  # rad/s
            self.dm_env.bind(self.head_turn_act).ctrl = action.head_turn  # rad/s

        if self.step_time is None:
            step_time = self.dm_env.timestep()
        else:
            step_time = self.step_time

        t0 = self.dm_env.data.time
        t = t0
        while (t < t0 + step_time) and (not self.terminated):
            self.dm_env.step()
            t = self.dm_env.data.time

        obs_d = self._get_obs()

        return (
            obs_d,
            self._get_reward(),
            self.terminated,
            self.truncated,
            self._get_info(),
        )

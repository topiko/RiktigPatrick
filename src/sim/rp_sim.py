import time

from typing import Optional

import numpy as np
import matplotlib.pyplot as plt

from dm_control import mjcf
from utils import display_video

from riktigpatric.patrick import Obs
from riktigpatric.patrick import StepReturn
from riktigpatric.patrick import StepAction


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


def quatrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q_vec = q[1:]
    q_real = q[0]

    # Way to represent q.v.q*:
    t = np.cross(q_vec, v)
    v_rot = v + q_real * t + np.cross(q_vec, t)
    return v_rot


class RPenvWrap:
    def __init__(self, simultime: float = 5, render: bool = False):
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

        self.frames = []
        self.framerate = 30
        self.simultime = simultime
        self.render = render
        self.obs = Obs()

    def _get_obs(self) -> Obs:
        # TODO: make obs.get_obs() --> final formatted observation that is fead into nn.
        obs = Obs()
        obs.acc = self.dm_env.bind(self.acc_sens).sensordata.copy()
        obs.gyro = self.dm_env.bind(self.gyro_sens).sensordata.copy()
        obs.head_pitch = self.dm_env.bind(self.head_picth_sens).sensordata.copy()[0]
        obs.head_turn = self.dm_env.bind(self.head_turn_sens).sensordata.copy()[0]

        self.obs = obs
        return obs

    def _get_reward(self, type: str = "time", obs: Optional[Obs] = None) -> float:
        """Function of obs and ?"""
        # TODO: device a propert reward.
        head_reward = self.obs.head_pitch**2 + self.obs.head_turn**2

        orient_reward = 1 - self._get_theta() ** 2

        if type == "time":
            return orient_reward + head_reward
        else:
            raise KeyError(f"Invalid reward type: {type}")

    def _get_theta(self) -> float:
        # TODO: state.get_theta()
        q = self.dm_env.named.data.xquat["frame/torso"]
        rot_z = quatrot(q, np.array([0, 0, 1]))
        theta = np.pi / 2 - np.arcsin(rot_z[2])
        return theta

    def reset(self, seed: Optional[int] = None):
        self.dm_env.reset()
        self.frames = []

        return self._get_obs()

    @property
    def terminated(self) -> bool:
        return self._get_theta() > (10 / 180 * np.pi)

    @property
    def truncated(self) -> bool:
        return self.dm_env.data.time >= self.simultime

    def record_frame(self):
        if not self.render:
            return

        if len(self.frames) < self.dm_env.data.time * self.framerate:
            pixels = self.dm_env.render(camera_id=0, height=480, width=640)
            self.frames.append(pixels)

    def step(
        self, action: Optional[StepAction] = None, step_time: Optional[float] = None
    ) -> StepReturn:
        if action is not None:
            self.dm_env.bind(self.left_wheel_act).ctrl = action.left_wheel
            self.dm_env.bind(self.right_wheel_act).ctrl = action.right_wheel  # rad/s
            self.dm_env.bind(self.head_pitch_act).ctrl = action.head_pitch  # rad/s
            self.dm_env.bind(self.head_turn_act).ctrl = action.head_turn  # rad/s

        if step_time is None:
            step_time = self.dm_env.timestep()

        t0 = self.dm_env.data.time
        t = t0
        while (t < t0 + step_time) and (not self.terminated):
            self.dm_env.step()
            t = self.dm_env.data.time
            self.record_frame()

        self._get_obs()

        sr = StepReturn()
        sr.obs = self.obs
        sr.reward = self._get_reward()
        sr.terminated = self.terminated
        sr.truncated = self.truncated

        return sr


if __name__ == "__main__":
    rpenv = RPenvWrap(simultime=5, render=True)

    sa = StepAction()
    sa.left_wheel = 0
    sa.right_wheel = 0
    sa.head_pitch = 0
    sa.head_turn = 0

    t0 = time.time()
    while True:
        sr = rpenv.step(sa, 0.1)
        if rpenv.dm_env.data.time > 0.2:
            sa.right_wheel = 5.14634413173
            sa.left_wheel = 5.14634413173
            sa.head_turn = 2

        if sr.terminated:
            simul_t = rpenv.dm_env.data.time
            break

    t1 = time.time()
    print(f"Runtime: {t1 -t0:.2f}")
    print(f"Simultime: {simul_t:.2f}")
    display_video(rpenv.frames, framerate=rpenv.framerate)

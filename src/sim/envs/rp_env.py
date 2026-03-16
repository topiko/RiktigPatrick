from typing import Any, Optional, Union

import gymnasium
import matplotlib.pyplot as plt
import numpy as np
from dm_control import mjcf
from gymnasium import spaces

from filters.qutils import q2eul
from riktigpatric.patrick import State, StepAction
from sim.sim_config import (
    MAX_V,
    MAX_WHEEL_ACC,
    MAX_WHEEL_VEL,
    N_ACTIONS,
    POLICY_TYPE,
    REWARD_CONFIG,
)

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
MAXV = MAX_V
MAXA = MAX_WHEEL_ACC


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
        step_time: float = 0.01,
        randomize: bool = False,
    ):
        self._randomize = randomize
        self._init_pitch_scale = 2.0
        self._init_wheel_vel_scale = 1.0
        self.lock_head = lock_head
        self.policy_type = POLICY_TYPE
        self.dm_env = self._reset_env()
        assert self.dm_env is not None

        self.simul_timestep = 0.002
        self.dm_env.model.opt.timestep = self.simul_timestep

        self.state = State(keys=state_keys, record=record)

        self.observation_space = self.state.to_obs_space()

        action_space = self._build_action_space()
        self.action_space = spaces.Dict(action_space)
        self.render_mode = render_mode
        self.step_time = step_time
        self.metadata["render_fps"] = int(1 / self.step_time)
        self._prev_action = StepAction()

    def _build_action_space(self) -> dict:
        if self.policy_type == "velocity":
            return {
                "act/velocity_left_wheel": spaces.Box(
                    -MAX_WHEEL_VEL, MAX_WHEEL_VEL, shape=(1,), dtype=float
                ),
                "act/velocity_right_wheel": spaces.Box(
                    -MAX_WHEEL_VEL, MAX_WHEEL_VEL, shape=(1,), dtype=float
                ),
            }
        elif self.policy_type == "acceleration":
            return {
                "act/acceleration_left_wheel": spaces.Discrete(N_ACTIONS),
                "act/acceleration_right_wheel": spaces.Discrete(N_ACTIONS),
            }
        else:
            raise ValueError(f"Invalid policy type: {self.policy_type}")

    @staticmethod
    def _idx_to_acc(idx: int) -> float:
        """Map discrete index to acceleration value.

        N=5 example: idx {0,1,2,3,4} → {-MAX, -MAX/2, 0, +MAX/2, +MAX}
        """
        return (2 * idx / (N_ACTIONS - 1) - 1) * MAX_WHEEL_ACC

    def _update_obs(self, action_time: float, obs_time: float):
        body_quat = self.dm_env.bind(self.body_quat).sensordata.copy()
        pitch = q2eul(body_quat)[1] / np.pi * 180

        self.state.update_obs(
            action_time=action_time,
            obs_time=obs_time,
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
        )

    def _get_obs(self) -> dict:
        d = self.state.get_state_dict(keys="all")
        return d

    def _get_reward(self) -> tuple[float, dict]:
        pitch = abs(self.state.euler[1])
        step_reward = REWARD_CONFIG["step"]
        pitch_penalty = -pitch * REWARD_CONFIG["pitch_coef"]
        action_penalty = -REWARD_CONFIG["action_coef"] * float(
            self._prev_action.left_wheel**2 + self._prev_action.right_wheel**2
        )
        yaw_penalty = -REWARD_CONFIG["yaw_coef"] * (
            float(self._prev_action.left_wheel - self._prev_action.right_wheel) ** 2
        )
        total = step_reward + pitch_penalty + action_penalty + yaw_penalty
        info = {
            "reward/step": step_reward,
            "reward/pitch": pitch_penalty,
            "reward/action": action_penalty,
            "reward/yaw": yaw_penalty,
        }

        # Add termination penalty if robot falls
        if self.terminated:
            termination_penalty = -REWARD_CONFIG.get("termination_penalty", 0.0)
            info["reward/termination"] = termination_penalty
            total += termination_penalty
        else:
            info["reward/termination"] = 0.0

        return total, info

    def _get_info(self) -> dict:
        return {}

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

        # Sensors:
        self.gyro_sens = rp.model.find("sensor", "gyro")
        self.acc_sens = rp.model.find("sensor", "accelerometer")
        self.head_pitch_sens = rp.model.find("sensor", "headpitch_sensor")
        self.head_turn_sens = rp.model.find("sensor", "headturn_sensor")
        self.body_quat = rp.model.find("sensor", "framequat_sensor")
        self.left_wheel_vel_sens = rp.model.find("sensor", "leftwheel_vel_sensor")
        self.right_wheel_vel_sens = rp.model.find("sensor", "rightwheel_vel_sensor")

        if not self.lock_head:
            self.head_pitch_act = rp.model.find("actuator", "headpitch_actuator")
            self.head_turn_act = rp.model.find("actuator", "headturn_actuator")
        # Make environment:
        physics = mjcf.Physics.from_mjcf_model(arena)

        # Set initial wheel velocity perturbation
        if self._randomize:
            init_wheel_vel = prng.normal(0, self._init_wheel_vel_scale)
            left_joint_id = physics.model.name2id("frame/leftwheel_joint", "joint")
            right_joint_id = physics.model.name2id("frame/rightwheel_joint", "joint")
            physics.data.qvel[left_joint_id] = init_wheel_vel
            physics.data.qvel[right_joint_id] = init_wheel_vel

        return physics

    def reset(
        self, options: Optional[Any] = None, seed: int | None = None
    ) -> tuple[dict, dict]:
        self.dm_env = self._reset_env(seed)
        self.state.reset()

        d, i = self._get_obs(), self._get_info()
        return d, i

    @property
    def terminated(self) -> bool:
        return abs(self.state.euler[1]) > 20

    @property
    def truncated(self) -> bool:
        return self.state.obs.action_time[0] >= 20.0  # 20 second time limit

    def render(self):
        if self.render_mode == "rgb_array":
            return self.dm_env.render(camera_id=0, height=480, width=640)

    def step(
        self, action: Optional[dict[str, np.ndarray]] = None
    ) -> tuple[dict, float, bool, bool, dict]:
        t0 = self.dm_env.data.time

        if action is not None:
            if self.policy_type == "velocity":
                left_vel = action["act/velocity_left_wheel"][0]
                right_vel = action["act/velocity_right_wheel"][0]
                self._prev_action = StepAction().from_dict(
                    {
                        "act/left_wheel": np.array([left_vel]),
                        "act/right_wheel": np.array([right_vel]),
                    }
                )

            elif self.policy_type == "acceleration":
                left_idx_arr = action["act/acceleration_left_wheel"]
                right_idx_arr = action["act/acceleration_right_wheel"]
                left_idx = int(
                    left_idx_arr.item()
                    if hasattr(left_idx_arr, "item")
                    else left_idx_arr
                )
                right_idx = int(
                    right_idx_arr.item()
                    if hasattr(right_idx_arr, "item")
                    else right_idx_arr
                )

                left_acc = self._idx_to_acc(left_idx)
                right_acc = self._idx_to_acc(right_idx)

                current_left_vel = self.state.obs.left_wheel_vel[0]
                current_right_vel = self.state.obs.right_wheel_vel[0]

                dt = self.step_time
                left_vel = current_left_vel + left_acc * dt
                right_vel = current_right_vel + right_acc * dt

                left_vel = np.clip(left_vel, -MAX_WHEEL_VEL, MAX_WHEEL_VEL)
                right_vel = np.clip(right_vel, -MAX_WHEEL_VEL, MAX_WHEEL_VEL)

                self._prev_action = StepAction().from_dict(
                    {
                        "act/left_wheel": np.array([left_vel]),
                        "act/right_wheel": np.array([right_vel]),
                    }
                )

            self.dm_env.bind(self.left_wheel_act).ctrl = self._prev_action.left_wheel
            self.dm_env.bind(self.right_wheel_act).ctrl = self._prev_action.right_wheel

        t = t0
        while t < t0 + self.step_time:
            self.dm_env.step()
            t = self.dm_env.data.time

        self._update_obs(action_time=t0, obs_time=t)

        reward, reward_info = self._get_reward()

        full_reward_info = {"reward": reward}
        full_reward_info.update(reward_info)
        self.state.update_rewards(t, full_reward_info)

        info = self._get_info()
        info.update(reward_info)

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

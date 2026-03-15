import numpy as np
from dm_control import mjcf

from controller.interfaces import Action, Controller, Observation
from filters.qutils import q2eul
from sim.envs.rp_env import make_arena, MujocoRP


class SimulatorController(Controller):
    def __init__(
        self,
        ctrl_mode: str = "vel",
        lock_head: bool = True,
        step_time: float = 0.01,
        randomize: bool = True,
    ):
        self.ctrl_mode = ctrl_mode
        self.lock_head = lock_head
        self.step_time = step_time
        self.randomize = randomize

        self._env = None
        self._prev_action = Action()

    def _build_env(self):
        rp = MujocoRP()
        arena = make_arena()

        init_pitch = 0.0 if not self.randomize else np.random.normal(0, 2.0)

        xpos, ypos, zpos = 0.0, 0.0, 0.05
        spawn_site = arena.worldbody.add(
            "site",
            name="rp_site",
            pos=[xpos, ypos, zpos],
            axisangle=[0, 1, 0, init_pitch],
            group=3,
        )
        spawn_site.attach(rp.model).add("freejoint")

        self._env = mjcf.Physics.from_mjcf_model(arena)

        self.left_wheel_act = rp.model.find("actuator", "leftwheel_actuator")
        self.right_wheel_act = rp.model.find("actuator", "rightwheel_actuator")
        self.gyro_sens = rp.model.find("sensor", "gyro")
        self.acc_sens = rp.model.find("sensor", "accelerometer")
        self.head_pitch_sens = rp.model.find("sensor", "headpitch_sensor")
        self.head_turn_sens = rp.model.find("sensor", "headturn_sensor")
        self.body_quat = rp.model.find("sensor", "framequat_sensor")
        self.left_wheel_vel_sens = rp.model.find("sensor", "leftwheel_vel_sensor")
        self.right_wheel_vel_sens = rp.model.find("sensor", "rightwheel_vel_sensor")

    def reset(self, seed: int | None = None) -> Observation:
        if self._env is None:
            self._build_env()

        self._prev_action = Action()
        self._update_state()

        return self._get_obs()

    def step(self, action: Action) -> tuple[Observation, float, bool, bool]:
        self._prev_action = action

        if self.ctrl_mode == "acc":
            mul_ = self.step_time
            vl = self._obs.left_wheel_vel
            vr = self._obs.right_wheel_vel
        else:
            mul_ = 1
            vl = 0
            vr = 0

        self._env.bind(self.left_wheel_act).ctrl = action.left_wheel * mul_ + vl
        self._env.bind(self.right_wheel_act).ctrl = action.right_wheel * mul_ + vr

        t0 = self._env.data.time
        t = t0
        while t < t0 + self.step_time:
            self._env.step()
            t = self._env.data.time

        self._update_state()

        return self._get_obs(), self._get_reward(), self._terminated, False

    def close(self):
        self._env = None

    def _update_state(self):
        body_quat = self._env.bind(self.body_quat).sensordata.copy()
        pitch = q2eul(body_quat)[1] / np.pi * 180

        self._obs = Observation(
            pitch=pitch,
            gyro=self._env.bind(self.gyro_sens).sensordata.copy(),
            acc=self._env.bind(self.acc_sens).sensordata.copy(),
            head_pitch=self._env.bind(self.head_pitch_sens).sensordata.copy()[0],
            head_turn=self._env.bind(self.head_turn_sens).sensordata.copy()[0],
            left_wheel_vel=self._env.bind(self.left_wheel_vel_sens).sensordata.copy()[0],
            right_wheel_vel=self._env.bind(self.right_wheel_vel_sens).sensordata.copy()[0],
        )

    def _get_obs(self) -> Observation:
        return self._obs

    def _get_reward(self) -> float:
        step_reward = 1.0
        action_penalty = -0.001 * (
            self._prev_action.left_wheel**2 + self._prev_action.right_wheel**2
        ) / (20**2)
        return step_reward + action_penalty

    @property
    def _terminated(self) -> bool:
        return abs(self._obs.pitch) > 20

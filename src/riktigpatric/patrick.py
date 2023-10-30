"""
This is the module that contains RiktigPatrick!
"""
from __future__ import annotations

import logging
import time
from typing import Optional, Union

import numpy as np
import pandas as pd
import torch
from filters.mahony import Mahony
from gymnasium import spaces
from relay.conversions import make_ctrl

from riktigpatric.servo import Servo

LOG = logging.getLogger("rp_logger")
G = 9.81
SPEEDSCALE = 5
MINTHETA = -28
MAXTHETA = 50
MINPHI = -40
MAXPHI = 40
SERVOMODE = "position"

htheta_params = {
    "name": "head_theta",
    "max_speed": (MAXTHETA - MINTHETA) * SPEEDSCALE,  # [deg/sec]
    "minlim": MINTHETA,
    "maxlim": MAXTHETA,
    "idx": 1,
    "a": -15,
    "b": 1611,
    "operation_mode": SERVOMODE,
}

hphi_params = {
    "name": "head_phi",
    "max_speed": (MAXPHI - MINPHI) * SPEEDSCALE,  # [deg/sec]
    "minlim": MINPHI,
    "maxlim": MAXPHI,
    "idx": 0,
    "a": -13,
    "b": 1664,
    "operation_mode": SERVOMODE,
}


class StepAction:
    left_wheel: Union[np.ndarray, float] = 0.0
    right_wheel: Union[np.ndarray, float] = 0.0
    head_pitch: Union[np.ndarray, float] = 0.0
    head_turn: Union[np.ndarray, float] = 0.0

    def __init__(self, lock_head: bool = True):
        self.lock_head = lock_head
        if self.lock_head:
            self._ndim = 2
        else:
            self._ndim = 4

    @property
    def ndim(self) -> int:
        return self._ndim

    def to_dict(self) -> dict[str, np.ndarray]:
        act_arr = {
            "act/left_wheel": self.left_wheel,
            "act/right_wheel": self.right_wheel,
        }
        if not self.lock_head:
            act_arr.update(
                {
                    "act/head_pitch": self.head_pitch,
                    "act/head_turn": self.head_turn,
                }
            )
        if isinstance(self.left_wheel, float):
            return {k: np.array([v]) for k, v in act_arr.items()}

        return act_arr

    def from_dict(self, d: dict[str, np.ndarray]) -> StepAction:
        if d["act/left_wheel"].ndim != 1:
            raise NotImplementedError()
        self.left_wheel = d["act/left_wheel"][0]
        self.right_wheel = d["act/right_wheel"][0]
        if not self.lock_head:
            self.head_pitch = d["act/head_pitch"][0]
            self.head_turn = d["act/head_turn"][0]
        return self

    def from_array(
        self,
        action: np.ndarray,
        dt: float,
        obs: Optional[dict[str, np.ndarray]] = None,
        lock_head: bool = True,
    ) -> StepAction:
        if action.ndim == 1:
            # print("Ndim == 1")
            action = action.reshape(1, -1)

        action = action.astype(np.float64)

        # TODO: implement control mapping here.
        self.left_wheel = action[:, 0].reshape(-1, 1)
        self.right_wheel = action[:, 1].reshape(-1, 1)

        if lock_head:
            return self

        self.lock_head = lock_head
        self.head_pitch = action[:, 2].reshape(-1, 1)
        self.head_turn = action[:, 3].reshape(-1, 1)
        return self

    def __str__(self) -> str:
        return f"left_wheel : {self.left_wheel}"


class Obs:
    def __init__(self):
        self._acc: np.ndarray = np.zeros(3)
        self._gyro: np.ndarray = np.zeros(3)
        self._head_pitch: np.ndarray = np.zeros(1)
        self._head_turn: np.ndarray = np.zeros(1)
        self._left_wheel_vel: np.ndarray = np.zeros(1)
        self._right_wheel_vel: np.ndarray = np.zeros(1)
        self._true_pitch: np.ndarray = np.zeros(1)
        self._t: float = 0

    @property
    def ndim(self) -> int:
        return 8

    @property
    def acc(self) -> np.ndarray:
        return self._acc

    @acc.setter
    def update_acc(self, acc: Union[np.ndarray, torch.Tensor]):
        if isinstance(acc, torch.Tensor):
            acc = acc.numpy()
        self._acc = acc

    @property
    def gyro(self) -> np.ndarray:
        return self._gyro

    @gyro.setter
    def update_gyro(self, gyro: Union[np.ndarray, torch.Tensor]):
        if isinstance(gyro, torch.Tensor):
            gyro = gyro.numpy()
        self._gyro = gyro

    @property
    def head_pitch(self) -> np.ndarray:
        return self._head_pitch

    @head_pitch.setter
    def update_head_pitch(self, head_pitch: float):
        self._head_pitch = np.array([head_pitch])

    @property
    def head_turn(self) -> np.ndarray:
        return self._head_turn

    @head_turn.setter
    def update_head_turn(self, head_turn: float):
        self._head_turn = np.array([head_turn])

    @property
    def left_wheel_vel(self) -> np.ndarray:
        return self._left_wheel_vel

    @left_wheel_vel.setter
    def update_left_wheel_vel(self, vel: float):
        self._left_wheel_vel = np.array([vel])

    @property
    def right_wheel_vel(self) -> np.ndarray:
        return self._right_wheel_vel

    @right_wheel_vel.setter
    def update_right_wheel_vel(self, vel: float):
        self._right_wheel_vel = np.array([vel])

    @property
    def true_pitch(self) -> np.ndarray:
        return self._true_pitch

    @true_pitch.setter
    def update_true_pitch(self, pitch: float):
        self._true_pitch = np.array([pitch])

    @property
    def t(self) -> float:
        return self._t

    @t.setter
    def update_t(self, t: float):
        self._t = t


class State:
    def __init__(
        self,
        keys: list[str] = ["sens/gyro", "filter/rp_pitch", "act/left_wheel"],
        record: bool = False,
    ):
        self.obs = Obs()
        self.keys = keys
        self.mahony = Mahony()
        self.prev_t = 0
        self._action_dict = StepAction().to_dict()
        self._info_dict = {}
        self._record = record
        self._history = []

    def update(
        self,
        *,  # Force names
        t: float,
        acc: np.ndarray,
        gyro: np.ndarray,
        head_pitch: float,
        head_turn: float,
        left_wheel_vel: float,
        right_wheel_vel: float,
        true_pitch: float,
        action: Optional[StepAction] = None,
        reward: Optional[float] = None,
    ):
        self.obs.update_t = t
        self.obs.update_gyro = gyro
        self.obs.update_acc = acc
        self.obs.update_head_pitch = head_pitch
        self.obs.update_head_turn = head_turn
        self.obs.update_left_wheel_vel = left_wheel_vel
        self.obs.update_right_wheel_vel = right_wheel_vel
        self.obs.update_true_pitch = true_pitch

        # Sens fusion:
        self.mahony.update(acc, gyro, t - self.prev_t)
        self.prev_t = t

        if action is not None:
            self._action_dict = action.to_dict()
            # TODO: infer the turning speed and put into state_d

        self._info_dict = {}
        if reward is not None:
            self._info_dict["reward"] = np.array([reward])

        if self._record:
            self._history.append(self.get_state_arr(keys="all"))

    @property
    def history(self) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        if not self._record:
            raise KeyError("You have not recorded history.")
        return (
            np.vstack(self._history),
            self.get_state_arr(keys="all", ret_idxs=True)[1],
        )

    @property
    def euler(self) -> np.ndarray:
        return self.mahony.eul

    def reset(self):
        self.mahony.reset()
        self.prev_t = 0
        self._history = []
        self._action_dict = StepAction().to_dict()
        self._info_dict = {"reward": 0}

    def get_state_dict(
        self, keys: Optional[Union[str, list[str]]] = None
    ) -> dict[str, np.ndarray]:
        d = {
            "sens/acc": self.obs.acc,
            "sens/gyro": self.obs.gyro,
            "sens/head_pitch": self.obs.head_pitch,
            "sens/head_turn": self.obs.head_turn,
            "sens/left_wheel_vel": self.obs.left_wheel_vel,
            "sens/right_wheel_vel": self.obs.right_wheel_vel,
            "filter/rp_pitch": np.array([self.euler[1]]),
            "simul/rp_pitch": self.obs.true_pitch,
            "time": np.array([self.obs.t]),
        }
        d.update(self._action_dict)
        d.update(self._info_dict)

        if keys is None:
            keys = self.keys
        elif keys == "all":
            keys = list(d.keys())
        elif isinstance(keys, list):
            pass
        else:
            raise ValueError("Invalid keys!")

        d = {k: d[k] for k in sorted(keys)}

        return d

    def get_state_arr(
        self, keys: Optional[Union[str, list[str]]] = None, ret_idxs: bool = False
    ) -> Union[np.ndarray, tuple[np.ndarray, dict[str, np.ndarray]]]:
        """

        Args:
            keys: list of keys to include.
            ret_idxs: Whether to return dictionary with idxs for the measurements.

        Returns:


        """
        state_d = self.get_state_dict(keys=keys)

        state_arr = np.concatenate([arr for arr in state_d.values()], axis=0)
        if ret_idxs:
            start_idx = 0
            idxs_d = {}
            for k, v in state_d.items():
                idxs_d[k] = np.arange(len(v)) + start_idx
                start_idx += len(v)

            return state_arr, idxs_d

        return state_arr

    def to_obs_space(self) -> spaces.Dict:
        return spaces.Dict(
            {
                k: spaces.Box(-np.inf, np.inf, shape=(len(v),), dtype=float)
                for k, v in self.get_state_dict().items()
            }
        )


class StepReturn:
    obs: Obs
    reward: float
    terminated: bool
    truncated: bool


class RPHead:
    def __init__(self):
        self.phiservo = Servo(**hphi_params)
        self.thetaservo = Servo(**htheta_params)
        self._servo_init = 0
        self._servosinited = False
        self._servo_operation_mode = SERVOMODE

    @property
    def servo_operation_mode(self):
        return self._servo_operation_mode

    @servo_operation_mode.setter
    def servo_operation_mode(self, mode):
        self._servo_operation_mode = mode
        self.phiservo.operation_mode = mode
        self.thetaservo.operation_mode = mode

    @property
    def state(self):
        return {
            "phiservo": self.phiservo.state,
            "thetaservo": self.thetaservo.state,
            "servo_init": self._servo_init,
            "servo_operation_mode": self.servo_operation_mode,
        }

    @property
    def servo_init_cmds(self) -> list:
        cmd_tuples = list(self.thetaservo.init_dict.values()) + list(
            self.phiservo.init_dict.values()
        )

        return [make_ctrl(*t) for t in cmd_tuples]

    @property
    def target_phi(self):
        return self.phiservo.target_angle

    @target_phi.setter
    def target_phi(self, tandangle):
        self.phiservo.target_angle = tandangle

    @property
    def target_theta(self):
        return self.thetaservo.target_angle

    @target_theta.setter
    def target_theta(self, tandangle):
        self.thetaservo.target_angle = tandangle

    @property
    def pulse_phi(self):
        return self.phiservo.pulse

    @pulse_phi.setter
    def pulse_phi(self, pulse):
        self.phiservo.pulse = pulse

    @property
    def pulse_theta(self):
        return self.thetaservo.pulse

    @pulse_theta.setter
    def pulse_theta(self, pulse):
        self.thetaservo.pulse = pulse

    def get_ctrl(self, deltaT=None, phispeed=None, thetaspeed=None):
        if self.servo_operation_mode == "position":
            phispeed = self.phiservo.speed2int(deltaT=deltaT)
            thetaspeed = self.thetaservo.speed2int(deltaT=deltaT)
        elif self.servo_operation_mode == "speed":
            phispeed = self.phiservo.speed2int(speed=phispeed)
            thetaspeed = self.thetaservo.speed2int(speed=thetaspeed)

        return make_ctrl(16, phispeed, thetaspeed)

    @property
    def asarray(self):
        return np.concatenate((self.phiservo.asarray, self.thetaservo.asarray))

    @property
    def arrayheader(self):
        return self.phiservo.arrayheader + self.thetaservo.arrayheader

    def __repr__(self):
        add = "  "
        repr_ = "Head:\n"
        repr_ += f"Phiservo\n"
        repr_ += self.phiservo.__repr__().replace("\n", f"\n{add*2}")
        repr_ += f"\nThetaservo\n"
        repr_ += self.thetaservo.__repr__().replace("\n", f"\n{add*2}")
        repr_ += "\n"

        return repr_


class RPatrick:
    def __init__(self, report_sock=None):
        self.head = RPHead()
        self._ahrs = Mahony()

        self.imu_a = np.zeros(3)
        self.imu_w = np.zeros(3)
        self.rpy = np.zeros(3)  # ahrs roll, pitch, yaw
        self.dt = 0
        self.dt_mean = 0
        self.rptime = 0
        self.rpmode = 0
        self._target_mode = 0
        self._count = 0
        self._mode = 0
        self.mytime = time.time()
        self.report_sock = report_sock

        self._n_collect = 1000

        if self._n_collect > -1:
            self.df = pd.DataFrame(
                data=np.zeros((self._n_collect, len(self.arrayheader))),
                columns=self.arrayheader,
            )

    def __repr__(self):
        add = "  "
        repr_ = "RiktigPatrick:\n"

        for k, v in self.state.items():
            if k == "head":
                continue
            repr_ += f"{add}{k} : {v}\n"
        repr_ += self.head.__repr__().replace("\n", f"\n{add}")

        return repr_

    @property
    def asarray(self):
        return np.concatenate(
            (
                np.array([self.rptime, self.mytime]),
                self.imu_a,
                self.imu_w,
                self.rpy,
                self.head.asarray,
            )
        )

    @property
    def arrayheader(self):
        xyz = "xyz"
        timehead = ["rptime", "mytime"]
        ahead = [f"a_{k}" for k in xyz]
        whead = [f"w_{k}" for k in xyz]
        rpyhead = ["roll", "pitch", "yaw"]

        return timehead + ahead + whead + rpyhead + self.head.arrayheader

    def send2monitor(self):
        if self.report_sock is None:
            return

        if self._count % 5 == 0:
            arr = np.concatenate(
                (
                    np.array(
                        [
                            self.rptime,
                            self.head.phiservo.angle,
                            self.head.phiservo.target_angle,
                            self.head.thetaservo.angle,
                            self.head.thetaservo.target_angle,
                        ]
                    ),
                    self.rpy,
                ),
                dtype="f",
            )

            if arr.dtype != "f":
                return

            self.report_sock.send(arr.tobytes())
            # self.report_sock.send(np2bytes(arr, fmt='f'))

    @property
    def state(self):
        return {
            "a": self.imu_a,
            "w": self.imu_w,
            "rpy": self.rpy,
            "rptime": self.rptime,
            "rpmode": self.rpmode,
            "head": self.head.state,
        }

    @state.setter
    def state(self, keydata):  # data : bytearray):
        key, data = keydata

        if key == "measurements":
            LOG.debug(data)
            self._count += 1
            (
                rptime,
                self.imu_a,
                self.imu_w,
                self.head.pulse_phi,
                self.head.pulse_theta,
                self.rpmode,
            ) = data

            rptime /= 1e6

            if (rptime - self.rptime) > 0.1:
                LOG.warning(f"Long break {(rptime - self.rptime)*1000:.0f} ms")
            else:
                self.dt = rptime - self.rptime
                # Update average on the fly
                self.dt_mean = self.dt_mean + (self.dt - self.dt_mean) / self._count
            self.rptime = rptime
            self._ahrs.update(
                self.imu_a / np.linalg.norm(self.imu_a),
                self.imu_w / 180 * np.pi,
                self.dt,
            )
            self.rpy = self._ahrs.eul.copy()
            self.mytime = time.time()

            if self._n_collect > -1:
                self.df.loc[self._count - 1, :] = self.asarray

        elif key == "external_input":
            LOG.info(f"External input : phi={data[0]:.2f}, theta={data[1]:.2f}")
            self.head.target_phi, self.head.target_theta = data
        elif key == "setmode-0":
            self._mode = 0
        elif key == "setmode-1":
            self._mode = 1
        else:
            raise KeyError(f'Invalid key "{key}" to update state.')

        # TESTING AREA
        # ======================
        # if self._count%100 == 0:
        #    for s in self.__repr__().split('\n'): LOG.info(s)

        if self._count == self._n_collect:
            LOG.info("Savin dataframe.")
            self._count = 0
            self.df.to_hdf("data/rp.hdf5", key="data")

        if self._count > 10:
            self.send2monitor()
        # ======================

    def get_ctrl(self):
        if not self.head._servosinited:
            LOG.warning(f"Servos not inited - curinit: {self.head._servo_init}...")
            cmd = self.head.servo_init_cmds[self.head._servo_init]
            self.head._servo_init += 1

            if self.head._servo_init == len(self.head.servo_init_cmds):
                self.head._servosinited = True

            return cmd

        if self._mode != self.rpmode:
            LOG.info(f"Update operation mode to {self._mode}")

            return make_ctrl(self._mode, 0, 0)

        # TODO: implement the controls...
        self.head.target_phi = (self.rptime, -self.rpy[0])
        self.head.target_theta = (self.rptime, -self.rpy[1])

        # TODO: think about this dt scheme a bit.
        cmd = self.head.get_ctrl(self.dt_mean)

        return cmd

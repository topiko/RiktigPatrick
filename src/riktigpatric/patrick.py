"""
This is the module that contains RiktigPatrick!
"""
from __future__ import annotations
from typing import Union
from typing import Optional

import time
import logging

import numpy as np
import pandas as pd
import torch
from gymnasium import spaces

from relay.conversions import make_ctrl
from filters.mahony import Mahony

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
    left_wheel: float = 0.
    right_wheel: float = 0.
    head_pitch: float = 0.
    head_turn: float = 0.

    @property
    def ndim(self) -> int:
        return 4

    def to_dict(self) -> dict[str, np.ndarray]:
        return {
            "act/left_wheel": np.array([self.left_wheel]),
            "act/right_wheel": np.array([self.right_wheel]),
            "act/head_pitch": np.array([self.head_pitch]),
            "act/head_turn": np.array([self.head_turn]),
        }

    def from_tensor(self, action: torch.Tensor) -> StepAction:
        self.left_wheel = action[0].item() + action[1].item()
        self.right_wheel = action[0].item() - action[1].item()
        self.head_pitch = action[2].item()
        self.head_turn = action[3].item()
        return self


class Obs:
    def __init__(self):
        self._acc: np.ndarray = np.zeros(3)
        self._gyro: np.ndarray = np.zeros(3)
        self._head_pitch: np.ndarray = np.zeros(1)
        self._head_turn: np.ndarray = np.zeros(1)
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
    def t(self) -> float:
        return self._t

    @t.setter
    def update_t(self, t: float):
        self._t = t


class State:
    def __init__(self, keys: list[str] = ["sens/gyro", "filter/rp_pitch", "act/left_wheel"]):
        self.obs = Obs()
        self.keys = keys
        self.mahony = Mahony()
        self.prev_t = 0
        self._action_dict = StepAction().to_dict()

    def update(self, action: Optional[StepAction] = None):
        # TODO: update the orientation filter.
        self.mahony.update(self.obs.acc, self.obs.gyro, self.obs.t - self.prev_t)
        self.prev_t = self.obs.t

        if action is not None:
            self._action_dict = action.to_dict()

    @property
    def euler(self) -> np.ndarray:
        return self.mahony.eul

    def reset(self):
        self.mahony.reset()
        self.prev_t = 0


    def get_state_dict(self, wlimits: bool = False) -> dict[str, np.ndarray]:
        d = {
            "sens/acc": self.obs.acc,
            "sens/gyro": self.obs.gyro,
            "sens/head_pitch": self.obs.head_pitch,
            "sens/heado_turn": self.obs.head_turn,
            "filter/rp_pitch": np.array([self.euler[1]]),
            "time": self.obs.t,
        }
        d.update(self._action_dict)

        d = {k: v for k, v in d.items() if k in self.keys}

        return d

    def get_state_arr(
        self, state_d: Optional[dict[str, np.ndarray]] = None
    ) -> np.ndarray:
        if state_d is None:
            state_d = self.get_state_dict()

        arrs = []
        for k in self.keys:
            arrs.append(state_d[k])

        return np.concatenate(arrs)

    def sdict2sarr(self, state_d: dict[str, np.ndarray]) -> np.ndarray:
        return self.get_state_arr(state_d)

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

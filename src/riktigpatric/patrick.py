"""
This is the module that contains RiktigPatrick!
"""

from __future__ import annotations

import logging
import time
from enum import Enum

import numpy as np
import pandas as pd

from filters.mahony import Mahony
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

HTHETA_PARAMS = {
    "name": "head_theta",
    "max_speed": (MAXTHETA - MINTHETA) * SPEEDSCALE,  # [deg/sec]
    "minlim": MINTHETA,
    "maxlim": MAXTHETA,
    "idx": 1,
    "a": -15,
    "b": 1611,
    "operation_mode": SERVOMODE,
}

HPHI_PARAMS = {
    "name": "head_phi",
    "max_speed": (MAXPHI - MINPHI) * SPEEDSCALE,  # [deg/sec]
    "minlim": MINPHI,
    "maxlim": MAXPHI,
    "idx": 0,
    "a": -13,
    "b": 1664,
    "operation_mode": SERVOMODE,
}


class Actions(str, Enum):
    ACC_LEFT_WHEEL = "act/accelerate_left_wheel"
    ACC_RIGHT_WHEEL = "act/accelerate_right_wheel"
    ACC_HEAD_PITCH = "act/accelerate_head_pitch"
    ACC_HEAD_TURN = "act/accelerate_head_turn"
    ACC_BOTH_WHEELS = "act/accelerate_both_wheels"
    ACC_YAW_TURN = "act/accelerate_yaw_turn"

    VEL_LEFT_WHEEL = "act/left_wheel_vel"
    VEL_RIGHT_WHEEL = "act/right_wheel_vel"
    VEL_HEAD_PITCH = "act/head_pitch_vel"  # rad/s - head pitch velocity control
    VEL_HEAD_TURN = "act/head_turn_vel"  # rad/s - head turn velocity control

    TIME = "act/time"

    @classmethod
    def from_str(cls, value: str) -> Actions:
        """Convert string value to Actions enum.

        Args:
            value: String value

        Returns:
            Corresponding Observables enum member

        Raises:
            ValueError: If no matching observable found
        """
        for act in cls:
            if act.value == value:
                return act
        raise ValueError(
            f"Unknown observable: '{value}'. Available: {[a.value for a in cls]}"
        )


class StepAction:
    def __init__(self, action_d: dict[Actions, np.ndarray]):
        self._ndim = len(action_d)
        self._actions = action_d.keys()

        self.from_dict(action_d)

    @property
    def ndim(self) -> int:
        return self._ndim

    def to_dict(self) -> dict[str | Actions, np.ndarray]:
        d_: dict[Actions, np.ndarray] = {}
        for a in self._actions:
            if (a_val := getattr(self, a)) is None:
                raise ValueError(f"Action {a} is not set!")

            if isinstance(a_val, float):
                a_val = np.array([a_val])
            elif isinstance(a_val, np.ndarray):
                pass
            else:
                raise ValueError(f"Invalid type for action {a} : {type(a_val)}")

            d_[a] = a_val

        return d_

    def from_dict(self, d: dict[Actions, np.ndarray]) -> StepAction:
        for key, val in d.items():
            if key not in self._actions:
                raise KeyError(f"Invalid action key {key} in input dict!")

            if not isinstance(val, np.ndarray):
                raise ValueError(f"Invalid type for action {key} : {type(val)}")

            setattr(self, key, val)

        return self


class Observables(str, Enum):
    ACC = "sens/acc"
    GYRO = "sens/gyro"
    HEAD_PITCH = "sens/head_pitch"
    HEAD_TURN = "sens/head_turn"
    LEFT_WHEEL_VEL = "sens/left_wheel_vel"
    RIGHT_WHEEL_VEL = "sens/right_wheel_vel"
    RP_PITCH = "filter/rp_pitch"
    TRUE_PITCH = "simul/rp_pitch"
    OBS_TIME = "env/obs_time"

    @classmethod
    def from_str(cls, value: str) -> Observables:
        """Convert string value to Observables enum.

        Args:
            value: String value (e.g., 'filter/rp_pitch', 'sens/gyro')

        Returns:
            Corresponding Observables enum member

        Raises:
            ValueError: If no matching observable found
        """
        for obs in cls:
            if obs.value == value:
                return obs
        raise ValueError(
            f"Unknown observable: '{value}'. Available: {[o.value for o in cls]}"
        )

    def dim(self) -> int:
        """Return the dimension (number of channels) for this observable.

        Returns:
            Number of channels (1 for scalars, 3 for vectors)
        """
        if self in {
            Observables.HEAD_PITCH,
            Observables.HEAD_TURN,
            Observables.LEFT_WHEEL_VEL,
            Observables.RIGHT_WHEEL_VEL,
            Observables.RP_PITCH,
            Observables.TRUE_PITCH,
            Observables.OBS_TIME,
        }:
            return 1
        if self in {Observables.ACC, Observables.GYRO}:
            return 3
        raise ValueError(f"Unknown observable {self}!")


class Obs:
    def __init__(self, obs_d: dict[Observables, np.ndarray]):
        self.set_observables(obs_d)

    def set_observable(self, obs: Observables, value: np.ndarray | float):
        if obs not in Observables:
            raise ValueError(f"Invalid observable {obs}!")
        if isinstance(value, float):
            value = np.array([value])
        elif isinstance(value, np.ndarray):
            pass
        else:
            raise ValueError(f"Invalid type for observable {obs} : {type(value)}")

        setattr(self, obs, value)

    def get_observable(self, obs: Observables) -> np.ndarray:
        if obs not in Observables:
            raise ValueError(f"Invalid observable {obs}!")

        if (val := getattr(self, obs)) is None:
            raise ValueError(f"Observable {obs} is not set!")

        if not isinstance(val, np.ndarray):
            raise ValueError(f"Invalid type for observable {obs} : {type(val)}")

        return val

    def set_observables(self, obs_d: dict[Observables, np.ndarray]):
        for obs, value in obs_d.items():
            self.set_observable(obs, value)

    def to_dict(self) -> dict[str | Observables, np.ndarray]:
        return {obs: self.get_observable(obs) for obs in Observables}


class State:
    def __init__(
        self,
        record: bool = False,
    ):
        self.mahony = Mahony()
        self._record = record
        self._history: list[np.ndarray] = []

    def step(self):
        obs_t = self.obs.get_observable(Observables.OBS_TIME)[0]
        self.mahony.update(
            self.obs.get_observable(Observables.ACC),
            self.obs.get_observable(Observables.GYRO),
            obs_t - self.prev_t,
        )
        self.prev_t = obs_t

        if self._record:
            arr, _ = self.get_state_arr()
            self._history.append(np.copy(arr))

    def update_action(self, action_t: float, action_dict: dict[Actions, np.ndarray]):
        action_dict[Actions.TIME] = np.array([action_t])
        self.action = StepAction(action_dict)

    def update_rewards(self, reward_dict: dict[str, np.ndarray]):
        self.reward_dict = reward_dict

    def update_obs(self, obs_dict: dict[Observables, np.ndarray]):
        self.obs = Obs(obs_dict)

    def get_state_dict(self) -> dict[str, np.ndarray]:
        d_ = self.obs.to_dict()
        d_.update(self.action.to_dict())
        d_.update(self.reward_dict)

        return d_

    def get_state_arr(self) -> tuple[np.ndarray, dict[str, int]]:
        d = self.get_state_dict()
        arr = None
        keys = []
        for k in sorted(d.keys()):
            arr_ = d[k]
            if not isinstance(arr_, np.ndarray):
                raise ValueError(f"Invalid type for state variable {k} : {type(arr_)}")

            if len(arr_) == 0:
                raise ValueError(f"Empty array for state variable {k}!")

            if arr_.ndim != 1:
                raise ValueError(f"State variable {k} has invalid shape {arr_.shape}!")

            if len(arr_) > 1:
                keys_ = [f"{k}_{i}" for i in range(len(arr_))]
            else:
                keys_ = [k]

            arr = np.concatenate((arr, arr_), axis=0)
            keys += keys_

        keys_d = {k: i for i, k in enumerate(keys)}
        return arr, keys_d

    @property
    def history(self) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        if not self._record:
            raise KeyError("You have not recorded history.")
        if not self._history:
            raise KeyError("History is empty.")

        history = np.vstack(self._history)
        idx_d = self.get_state_arr(keys="all", ret_idxs=True)[1]

        return history, idx_d

    @property
    def euler(self) -> np.ndarray:
        return self.mahony.eul

    def reset(self):
        self.mahony.reset()
        self.prev_t = 0.0
        self._history = []


class RPHead:
    def __init__(self):
        self.phiservo = Servo(**HPHI_PARAMS)
        self.thetaservo = Servo(**HTHETA_PARAMS)
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
        repr_ += "Phiservo\n"
        repr_ += self.phiservo.__repr__().replace("\n", f"\n{add * 2}")
        repr_ += "\nThetaservo\n"
        repr_ += self.thetaservo.__repr__().replace("\n", f"\n{add * 2}")
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
                LOG.warning(f"Long break {(rptime - self.rptime) * 1000:.0f} ms")
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

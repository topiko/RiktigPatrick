from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union

import numpy as np


@dataclass
class Observation:
    pitch: float
    gyro: np.ndarray
    head_pitch: float
    head_turn: float
    left_wheel_vel: float
    right_wheel_vel: float
    acc: np.ndarray

    def to_dict(self) -> dict[str, np.ndarray]:
        return {
            "filter/rp_pitch": np.array([self.pitch]),
            "sens/gyro": self.gyro,
            "sens/head_pitch": np.array([self.head_pitch]),
            "sens/head_turn": np.array([self.head_turn]),
            "sens/left_wheel_vel": np.array([self.left_wheel_vel]),
            "sens/right_wheel_vel": np.array([self.right_wheel_vel]),
            "sens/acc": self.acc,
        }


@dataclass
class Action:
    left_wheel: float = 0.0
    right_wheel: float = 0.0
    head_pitch: float = 0.0
    head_turn: float = 0.0

    def to_dict(self, lock_head: bool = True) -> dict[str, np.ndarray]:
        d = {
            "act/left_wheel": np.array([self.left_wheel]),
            "act/right_wheel": np.array([self.right_wheel]),
        }
        if not lock_head:
            d.update({
                "act/head_pitch": np.array([self.head_pitch]),
                "act/head_turn": np.array([self.head_turn]),
            })
        return d

    @classmethod
    def from_dict(cls, d: dict[str, np.ndarray], lock_head: bool = True) -> "Action":
        return cls(
            left_wheel=float(d["act/left_wheel"][0]),
            right_wheel=float(d["act/right_wheel"][0]),
            head_pitch=float(d["act/head_pitch"][0]) if not lock_head else 0.0,
            head_turn=float(d["act/head_turn"][0]) if not lock_head else 0.0,
        )


class Controller(ABC):
    @abstractmethod
    def reset(self, seed: int | None = None) -> Observation:
        pass

    @abstractmethod
    def step(self, action: Action) -> tuple[Observation, float, bool, bool]:
        pass

    @abstractmethod
    def close(self):
        pass

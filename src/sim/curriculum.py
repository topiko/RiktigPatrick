"""Four-stage curriculum: balance, hold position, move/turn and control gaze."""

import logging
from itertools import product

import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent, continuous_action_settings
from riktigpatric.patrick import Actions, DerivedObs, Observable, Target
from sim.rewards import validate_pitch_deadband
from sim.utils import EpisodeBuffer

LOG = logging.getLogger(__name__)
STAGES = ("balance", "hold_position", "locomotion", "full_control")
HEAD_ACTIONS = (Actions.VEL_HEAD_PITCH, Actions.VEL_HEAD_TURN)
HEAD_REWARDS = (
    Observable.REWARD_HEAD_PITCH, Observable.REWARD_CAMERA_PITCH,
    Observable.REWARD_HEAD_YAW,
)


class Curriculum:
    VERSION = 3  # v1 combined balance/stop; v2 added stop; v3 holds position there.

    def __init__(self, cfg: DictConfig):
        self.settings = OmegaConf.to_container(cfg.curriculum, resolve=True)
        self.validation_settings = {
            "seed": cfg.guard.seed, "episodes": cfg.guard.episodes,
            "step_time": cfg.env.step_time,
            "max_episode_steps": cfg.env.max_episode_steps,
            "random_scale": cfg.env.random_scale,
        }
        self.cfg = cfg.curriculum
        self._validate_task(cfg)
        self._validate_settings()
        self.command_pairs = np.array(list(product(
            self.cfg.forward_velocities, self.cfg.yaw_rates
        )))
        if cfg.guard.episodes < len(self.command_pairs):
            raise ValueError("Validation must cover every curriculum command pair")
        self.stage = "balance"
        self.success_streak = 0

    @staticmethod
    def _validate_task(cfg):
        if (
            cfg.env.tracking_mode != "velocity"
            or not cfg.env.yaw_tracking or not cfg.env.head_tracking
        ):
            raise ValueError("Curriculum requires velocity, yaw and head tracking")
        expected = {Actions.ACC_BOTH_WHEELS, Actions.VEL_WHEEL_DIFF, *HEAD_ACTIONS}
        if set(cfg.policy.actions) != expected:
            raise ValueError("Curriculum requires the four default action heads")
        for action in (Actions.VEL_WHEEL_DIFF, *HEAD_ACTIONS):
            spec = cfg.policy.actions[action.value]
            if spec.type == "continuous":
                continuous_action_settings(spec)
                continue
            bins = np.asarray(spec.get("bins", []), dtype=np.float32)
            if (
                spec.type != "discrete" or bins.ndim != 1 or len(bins) < 2
                or not np.isfinite(bins).all() or np.count_nonzero(bins == 0) != 1
            ):
                raise ValueError(f"Curriculum head {action.value} needs one zero bin")
        if cfg.policy.restore_id is not None:
            raise ValueError("Resume curriculum from a training checkpoint")
        if any(cfg.train[key] is not None for key in (
            "target_positions", "target_trajectories", "target_velocities",
            "target_yaw_rates", "head_targets", "head_trajectories",
        )) or any((
            cfg.env.target_vel != 0, cfg.env.target_yaw_rate != 0,
            cfg.env.target_pos != 0, cfg.env.target_trajectory is not None,
            any(cfg.env.head_target), cfg.env.head_trajectory is not None,
        )):
            raise ValueError(
                "Curriculum owns references; configure its command grids "
                "or set curriculum.enabled=false for manual target overrides"
            )

    def _validate_settings(self):
        c = self.cfg
        validate_pitch_deadband(c.pitch_deadband)
        durations = (c.balance_seconds, c.hold_seconds, c.locomotion_seconds)
        positive = (
            *durations, c.position_mae, c.velocity_mae, c.yaw_rate_mae
        )
        if not all(np.isfinite(v) and v > 0 for v in positive):
            raise ValueError("Curriculum durations and thresholds must be positive")
        if (
            not 0 < c.neutral_probability < 1 or not 0 < c.survival_fraction <= 1
            or not 0 <= c.hold_velocity_weight <= 1
            or not 0 <= c.startup_seconds < min(durations)
            or isinstance(c.consecutive_passes, bool)
            or not isinstance(c.consecutive_passes, int) or c.consecutive_passes < 1
        ):
            raise ValueError("Invalid curriculum probability, startup or pass count")
        for grid in (c.forward_velocities, c.yaw_rates):
            values = np.asarray(grid, dtype=np.float64)
            if (
                values.ndim != 1 or not len(values) or not np.isfinite(values).all()
                or np.any(abs(values) > float(np.finfo(np.float32).max))
            ):
                raise ValueError("Command grids must contain finite float32 values")

    @property
    def index(self) -> int:
        return STAGES.index(self.stage)

    @property
    def inactive_actions(self) -> tuple[Actions, ...]:
        stationary = (Actions.VEL_WHEEL_DIFF, *HEAD_ACTIONS)
        return (stationary, stationary, HEAD_ACTIONS, ())[self.index]

    @property
    def disabled_rewards(self) -> tuple[Observable, ...]:
        stationary = (Observable.REWARD_YAW_RATE, *HEAD_REWARDS)
        return (
            (Observable.REWARD_VEL, *stationary), stationary, HEAD_REWARDS, ()
        )[self.index]

    def apply_policy(self, agent: Agent):
        agent.inactive_actions = self.inactive_actions

    @property
    def tracking_mode(self) -> str:
        return "position_velocity" if self.stage == "hold_position" else "velocity"

    @property
    def velocity_reward_weight(self) -> float:
        return self.cfg.hold_velocity_weight if self.stage == "hold_position" else 1.0

    def prepare_rollout(self, env, agent: Agent, *, evaluation: bool = False) -> dict:
        """Configure the objective before reset and supply episode-level references."""
        self.apply_policy(agent)
        env.set_attr("disabled_rewards", [self.disabled_rewards] * env.num_envs)
        env.set_attr("pitch_deadband", self.cfg.pitch_deadband)
        env.set_attr("tracking_mode", self.tracking_mode)
        env.set_attr("velocity_reward_weight", self.velocity_reward_weight)
        env.set_attr("target_pos", 0.0)
        if self.stage in ("balance", "hold_position"):
            commands = np.zeros((env.num_envs, 2))
        else:
            indices = (
                np.arange(env.num_envs) % len(self.command_pairs)
                if evaluation else np.random.randint(
                    len(self.command_pairs), size=env.num_envs
                )
            )
            commands = self.command_pairs[indices]
        return {
            "target_velocities": commands[:, 0],
            "target_yaw_rates": commands[:, 1],
            "head_targets": [[0.0, 0.0]] * env.num_envs,
            "head_trajectories": None,
        }

    def validation_metrics(self, buffers: list[EpisodeBuffer]) -> dict[str, float]:
        seconds = {
            "balance": self.cfg.balance_seconds, "hold_position": self.cfg.hold_seconds,
        }.get(self.stage, self.cfg.locomotion_seconds)
        survived, position_errors, velocity_errors, yaw_errors = [], [], [], []
        for buffer in buffers:
            time = buffer.get_observable(Observable.OBS_TIME)[:, 0]
            elapsed = time - time[0]
            survived.append(
                elapsed[-1] >= seconds - 1e-6
                and (not buffer.terminated or elapsed[-1] > seconds + 1e-6)
            )
            after_startup = elapsed[1:] >= self.cfg.startup_seconds
            # Short failed episodes still contribute errors, rather than disappearing.
            if not after_startup.any():
                after_startup[:] = True
            for actual, target, errors in (
                (DerivedObs.CURRENT_POS, Target.TARGET_POS, position_errors),
                (DerivedObs.CURRENT_VEL, Target.TARGET_VEL, velocity_errors),
                (DerivedObs.YAW_RATE, Target.YAW_RATE, yaw_errors),
            ):
                error = abs(buffer.get_observable(actual)[1:, 0]
                            - buffer.get_observable(target)[1:, 0])
                errors.append(float(error[after_startup].mean()))
        return {
            "validation/survival_fraction": float(np.mean(survived)),
            "validation/position_mae": float(np.mean(position_errors)),
            "validation/velocity_mae": float(np.mean(velocity_errors)),
            "validation/yaw_rate_mae": float(np.mean(yaw_errors)),
        }

    def observe(self, metrics: dict[str, float]) -> bool:
        """Record one scheduled evaluation; a true result requests promotion."""
        if self.stage == "full_control":
            return False
        passed = (
            metrics["validation/survival_fraction"] >= self.cfg.survival_fraction
            and (self.stage == "balance"
                 or metrics["validation/velocity_mae"] <= self.cfg.velocity_mae)
            and (self.stage != "hold_position"
                 or metrics["validation/position_mae"] <= self.cfg.position_mae)
            and (self.stage in ("balance", "hold_position")
                 or metrics["validation/yaw_rate_mae"] <= self.cfg.yaw_rate_mae)
        )
        self.success_streak = self.success_streak + 1 if passed else 0
        return self.success_streak >= self.cfg.consecutive_passes

    def advance(self, agent: Agent, optimizer: torch.optim.Optimizer):
        if self.stage == "full_control":
            raise ValueError("Already at the final curriculum stage")
        next_stage = STAGES[self.index + 1]
        newly_active = {
            "hold_position": (), "locomotion": (Actions.VEL_WHEEL_DIFF,),
            "full_control": HEAD_ACTIONS,
        }[next_stage]
        for action in newly_active:
            agent.initialize_neutral_head(action, self.cfg.neutral_probability)
            for parameter in agent.action_heads[action.value].parameters():
                optimizer.state.pop(parameter, None)
                parameter.grad = None
        self.stage = next_stage
        self.success_streak = 0
        self.apply_policy(agent)

    def state_dict(self) -> dict:
        return {"version": self.VERSION, "stage": self.stage,
                "success_streak": self.success_streak,
                "settings": self.settings, "validation": self.validation_settings}

    def validate_state(self, state: dict):
        version = state.get("version", 1)
        if type(version) is not int or version not in (1, 2, self.VERSION):
            raise ValueError("Unsupported checkpoint curriculum version")
        stages = STAGES if version == self.VERSION else (
            ("balance", "locomotion", "full_control") if version == 1
            else ("balance", "stop", "locomotion", "full_control")
        )
        if (
            state.get("stage") not in stages
            or type(state.get("success_streak")) is not int
            or state["success_streak"] < 0
        ):
            raise ValueError("Invalid checkpoint curriculum stage/progress")

    def load_state_dict(self, state: dict):
        self.validate_state(state)
        version = state.get("version", 1)
        legacy = version != self.VERSION
        self.stage = (
            "hold_position" if state["stage"] == "stop"
            or (version == 1 and state["stage"] == "balance") else state["stage"]
        )
        if legacy:
            LOG.info("Migrated curriculum %s -> %s; reset promotion streak",
                     state["stage"], self.stage)
        # Changed stage definitions, criteria or commands invalidate the streak.
        self.success_streak = (
            state["success_streak"] if (
                not legacy and state.get("settings") == self.settings
                and state.get("validation") == self.validation_settings
            ) else 0
        )

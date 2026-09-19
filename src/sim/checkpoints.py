"""Portable CPU/CUDA training checkpoints and conservative validation rollback."""

from __future__ import annotations

import logging
import math
import os
import random
from collections.abc import Iterator
from copy import deepcopy
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import torch
from omegaconf import OmegaConf

from nn_ctrl.nns import Agent
from sim.checkpoint_migrations import add_position_inputs

if TYPE_CHECKING:
    from sim.curriculum import Curriculum

LOG = logging.getLogger(__name__)


def _leaves(value: Any, path: str = "") -> Iterator[tuple[str, Any]]:
    """Walk the standard containers used by model and optimizer state dictionaries."""
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _leaves(child, f"{path}/{key}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            yield from _leaves(child, f"{path}/{index}")
    else:
        yield path, value


def require_finite(value: Any, name: str) -> None:
    """Reject invalid state, synchronizing once per device on the normal path."""
    groups: dict[torch.device, list[tuple[str, torch.Tensor]]] = {}

    for path, item in _leaves(value, name):
        if isinstance(item, torch.Tensor):
            groups.setdefault(item.device, []).append((path, item))
        elif isinstance(item, float) and not math.isfinite(item):
            raise FloatingPointError(f"Non-finite {path}")
    for tensors in groups.values():
        flags = torch.stack([torch.isfinite(tensor).all() for _, tensor in tensors])
        if not flags.all():
            # Naming the offending tensor may synchronize again, only on failure.
            index = int((~flags).nonzero()[0].item())
            raise FloatingPointError(f"Non-finite {tensors[index][0]}")


def cpu_snapshot(value: Any) -> Any:
    """Deep-copy tensor trees to host RAM, preserving aliases/state_dict metadata."""
    memo = {}

    for _, item in _leaves(value):
        if isinstance(item, torch.Tensor) and id(item) not in memo:
            memo[id(item)] = item.detach().to(device="cpu", copy=True)
    return deepcopy(value, memo)


def policy_spec(agent: Agent) -> dict:
    actions: Any = agent.actions
    if OmegaConf.is_config(actions):
        actions = OmegaConf.to_container(actions, resolve=True)
    return {"inputs": list(agent.inputs), "actions": deepcopy(actions)}


def capture_state(
    agent: Agent, optimizer: torch.optim.Optimizer, next_iteration: int,
    curriculum: Curriculum | None = None,
) -> dict:
    """Capture host-side recovery data without retaining GPU/autograd storage."""
    np_rng = np.random.get_state()
    assert isinstance(np_rng, tuple)
    return {
        "policy": policy_spec(agent),
        "model": cpu_snapshot(agent.state_dict()),
        "optimizer": cpu_snapshot(optimizer.state_dict()),
        "next_iteration": next_iteration,
        "curriculum": (
            deepcopy(curriculum.state_dict()) if curriculum is not None else None
        ),
        "device": str(agent.device),
        "torch_rng": torch.get_rng_state().clone(),
        "cuda_rng": (
            torch.cuda.get_rng_state(agent.device).clone()
            if agent.device.type == "cuda" else None
        ),
        "numpy_rng": [np_rng[0], np_rng[1].tolist(), np_rng[2], np_rng[3], np_rng[4]],
        "python_rng": random.getstate(),
    }


def _restore_rng(state: dict, device: torch.device):
    torch.set_rng_state(state["torch_rng"].cpu())
    if device.type == "cuda" and state.get("cuda_rng") is not None:
        # Map the saved policy's GPU stream onto the selected destination GPU.
        torch.cuda.set_rng_state(state["cuda_rng"].cpu(), device)
    name, keys, position, has_gauss, cached_gaussian = state["numpy_rng"]
    np.random.set_state((name, np.asarray(keys, dtype=np.uint32), position,
                         has_gauss, cached_gaussian))
    random.setstate(state["python_rng"])


def restore_state(
    agent: Agent, optimizer: torch.optim.Optimizer, state: dict,
    *, restore_rng: bool = True, curriculum: Curriculum | None = None,
    restore_curriculum: bool = True,
) -> int:
    progress = state.get("curriculum")
    if (progress is None) != (curriculum is None):
        raise ValueError("Checkpoint curriculum differs; match curriculum.enabled")
    if curriculum is not None:
        if not isinstance(progress, dict):
            raise ValueError("Invalid checkpoint curriculum state")
        curriculum.validate_state(progress)
        if not restore_curriculum and (
            progress.get("version") != curriculum.VERSION
            or progress["stage"] != curriculum.stage
        ):
            raise ValueError("Cannot roll back across curriculum stages")
    # Input order matters for the GRU's concatenated feature vector.
    if state["policy"] != policy_spec(agent):
        raise ValueError("Checkpoint inputs/actions do not match this configuration")
    expected = agent.state_dict()
    if state["model"].keys() != expected.keys() or any(
        not isinstance(value, torch.Tensor) or value.shape != expected[key].shape
        for key, value in state["model"].items()
    ):
        raise ValueError("Checkpoint shape differs; match policy.hsize/n_rnnlayers")
    require_finite(state["model"], "checkpoint/model")
    require_finite(state["optimizer"], "checkpoint/optimizer")
    agent.load_state_dict(state["model"])
    # Release old GPU moments before allocating their replacements.
    optimizer.state.clear()
    # PyTorch places moments with their parameters and keeps non-capturable Adam
    # step counters on CPU. Do not blindly move every optimizer tensor to CUDA.
    optimizer.load_state_dict(deepcopy(state["optimizer"]))
    optimizer.zero_grad(set_to_none=True)
    if curriculum is not None:
        assert isinstance(progress, dict)
        if restore_curriculum:
            curriculum.load_state_dict(progress)
        curriculum.apply_policy(agent)
    if restore_rng:
        _restore_rng(state, agent.device)
    return int(state["next_iteration"])


def save_checkpoint(
    path: Path, state: dict, config: dict, score: float | None = None
) -> Path:
    """Atomically save data loadable with weights_only=True, without a Python Agent."""
    require_finite(state["model"], "checkpoint/model")
    require_finite(state["optimizer"], "checkpoint/optimizer")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        with temporary.open("wb") as stream:
            torch.save({"format_version": 1, "state": state, "config": config,
                        "validation_score": score}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def load_checkpoint(
    path: str | Path, agent: Agent, optimizer: torch.optim.Optimizer,
    curriculum: Curriculum | None = None,
    *, learning_rate: float | None = None,
) -> int:
    if learning_rate is not None and (
        not math.isfinite(learning_rate) or learning_rate <= 0
    ):
        raise ValueError("Resume learning rate must be positive and finite")
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if checkpoint.get("format_version") != 1:
        raise ValueError("Unsupported training checkpoint format")
    state = checkpoint["state"]
    if curriculum is not None and state.get("curriculum") is not None:
        state = add_position_inputs(agent, optimizer, state, policy_spec(agent))
        if state is not checkpoint["state"]:
            LOG.info("Added position inputs with zero GRU influence; kept Adam moments")
    iteration = restore_state(agent, optimizer, state, curriculum=curriculum)
    if learning_rate is not None:
        for group in optimizer.param_groups:
            group["lr"] = learning_rate
    return iteration


class PolicyGuard:
    """Keep a best validation state; roll back only sustained, substantial drops.

    A fresh/resumed run establishes a new validation baseline. Rollback restores
    weights AND Adam state, but keeps current RNG/iteration progress so training
    does not replay the exact same samples forever.
    After a rollback, check every update against the same degradation threshold.
    Consecutive rejected attempts stop recovery instead of repeatedly shrinking LR.
    """

    def __init__(
        self, agent: Agent, optimizer: torch.optim.Optimizer, *,
        drop_fraction: float = 0.5, absolute_drop: float = 25.0,
        patience: int = 2, min_best_return: float = 50.0,
        lr_factor: float = 0.5, min_lr: float = 1e-6,
        recovery_attempts: int = 3,
        curriculum: Curriculum | None = None,
    ):
        if not 0 < drop_fraction < 1 or not 0 < lr_factor < 1:
            raise ValueError("Guard fractions must lie strictly between 0 and 1")
        if patience < 1 or absolute_drop < 0 or min_lr <= 0:
            raise ValueError("Invalid guard patience, absolute_drop or min_lr")
        if not all(math.isfinite(v) for v in (absolute_drop, min_best_return, min_lr)):
            raise ValueError("Guard thresholds must be finite")
        if type(recovery_attempts) is not int or recovery_attempts < 0:
            raise ValueError("Guard recovery_attempts must be a nonnegative integer")
        self.agent = agent
        self.optimizer = optimizer
        self.curriculum = curriculum
        self.drop_fraction = drop_fraction
        self.absolute_drop = absolute_drop
        self.patience = patience
        self.min_best_return = min_best_return
        self.lr_factor = lr_factor
        self.min_lr = min_lr
        self.recovery_attempts = recovery_attempts
        self.recovering = False
        self.recovery_failures = 0
        self.stop_requested = False
        self.best_state: dict | None = None
        self.best_score: float | None = None
        self.bad_evaluations = 0
        self.rollbacks = 0

    def reset_baseline(self):
        """A new curriculum objective gets its own best policy and return baseline."""
        self.best_state = None
        self.best_score = None
        self.bad_evaluations = 0
        self.recovering = False
        self.recovery_failures = 0
        self.stop_requested = False

    def observe(
        self, score: float, next_iteration: int, *, allow_rollback: bool = True
    ) -> str:
        if not math.isfinite(score):
            raise FloatingPointError("Non-finite validation return")
        if self.best_score is None or score > self.best_score:
            self.best_state = capture_state(
                self.agent, self.optimizer, next_iteration, self.curriculum
            )
            self.best_score = score
            self.bad_evaluations = 0
            self.recovery_failures = 0
            return "best"

        if not allow_rollback:
            return "disabled"

        threshold = self.rejection_threshold
        assert threshold is not None
        degraded = self.best_score >= self.min_best_return and score < threshold
        if self.recovering:
            if not degraded:
                self.recovery_failures = 0
                return "recovery_accepted"
            self._restore_best(reduce_lr=False)
            self.recovery_failures += 1
            self.stop_requested = self.recovery_failures >= self.recovery_attempts
            return "recovery_stop" if self.stop_requested else "recovery_rejected"

        self.bad_evaluations = self.bad_evaluations + 1 if degraded else 0
        if self.bad_evaluations < self.patience:
            return "keep"

        self._restore_best(reduce_lr=True)
        self.recovering = self.recovery_attempts > 0
        return "rollback"

    @property
    def rejection_threshold(self) -> float | None:
        if self.best_score is None:
            return None
        drop = max(abs(self.best_score) * self.drop_fraction, self.absolute_drop)
        return self.best_score - drop

    def should_evaluate(self, next_iteration: int, interval: int) -> bool:
        return self.recovering or next_iteration % interval == 0

    def _restore_best(self, *, reduce_lr: bool):
        assert self.best_state is not None
        previous_rates = [group["lr"] for group in self.optimizer.param_groups]
        restore_state(
            self.agent, self.optimizer, self.best_state, restore_rng=False,
            curriculum=self.curriculum, restore_curriculum=False,
        )
        if self.curriculum is not None:
            self.curriculum.success_streak = 0
        for group, previous in zip(self.optimizer.param_groups, previous_rates):
            group["lr"] = min(previous, max(
                self.min_lr, min(previous, group["lr"]) * self.lr_factor
            )) if reduce_lr else previous
        self.bad_evaluations = 0
        self.rollbacks += 1

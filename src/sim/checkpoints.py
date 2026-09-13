"""Portable CPU/CUDA training checkpoints and conservative validation rollback."""

import math
import os
import random
from collections.abc import Iterator
from copy import deepcopy
from pathlib import Path
from typing import Any

import numpy as np
import torch
from omegaconf import OmegaConf

from nn_ctrl.nns import Agent


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
    agent: Agent, optimizer: torch.optim.Optimizer, next_iteration: int
) -> dict:
    """Capture host-side recovery data without retaining GPU/autograd storage."""
    np_rng = np.random.get_state()
    assert isinstance(np_rng, tuple)
    return {
        "policy": policy_spec(agent),
        "model": cpu_snapshot(agent.state_dict()),
        "optimizer": cpu_snapshot(optimizer.state_dict()),
        "next_iteration": next_iteration,
        "device": str(agent.device),
        "torch_rng": torch.get_rng_state().clone(),
        "cuda_rng": (
            torch.cuda.get_rng_state(agent.device).clone()
            if agent.device.type == "cuda" else None
        ),
        "numpy_rng": [np_rng[0], np_rng[1].tolist(), np_rng[2], np_rng[3], np_rng[4]],
        "python_rng": random.getstate(),
    }


def restore_state(
    agent: Agent, optimizer: torch.optim.Optimizer, state: dict,
    *, restore_rng: bool = True,
) -> int:
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
    if restore_rng:
        torch.set_rng_state(state["torch_rng"].cpu())
        if agent.device.type == "cuda" and state.get("cuda_rng") is not None:
            # Map the saved policy's GPU stream onto the selected destination GPU.
            torch.cuda.set_rng_state(state["cuda_rng"].cpu(), agent.device)
        name, keys, position, has_gauss, cached_gaussian = state["numpy_rng"]
        np.random.set_state((name, np.asarray(keys, dtype=np.uint32), position,
                             has_gauss, cached_gaussian))
        random.setstate(state["python_rng"])
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
    path: str | Path, agent: Agent, optimizer: torch.optim.Optimizer
) -> int:
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if checkpoint.get("format_version") != 1:
        raise ValueError("Unsupported training checkpoint format")
    return restore_state(agent, optimizer, checkpoint["state"])


class PolicyGuard:
    """Keep a best validation state; roll back only sustained, substantial drops.

    A fresh/resumed run establishes a new validation baseline. Rollback restores
    weights AND Adam state, but keeps current RNG/iteration progress so training
    does not replay the exact same samples forever.
    """

    def __init__(
        self, agent: Agent, optimizer: torch.optim.Optimizer, *,
        drop_fraction: float = 0.5, absolute_drop: float = 25.0,
        patience: int = 2, min_best_return: float = 50.0,
        lr_factor: float = 0.5, min_lr: float = 1e-6,
    ):
        if not 0 < drop_fraction < 1 or not 0 < lr_factor < 1:
            raise ValueError("Guard fractions must lie strictly between 0 and 1")
        if patience < 1 or absolute_drop < 0 or min_lr <= 0:
            raise ValueError("Invalid guard patience, absolute_drop or min_lr")
        if not all(math.isfinite(v) for v in (absolute_drop, min_best_return, min_lr)):
            raise ValueError("Guard thresholds must be finite")
        self.agent = agent
        self.optimizer = optimizer
        self.drop_fraction = drop_fraction
        self.absolute_drop = absolute_drop
        self.patience = patience
        self.min_best_return = min_best_return
        self.lr_factor = lr_factor
        self.min_lr = min_lr
        self.best_state: dict | None = None
        self.best_score: float | None = None
        self.bad_evaluations = 0
        self.rollbacks = 0

    def observe(self, score: float, next_iteration: int) -> str:
        if not math.isfinite(score):
            raise FloatingPointError("Non-finite validation return")
        if self.best_score is None or score > self.best_score:
            self.best_state = capture_state(self.agent, self.optimizer, next_iteration)
            self.best_score = score
            self.bad_evaluations = 0
            return "best"

        drop = max(abs(self.best_score) * self.drop_fraction, self.absolute_drop)
        degraded = (
            self.best_score >= self.min_best_return and score < self.best_score - drop
        )
        self.bad_evaluations = self.bad_evaluations + 1 if degraded else 0
        if self.bad_evaluations < self.patience:
            return "keep"

        assert self.best_state is not None
        previous_rates = [group["lr"] for group in self.optimizer.param_groups]
        restore_state(self.agent, self.optimizer, self.best_state, restore_rng=False)
        for group, previous in zip(self.optimizer.param_groups, previous_rates):
            group["lr"] = min(previous, max(
                self.min_lr, min(previous, group["lr"]) * self.lr_factor
            ))
        self.bad_evaluations = 0
        self.rollbacks += 1
        return "rollback"

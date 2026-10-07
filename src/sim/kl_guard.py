"""Backtrack tentative Adam steps using full-history recurrent policy KL."""

import logging
import math
import time
from dataclasses import dataclass

from sim.checkpoints import require_finite, restore_state
from sim.policy_probe import RecurrentPolicyReplay

LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class KLGuardSettings:
    enabled: bool = False
    max_kl: float = 0.02
    max_backtracks: int = 6
    lr_factor: float = 0.5
    min_lr: float = 1e-6
    diagnostics: bool = False
    loo_top_k: int = 3

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError("kl_guard.enabled must be a boolean")
        if type(self.max_backtracks) is not int or self.max_backtracks < 0:
            raise ValueError("kl_guard.max_backtracks must be a nonnegative integer")
        if not all(math.isfinite(value) and value > 0
                   for value in (self.max_kl, self.min_lr)):
            raise ValueError("kl_guard.max_kl/min_lr must be positive and finite")
        if not math.isfinite(self.lr_factor) or not 0 < self.lr_factor < 1:
            raise ValueError("kl_guard.lr_factor must be between zero and one")
        if type(self.diagnostics) is not bool:
            raise ValueError("kl_guard.diagnostics must be a boolean")
        if type(self.loo_top_k) is not int or self.loo_top_k < 0:
            raise ValueError("kl_guard.loo_top_k must be a nonnegative integer")


def _restore_candidate(agent, optimizer, before, rates, curriculum):
    restore_state(agent, optimizer, before, restore_rng=False,
                  curriculum=curriculum, restore_curriculum=False)
    for group, rate in zip(optimizer.param_groups, rates, strict=True):
        group["lr"] = rate


def _smaller_rates(optimizer, rates, settings):
    return [
        rate if group.get("name") == "critic" else min(
            rate, max(settings.min_lr, rate * settings.lr_factor)
        )
        for group, rate in zip(optimizer.param_groups, rates, strict=True)
    ]


def _metrics(candidates, accepted, started):
    candidate = candidates[-1]
    attempts = len(candidates)
    return {
        "kl_guard/accepted": float(accepted),
        "kl_guard/skipped_update": float(not accepted),
        "kl_guard/attempts": float(attempts),
        "kl_guard/backtracks": float(attempts - 1),
        "kl_guard/kl_mean": candidate["mean"] if accepted else 0.0,
        "kl_guard/kl_max": candidate["max"] if accepted else 0.0,
        "kl_guard/last_candidate_kl_mean": candidate["mean"],
        "kl_guard/last_candidate_kl_max": candidate["max"],
        "kl_guard/initial_kl_mean": candidates[0]["mean"],
        "kl_guard/initial_kl_max": candidates[0]["max"],
        "kl_guard/replay_samples": candidate["samples"],
        "timing/kl_guard_seconds": time.perf_counter() - started,
        **{
            f"kl_guard/initial_{key}": candidates[0][key]
            for key in ("episode_mean_max", "episode_mean_p95", "worst_episode")
            if key in candidates[0]
        },
        **{
            f"kl_guard/last_candidate_{key}": candidate[key]
            for key in ("episode_mean_max", "episode_mean_p95", "worst_episode")
            if key in candidate
        },
    }


def guarded_optimizer_step(
    agent, optimizer, before, buffers, settings: KLGuardSettings, curriculum=None,
    *, diagnostics=None,
):
    """Reuse clipped gradients, restoring model and Adam before each smaller trial.

    Accepted trials commit exactly one optimizer update, including the critic.
    Exhaustion skips both updates and retains the last tested policy LR. Rollout
    RNG/iteration progression is never rewound. The caller handles numerical
    failures with the existing transactional training recovery.
    """
    if not settings.enabled:
        optimizer.step()
        return {}
    started = time.perf_counter()
    replay = RecurrentPolicyReplay(agent, buffers)
    gradients = [None if p.grad is None else p.grad.detach().clone()
                 for p in agent.parameters()]
    rates = [group["lr"] for group in optimizer.param_groups]
    candidates = []
    for backtrack in range(settings.max_backtracks + 1):
        if backtrack:
            _restore_candidate(agent, optimizer, before, rates, curriculum)
        for parameter, gradient in zip(agent.parameters(), gradients, strict=True):
            parameter.grad = None if gradient is None else gradient.clone()
        optimizer.step()
        require_finite(agent.state_dict(), "KL candidate model")
        require_finite(optimizer.state_dict(), "KL candidate optimizer")
        candidate = replay.metrics(agent)
        require_finite(candidate, "replayed policy KL")
        candidates.append(candidate)
        _record_trial(diagnostics, replay, candidate, rates)
        attempts = backtrack + 1
        if candidate["mean"] <= settings.max_kl:
            if backtrack:
                LOG.info(
                    "KL accepted after %d backtracks: %.6g <= %.6g; policy LR %.3g",
                    backtrack, candidate["mean"], settings.max_kl, rates[0],
                )
            return _metrics(candidates, True, started)
        smaller = _smaller_rates(optimizer, rates, settings)
        if backtrack == settings.max_backtracks or smaller == rates:
            break
        rates = smaller
    _restore_candidate(agent, optimizer, before, rates, curriculum)
    LOG.warning(
        "Skipped optimizer update after %d KL trials: %.6g > %.6g; policy LR %.3g",
        attempts, candidate["mean"], settings.max_kl, rates[0],
    )
    return _metrics(candidates, False, started)


def _record_trial(diagnostics, replay, candidate, rates):
    if diagnostics is not None:
        diagnostics.record_trial(replay, candidate, rates)

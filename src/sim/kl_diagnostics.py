"""Rejection-only trajectory attribution on an isolated model/Adam copy."""

import json
import logging
import random
import time
from contextlib import contextmanager
from copy import deepcopy
from pathlib import Path

import mlflow
import numpy as np
import torch

from riktigpatric.patrick import Target
from sim.checkpoints import require_finite, restore_state
from sim.utils import clip_agent_gradients

LOG = logging.getLogger(__name__)


@contextmanager
def _isolated_rng(device):
    numpy_state, python_state = np.random.get_state(), random.getstate()
    devices = [] if device.type != "cuda" else [
        device.index if device.index is not None else torch.cuda.current_device()
    ]
    try:
        with torch.random.fork_rng(devices=devices):
            yield
    finally:
        np.random.set_state(numpy_state)
        random.setstate(python_state)


def _vector(gradients, parameters, indices):
    return torch.cat([
        torch.zeros(parameters[i].numel(), dtype=torch.float64)
        if gradients[i] is None else gradients[i].flatten().double().cpu()
        for i in indices
    ])


def _sum_gradients(episodes):
    result = []
    for column in zip(*episodes, strict=True):
        present = [gradient for gradient in column if gradient is not None]
        result.append(torch.stack(present).sum(dim=0) if present else None)
    return result


def _leave_out_gradient(batch, episode, samples, removed):
    if removed >= samples:
        raise ValueError("Cannot omit the only trajectory in the batch")
    scale = samples / (samples - removed)
    return [
        None if whole is None else (whole - (0 if part is None else part)) * scale
        for whole, part in zip(batch, episode, strict=True)
    ]


class KLDiagnostics:
    def __init__(
        self, cfg, settings, agent, buffers, targets, values, logps, iteration,
    ):
        self.settings = settings
        self.buffers = buffers
        self.targets = targets.detach()
        self.values = values.detach()
        self.logps = logps.detach()
        self.advantages = self.targets - self.values
        self.raw_gradients = [None if p.grad is None else p.grad.detach().cpu().clone()
                              for p in agent.parameters()]
        self.tbptt_steps = cfg.train.tbptt_steps
        self.value_coef = cfg.rl.value_loss_coef
        self.grad_clip = cfg.train.grad_clip
        self.iteration = iteration
        self.seed = cfg.seed + iteration
        self.trials = []
        self.replay = None

    def record_trial(self, replay, candidate, rates):
        self.replay = replay
        trial = {**candidate, "attempt": len(self.trials) + 1,
                 "learning_rates": {"policy": rates[0], "critic": rates[-1]},
                 "rejected": candidate["mean"] > self.settings.max_kl,
                 "episodes": deepcopy(replay.episode_metrics)}
        self.trials.append(trial)
        if candidate["mean"] > self.settings.max_kl:
            LOG.info("KL rejection at update %d, trial %d: mean %.6g, max %.6g; "
                     "worst trajectory %s (mean %s)", self.iteration, trial["attempt"],
                     candidate["mean"], candidate["max"],
                     candidate.get("worst_episode"),
                     candidate.get("episode_mean_max"))

    def _metadata(self):
        episodes = []
        for index, buffer in enumerate(self.buffers):
            def target(key, final=False):
                values = buffer.obs_l[-1 if final else 0].get(key)
                return None if values is None else float(values[0])

            advantages = self.advantages[index, :buffer.seq_len]
            episodes.append({
                "index": index, "environment_seed": self.seed + index,
                "steps": buffer.seq_len,
                "duration": float(buffer.get_step_times().sum()),
                "return": float(sum(buffer.rewards_l)),
                "terminated": buffer.terminated, "truncated": buffer.truncated,
                "target_velocity": target(Target.TARGET_VEL),
                "target_yaw_rate": target(Target.YAW_RATE),
                "target_position_start": target(Target.TARGET_POS),
                "target_position_end": target(Target.TARGET_POS, final=True),
                "advantage_mean": float(advantages.mean()),
                "advantage_rms": float(advantages.square().mean().sqrt()),
            })
        return episodes

    def _replay_losses(self, agent):
        replay = self.replay
        assert replay is not None
        recorded = {}
        for action in replay.actions:
            shape = np.asarray(self.buffers[0].action_l[0][action]).shape
            values = np.zeros(
                (len(self.buffers), replay.steps, *shape), dtype=np.float32
            )
            for index, buffer in enumerate(self.buffers):
                values[index, :buffer.seq_len] = buffer.get_action(action)
            recorded[action] = torch.from_numpy(values).to(agent.device)
        lengths = torch.tensor(replay.lengths, device=agent.device)
        hidden = None
        logps, values = [], []
        for step in range(replay.steps):
            observations = {key: value[:, step]
                            for key, value in replay.observations.items()}
            actions, logp, value, hidden = agent.act(observations, hidden)
            valid = replay.valid[:, step]
            for action in replay.actions:
                torch.testing.assert_close(actions[action][valid],
                                           recorded[action][:, step][valid],
                                           rtol=1e-5, atol=1e-6)
            logps.append(logp[:, 0])
            values.append(value[:, 0])
            if hidden is not None:
                done = lengths == step + 1
                hidden = torch.where(
                    done[None, :, None], torch.zeros_like(hidden), hidden
                )
                if (step + 1) % self.tbptt_steps == 0:
                    hidden = hidden.detach()
        logps, values = torch.stack(logps, dim=1), torch.stack(values, dim=1)
        torch.testing.assert_close(logps[replay.valid], self.logps[replay.valid],
                                   rtol=1e-4, atol=1e-5)
        loss = (
            -logps * self.advantages
            + self.value_coef * (values - self.targets).square()
        )
        return (loss * replay.valid).sum(dim=1) / replay.samples

    def _episode_gradients(self, agent):
        losses = self._replay_losses(agent)
        parameters = list(agent.parameters())
        gradients = []
        for index in range(len(self.buffers)):
            row = torch.autograd.grad(losses[index], parameters, allow_unused=True,
                                      retain_graph=index + 1 < len(self.buffers))
            row = [None if value is None else value.detach().cpu() for value in row]
            require_finite(row, "trajectory diagnostic gradients")
            gradients.append(row)
        return gradients

    def _gradient_summary(self, agent, gradients, report):
        named = list(agent.named_parameters())
        parameters = [parameter for _, parameter in named]
        policy = [i for i, (name, _) in enumerate(named)
                  if not name.startswith("value_head.")]
        critic = [i for i, (name, _) in enumerate(named)
                  if name.startswith("value_head.")]
        batch_policy = _vector(self.raw_gradients, parameters, policy)
        vectors = [_vector(row, parameters, policy) for row in gradients]
        norms = [float(vector.norm()) for vector in vectors]
        norm_sum = sum(norms)
        for episode, row, vector, norm in zip(
            report["episodes"], gradients, vectors, norms, strict=True
        ):
            episode.update({
                "policy_gradient_norm": norm,
                "critic_gradient_norm": float(_vector(row, parameters, critic).norm()),
                "policy_norm_share": norm / norm_sum if norm_sum else 0.0,
                "policy_batch_cosine": float(torch.nn.functional.cosine_similarity(
                    vector, batch_policy, dim=0
                )),
            })
        whole = _vector(self.raw_gradients, parameters, range(len(named)))
        reconstructed = _vector(
            _sum_gradients(gradients), parameters, range(len(named))
        )
        error = float((whole - reconstructed).norm() / whole.norm().clamp_min(1e-12))
        report["policy_gradient_source"] = (
            "actor_only" if getattr(agent, "detach_critic", False)
            else "actor_and_shared_critic"
        )
        return {
            "kl_diagnostics/policy_gradient_norm": float(batch_policy.norm()),
            "kl_diagnostics/max_policy_norm_share": (
                max(norms) / norm_sum if norm_sum else 0.
            ),
            "kl_diagnostics/cancellation_ratio": (
                float(torch.stack(vectors).sum(dim=0).norm()) / norm_sum
                if norm_sum else 0.
            ),
            "kl_diagnostics/gradient_reconstruction_error": error,
        }

    def _leave_one_out(self, agent, optimizer, before, curriculum, gradients, report):
        replay = self.replay
        assert replay is not None
        if len(self.buffers) < 2:
            report["leave_one_out_skipped"] = "Only one trajectory in the batch"
            return
        selected = sorted(
            report["episodes"], key=lambda row: -row["policy_gradient_norm"]
        )
        for episode in selected[:self.settings.loo_top_k]:
            index = episode["index"]
            restore_state(
                agent, optimizer, before, curriculum=curriculum, restore_rng=False
            )
            gradient = _leave_out_gradient(self.raw_gradients, gradients[index],
                                           replay.samples, replay.lengths[index])
            for parameter, value in zip(agent.parameters(), gradient, strict=True):
                parameter.grad = (
                    None if value is None else value.to(agent.device).clone()
                )
            clip_agent_gradients(agent, self.grad_clip)
            optimizer.step()
            require_finite(agent.state_dict(), "leave-one-out candidate")
            result = replay.metrics(agent)
            require_finite(result, "leave-one-out KL")
            report["leave_one_out"].append({
                "omitted_episode": index, **result,
                "kl_reduction": self.trials[0]["mean"] - result["mean"],
                "passes_kl_budget": result["mean"] <= self.settings.max_kl,
                "episodes": deepcopy(replay.episode_metrics),
            })

    def finish(self, agent, optimizer, before, curriculum, outcome):
        if not self.trials or self.trials[0]["mean"] <= self.settings.max_kl:
            return {}, None
        started = time.perf_counter()
        report = {
            "iteration": self.iteration, "rollout_seed": self.seed,
            "stage": curriculum.stage if curriculum is not None else None,
            "normalization": "sum of episode losses / total valid transitions",
            "leave_one_out_reference": (
                "all original trajectories, including omitted episode"
            ),
            "tbptt_steps": self.tbptt_steps, "value_loss_coef": self.value_coef,
            "max_kl": self.settings.max_kl, "trials": self.trials,
            "episodes": [], "leave_one_out": [],
        }
        metrics = {"kl_diagnostics/failed": 0.0}
        try:
            report["episodes"] = self._metadata()
            with _isolated_rng(agent.device):
                shadow, shadow_optimizer = deepcopy((agent, optimizer))
                shadow_curriculum = deepcopy(curriculum)
                restore_state(
                    shadow, shadow_optimizer, before, curriculum=shadow_curriculum
                )
                gradients = self._episode_gradients(shadow)
                metrics.update(self._gradient_summary(shadow, gradients, report))
                if metrics["kl_diagnostics/gradient_reconstruction_error"] <= 1e-3:
                    self._leave_one_out(shadow, shadow_optimizer, before,
                                        shadow_curriculum, gradients, report)
                else:
                    report["leave_one_out_skipped"] = (
                        "Gradient reconstruction error > 1e-3"
                    )
                    LOG.warning("Skipping leave-one-out at update %d: gradient "
                                "reconstruction error %.6g", self.iteration,
                                metrics["kl_diagnostics/gradient_reconstruction_error"])
        except Exception as error:
            metrics["kl_diagnostics/failed"] = 1.0
            report["error"] = f"{type(error).__name__}: {error}"
            LOG.exception("KL rejection analysis failed at update %d", self.iteration)
        if report["leave_one_out"]:
            metrics["kl_diagnostics/best_loo_kl_reduction"] = max(
                row["kl_reduction"] for row in report["leave_one_out"]
            )
        metrics["timing/kl_diagnostics_seconds"] = time.perf_counter() - started
        report["metrics"] = {**outcome, **metrics}
        _log_report(report)
        return metrics, report


def _log_report(report):
    initial = report["trials"][0]["episodes"]
    for episode in report["episodes"]:
        index = episode["index"]
        kl = initial[index]["mean"] if index < len(initial) else None
        LOG.info("KL trajectory %d: seed=%d steps=%d duration=%.3fs return=%.3f "
                 "v=%s yaw=%s KL=%s policy_grad=%s norm_share=%s cosine=%s",
                 index, episode["environment_seed"], episode["steps"],
                 episode["duration"], episode["return"], episode["target_velocity"],
                 episode["target_yaw_rate"], kl, episode.get("policy_gradient_norm"),
                 episode.get("policy_norm_share"),
                 episode.get("policy_batch_cosine"))
    for row in report["leave_one_out"]:
        LOG.info("KL without trajectory %d: mean %.6g, reduction %.6g, passes=%s",
                 row["omitted_episode"], row["mean"], row["kl_reduction"],
                 row["passes_kl_budget"])


def prepare_kl_diagnostics(
    cfg, settings, agent, buffers, targets, values, logps, iteration,
):
    if not settings.enabled or not settings.diagnostics:
        return None
    return KLDiagnostics(
        cfg, settings, agent, buffers, targets, values, logps, iteration
    )


def finish_kl_diagnostics(
    diagnostics, agent, optimizer, before, curriculum, outcome, reports,
):
    if diagnostics is None:
        return {}
    metrics, report = diagnostics.finish(agent, optimizer, before, curriculum, outcome)
    if report is not None and reports is not None:
        reports.append(report)
    return metrics


def publish_kl_report(cfg, report, *, log_metrics=True):
    """Keep rejection diagnostics even between ordinary metric logging intervals."""
    try:
        directory = Path(cfg.checkpoints.dir).parent / "kl_diagnostics"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"iteration_{report['iteration']:06d}.json"
        path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        LOG.info("Saved KL rejection analysis: %s", path)
        if cfg.logging.mlflow.enabled:
            mlflow.log_artifact(str(path), artifact_path="diagnostics/kl")
            if log_metrics:
                mlflow.log_metrics(report["metrics"], step=report["iteration"])
    except Exception:
        LOG.exception("Could not publish KL rejection analysis at update %d",
                      report["iteration"])

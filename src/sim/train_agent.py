import json
import logging
import os
import random
import traceback
from collections.abc import Sequence
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path

import dotenv
import hydra
import matplotlib.pyplot as plt
import mlflow
import numpy as np
import torch
from gymnasium import Env
from gymnasium.vector import SyncVectorEnv
from gymnasium.wrappers import RecordVideo
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import (
    Actions,
    DerivedObs,
    Observable,
    StateVar,
    StateVarKey,
    Target,
)
from riktigpatric.trajectory import (
    HeadTrajectory,
    PositionTrajectory,
    validate_head_target,
)
from sim.checkpoints import (
    PolicyGuard,
    capture_state,
    load_checkpoint,
    policy_spec,
    require_finite,
    restore_state,
    save_checkpoint,
)
from sim.curriculum import Curriculum
from sim.devices import evaluation_rng, resolve_device, seed_torch
from sim.episode_io import resample_video_frames, save_episode_csv
from sim.plot_utils import plot_episode
from sim.policy_probe import PolicyProbe
from sim.utils import (
    Episode,
    EpisodeBuffer,
    MiscKeys,
    SingleEnvWrapper,
    actor_critic_losses,
    ebufs2batchd,
    ebufs2step_times,
    flatten_dict,
    get_advantages,
    get_returns,
    npd2tensord,
    register_and_make_env,
    tensord2npd,
)

dotenv.load_dotenv()  # Load environment variables from .env file
LOG = logging.getLogger(__name__)

HYDRA_CONFIG_DIR = os.getenv(
    "HYDRA_CONFIG_DIR", str(Path(__file__).resolve().parents[2] / "config")
)

PlotKey = tuple[
    StateVarKey | Actions,
    tuple[StateVarKey | Actions | MiscKeys, ...],
]

PLOTKS = [
    (Observable.OBS_TIME, (Observable.RP_PITCH, Observable.TRUE_PITCH)),
    (
        Observable.OBS_TIME,
        (Observable.LEFT_WHEEL_VEL, Observable.RIGHT_WHEEL_VEL),
    ),
    (
        Observable.OBS_TIME,
        (Observable.HEAD_PITCH, Observable.HEAD_TURN),
    ),
]

TRACKING_INPUTS = {
    "position": (Target.TARGET_POS, DerivedObs.CURRENT_POS),
    "velocity": (Target.TARGET_VEL, DerivedObs.CURRENT_VEL),
    "position_velocity": (
        Target.TARGET_POS, DerivedObs.CURRENT_POS,
        Target.TARGET_VEL, DerivedObs.CURRENT_VEL,
    ),
    "none": (),
}

HEAD_INPUTS = (
    Target.CAMERA_PITCH_WORLD, Target.HEAD_YAW_NECK, DerivedObs.CAMERA_PITCH_WORLD,
    Observable.RP_ROLL, Observable.HEAD_PITCH_VEL, Observable.HEAD_TURN_VEL,
)

YAW_INPUTS = (Target.YAW_RATE, DerivedObs.YAW_RATE)


def get_policy_inputs(cfg: DictConfig) -> list[str]:
    """Append task inputs to the shared sensor inputs, independently of the agent."""
    return list(dict.fromkeys([
        *cfg.policy.inputs,
        *[key.value for key in TRACKING_INPUTS[cfg.env.tracking_mode]],
        *[key.value for key in HEAD_INPUTS if cfg.env.head_tracking],
        *[key.value for key in YAW_INPUTS if cfg.env.yaw_tracking],
        *[key.value for key in TRACKING_INPUTS["position"]
          if cfg.get("curriculum", {}).get("enabled", False)],
    ]))


def _take_idx_from_d(d, idx: int):
    return {k: v[idx].copy() for k, v in d.items()}


def _require_finite_observations(observations: dict, seed: int):
    for key, value in observations.items():
        if not np.isfinite(value).all():
            raise FloatingPointError(
                f"Non-finite observation {key}, rollout seed {seed}"
            )


def _zero_hidden_state(
    h: torch.Tensor | None, done: np.ndarray
) -> torch.Tensor | None:
    if h is None or not done.any():
        return h
    done_t = torch.from_numpy(done.astype(bool)).to(device=h.device)
    return torch.where(done_t.view(1, -1, 1), torch.zeros_like(h), h)


def add_targets(
    obs_d: dict[StateVarKey, np.ndarray],
    rp_env: SingleEnvWrapper | SyncVectorEnv,
    target_positions: list[float] | np.ndarray | None = None,
    target_trajectories: Sequence[Sequence[Sequence[float]]] | None = None,
    target_velocities: list[float] | np.ndarray | None = None,
):
    """Set a batch of position/velocity targets or position trajectories."""
    if sum(value is not None for value in (
        target_positions, target_trajectories, target_velocities
    )) > 1:
        raise ValueError("Supply only one type of target override per batch")
    if target_trajectories is not None:
        if len(target_trajectories) != rp_env.num_envs:
            raise ValueError(f"Expected {rp_env.num_envs} target trajectories")
        # Validate the entire batch before changing any environment.
        trajectories = [PositionTrajectory(points) for points in target_trajectories]
        rp_env.set_attr("target_trajectory", trajectories)
        obs_d[Target.TARGET_POS] = np.array(
            [
                [trajectory.position_at(float(time[0]))]
                for trajectory, time in zip(trajectories, obs_d[Observable.OBS_TIME])
            ],
            dtype=np.float32,
        )
        return obs_d
    key, attribute, values = (
        (Target.TARGET_VEL, "target_vel", target_velocities)
        if target_velocities is not None
        else (Target.TARGET_POS, "target_pos", target_positions)
    )
    return _add_scalar_targets(obs_d, rp_env, key, attribute, values)


def _add_scalar_targets(obs_d, rp_env, key, attribute, values):
    """Validate the whole batch before changing scalar references in any backend."""
    if values is None:
        return obs_d
    targets = np.asarray(values, dtype=np.float64)
    if (
        targets.shape != (rp_env.num_envs,)
        or not np.isfinite(targets).all()
        or np.any(np.abs(targets) > float(np.finfo(np.float32).max))
    ):
        raise ValueError(
            f"Expected {rp_env.num_envs} finite float32 {key.value} targets with shape "
            f"({rp_env.num_envs},), got {targets}"
        )
    targets = targets.astype(np.float32)
    # Gymnasium distributes lists/tuples; an ndarray would be broadcast whole.
    rp_env.set_attr(attribute, targets.tolist())
    obs_d[key] = targets[:, None].copy()
    return obs_d


def add_yaw_targets(obs_d, rp_env, target_yaw_rates=None):
    """Set independent body yaw-rate references, in rad/s; never wheel commands."""
    return _add_scalar_targets(
        obs_d, rp_env, Target.YAW_RATE, "target_yaw_rate", target_yaw_rates
    )


def add_head_targets(
    obs_d: dict[StateVarKey, np.ndarray],
    rp_env: SingleEnvWrapper | SyncVectorEnv,
    head_targets: Sequence[Sequence[float]] | np.ndarray | None = None,
    head_trajectories: Sequence[Sequence[Sequence[float]]] | None = None,
):
    """Set head references independently of the locomotion task; no actuation."""
    if head_targets is not None and head_trajectories is not None:
        raise ValueError("Choose fixed head targets or head trajectories, not both")
    if head_trajectories is not None:
        if len(head_trajectories) != rp_env.num_envs:
            raise ValueError(f"Expected {rp_env.num_envs} head trajectories")
        trajectories = [HeadTrajectory(points) for points in head_trajectories]
        values = np.stack([
            trajectory.angles_at(float(time[0]))
            for trajectory, time in zip(trajectories, obs_d[Observable.OBS_TIME])
        ])
        rp_env.set_attr("head_trajectory", trajectories)
    elif head_targets is not None:
        if np.asarray(head_targets).shape != (rp_env.num_envs, 2):
            raise ValueError(f"Expected head target shape ({rp_env.num_envs}, 2)")
        values = np.stack([validate_head_target(angles) for angles in head_targets])
        rp_env.set_attr("head_target", values.tolist())
    else:
        return obs_d
    for column, key in enumerate((Target.CAMERA_PITCH_WORLD, Target.HEAD_YAW_NECK)):
        obs_d[key] = values[:, column:column + 1].copy()
    return obs_d


def rollout(
    rp_env: SingleEnvWrapper | SyncVectorEnv,
    agent: Agent,
    seed: int = 0,
    target_positions: list[float] | np.ndarray | None = None,
    target_trajectories: Sequence[Sequence[Sequence[float]]] | None = None,
    target_velocities: list[float] | np.ndarray | None = None,
    head_targets: Sequence[Sequence[float]] | np.ndarray | None = None,
    head_trajectories: Sequence[Sequence[Sequence[float]]] | None = None,
    tbptt_steps: int | None = None,
    target_yaw_rates: list[float] | np.ndarray | None = None,
    curriculum: Curriculum | None = None,
    evaluation: bool = False,
    deterministic: bool = False,
    policy_probe: PolicyProbe | None = None,
) -> list[EpisodeBuffer]:
    """Collect episodes, optionally detaching recurrent history every N steps."""
    if tbptt_steps is not None:
        _validate_tbptt_steps(tbptt_steps)
    if curriculum is not None:
        commands = curriculum.prepare_rollout(rp_env, agent, evaluation=evaluation)
        target_velocities = commands["target_velocities"]
        target_yaw_rates = commands["target_yaw_rates"]
        head_targets = commands["head_targets"]
        head_trajectories = commands["head_trajectories"]
    num_envs = rp_env.num_envs
    active = np.ones(num_envs, dtype=bool)
    episode_buffers = [EpisodeBuffer() for _ in range(num_envs)]

    h = None
    step = 0
    obs_d, _ = rp_env.reset(seed=seed)
    obs_d = add_targets(
        obs_d, rp_env, target_positions, target_trajectories, target_velocities
    )
    obs_d = add_head_targets(obs_d, rp_env, head_targets, head_trajectories)
    obs_d = add_yaw_targets(obs_d, rp_env, target_yaw_rates)
    _require_finite_observations(obs_d, seed)
    while active.any():
        policy_obs = {key: obs_d[key] for key in (*agent.inputs, Observable.OBS_TIME)}
        obs_d_t = npd2tensord(policy_obs, device=agent.device)

        if policy_probe is not None and step % policy_probe.every == 0:
            policy_probe.capture(agent, obs_d_t, h, active)
        action, logp, value, h = agent.act(obs_d_t, h, deterministic=deterministic)
        require_finite((action, logp, value), "policy output")
        action_np = tensord2npd(action)
        next_obs_d, reward, terminated, truncated, info = rp_env.step(action_np)
        _require_finite_observations(next_obs_d, seed)
        if not np.isfinite(reward).all():
            raise FloatingPointError(f"Non-finite reward, rollout seed {seed}")

        done = (terminated | truncated) & active

        for env_idx in np.flatnonzero(active):
            episode_buffers[env_idx].add_step(
                obs_t=_take_idx_from_d(obs_d, env_idx),
                action_t=_take_idx_from_d(action_np, env_idx),
                reward_t=reward[env_idx],
                logp_t=logp[env_idx, 0],
                value_t=value[env_idx, 0],
                step_time=(
                    float(info["step_time"][env_idx]) if "step_time" in info else None
                ),
            )

        for env_idx in np.flatnonzero(done):
            episode_buffers[env_idx].finish(
                _take_idx_from_d(next_obs_d, env_idx),
                terminated=bool(terminated[env_idx]),
            )

        # If an episode is done, mark it as inactive and zero out its hidden state
        active = active & ~done
        h = _zero_hidden_state(h, done)
        step += 1
        if tbptt_steps is not None and step % tbptt_steps == 0 and h is not None:
            h = h.detach()  # Preserve memory values; cut only gradient history.

        # Update obs_d for the next step
        obs_d = next_obs_d

    return episode_buffers


@hydra.main(config_path=HYDRA_CONFIG_DIR, config_name="rlrp", version_base=None)
def main(cfg: DictConfig):
    device = resolve_device(cfg.train.device)
    seed_torch(cfg.seed, device)
    np.random.seed(cfg.seed)
    random.seed(cfg.seed)
    with ExitStack() as resources:
        train(cfg, resources, device=device)


def _log_run_config(cfg: DictConfig):
    """Keep searchable parameters and a complete, resolved YAML in the active run."""
    resolved = OmegaConf.to_container(cfg, resolve=True)
    assert isinstance(resolved, dict)
    mlflow.log_params({str(k): str(v) for k, v in flatten_dict(resolved).items()})
    mlflow.log_text(
        OmegaConf.to_yaml(OmegaConf.create(resolved)), "config/resolved.yaml"
    )


def train(cfg: DictConfig, resources: ExitStack, device: torch.device | None = None):
    device = resolve_device(cfg.train.device if device is None else device)
    _validate_tbptt_steps(cfg.train.tbptt_steps)
    if cfg.checkpoints.every < 1 or cfg.guard.every < 1 or cfg.guard.episodes < 1:
        raise ValueError(
            "Checkpoint/validation intervals and episode count must be positive"
        )
    if cfg.train.resume_from is not None and cfg.policy.restore_id is not None:
        raise ValueError("Choose a training checkpoint or an MLflow policy, not both")
    if cfg.train.resume_lr is not None and cfg.train.resume_from is None:
        raise ValueError("train.resume_lr requires train.resume_from")
    curriculum = Curriculum(cfg) if cfg.curriculum.enabled else None
    # Setup MLflow if enabled
    if cfg.logging.mlflow.enabled:
        if (tracking_uri := os.getenv("MLFLOW_TRACKING_URI")) is None:
            raise ValueError("MLFLOW_TRACKING_URI is not set")
        mlflow.set_tracking_uri(tracking_uri)
        mlflow.set_experiment(cfg.logging.mlflow.experiment_name)
        resources.enter_context(mlflow.start_run())
        mlflow.set_tags({
            "training.device": str(device),
            "training.pytorch_version": str(torch.__version__),
            "training.cuda_version": torch.version.cuda or "none",
        })

        _log_run_config(cfg)

    # Create environment
    rp_env = register_and_make_env(cfg)
    resources.callback(rp_env.close)

    if isinstance(rp_env, Env):
        rp_env = SingleEnvWrapper(rp_env)

    rp_video_env = make_video_env(cfg, resources)

    agent = make_agent(cfg, device)
    if curriculum is not None:
        curriculum.apply_policy(agent)
    LOG.info("Policy device %s; GRU hidden size %d, layers %d",
             agent.device, agent.rnn.hidden_size, agent.rnn.num_layers)
    optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
    next_iteration = 0
    if cfg.train.resume_from is not None:
        next_iteration = load_checkpoint(
            cfg.train.resume_from, agent, optimizer, curriculum,
            learning_rate=cfg.train.resume_lr,
        )
        LOG.info("Resumed at iteration %d; Adam LR %.3g",
                 next_iteration, optimizer.param_groups[0]["lr"])
    agent.train()
    require_finite(agent.state_dict(), "model")
    guard = PolicyGuard(
        agent, optimizer, drop_fraction=cfg.guard.drop_fraction,
        absolute_drop=cfg.guard.absolute_drop, patience=cfg.guard.patience,
        min_best_return=cfg.guard.min_best_return,
        lr_factor=cfg.guard.lr_factor, min_lr=cfg.guard.min_lr,
        recovery_attempts=cfg.guard.get("recovery_attempts", 3),
        curriculum=curriculum,
    )
    validation_env = (
        make_validation_env(cfg, resources, curriculum)
        if cfg.guard.enabled or curriculum is not None
        or cfg.logging.get("record_best", True) else None
    )
    stage_runs = (
        resources.enter_context(ExitStack())
        if cfg.logging.mlflow.enabled and curriculum is not None else None
    )
    run_training_loop(
        cfg, rp_env, rp_video_env, validation_env,
        agent, optimizer, guard, next_iteration, curriculum, stage_runs,
    )


def make_agent(cfg: DictConfig, device: torch.device | None = None) -> Agent:
    device = resolve_device(cfg.train.device if device is None else device)
    if cfg.policy.restore_id is not None:
        LOG.info("Restoring agent from MLflow model ID: %s", cfg.policy.restore_id)
        agent = mlflow.pytorch.load_model(
            mlflow.get_logged_model(cfg.policy.restore_id).model_uri,
            map_location=device,
        )
        if (
            set(agent.inputs) != set(get_policy_inputs(cfg))
            or policy_spec(agent)["actions"] != OmegaConf.to_container(
                cfg.policy.actions, resolve=True
            )
            or agent.rnn.hidden_size != cfg.policy.hsize
            or agent.rnn.num_layers != cfg.policy.n_rnnlayers
        ):
            raise ValueError(
                "Restored policy architecture/inputs/actions do not match this task"
            )
        agent.inactive_actions = ()  # Non-curriculum fine-tuning enables every head.
        return agent.to(device)
    return Agent(
        inputs=get_policy_inputs(cfg), actions=cfg.policy.actions,
        hsize=cfg.policy.hsize, n_rnnlayers=cfg.policy.n_rnnlayers,
    ).to(device)


def _validate_tbptt_steps(steps: int) -> None:
    if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
        raise ValueError("train.tbptt_steps must be a positive integer")


def _restore_after_failure(error, before, agent, optimizer, curriculum):
    # Keep the CPU backup saveable even if a device error prevents in-memory
    # restoration. Preserve the original failure and its recovery snapshot.
    setattr(error, "training_state", before)
    try:
        restore_state(agent, optimizer, before, curriculum=curriculum)
    except Exception:
        LOG.exception("In-memory rollback failed; CPU recovery snapshot retained")


def training_rollout(cfg, rp_env, agent, seed, curriculum=None, policy_probe=None):
    """Collect the configured training task for an update or gradient diagnostic."""
    return rollout(
        rp_env, agent, seed=seed,
        target_positions=cfg.train.target_positions,
        target_trajectories=cfg.train.target_trajectories,
        target_velocities=cfg.train.target_velocities,
        head_targets=cfg.train.head_targets,
        head_trajectories=cfg.train.head_trajectories,
        tbptt_steps=cfg.train.tbptt_steps,
        target_yaw_rates=cfg.train.target_yaw_rates,
        curriculum=curriculum, policy_probe=policy_probe,
    )


def training_update(
    cfg, rp_env, agent, optimizer, iteration: int, curriculum: Curriculum | None = None
) -> dict[str, float]:
    """One transactional update: a failed/non-finite update restores its input state."""
    if curriculum is not None:
        curriculum.apply_policy(agent)
    before = capture_state(agent, optimizer, iteration, curriculum)
    try:
        optimizer.zero_grad(set_to_none=True)
        _validate_tbptt_steps(cfg.train.tbptt_steps)
        coefficient = cfg.rl.value_loss_coef
        if not np.isfinite(coefficient) or coefficient < 0:
            raise ValueError("rl.value_loss_coef must be finite and nonnegative")
        probe_every = cfg.train.kl_probe_every
        if type(probe_every) is not int or probe_every < 0:
            raise ValueError("train.kl_probe_every must be a nonnegative integer")
        probe = PolicyProbe(agent, probe_every) if probe_every else None
        episode_buf_l = training_rollout(
            cfg, rp_env, agent, cfg.seed + iteration, curriculum, probe,
        )

        logps, rewards, values, seq_lens, valid_mask = ebufs2batchd(
            episode_buf_l, device=agent.device
        )

        step_times = ebufs2step_times(episode_buf_l, device=agent.device)
        G_t = get_returns(
            rewards, discount=cfg.rl.discount, step_times=step_times,
            reference_step_time=cfg.env.step_time,
        )
        policy_loss, values_loss = actor_critic_losses(logps, G_t, values, valid_mask)

        weighted_value_loss = coefficient * values_loss
        loss = policy_loss + weighted_value_loss
        require_finite((policy_loss, values_loss, loss), "loss")
        loss.backward()
        gradient_norm = torch.nn.utils.clip_grad_norm_(
            agent.parameters(), max_norm=cfg.train.grad_clip, error_if_nonfinite=True
        )
        optimizer.step()
        require_finite(agent.state_dict(), "updated model")
        require_finite(optimizer.state_dict(), "updated optimizer")
        probe_metrics = probe.metrics(agent) if probe is not None else {}
        require_finite(probe_metrics, "policy change")
        # CPU metric transfers are part of accepting the update too.
        episode_returns = (rewards * valid_mask).sum(dim=1).cpu().numpy()
        durations = step_times.sum(dim=1).cpu().numpy()
        valid_times = step_times[valid_mask.bool()]
        return {
            "losses/policy": policy_loss.item(), "losses/value": values_loss.item(),
            "losses/value_weighted": weighted_value_loss.item(),
            "losses/total": loss.item(), "returns/mean": float(episode_returns.mean()),
            "returns/max": float(episode_returns.max()),
            "returns/min": float(episode_returns.min()),
            "returns/std": float(episode_returns.std()),
            "episodes/length/min": float(min(seq_lens)),
            "episodes/length/max": float(max(seq_lens)),
            "episodes/length/mean": float(seq_lens.mean()),
            "episodes/duration/mean": float(durations.mean()),
            "episodes/duration/min": float(durations.min()),
            "episodes/duration/max": float(durations.max()),
            "timing/step_time/mean": float(valid_times.mean()),
            "timing/step_time/min": float(valid_times.min()),
            "timing/step_time/max": float(valid_times.max()),
            "timing/step_time/std": float(valid_times.std(unbiased=False)),
            "optimization/gradient_norm": float(gradient_norm),
            "optimization/learning_rate": float(optimizer.param_groups[0]["lr"]),
            **probe_metrics,
            **{
                f"policy/std/{action}": float(head.std.detach())
                for action, head in agent.action_heads.items()
                if isinstance(head, ContinuousHead)
            },
        }
    except BaseException as error:
        _restore_after_failure(error, before, agent, optimizer, curriculum)
        raise


def write_checkpoint(cfg, state, name: str, score=None, *, upload: bool = True) -> Path:
    config = OmegaConf.to_container(cfg, resolve=True)
    assert isinstance(config, dict)
    path = save_checkpoint(Path(cfg.checkpoints.dir) / name, state, config, score)
    if upload and cfg.logging.mlflow.enabled:
        mlflow.log_artifact(str(path), artifact_path="checkpoints")
    return path


def make_validation_env(
    cfg, resources: ExitStack, curriculum: Curriculum | None = None
):
    validation_cfg = deepcopy(cfg)
    validation_cfg.env.n_parallel = cfg.guard.episodes
    if curriculum is not None:
        validation_cfg.env.randomize = True
    env = register_and_make_env(validation_cfg)
    resources.callback(env.close)
    return SingleEnvWrapper(env) if isinstance(env, Env) else env


def evaluation_rollout(
    env, agent: Agent, seed: int, curriculum: Curriculum | None = None,
    *, deterministic: bool = False,
) -> list[EpisodeBuffer]:
    """Use eval() without gradients and isolate all evaluation random streams."""
    was_training = agent.training
    try:
        agent.eval()
        with evaluation_rng(agent.device, seed), torch.no_grad():
            return rollout(
                env, agent, seed=seed, curriculum=curriculum, evaluation=True,
                deterministic=deterministic,
            )
    finally:
        agent.train(was_training)


def validate_policy(
    cfg, env, agent, curriculum: Curriculum | None = None
) -> dict[str, float]:
    """Fixed stochastic action/environment seeds, without affecting training RNG."""
    buffers = evaluation_rollout(env, agent, cfg.guard.seed, curriculum)
    metrics = _validation_metrics(buffers, curriculum)
    if cfg.guard.get("compare_deterministic", False):
        deterministic = evaluation_rollout(
            env, agent, cfg.guard.seed, curriculum, deterministic=True
        )
        metrics.update({
            key.replace("validation/", "validation_deterministic/", 1): value
            for key, value in _validation_metrics(deterministic, curriculum).items()
        })
    return metrics


def _validation_metrics(buffers, curriculum: Curriculum | None):
    returns = np.array([sum(buffer.rewards_l) for buffer in buffers])
    if not np.isfinite(returns).all():
        raise FloatingPointError("Non-finite validation returns")
    metrics = {
        "validation/returns/mean": float(returns.mean()),
        "validation/returns/min": float(returns.min()),
        "validation/returns/max": float(returns.max()),
        "validation/returns/std": float(returns.std()),
        "validation/episodes/length/mean": float(
            np.mean([b.seq_len for b in buffers])
        ),
        "validation/episodes/duration/mean": float(
            np.mean([b.get_step_times().sum() for b in buffers])
        ),
    }
    if curriculum is not None:
        metrics.update(curriculum.validation_metrics(buffers))
    return metrics


def start_stage_run(cfg, agent, curriculum: Curriculum, iteration: int, stage_runs):
    """Close the completed child run, keeping the overall session's parent open."""
    if stage_runs is None:
        return
    stage_runs.close()
    stage_runs.enter_context(mlflow.start_run(run_name=curriculum.stage, nested=True))
    _log_run_config(cfg)
    mlflow.set_tags({
        "curriculum.stage": curriculum.stage,
        "curriculum.version": curriculum.VERSION,
        "curriculum.stage_index": curriculum.index,
        "training.start_iteration": iteration,
        "training.resume_from": cfg.train.resume_from or "",
        "training.device": str(agent.device),
        "training.inactive_actions": ",".join(curriculum.inactive_actions),
        "training.disabled_rewards": ",".join(curriculum.disabled_rewards),
        "training.tracking_mode": curriculum.tracking_mode,
        "training.velocity_reward_weight": curriculum.velocity_reward_weight,
    })


def _log_guard_decision(guard, decision, score):
    if decision == "rollback":
        LOG.warning("Policy degradation: restored best return %.2f; LR now %.3g",
                    guard.best_score, guard.optimizer.param_groups[0]["lr"])
        if guard.recovering:
            LOG.warning("Checking every subsequent update against rejection threshold "
                        "%.2f; stopping after %d consecutive degraded attempts",
                        guard.rejection_threshold, guard.recovery_attempts)
    elif decision in ("recovery_rejected", "recovery_stop"):
        LOG.warning("Recovery attempt %d/%d rejected: candidate %.2f below %.2f "
                    "(best %.2f); restored weights and Adam, kept LR %.3g",
                    guard.recovery_failures, guard.recovery_attempts, score,
                    guard.rejection_threshold, guard.best_score,
                    guard.optimizer.param_groups[0]["lr"])
    elif decision == "recovery_accepted":
        LOG.info("Recovery candidate accepted: %.2f >= %.2f (best %.2f)",
                 score, guard.rejection_threshold, guard.best_score)


def _write_recovery_stop(cfg, guard, iteration, score, metrics, curriculum):
    if not guard.stop_requested:
        return
    report = {
        "reason": "recovery_exhausted", "next_iteration": iteration,
        "stage": curriculum.stage if curriculum is not None else None,
        "best_iteration": guard.best_state["next_iteration"],
        "best_return": guard.best_score, "rejected_return": score,
        "rejection_threshold": guard.rejection_threshold,
        "recovery_failures": guard.recovery_failures,
        "learning_rates": [g["lr"] for g in guard.optimizer.param_groups],
        "candidate_metrics": metrics,
    }
    path = Path(cfg.checkpoints.dir) / "training_stop.json"
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    LOG.warning("Recovery exhausted; stopped with the best policy restored. "
                "Resume checkpoint: %s; diagnostics: %s",
                path.with_name("latest.pt"), path)
    if cfg.logging.mlflow.enabled:
        mlflow.log_artifact(str(path), artifact_path="checkpoints")


def _record_best_policy(cfg, video_env, agent, iteration, curriculum, decision):
    if (
        decision != "best" or video_env is None
        or not cfg.logging.get("record_best", True)
    ):
        return False
    stage = f"{curriculum.stage}_" if curriculum is not None else ""
    evaluate_and_plot(
        cfg, video_env, agent, iteration, curriculum,
        artifact_name=f"best_{stage}iter_{iteration:06d}",
    )
    return True


def check_policy_guard(
    cfg, env, agent, guard, next_iteration: int,
    curriculum: Curriculum | None = None, stage_runs=None, *, advance: bool = True,
    video_env=None,
):
    """Validate and save progress; return whether the current policy was recorded."""
    metrics = validate_policy(cfg, env, agent, curriculum)
    score = metrics["validation/returns/mean"]
    promote = (
        curriculum.observe(metrics) if curriculum is not None and advance else False
    )
    decision = guard.observe(
        score, next_iteration, allow_rollback=cfg.guard.get("enabled", True)
    )
    rejected = decision in ("rollback", "recovery_rejected", "recovery_stop")
    if rejected:
        promote = False
    if decision == "best":
        write_checkpoint(cfg, guard.best_state, "best.pt", score)
    _log_guard_decision(guard, decision, score)
    if guard.best_score is not None:
        metrics.update({
            "guard/best_return": guard.best_score,
            "guard/rejection_threshold": guard.rejection_threshold,
            "guard/bad_evaluations": guard.bad_evaluations,
            "guard/rollbacks": guard.rollbacks,
            "guard/rolled_back": int(rejected),
            "guard/learning_rate": guard.optimizer.param_groups[0]["lr"],
            "guard/recovering": int(guard.recovering),
            "guard/recovery_failures": guard.recovery_failures,
            "guard/stop_requested": int(guard.stop_requested),
        })
    if curriculum is not None:
        metrics.update({"curriculum/stage": curriculum.index,
                        "curriculum/success_streak": curriculum.success_streak})
        LOG.info(
            "Curriculum %s%s: survival %.0f%%, position MAE %.3f m, "
            "velocity MAE %.3f m/s, yaw MAE %.3f rad/s; passing evaluations %d",
            curriculum.stage, " (rejected candidate)" if rejected else "",
            100 * metrics["validation/survival_fraction"],
            metrics["validation/position_mae"],
            metrics["validation/velocity_mae"], metrics["validation/yaw_rate_mae"],
            curriculum.success_streak,
        )
    if decision in ("best", "recovery_accepted") or rejected or curriculum is not None:
        # Save progress even without a new best score, including a rollback's LR.
        write_checkpoint(
            cfg, capture_state(agent, guard.optimizer, next_iteration, curriculum),
            "latest.pt",
        )
    LOG.info("Validation at %d: %.2f; best %s (%s)",
             next_iteration, score, guard.best_score, decision)
    if cfg.logging.mlflow.enabled:
        mlflow.log_metrics(metrics, step=next_iteration)
    _write_recovery_stop(cfg, guard, next_iteration, score, metrics, curriculum)
    recorded = _record_best_policy(
        cfg, video_env, agent, next_iteration, curriculum, decision,
    )
    if promote:
        assert curriculum is not None
        before = capture_state(agent, guard.optimizer, next_iteration, curriculum)
        write_checkpoint(cfg, before, f"stage_{curriculum.stage}_complete.pt", score)
        try:
            curriculum.advance(agent, guard.optimizer)
            require_finite(agent.state_dict(), "promoted model")
        except BaseException as error:
            _restore_after_failure(error, before, agent, guard.optimizer, curriculum)
            raise
        guard.reset_baseline()
        LOG.info("Curriculum promoted to %s at iteration %d",
                 curriculum.stage, next_iteration)
        start_stage_run(cfg, agent, curriculum, next_iteration, stage_runs)
        write_checkpoint(
            cfg, capture_state(agent, guard.optimizer, next_iteration, curriculum),
            "latest.pt",
        )
        # Establish a comparable baseline before learning under the new objective.
        recorded = check_policy_guard(
            cfg, env, agent, guard, next_iteration, curriculum, stage_runs,
            advance=False, video_env=video_env,
        ) or recorded
    return recorded


def preserve_training_failure(
    cfg, agent, optimizer, next_iteration: int, report: str,
    *, state: dict | None = None,
    curriculum: Curriculum | None = None,
):
    LOG.error("Training stopped; preserving the last finite state:\n%s", report)
    try:
        if state is None:
            state = capture_state(agent, optimizer, next_iteration, curriculum)
        path = write_checkpoint(cfg, state, "latest.pt", upload=False)
        failure = path.parent / "training_failure.txt"
        failure.write_text(report, encoding="utf-8")
        if cfg.logging.mlflow.enabled:
            mlflow.log_artifact(str(path), artifact_path="checkpoints")
            mlflow.log_artifact(str(failure), artifact_path="checkpoints")
    except Exception:
        LOG.exception("Could not persist/upload failure artifacts")


def _log_training_update(cfg, iteration, metrics):
    LOG.info("Step %4d: p_l=%.4f, v_l=%.4f, ret=%.2f, mean_ep_len=%.0f",
             iteration, metrics["losses/policy"], metrics["losses/value"],
             metrics["returns/mean"], metrics["episodes/length/mean"])
    if cfg.logging.mlflow.enabled and iteration % cfg.logging.mlflow.push_freq == 0:
        mlflow.log_metrics(metrics, step=iteration)


def run_training_loop(
    cfg, env, video_env, validation_env, agent, optimizer, guard, start,
    curriculum: Curriculum | None = None, stage_runs=None,
):
    next_iteration = start
    try:
        if curriculum is not None:
            curriculum.apply_policy(agent)
            start_stage_run(cfg, agent, curriculum, start, stage_runs)
        write_checkpoint(
            cfg, capture_state(agent, optimizer, start, curriculum), "latest.pt"
        )
        if validation_env is not None:
            check_policy_guard(
                cfg, validation_env, agent, guard, start, curriculum, stage_runs,
                advance=False, video_env=video_env,
            )
        while (
            cfg.train.max_iterations is None
            or next_iteration < cfg.train.max_iterations
        ):
            iteration = next_iteration
            metrics = training_update(cfg, env, agent, optimizer, iteration, curriculum)
            next_iteration = iteration + 1
            _log_training_update(cfg, iteration, metrics)
            scheduled = next_iteration % cfg.guard.every == 0
            best_recorded = False
            if validation_env is not None and guard.should_evaluate(
                next_iteration, cfg.guard.every
            ):
                best_recorded = check_policy_guard(
                    cfg, validation_env, agent, guard, next_iteration, curriculum,
                    stage_runs, advance=scheduled, video_env=video_env,
                )
                if guard.stop_requested:
                    break
            if next_iteration % cfg.checkpoints.every == 0:
                state = capture_state(agent, optimizer, next_iteration, curriculum)
                write_checkpoint(cfg, state, f"iteration_{iteration:06d}.pt")
                write_checkpoint(cfg, state, "latest.pt")
            _record_periodic_policy(
                cfg, video_env, agent, iteration, curriculum, best_recorded,
            )
            if cfg.logging.mlflow.enabled and iteration % cfg.logging.save_freq == 0:
                mlflow.pytorch.log_model(
                    agent, name=f"agent_{iteration:04d}", step=iteration
                )
        write_checkpoint(
            cfg, capture_state(agent, optimizer, next_iteration, curriculum),
            "latest.pt",
        )
    except BaseException as error:
        preserve_training_failure(
            cfg, agent, optimizer, next_iteration, traceback.format_exc(),
            state=getattr(error, "training_state", None),
            curriculum=curriculum,
        )
        raise


def _record_periodic_policy(
    cfg, video_env, agent, iteration, curriculum, best_recorded,
):
    if (
        video_env is not None and not best_recorded and cfg.logging.plot_freq > 0
        and iteration % cfg.logging.plot_freq == 0
    ):
        evaluate_and_plot(cfg, video_env, agent, iteration, curriculum)


def make_video_env(cfg: DictConfig, resources: ExitStack) -> SingleEnvWrapper | None:
    if cfg.logging.plot_freq <= 0 and not cfg.logging.get("record_best", True):
        return None
    env = register_and_make_env(cfg, force_single_env=True, force_non_random=True)
    assert isinstance(env, Env)
    resources.callback(env.close)
    video = RecordVideo(env, "video/", episode_trigger=lambda _: True)
    resources.callback(video.close)
    return SingleEnvWrapper(video)


def evaluate_and_plot(
    cfg: DictConfig, env: SingleEnvWrapper, agent: Agent, i: int,
    curriculum: Curriculum | None = None, *, artifact_name: str | None = None,
):
    artifact_name = artifact_name or f"train_iter_{i:06d}"
    env.name_prefix = artifact_name
    buffers = evaluation_rollout(env, agent, cfg.seed + i + 10000, curriculum)
    _, rewards, values, _, _ = ebufs2batchd(buffers, device=agent.device)
    if cfg.env.step_time_std > 0:
        env.recorded_frames = resample_video_frames(
            env.recorded_frames, buffers[0].get_step_times(), env.frames_per_sec
        )
    env.stop_recording()
    recorded_path = Path(env.video_folder) / (
        f"{env.name_prefix}-episode-{env.episode_id}.mp4"
    )
    video_path = recorded_path.with_name(f"{artifact_name}.mp4")
    recorded_path.replace(video_path)  # Drop Gymnasium's recording-episode counter.

    returns = get_returns(
        rewards, discount=cfg.rl.discount,
        step_times=ebufs2step_times(buffers, device=agent.device),
        reference_step_time=cfg.env.step_time,
    )
    advantages = get_advantages(returns, values)
    eps = Episode(
        buffers[0],
        value_estimates=values.cpu().numpy(),
        returns=returns.cpu().numpy(),
        advantages=advantages.cpu().numpy(),
    )

    plot_path = Path("plots") / f"{artifact_name}.png"
    trace_path = save_episode_csv(
        buffers[0],
        plot_path.with_suffix(".csv"),
        returns=returns[0].cpu().numpy(),
        advantages=advantages[0].cpu().numpy(),
    )
    fig = plot_episode(
        eps, keys=get_plot_keys(cfg, agent, curriculum), save_path=plot_path
    )
    plt.close(fig)
    if cfg.logging.mlflow.enabled:
        for artifact in (plot_path, trace_path, video_path):
            mlflow.log_artifact(str(artifact))
        if cfg.env.head_tracking:
            errors = {
                "evaluation/head/camera_pitch_mae": (
                    Observable.TRUE_CAMERA_PITCH, Target.CAMERA_PITCH_WORLD
                ),
                "evaluation/head/neck_yaw_mae": (
                    Observable.HEAD_TURN, Target.HEAD_YAW_NECK
                ),
                "evaluation/head/camera_estimation_mae": (
                    DerivedObs.CAMERA_PITCH_WORLD, Observable.TRUE_CAMERA_PITCH
                ),
            }
            mlflow.log_metrics({
                name: float(
                    np.abs(eps.get_data(actual)[1:] - eps.get_data(target)[1:]).mean()
                )
                for name, (actual, target) in errors.items()
            }, step=i)
    print(f"  📊 Saved plot: {plot_path}")
    print(f"  Saved evaluation trace: {trace_path}")
    print(f"  Saved video: {video_path}")


def get_plot_keys(
    cfg: DictConfig, agent: Agent, curriculum: Curriculum | None = None
) -> list[PlotKey]:
    """Plot the selected tracking task, policy inputs, and active reward terms."""
    plot_keys: list[PlotKey] = list(PLOTKS)
    mode = curriculum.tracking_mode if curriculum is not None else cfg.env.tracking_mode
    tracking_modes = (
        ("position", "velocity") if mode == "position_velocity" else (mode,)
    )
    plot_keys.extend(
        (Observable.OBS_TIME, TRACKING_INPUTS[kind])
        for kind in tracking_modes if TRACKING_INPUTS[kind]
    )
    if cfg.env.yaw_tracking:
        plot_keys.append((Observable.OBS_TIME, YAW_INPUTS))
    if cfg.env.head_tracking:
        plot_keys.extend([
            (Observable.OBS_TIME, (
                Target.CAMERA_PITCH_WORLD, DerivedObs.CAMERA_PITCH_WORLD,
                Observable.TRUE_CAMERA_PITCH,
            )),
            (Observable.OBS_TIME, (Target.HEAD_YAW_NECK, Observable.HEAD_TURN)),
        ])
    seen_observables = {
        Observable.OBS_TIME.value,
        *[key.value for _, keys in plot_keys for key in keys],
    }
    for observable in agent.inputs:
        if observable not in seen_observables:
            plot_keys.append((Observable.OBS_TIME, (StateVar.from_str(observable),)))
    for action_name in agent.actions:
        plot_keys.append((Actions.TIME, (Actions.from_str(action_name),)))
    inactive_rewards = {
        "position": {Observable.REWARD_VEL},
        "velocity": {Observable.REWARD_POS, Observable.REWARD_WHEEL_VEL},
        "position_velocity": {Observable.REWARD_WHEEL_VEL},
        "none": {Observable.REWARD_POS, Observable.REWARD_VEL},
    }[mode]
    if curriculum is not None:
        inactive_rewards.update(curriculum.disabled_rewards)
    if not cfg.env.yaw_tracking:
        inactive_rewards.add(Observable.REWARD_YAW_RATE)
    if cfg.env.head_tracking:
        inactive_rewards.add(Observable.REWARD_HEAD_PITCH)
    else:
        inactive_rewards.update({
            Observable.REWARD_CAMERA_PITCH, Observable.REWARD_HEAD_YAW
        })
    reward_keys = tuple(
        Observable.from_str(key) for key in cfg.reward if key not in inactive_rewards
    )
    plot_keys.append((Observable.OBS_TIME, reward_keys))
    plot_keys.append(
        (Actions.TIME, (MiscKeys.VALUE_ESTIM, MiscKeys.RETURNS, MiscKeys.ADVANTAGES))
    )

    return plot_keys


if __name__ == "__main__":
    main()

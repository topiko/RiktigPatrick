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

from nn_ctrl.nns import Agent
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
    require_finite,
    restore_state,
    save_checkpoint,
)
from sim.devices import evaluation_rng, resolve_device, seed_torch
from sim.episode_io import save_episode_csv
from sim.plot_utils import plot_episode
from sim.utils import (
    Episode,
    EpisodeBuffer,
    MiscKeys,
    SingleEnvWrapper,
    ebufs2batchd,
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
    "none": (),
}

HEAD_INPUTS = (
    Target.CAMERA_PITCH_WORLD, Target.HEAD_YAW_NECK, DerivedObs.CAMERA_PITCH_WORLD,
    Observable.RP_ROLL, Observable.HEAD_PITCH_VEL, Observable.HEAD_TURN_VEL,
)


def get_policy_inputs(cfg: DictConfig) -> list[str]:
    """Append task inputs to the shared sensor inputs, independently of the agent."""
    return list(dict.fromkeys([
        *cfg.policy.inputs,
        *[key.value for key in TRACKING_INPUTS[cfg.env.tracking_mode]],
        *[key.value for key in HEAD_INPUTS if cfg.env.head_tracking],
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
) -> list[EpisodeBuffer]:
    num_envs = rp_env.num_envs
    active = np.ones(num_envs, dtype=bool)
    episode_buffers = [EpisodeBuffer() for _ in range(num_envs)]

    h = None
    obs_d, _ = rp_env.reset(seed=seed)
    obs_d = add_targets(
        obs_d, rp_env, target_positions, target_trajectories, target_velocities
    )
    obs_d = add_head_targets(obs_d, rp_env, head_targets, head_trajectories)
    _require_finite_observations(obs_d, seed)
    while active.any():
        policy_obs = {key: obs_d[key] for key in (*agent.inputs, Observable.OBS_TIME)}
        obs_d_t = npd2tensord(policy_obs, device=agent.device)

        action, logp, value, h = agent.act(obs_d_t, h)
        require_finite((action, logp, value), "policy output")
        action_np = tensord2npd(action)
        next_obs_d, reward, terminated, truncated, _ = rp_env.step(action_np)
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
            )

        for env_idx in np.flatnonzero(done):
            episode_buffers[env_idx].finish(_take_idx_from_d(next_obs_d, env_idx))

        # If an episode is done, mark it as inactive and zero out its hidden state
        active = active & ~done
        h = _zero_hidden_state(h, done)

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


def train(cfg: DictConfig, resources: ExitStack, device: torch.device | None = None):
    device = resolve_device(cfg.train.device if device is None else device)
    if cfg.checkpoints.every < 1 or cfg.guard.every < 1 or cfg.guard.episodes < 1:
        raise ValueError(
            "Checkpoint/validation intervals and episode count must be positive"
        )
    if cfg.train.resume_from is not None and cfg.policy.restore_id is not None:
        raise ValueError("Choose a training checkpoint or an MLflow policy, not both")
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

        # Log config parameters
        flat_params = OmegaConf.to_container(cfg, resolve=True)
        mlflow.log_params(
            {str(k): str(v) for k, v in flatten_dict(flat_params).items()}
        )

    # Create environment
    rp_env = register_and_make_env(cfg)
    resources.callback(rp_env.close)

    if isinstance(rp_env, Env):
        rp_env = SingleEnvWrapper(rp_env)

    rp_video_env = make_video_env(cfg, resources)

    agent = make_agent(cfg, device)
    LOG.info("Policy device %s; GRU hidden size %d, layers %d",
             agent.device, agent.rnn.hidden_size, agent.rnn.num_layers)
    optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
    next_iteration = 0
    if cfg.train.resume_from is not None:
        next_iteration = load_checkpoint(cfg.train.resume_from, agent, optimizer)
        LOG.info("Resumed training at iteration %d", next_iteration)
    agent.train()
    require_finite(agent.state_dict(), "model")
    guard = PolicyGuard(
        agent, optimizer, drop_fraction=cfg.guard.drop_fraction,
        absolute_drop=cfg.guard.absolute_drop, patience=cfg.guard.patience,
        min_best_return=cfg.guard.min_best_return,
        lr_factor=cfg.guard.lr_factor, min_lr=cfg.guard.min_lr,
    )
    validation_env = make_validation_env(cfg, resources) if cfg.guard.enabled else None
    run_training_loop(
        cfg, rp_env, rp_video_env, validation_env,
        agent, optimizer, guard, next_iteration,
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
            or set(agent.actions) != set(cfg.policy.actions)
            or agent.rnn.hidden_size != cfg.policy.hsize
            or agent.rnn.num_layers != cfg.policy.n_rnnlayers
        ):
            raise ValueError(
                "Restored policy architecture/inputs/actions do not match this task"
            )
        return agent.to(device)
    return Agent(
        inputs=get_policy_inputs(cfg), actions=cfg.policy.actions,
        hsize=cfg.policy.hsize, n_rnnlayers=cfg.policy.n_rnnlayers,
    ).to(device)


def training_update(cfg, rp_env, agent, optimizer, iteration: int) -> dict[str, float]:
    """One transactional update: a failed/non-finite update restores its input state."""
    before = capture_state(agent, optimizer, iteration)
    try:
        optimizer.zero_grad(set_to_none=True)
        episode_buf_l = rollout(
            rp_env,
            agent,
            seed=cfg.seed + iteration,
            target_positions=cfg.train.target_positions,
            target_trajectories=cfg.train.target_trajectories,
            target_velocities=cfg.train.target_velocities,
            head_targets=cfg.train.head_targets,
            head_trajectories=cfg.train.head_trajectories,
        )

        logps, rewards, values, seq_lens, valid_mask = ebufs2batchd(
            episode_buf_l, device=agent.device
        )

        G_t = get_returns(rewards, discount=cfg.rl.discount)
        advantages = get_advantages(G_t, values)

        policy_loss = -((logps * advantages) * valid_mask).sum() / valid_mask.sum()
        values_loss = (((values - G_t) ** 2) * valid_mask).sum() / valid_mask.sum()

        loss = policy_loss + values_loss
        require_finite((policy_loss, values_loss, loss), "loss")
        loss.backward()
        gradient_norm = torch.nn.utils.clip_grad_norm_(
            agent.parameters(), max_norm=cfg.train.grad_clip, error_if_nonfinite=True
        )
        optimizer.step()
        require_finite(agent.state_dict(), "updated model")
        require_finite(optimizer.state_dict(), "updated optimizer")
        # CPU metric transfers are part of accepting the update too.
        episode_returns = (rewards * valid_mask).sum(dim=1).cpu().numpy()
        return {
            "losses/policy": policy_loss.item(), "losses/value": values_loss.item(),
            "losses/total": loss.item(), "returns/mean": float(episode_returns.mean()),
            "returns/max": float(episode_returns.max()),
            "returns/min": float(episode_returns.min()),
            "returns/std": float(episode_returns.std()),
            "episodes/length/min": float(min(seq_lens)),
            "episodes/length/max": float(max(seq_lens)),
            "episodes/length/mean": float(seq_lens.mean()),
            "optimization/gradient_norm": float(gradient_norm),
            "optimization/learning_rate": float(optimizer.param_groups[0]["lr"]),
        }
    except BaseException as error:
        # Host-side recovery remains saveable even if a CUDA error prevents an
        # in-memory restore. Preserve the original exception, not a restore error.
        setattr(error, "training_state", before)
        try:
            restore_state(agent, optimizer, before)
        except Exception:
            LOG.exception("In-memory rollback failed; CPU recovery snapshot retained")
        raise


def write_checkpoint(cfg, state, name: str, score=None, *, upload: bool = True) -> Path:
    config = OmegaConf.to_container(cfg, resolve=True)
    assert isinstance(config, dict)
    path = save_checkpoint(Path(cfg.checkpoints.dir) / name, state, config, score)
    if upload and cfg.logging.mlflow.enabled:
        mlflow.log_artifact(str(path), artifact_path="checkpoints")
    return path


def make_validation_env(cfg, resources: ExitStack):
    validation_cfg = deepcopy(cfg)
    validation_cfg.env.n_parallel = cfg.guard.episodes
    env = register_and_make_env(validation_cfg)
    resources.callback(env.close)
    return SingleEnvWrapper(env) if isinstance(env, Env) else env


def evaluation_rollout(env, agent: Agent, seed: int) -> list[EpisodeBuffer]:
    """Use eval() without gradients and isolate all evaluation random streams."""
    was_training = agent.training
    try:
        agent.eval()
        with evaluation_rng(agent.device, seed), torch.no_grad():
            return rollout(env, agent, seed=seed)
    finally:
        agent.train(was_training)


def validate_policy(cfg, env, agent) -> dict[str, float]:
    """Fixed stochastic action/environment seeds, without affecting training RNG."""
    buffers = evaluation_rollout(env, agent, cfg.guard.seed)
    returns = np.array([sum(buffer.rewards_l) for buffer in buffers])
    if not np.isfinite(returns).all():
        raise FloatingPointError("Non-finite validation returns")
    return {
        "validation/returns/mean": float(returns.mean()),
        "validation/returns/min": float(returns.min()),
        "validation/returns/max": float(returns.max()),
        "validation/returns/std": float(returns.std()),
        "validation/episodes/length/mean": float(
            np.mean([b.seq_len for b in buffers])
        ),
    }


def check_policy_guard(cfg, env, agent, guard, next_iteration: int):
    metrics = validate_policy(cfg, env, agent)
    score = metrics["validation/returns/mean"]
    decision = guard.observe(score, next_iteration)
    if decision == "best":
        write_checkpoint(cfg, guard.best_state, "best.pt", score)
        write_checkpoint(cfg, guard.best_state, "latest.pt", score)
    if decision == "rollback":
        LOG.warning("Policy degradation: restored best return %.2f; LR now %.3g",
                    guard.best_score, guard.optimizer.param_groups[0]["lr"])
        # Persist the restored state and reduced optimizer LR immediately.
        write_checkpoint(
            cfg, capture_state(agent, guard.optimizer, next_iteration), "latest.pt"
        )
    metrics.update({
        "guard/best_return": guard.best_score,
        "guard/bad_evaluations": guard.bad_evaluations,
        "guard/rollbacks": guard.rollbacks,
        "guard/rolled_back": int(decision == "rollback"),
        "guard/learning_rate": guard.optimizer.param_groups[0]["lr"],
    })
    LOG.info("Validation at %d: %.2f; best %.2f (%s)",
             next_iteration, score, guard.best_score, decision)
    if cfg.logging.mlflow.enabled:
        mlflow.log_metrics(metrics, step=next_iteration)


def preserve_training_failure(
    cfg, agent, optimizer, next_iteration: int, report: str,
    *, state: dict | None = None,
):
    LOG.error("Training stopped; preserving the last finite state:\n%s", report)
    try:
        if state is None:
            state = capture_state(agent, optimizer, next_iteration)
        path = write_checkpoint(cfg, state, "latest.pt", upload=False)
        failure = path.parent / "training_failure.txt"
        failure.write_text(report, encoding="utf-8")
        if cfg.logging.mlflow.enabled:
            mlflow.log_artifact(str(path), artifact_path="checkpoints")
            mlflow.log_artifact(str(failure), artifact_path="checkpoints")
    except Exception:
        LOG.exception("Could not persist/upload failure artifacts")


def run_training_loop(
    cfg, env, video_env, validation_env, agent, optimizer, guard, start
):
    next_iteration = start
    try:
        write_checkpoint(cfg, capture_state(agent, optimizer, start), "latest.pt")
        if validation_env is not None:
            check_policy_guard(cfg, validation_env, agent, guard, start)
        while (
            cfg.train.max_iterations is None
            or next_iteration < cfg.train.max_iterations
        ):
            iteration = next_iteration
            metrics = training_update(cfg, env, agent, optimizer, iteration)
            next_iteration = iteration + 1
            LOG.info("Step %4d: p_l=%.4f, v_l=%.4f, ret=%.2f, mean_ep_len=%.0f",
                     iteration, metrics["losses/policy"], metrics["losses/value"],
                     metrics["returns/mean"], metrics["episodes/length/mean"])
            if (
                cfg.logging.mlflow.enabled
                and iteration % cfg.logging.mlflow.push_freq == 0
            ):
                mlflow.log_metrics(metrics, step=iteration)
            if validation_env is not None and next_iteration % cfg.guard.every == 0:
                check_policy_guard(cfg, validation_env, agent, guard, next_iteration)
            if next_iteration % cfg.checkpoints.every == 0:
                state = capture_state(agent, optimizer, next_iteration)
                write_checkpoint(cfg, state, f"iteration_{iteration:06d}.pt")
                write_checkpoint(cfg, state, "latest.pt")
            if video_env is not None and iteration % cfg.logging.plot_freq == 0:
                evaluate_and_plot(cfg, video_env, agent, iteration)
            if cfg.logging.mlflow.enabled and iteration % cfg.logging.save_freq == 0:
                mlflow.pytorch.log_model(
                    agent, name=f"agent_{iteration:04d}", step=iteration
                )
        write_checkpoint(
            cfg, capture_state(agent, optimizer, next_iteration), "latest.pt"
        )
    except BaseException as error:
        preserve_training_failure(
            cfg, agent, optimizer, next_iteration, traceback.format_exc(),
            state=getattr(error, "training_state", None),
        )
        raise


def make_video_env(cfg: DictConfig, resources: ExitStack) -> SingleEnvWrapper | None:
    if cfg.logging.plot_freq <= 0:
        return None
    env = register_and_make_env(cfg, force_single_env=True, force_non_random=True)
    assert isinstance(env, Env)
    resources.callback(env.close)
    video = RecordVideo(env, "video/", episode_trigger=lambda _: True)
    resources.callback(video.close)
    return SingleEnvWrapper(video)


def evaluate_and_plot(cfg: DictConfig, env: SingleEnvWrapper, agent: Agent, i: int):
    env.name_prefix = f"rp_iter_{i:04d}"
    buffers = evaluation_rollout(env, agent, cfg.seed + i + 10000)
    _, rewards, values, _, _ = ebufs2batchd(buffers, device=agent.device)
    env.stop_recording()
    video_path = Path(env.video_folder) / (
        f"{env.name_prefix}-episode-{env.episode_id}.mp4"
    )

    returns = get_returns(rewards, discount=cfg.rl.discount)
    advantages = get_advantages(returns, values)
    eps = Episode(
        buffers[0],
        value_estimates=values.cpu().numpy(),
        returns=returns.cpu().numpy(),
        advantages=advantages.cpu().numpy(),
    )

    plot_path = Path(f"plots/episode_iter_{i:04d}.png")
    trace_path = save_episode_csv(
        buffers[0],
        plot_path.with_suffix(".csv"),
        returns=returns[0].cpu().numpy(),
        advantages=advantages[0].cpu().numpy(),
    )
    fig = plot_episode(eps, keys=get_plot_keys(cfg, agent), save_path=plot_path)
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
    print(f"  Saved episode trace: {trace_path}")


def get_plot_keys(cfg: DictConfig, agent: Agent) -> list[PlotKey]:
    """Plot the selected tracking task, policy inputs, and active reward terms."""
    plot_keys: list[PlotKey] = list(PLOTKS)
    mode = cfg.env.tracking_mode
    if tracking_keys := TRACKING_INPUTS[mode]:
        plot_keys.append((Observable.OBS_TIME, tracking_keys))
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
        "none": {Observable.REWARD_POS, Observable.REWARD_VEL},
    }[mode]
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

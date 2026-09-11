import os
from collections.abc import Sequence
from contextlib import ExitStack
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
from riktigpatric.trajectory import PositionTrajectory
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
    (
        Observable.OBS_TIME,
        (Target.TARGET_POS, DerivedObs.CURRENT_POS),
    ),
]


def _take_idx_from_d(d, idx: int):
    return {k: v[idx].copy() for k, v in d.items()}


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
):
    """Set fixed targets or trajectories for a batch and refresh policy inputs."""
    if target_trajectories is not None:
        if target_positions is not None:
            raise ValueError("Choose target_positions or target_trajectories, not both")
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
    if target_positions is None:
        return obs_d
    targets = np.asarray(target_positions, dtype=np.float64)
    if (
        targets.shape != (rp_env.num_envs,)
        or not np.isfinite(targets).all()
        or np.any(np.abs(targets) > float(np.finfo(np.float32).max))
    ):
        raise ValueError(
            f"Expected {rp_env.num_envs} finite float32 position targets with shape "
            f"({rp_env.num_envs},), got {targets}"
        )
    targets = targets.astype(np.float32)
    # Gymnasium distributes lists/tuples; an ndarray would be broadcast whole.
    rp_env.set_attr("target_pos", targets.tolist())
    obs_d[Target.TARGET_POS] = targets[:, None].copy()
    return obs_d


def rollout(
    rp_env: SingleEnvWrapper | SyncVectorEnv,
    agent: Agent,
    seed: int = 0,
    target_positions: list[float] | np.ndarray | None = None,
    target_trajectories: Sequence[Sequence[Sequence[float]]] | None = None,
) -> list[EpisodeBuffer]:
    num_envs = rp_env.num_envs
    active = np.ones(num_envs, dtype=bool)
    episode_buffers = [EpisodeBuffer() for _ in range(num_envs)]

    h = None
    obs_d, _ = rp_env.reset(seed=seed)
    obs_d = add_targets(obs_d, rp_env, target_positions, target_trajectories)
    while active.any():
        obs_d_t = npd2tensord(obs_d)

        action, logp, value, h = agent.act(obs_d_t, h)
        action_np = tensord2npd(action)
        next_obs_d, reward, terminated, truncated, _ = rp_env.step(action_np)

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
    torch.manual_seed(cfg.seed)
    np.random.seed(cfg.seed)
    with ExitStack() as resources:
        train(cfg, resources)


def train(cfg: DictConfig, resources: ExitStack):
    # Setup MLflow if enabled
    if cfg.logging.mlflow.enabled:
        if (tracking_uri := os.getenv("MLFLOW_TRACKING_URI")) is None:
            raise ValueError("MLFLOW_TRACKING_URI is not set")
        mlflow.set_tracking_uri(tracking_uri)
        mlflow.set_experiment(cfg.logging.mlflow.experiment_name)
        resources.enter_context(mlflow.start_run())

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

    # Create agent
    if cfg.policy.restore_id is not None:
        print(f"Restoring agent from MLflow model ID: {cfg.policy.restore_id}")
        agent = mlflow.pytorch.load_model(
            mlflow.get_logged_model(cfg.policy.restore_id).model_uri, map_location="cpu"
        )
    else:
        agent = Agent(inputs=cfg.policy.inputs, actions=cfg.policy.actions)

    optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)

    i = 0
    while cfg.train.max_iterations is None or i < cfg.train.max_iterations:
        optimizer.zero_grad()

        episode_buf_l = rollout(
            rp_env,
            agent,
            seed=cfg.seed + i,
            target_positions=cfg.train.target_positions,
            target_trajectories=cfg.train.target_trajectories,
        )

        logps, rewards, values, seq_lens, valid_mask = ebufs2batchd(episode_buf_l)

        G_t = get_returns(rewards, discount=cfg.rl.discount)
        advantages = get_advantages(G_t, values)

        policy_loss = -((logps * advantages) * valid_mask).sum() / valid_mask.sum()
        values_loss = (((values - G_t) ** 2) * valid_mask).sum() / valid_mask.sum()

        loss = policy_loss + values_loss

        loss.backward()

        # Gradient clipping
        torch.nn.utils.clip_grad_norm_(agent.parameters(), max_norm=cfg.train.grad_clip)

        optimizer.step()

        episode_returns = (rewards * valid_mask).sum(dim=1).cpu().numpy()
        mean_return = episode_returns.mean()

        print(
            f"Step {i:4d}: p_l={policy_loss.item():.4f}, "
            f"v_l={values_loss.item():.4f}, l={loss.item():.4f}, "
            f"ret={mean_return:.2f}, mean_ep_len={seq_lens.mean():.0f}"
        )

        # Log metrics to MLflow
        if cfg.logging.mlflow.enabled and i % cfg.logging.mlflow.push_freq == 0:
            mlflow.log_metrics(
                {
                    "policy_loss": policy_loss.item(),
                    "value_loss": values_loss.item(),
                    "total_loss": loss.item(),
                    "mean_return": mean_return,
                    "min_episode_length": min(seq_lens),
                    "max_episode_length": max(seq_lens),
                    "mean_episode_length": seq_lens.mean(),
                    "max_return": episode_returns.max(),
                    "min_return": episode_returns.min(),
                    "std_return": episode_returns.std(),
                },
                step=i,
            )

        if rp_video_env is not None and i % cfg.logging.plot_freq == 0:
            evaluate_and_plot(cfg, rp_video_env, agent, i)

        if cfg.logging.mlflow.enabled and (i % cfg.logging.save_freq == 0):
            mlflow.pytorch.log_model(agent, name=f"agent_{i:04d}", step=i)
            print("  💾 Saved model")

        i += 1


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
    # Evaluation sampling must not change subsequent training action samples.
    with torch.random.fork_rng(devices=[]), torch.no_grad():
        torch.manual_seed(cfg.seed + i + 10000)
        env.name_prefix = f"rp_iter_{i:04d}"
        buffers = rollout(env, agent, seed=cfg.seed + i + 10000)
        _, rewards, values, _, _ = ebufs2batchd(buffers)
        env.stop_recording()

    returns = get_returns(rewards, discount=cfg.rl.discount)
    advantages = get_advantages(returns, values)
    eps = Episode(
        buffers[0],
        value_estimates=values.cpu().numpy(),
        returns=returns.cpu().numpy(),
        advantages=advantages.cpu().numpy(),
    )

    plot_keys: list[PlotKey] = list(PLOTKS)
    seen_observables = {
        Observable.OBS_TIME.value,
        *[key.value for _, keys in PLOTKS for key in keys],
    }
    for observable in agent.inputs:
        if observable not in seen_observables:
            plot_keys.append((Observable.OBS_TIME, (StateVar.from_str(observable),)))
    for action_name in agent.actions:
        plot_keys.append((Actions.TIME, (Actions.from_str(action_name),)))
    reward_keys = tuple(Observable.from_str(key) for key in cfg.reward)
    plot_keys.append((Observable.OBS_TIME, reward_keys))
    plot_keys.append(
        (Actions.TIME, (MiscKeys.VALUE_ESTIM, MiscKeys.RETURNS, MiscKeys.ADVANTAGES))
    )

    plot_path = Path(f"plots/episode_iter_{i:04d}.png")
    fig = plot_episode(eps, keys=plot_keys, save_path=plot_path)
    plt.close(fig)
    if cfg.logging.mlflow.enabled:
        mlflow.log_artifact(str(plot_path))
    print(f"  📊 Saved plot: {plot_path}")


if __name__ == "__main__":
    main()

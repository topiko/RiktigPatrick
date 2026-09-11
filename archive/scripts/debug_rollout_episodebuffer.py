#!/usr/bin/env python3
"""Debug rollout / EpisodeBuffer invariants without touching training code.

This script mirrors the current rollout logic in src/sim/train_agent.py and
prints when the invariants drift before EpisodeBuffer.finish raises.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from nn_ctrl.nns import Agent  # noqa: E402
from sim.train_agent import _take_idx_from_d  # noqa: E402
from sim.utils import (  # noqa: E402
    EpisodeBuffer,
    SingleEnvWrapper,
    npd2tensord,
    register_and_make_env,
    tensord2npd,
)


def _buf_stats(buf: EpisodeBuffer) -> str:
    return (
        f"obs={len(buf.obs_l)} act={len(buf.action_l)} rew={len(buf.rewards_l)} "
        f"logp={len(buf.logps_l)} val={len(buf.values_l)} finished={buf.finished}"
    )


def _zero_hidden_state(h: torch.Tensor | None, done: np.ndarray) -> torch.Tensor | None:
    if h is None or not done.any():
        return h
    done_t = torch.from_numpy(done.astype(bool)).to(device=h.device)
    return torch.where(done_t.view(1, -1, 1), torch.zeros_like(h), h)


def debug_rollout(seed: int, n_parallel: int, max_steps: int) -> int:
    np.random.seed(seed)
    torch.manual_seed(seed)

    cfg = OmegaConf.load(ROOT / "config" / "rlrp.yaml")
    if not isinstance(cfg, DictConfig):
        raise TypeError("Expected DictConfig from config/rlrp.yaml")
    cfg.env.n_parallel = n_parallel

    env = register_and_make_env(cfg)
    if n_parallel == 1:
        env = SingleEnvWrapper(env)

    agent = Agent(inputs=cfg.policy.inputs, actions=cfg.policy.actions)

    num_envs = int(getattr(env, "num_envs", 1))
    active = np.ones(num_envs, dtype=bool)
    buffers = [EpisodeBuffer() for _ in range(num_envs)]
    finish_count = np.zeros(num_envs, dtype=np.int64)

    h = None
    obs_d, _ = env.reset(seed=seed)

    for step in range(max_steps):
        if not active.any():
            print(f"all envs inactive at step={step}")
            break

        active_prev = active.copy()

        obs_d_t = npd2tensord(obs_d)
        action, logp, value, h = agent.act(obs_d_t, h)
        action_np = tensord2npd(action)
        next_obs_d, reward, terminated, truncated, _ = env.step(action_np)

        reward = np.ravel(reward)
        done = np.ravel(terminated | truncated)

        # Current training code update order.
        active = active & ~done

        skipped_step = np.flatnonzero(active_prev & done)
        if skipped_step.size > 0:
            print(
                f"step={step} done_while_active={skipped_step.tolist()} "
                f"(these envs are skipped by add_step in current logic)"
            )

        # add_step only for active AFTER update (mirrors current code)
        for env_idx in np.flatnonzero(active):
            env_idx = int(env_idx)
            buffers[env_idx].add_step(
                obs_t=_take_idx_from_d(obs_d, env_idx),
                action_t=_take_idx_from_d(action_np, env_idx),
                reward_t=reward[env_idx],
                logp_t=logp[env_idx, 0],
                value_t=value[env_idx, 0],
            )

        newly_done = done & ~active

        spurious_finish = np.flatnonzero(newly_done & ~active_prev)
        if spurious_finish.size > 0:
            print(
                f"step={step} spurious_newly_done={spurious_finish.tolist()} "
                f"(already inactive envs finishing again)"
            )

        for env_idx in np.flatnonzero(newly_done):
            env_idx = int(env_idx)
            finish_count[env_idx] += 1
            if finish_count[env_idx] > 1:
                print(f"step={step} env={env_idx} finish_count={finish_count[env_idx]}")

            try:
                buffers[env_idx].finish(
                    _take_idx_from_d(next_obs_d, env_idx),
                    reward=reward[env_idx],
                )
            except Exception as exc:  # noqa: BLE001
                print(f"\nfinish raised at step={step}, env={env_idx}: {exc}")
                print(f"  done={done.tolist()} active_prev={active_prev.tolist()}")
                print(f"  buffer_stats: {_buf_stats(buffers[env_idx])}")
                print(f"  finish_count={finish_count.tolist()}")
                return 1

        if done.any():
            next_obs_d, _ = env.reset(options={"reset_mask": done})

        h = _zero_hidden_state(h, done)
        obs_d = next_obs_d

    print("\nNo finish exception seen within max_steps")
    print(f"finish_count={finish_count.tolist()}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--n-parallel", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=400)
    args = parser.parse_args()

    raise SystemExit(debug_rollout(args.seed, args.n_parallel, args.max_steps))


if __name__ == "__main__":
    main()

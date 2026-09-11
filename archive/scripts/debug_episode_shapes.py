#!/usr/bin/env python3
"""Validate EpisodeBuffer/Episode shape invariants across rollout seeds."""

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
from sim.train_agent import rollout  # noqa: E402
from sim.utils import (  # noqa: E402
    Episode,
    MiscKeys,
    SingleEnvWrapper,
    ebufs2batchd,
    get_advantages,
    get_returns,
    register_and_make_env,
)


def _check_ebuf_invariants(buf, seed: int, env_idx: int) -> None:
    if not buf.finished:
        raise AssertionError(f"seed={seed} env={env_idx} buffer not finished")

    seq_len = buf.seq_len
    if len(buf.rewards_l) != seq_len:
        raise AssertionError(
            f"seed={seed} env={env_idx} rewards len={len(buf.rewards_l)} seq_len={seq_len}"
        )
    if len(buf.logps_l) != seq_len:
        raise AssertionError(
            f"seed={seed} env={env_idx} logps len={len(buf.logps_l)} seq_len={seq_len}"
        )
    if len(buf.values_l) != seq_len:
        raise AssertionError(
            f"seed={seed} env={env_idx} values len={len(buf.values_l)} seq_len={seq_len}"
        )

    for key in buf.obs_l[0].keys():
        arr = buf.get_observable(key)
        if arr.shape[0] != seq_len + 1:
            raise AssertionError(
                f"seed={seed} env={env_idx} obs={key} len={arr.shape[0]} expected={seq_len + 1}"
            )

    for key in buf.action_l[0].keys():
        arr = buf.get_action(key)
        if arr.shape[0] != seq_len:
            raise AssertionError(
                f"seed={seed} env={env_idx} act={key} len={arr.shape[0]} expected={seq_len}"
            )


def _check_episode_shapes(episode, seq_len: int, seed: int, env_idx: int) -> None:
    for attr in dir(episode):
        if attr.startswith("OBS_"):
            arr = getattr(episode, attr)
            if arr.shape[0] != seq_len + 1:
                raise AssertionError(
                    f"seed={seed} env={env_idx} {attr} len={arr.shape[0]} expected={seq_len + 1}"
                )
        elif attr.startswith("ACT_"):
            arr = getattr(episode, attr)
            if arr.shape[0] != seq_len:
                raise AssertionError(
                    f"seed={seed} env={env_idx} {attr} len={arr.shape[0]} expected={seq_len}"
                )
        elif attr.startswith("MISC_"):
            arr = getattr(episode, attr)
            if arr.shape[0] != seq_len:
                raise AssertionError(
                    f"seed={seed} env={env_idx} {attr} len={arr.shape[0]} expected={seq_len}"
                )


def run_checks(cfg: DictConfig, seeds: int, single: bool) -> None:
    env = register_and_make_env(cfg, force_single_env=single)
    if single:
        env = SingleEnvWrapper(env)

    agent = Agent(inputs=cfg.policy.inputs, actions=cfg.policy.actions)

    for seed in range(seeds):
        torch.manual_seed(seed)
        np.random.seed(seed)

        ebuf_l = rollout(env, agent, seed=seed)
        logps, rewards, values, seq_lens, _ = ebufs2batchd(ebuf_l)

        returns = get_returns(rewards, discount=cfg.rl.discount)
        advantages = get_advantages(returns, values)

        for env_idx, buf in enumerate(ebuf_l):
            _check_ebuf_invariants(buf, seed=seed, env_idx=env_idx)

            seq_len = int(seq_lens[env_idx])
            eps = Episode(
                buf,
                value_estimates=values[env_idx : env_idx + 1, :seq_len]
                .detach()
                .cpu()
                .numpy(),
                returns=returns[env_idx : env_idx + 1, :seq_len].detach().cpu().numpy(),
                advantages=advantages[env_idx : env_idx + 1, :seq_len]
                .detach()
                .cpu()
                .numpy(),
            )
            _check_episode_shapes(eps, seq_len=seq_len, seed=seed, env_idx=env_idx)

    env.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=100)
    parser.add_argument("--n-parallel", type=int, default=3)
    args = parser.parse_args()

    cfg = OmegaConf.load(ROOT / "config" / "rlrp.yaml")
    if not isinstance(cfg, DictConfig):
        raise TypeError("Expected DictConfig")
    cfg.env.n_parallel = args.n_parallel

    run_checks(cfg, seeds=args.seeds, single=False)
    run_checks(cfg, seeds=max(20, args.seeds // 2), single=True)
    print("EpisodeBuffer and Episode shapes look consistent")


if __name__ == "__main__":
    main()

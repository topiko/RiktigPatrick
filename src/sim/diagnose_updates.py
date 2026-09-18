"""Compare independent-batch and averaged updates from one frozen checkpoint.

Run with ``python -m sim.diagnose_updates --help``. Writes only a JSON report;
never replaces the source checkpoint or submits metrics to its MLflow run.
"""

import argparse
import json
import random
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path

import numpy as np
import torch
from gymnasium import Env
from omegaconf import OmegaConf

from sim.checkpoints import (
    capture_state,
    load_checkpoint,
    require_finite,
    restore_state,
)
from sim.curriculum import Curriculum
from sim.devices import seed_torch
from sim.train_agent import (
    make_agent,
    make_validation_env,
    training_rollout,
    validate_policy,
)
from sim.utils import (
    SingleEnvWrapper,
    actor_critic_losses,
    ebufs2batchd,
    ebufs2step_times,
    get_returns,
    register_and_make_env,
)


def mean_gradients(batches):
    """Equal-weight batch mean; preserve unused parameters' absent Adam gradients."""
    result = []
    for column in zip(*batches, strict=True):
        if all(g is None for g in column):
            result.append(None)
        elif any(g is None for g in column):
            raise ValueError("Gradient batches disagree on active parameters")
        else:
            result.append(torch.stack([g for g in column if g is not None]).mean(dim=0))
    return result


def _vector(grads):
    return torch.cat([g.flatten() for g in grads if g is not None])


def collect_gradients(cfg, env, agent, curriculum, seed, loss):
    seed_torch(seed, agent.device)
    np.random.seed(seed)
    random.seed(seed)
    agent.zero_grad(set_to_none=True)
    buffers = training_rollout(cfg, env, agent, seed, curriculum)
    logps, rewards, values, _, mask = ebufs2batchd(buffers, device=agent.device)
    returns = get_returns(
        rewards, cfg.rl.discount,
        step_times=ebufs2step_times(buffers, device=agent.device),
        reference_step_time=cfg.env.step_time,
    )
    actor, critic = actor_critic_losses(logps, returns, values, mask)
    named = list(agent.named_parameters())
    parameters = [p for _, p in named]
    actor_grads = torch.autograd.grad(
        actor, parameters, retain_graph=True, allow_unused=True,
    )
    critic_grads = torch.autograd.grad(
        cfg.rl.value_loss_coef * critic, parameters, allow_unused=True,
    )
    shared = [i for i, (name, _) in enumerate(named)
              if not name.startswith(("value_head.", "action_heads."))]
    # Both losses use the same shared parameters; align missing gradients with zero.
    a, c = [torch.cat([
        (torch.zeros_like(parameters[i]) if grads[i] is None else grads[i]).flatten()
        for i in shared
    ]) for grads in (actor_grads, critic_grads)]
    selected = []
    for ga, gc in zip(actor_grads, critic_grads):
        terms = [g for g in (ga, gc if loss == "joint" else None) if g is not None]
        selected.append(torch.stack(terms).sum(dim=0).detach().cpu() if terms else None)
    require_finite(selected, "diagnostic gradients")
    return selected, {
        "seed": seed, "return_mean": float(rewards.sum(dim=1).mean()),
        "actor_loss": float(actor.detach()), "critic_mse": float(critic.detach()),
        "actor_shared_norm": float(a.norm()),
        "weighted_critic_shared_norm": float(c.norm()),
        "actor_critic_shared_cosine": float(
            torch.nn.functional.cosine_similarity(a, c, dim=0)
        ),
        "gradient_norm": float(_vector(selected).norm()),
    }


def evaluate_suites(cfg, env, agent, curriculum, seeds):
    evaluation_cfg = deepcopy(cfg)
    evaluation_cfg.guard.compare_deterministic = False
    suites = {}
    for seed in seeds:
        evaluation_cfg.guard.seed = seed
        metrics = validate_policy(evaluation_cfg, env, agent, curriculum)
        suites[str(seed)] = metrics
    return suites


def apply_gradients(agent, optimizer, gradient, max_norm, *, fresh_adam=False):
    if fresh_adam:
        optimizer.state.clear()
    for parameter, g in zip(agent.parameters(), gradient, strict=True):
        parameter.grad = None if g is None else g.to(agent.device).clone()
    norm = torch.nn.utils.clip_grad_norm_(
        agent.parameters(), max_norm, error_if_nonfinite=True,
    )
    optimizer.step()
    require_finite(agent.state_dict(), "diagnostic candidate")
    return float(norm)


def compare_updates(
    cfg, checkpoint, *, batches=4, seed=20000, learning_rate=None, loss="actor",
    fresh_adam=False, hold_velocity_weight=None, discount=None, validation_seeds=None,
):
    """Restore the same model/Adam before every batch and every candidate update.

    Reward/discount overrides affect only gradient collection. Candidate evaluation
    always uses the original checkpoint objective, including its speed penalty.
    """
    if type(batches) is not int or batches < 2 or loss not in ("actor", "joint"):
        raise ValueError("Use at least two batches and loss=actor or joint")
    training_cfg = deepcopy(cfg)
    training_cfg.policy.restore_id = None
    if hold_velocity_weight is not None:
        if not training_cfg.curriculum.enabled:
            raise ValueError("hold_velocity_weight requires the curriculum")
        training_cfg.curriculum.hold_velocity_weight = hold_velocity_weight
    if discount is not None:
        training_cfg.rl.discount = discount
    curriculum = Curriculum(training_cfg) if training_cfg.curriculum.enabled else None
    agent = make_agent(training_cfg)
    optimizer = torch.optim.Adam(agent.parameters(), lr=training_cfg.train.policy_lr)
    iteration = load_checkpoint(checkpoint, agent, optimizer, curriculum,
                                learning_rate=learning_rate)
    if hold_velocity_weight is not None and (
        curriculum is None or curriculum.stage != "hold_position"
    ):
        raise ValueError("hold_velocity_weight applies only to hold_position")
    agent.train()
    before = capture_state(agent, optimizer, iteration, curriculum)
    evaluation_curriculum = Curriculum(cfg) if curriculum is not None else None
    if evaluation_curriculum is not None:
        evaluation_curriculum.load_state_dict(before["curriculum"])
    seeds = validation_seeds or [cfg.guard.seed, cfg.guard.seed + 100000]
    report = {
        "checkpoint": str(checkpoint), "iteration": iteration,
        "loss": loss, "fresh_adam": fresh_adam,
        "learning_rate": optimizer.param_groups[0]["lr"],
        "training_discount": training_cfg.rl.discount,
        "training_hold_velocity_weight": (
            training_cfg.curriculum.hold_velocity_weight if curriculum else None
        ),
        "validation_discount": cfg.rl.discount,
        "validation_hold_velocity_weight": (
            cfg.curriculum.hold_velocity_weight if curriculum else None
        ),
        "batch_episodes": cfg.env.n_parallel, "validation_episodes": cfg.guard.episodes,
    }
    with ExitStack() as resources:
        env = register_and_make_env(training_cfg)
        resources.callback(env.close)
        if isinstance(env, Env):
            env = SingleEnvWrapper(env)
        validation_env = make_validation_env(cfg, resources, evaluation_curriculum)
        report["baseline"] = evaluate_suites(
            cfg, validation_env, agent, evaluation_curriculum, seeds,
        )
        gradients, batch_metrics = [], []
        for index in range(batches):
            restore_state(agent, optimizer, before, curriculum=curriculum)
            gradient, metrics = collect_gradients(
                training_cfg, env, agent, curriculum,
                seed + index * env.num_envs, loss,
            )
            gradients.append(gradient)
            batch_metrics.append(metrics)
            print(f"Collected batch {index + 1}/{batches}", flush=True)
        report["batches"] = batch_metrics
        vectors = torch.stack([_vector(g) for g in gradients])
        unit = torch.nn.functional.normalize(vectors, dim=1)
        report["gradient_cosines"] = (unit @ unit.T).tolist()
        candidates = {}
        for label, gradient in [
            *[(f"batch_{i}", g) for i, g in enumerate(gradients)],
            ("mean", mean_gradients(gradients)),
        ]:
            restore_state(agent, optimizer, before, curriculum=curriculum)
            norm = apply_gradients(agent, optimizer, gradient, cfg.train.grad_clip,
                                   fresh_adam=fresh_adam)
            suites = evaluate_suites(
                cfg, validation_env, agent, evaluation_curriculum, seeds,
            )
            candidates[label] = {"gradient_norm": float(norm), "validation": suites}
            scores = [round(m["validation/returns/mean"], 2) for m in suites.values()]
            print(f"{label}: validation returns {scores}", flush=True)
        report["candidates"] = candidates
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--batches", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20000)
    parser.add_argument("--learning-rate", type=float)
    parser.add_argument("--loss", choices=("actor", "joint"), default="actor")
    parser.add_argument("--fresh-adam", action="store_true")
    parser.add_argument("--hold-velocity-weight", type=float)
    parser.add_argument("--discount", type=float)
    parser.add_argument("--validation-seeds", type=int, nargs="+")
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    if args.output.resolve() == args.checkpoint.resolve():
        parser.error("Output must not overwrite the checkpoint")
    torch.set_num_threads(1)
    payload = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    cfg = OmegaConf.create(payload["config"])
    cfg.train.device = args.device
    report = compare_updates(
        cfg, args.checkpoint, batches=args.batches, seed=args.seed,
        learning_rate=args.learning_rate, loss=args.loss, fresh_adam=args.fresh_adam,
        hold_velocity_weight=args.hold_velocity_weight, discount=args.discount,
        validation_seeds=args.validation_seeds,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"Saved diagnostics: {args.output}")


if __name__ == "__main__":
    main()

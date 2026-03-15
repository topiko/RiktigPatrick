from __future__ import annotations

import logging
import logging.config
import os

from datetime import datetime
from dotenv import load_dotenv

import gymnasium as gym
import mlflow
import numpy as np
import torch
import yaml
from gymnasium import ActionWrapper
import mlflow
import numpy as np
import torch
from gymnasium import ActionWrapper
from gymnasium.envs.registration import register
from gymnasium.wrappers import (
    RecordEpisodeStatistics,
    RecordVideo,
    TransformObservation,
)
from mlflow import MlflowClient

from sim.algos import REINFORCE
from sim.envs.rp_env import MAXA, MAXV
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.utils import Tape, register_and_make_env

load_dotenv()  # Loads .env from project root

log_config = {
    "version": 1,
    "disable_existing_loggers": True,
    "formatters": {
        "standard": {
            "format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
            "datefmt": "%Y-%m-%d %H:%M:%S",
        },
    },
    "handlers": {
        "default": {
            "level": "INFO",
            "formatter": "standard",
            "class": "logging.StreamHandler",
            "stream": "ext://sys.stdout",  # Default is stderr
        },
    },
    "loggers": {
        "": {  # root logger
            "handlers": ["default"],
            "level": "INFO",
            "propagate": False,
        }
    },
}
logging.config.dictConfig(log_config)
log = logging.getLogger(__name__)


if __name__ == "__main__":
    USE_BASELINE = False  # True
    BATCH_SIZE = 64

    rpenv_p = register_and_make_env(ENV_CONFIG, OBS_SPACE, True, BATCH_SIZE)

    rpenv_p.reset()

    indim = sum(
        v.shape[0]
        for k, v in rpenv_p.single_observation_space.items()
        if k in MODEL_INPUT
    )
    actiondim = sum(v.shape[0] for v in rpenv_p.single_action_space.values())

    agent = REINFORCE(
        indim,
        actiondim,
        MODEL_INPUT,
        use_baseline=USE_BASELINE,
        init2zeros=True,
        load_net=False,
    )

    RUN_NAME = "rp_test_parallel"
    mlflow.set_tracking_uri("https://ml.twohands.dev")
    mlflow.set_experiment(RUN_NAME)
    client = MlflowClient()

    MAX_RETURN = 0
    seed = 42
    max_steps = 2000

    with mlflow.start_run():
        # Load and log config parameters
        with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "sim/config.yaml"), "r") as f:
            config = yaml.safe_load(f)

        # Flatten config for mlflow params
        flat_params = {}
        for section, params in config.items():
            if isinstance(params, dict):
                for key, value in params.items():
                    flat_params[f"{section}_{key}"] = value
        mlflow.log_params(flat_params)

        for episode in range(200001):
            obs, _ = rpenv_p.reset(seed=seed)

            tapes = [Tape(i) for i in range(BATCH_SIZE)]
            full_tapes = []
            ready_ = np.zeros(BATCH_SIZE, dtype=bool)
            step_count = 0
            while True:
                result = agent.sample_action(obs)
                if len(result) == 4:
                    actions, probs, values, entropies = result
                else:
                    actions, probs, values = result
                    entropies = torch.zeros_like(probs)

                obs, rewards, terminated, truncated, _ = rpenv_p.step(actions)

                for i, t in enumerate(tapes):
                    t.rewards.append(rewards[i])
                    t.probs.append(probs[i])
                    t.values.append(values[i])
                    t.entropies.append(entropies[i])
                    if terminated[i] or truncated[i]:
                        full_tapes.append(t.build())
                        ready_[i] = True

                        tapes[i] = Tape(i)

                step_count += 1
                if all(ready_) or step_count >= max_steps:
                    break

            assert (
                len(full_tapes) >= BATCH_SIZE
            ), f"len(full_tapes) = {len(full_tapes)}, {len(ready_)}"
            rets, val_loss = agent.update(full_tapes)

            # Record stats:
            mean_return = rets.mean()
            std_return = rets.std()
            mlflow.log_metrics(
                {
                    "mean_ret": mean_return,
                    "std_ret": std_return,
                    "max_ret": rets.max(),
                    "min_ret": rets.min(),
                    "value_loss": val_loss,
                },
                step=episode,
            )
            if mean_return > (MAX_RETURN + 1):
                if episode >= 0:
                    log.info(f"Best return {mean_return:.02f} -> saving")
                    agent.net.store()
                MAX_RETURN = mean_return

            log.info(
                f"Episode {episode:<6d} (bs={len(full_tapes):4d}) --> {mean_return:6.02f} \u00B1 {std_return:5.02f}, min={min(rets):6.02f} max={max(rets):6.02f}, V_loss={val_loss:.02f}"
            )

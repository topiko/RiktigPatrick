from __future__ import annotations

import gymnasium as gym
import mlflow
import numpy as np
import torch
from gymnasium.envs.registration import register
from gymnasium.wrappers import RecordEpisodeStatistics, RecordVideo
from mlflow.client import MlflowClient
from riktigpatric.patrick import StepAction
from torch import nn
from torch.distributions.normal import Normal

from sim.algos import REINFORCE
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.try_policy import run_episode
from sim.utils import Tape, actiondim, model_indim, register_and_make_env


def record(ep_id) -> bool:
    return ep_id % 10000 == 0


if __name__ == "__main__":
    NROLLOUTS = 1

    rpenv = register_and_make_env(ENV_CONFIG, OBS_SPACE, vector_env=False)

    rpenv = RecordVideo(rpenv, "./video", episode_trigger=record, name_prefix="rp")
    rpenv = RecordEpisodeStatistics(rpenv, deque_size=NROLLOUTS)

    rpenv.reset()

    indim = model_indim(rpenv, MODEL_INPUT)
    actdim = actiondim(rpenv)
    agent = REINFORCE(indim, actdim, MODEL_INPUT)

    RUN_NAME = "rp_test"
    client = MlflowClient()
    try:
        experiment_id = client.create_experiment(RUN_NAME)
    except:
        experiment_id = client.get_experiment_by_name(RUN_NAME).experiment_id

    MAX_RETURN = 0
    with mlflow.start_run(experiment_id=experiment_id, run_name=RUN_NAME):
        for episode in range(10001):
            tapes = [Tape(i) for i in range(NROLLOUTS)]
            tapes = run_episode(agent, rpenv, nrollouts=NROLLOUTS, tapes=tapes)

            rets, val_loss = agent.update(tapes)
            if episode % 100 == 0:
                mlflow.log_metrics(
                    {
                        f"episode_dur_{i:02d}": v[0]
                        for i, v in enumerate(rpenv.length_queue)
                    },
                    step=episode,
                )
                mlflow.log_metrics(
                    {
                        f"episode_ret_{i:02d}": v[0]
                        for i, v in enumerate(rpenv.return_queue)
                    },
                    step=episode,
                )
                if (mean_return := np.array(rets).mean()) > (MAX_RETURN + 5):
                    print(f"Best return {mean_return:.02f} -> saving")
                    agent.net.store()
                    MAX_RETURN = mean_return
                print(
                    f"Episode {episode:<6d} ({NROLLOUTS} rollouts) --> {mean_return:.3f}"
                )

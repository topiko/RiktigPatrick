from __future__ import annotations

import sys
print("Starting...", flush=True)

import gymnasium as gym
import numpy as np
import torch
from gymnasium.envs.registration import register
from gymnasium.wrappers import RecordEpisodeStatistics

from sim.algos import REINFORCE
print("Imported algos", flush=True)

from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.try_policy import run_episode
from sim.utils import Tape, actiondim, model_indim, register_and_make_env

print("Making env...", flush=True)

if __name__ == "__main__":
    NROLLOUTS = 16

    rpenv = register_and_make_env(ENV_CONFIG, OBS_SPACE, vector_env=False)
    print("Env created", flush=True)
    rpenv = RecordEpisodeStatistics(rpenv, buffer_length=1000)
    print("Stats wrapper added", flush=True)

    rpenv.reset()
    print("Env reset", flush=True)

    indim = model_indim(rpenv, MODEL_INPUT)
    actdim = actiondim(rpenv)
    print(f"indim={indim}, actdim={actdim}", flush=True)
    
    agent = REINFORCE(
        indim, actdim, MODEL_INPUT, use_baseline=True, init2zeros=True, load_net=False
    )
    print("Agent created", flush=True)

    MAX_RETURN = 0
    for episode in range(20001):
        tapes = [Tape(i) for i in range(NROLLOUTS)]
        tapes = run_episode(agent, rpenv, nrollouts=NROLLOUTS, tapes=tapes)

        rets, val_loss = agent.update(tapes)
        if episode % 10 == 0:
            mean_ret = float(np.mean(rets))
            if mean_ret > (MAX_RETURN + 5):
                print(f"Best return {mean_ret:.02f} -> saving", flush=True)
                agent.net.store()
                agent.value_net.store()
                MAX_RETURN = mean_ret
            print(f"Episode {episode:<6d} ({NROLLOUTS} rollouts) --> {mean_ret:.3f}", flush=True)

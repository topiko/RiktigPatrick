from __future__ import annotations

import logging
from typing import Optional

import gymnasium as gym
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
from mlflow.client import MlflowClient
from riktigpatric.patrick import StepAction
from torch import nn
from torch.distributions.normal import Normal

from sim.envs.rp_env import MAXV

config = {
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
logging.config.dictConfig(config)
log = logging.getLogger(__name__)


def dict2tensor(obs_d: dict[str, np.ndarray]) -> torch.Tensor:
    ndim = list(obs_d.values())[0].ndim
    if ndim == 1:
        obs_arr = np.concatenate(list(obs_d.values()), axis=0)
    elif ndim == 2:
        obs_arr = np.concatenate(list(obs_d.values()), axis=1)
    else:
        raise TypeError()

    return torch.Tensor(obs_arr)


class ValueNet(nn.Module):
    NETF = "nets/val_net.pth"

    def __init__(self, obs_space_dim: int):
        super().__init__()

        h1 = 32
        h2 = 32

        self.net = nn.Sequential(
            nn.Linear(obs_space_dim, h1),
            nn.LeakyReLU(),
            nn.Linear(h1, h2),
            nn.LeakyReLU(),
            nn.Linear(h2, 1),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)

    def store(self, fname: str = NETF):
        torch.save(self, fname)

    def load(self, fname: str = NETF) -> ValueNet:
        try:
            return torch.load(fname)
        except FileNotFoundError:
            log.warning("Failed to load value network.")
            return self


class Policy_Network(nn.Module):
    """Parametrized Policy Network."""

    NETF = "nets/rpnet_p.pth"

    def __init__(self, obs_space_dims: int, action_space_dims: int):
        """Initializes a neural network that estimates the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """
        super().__init__()

        hidden_space1 = 512  # 32  # Nothing special with 16, feel free to change
        hidden_space2 = 512  # 32  # Nothing special with 32, feel free to change
        hidden_space3 = 512  # 32  # Nothing special with 32, feel free to change
        hidden_space4 = 512  # 32  # Nothing special with 32, feel free to change

        # Shared Network
        self.shared_net = nn.Sequential(
            nn.Linear(obs_space_dims, hidden_space1),
            nn.LeakyReLU(),
            nn.Linear(hidden_space1, hidden_space2),
            nn.LeakyReLU(),
            nn.Linear(hidden_space2, hidden_space3),
            nn.LeakyReLU(),
            nn.Linear(hidden_space3, hidden_space4),
            nn.LeakyReLU(),
        )

        # Policy Mean specific Linear Layer
        self.policy_mean_net = nn.Sequential(
            nn.Linear(hidden_space2, hidden_space2),
            nn.LeakyReLU(),
            nn.Linear(hidden_space2, action_space_dims),
            nn.Sigmoid(),
        )

        # Policy Std Dev specific Linear Layer
        self.policy_stddev_net = nn.Sequential(
            nn.Linear(hidden_space2, action_space_dims),
        )

    def forward(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Conditioned on the observation, returns the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            x: Observation from the environment

        Returns:
            action_means: predicted mean of the normal distribution
            action_stddevs: predicted standard deviation of the normal distribution
        """
        shared_features = self.shared_net(x.float())

        action_means = (self.policy_mean_net(shared_features) - 0.5) * 2 * MAXV
        action_stddevs = torch.log(
            1 + torch.exp(self.policy_stddev_net(shared_features))
        )

        return action_means, action_stddevs

    def store(self, fname: str = NETF):
        torch.save(self, fname)

    def load(self, fname: str = NETF) -> Policy_Network:
        try:
            return torch.load(fname)
        except FileNotFoundError:
            log.warning("Failed to load policy network.")
            return self


class REINFORCE:
    """REINFORCE algorithm."""

    def __init__(
        self, obs_space_dims: int, action_space_dims: int, use_baseline: bool = False
    ):
        """Initializes an agent that learns a policy via REINFORCE algorithm [1]
        to solve the task at hand (Inverted Pendulum v4).

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """

        # Hyperparameters
        self.learning_rate = 1e-5  # orig = 1e-4 Learning rate for policy optimization
        self.gamma = 0.9  # Discount factor
        self.eps = 1e-6  # small number for mathematical stability

        self.net = Policy_Network(obs_space_dims, action_space_dims).load()
        self.polizy_optimizer = torch.optim.SGD(
            self.net.parameters(), lr=self.learning_rate, momentum=0.1
        )

        self.value_net = ValueNet(obs_space_dims).load()
        self.value_optimizer = torch.optim.AdamW(self.value_net.parameters(), lr=1e-3)
        self.value_loss = torch.nn.MSELoss(reduction="sum")

        self.use_baseline = use_baseline

    def sample_action(
        self, obs: torch.Tensor
    ) -> tuple[dict[str, np.ndarray], torch.Tensor, torch.Tensor]:
        """Returns an action, conditioned on the policy and observation.

        Args:
            state: Observation from the environment

        Returns:
            action: Action to be performed
        """

        action_means, action_stddevs = self.net(obs)

        # create a normal distribution from the predicted
        # mean and standard deviation and sample an action
        distrib = Normal(action_means, action_stddevs + self.eps)

        action = distrib.sample()
        probs = distrib.log_prob(action)

        action = action.numpy()

        value = self.value_net(obs)

        return StepAction().from_array(action).to_dict(), probs, value

    def _step_value(self, tapes: list[Tape]) -> float:
        value_loss = 0
        l = 0
        for t in tapes:
            true_val = rews2returns(t.rewards, 1)
            value_loss += self.value_loss(t.values, torch.Tensor(true_val))
            l += len(t)

        value_loss /= l
        self.value_optimizer.zero_grad()
        value_loss.backward()
        self.value_optimizer.step()

        return float(value_loss.detach().numpy())

    def update(self, tapes: list[Tape]) -> tuple[np.ndarray, float]:
        """Updates the policy network's weights."""
        losses = torch.zeros(len(tapes))

        # TODO: should rewards be normalized somehow?
        # TODO: Yes! implement the -b thing from here -> https://mcneela.github.io/machine_learning/2019/06/03/The-Problem-With-Policy-Gradient.html

        rets = []

        for i, tape in enumerate(tapes):
            G = rews2returns(tape.rewards, 1)

            deltas = G

            if self.use_baseline:
                # Values are constants here. You do not want to backward over them!
                deltas -= tape.values.detach().numpy()

            rets.append(tape.ep_return)

            # minimize -1 * prob * reward obtained
            for log_prob, delta in zip(tape.probs, deltas):
                losses[i] -= log_prob.sum() * delta

        val_loss = self._step_value(tapes)

        # print(G)
        # print(values)
        # print()
        loss = torch.mean(losses)
        # print(losses[-1], G[0])

        # Update the policy network
        self.polizy_optimizer.zero_grad()
        loss.backward()
        self.polizy_optimizer.step()

        return np.array(rets), val_loss


def rews2returns(
    rewards: np.ndarray,
    discount: Optional[float] = 1,
) -> torch.Tensor:
    values = np.zeros(len(rewards))
    i = -1
    for R in rewards[::-1]:
        values[i] = R + values[i + 1] * discount
        i -= 1
    return values


class Tape:
    def __init__(self, idx: int):
        self.idx = idx
        self.probs = []
        self.values = []
        self.rewards = []

    @property
    def ep_return(self) -> float:
        return sum(self.rewards)

    def build(self) -> Tape:
        self._is_ready = True
        self.probs = torch.concat(self.probs, axis=0)
        self.rewards = np.array(self.rewards)
        self.values = torch.concat(self.values, axis=0)
        return self

    def __len__(self) -> int:
        return len(self.probs)


OBS_SPACE = [
    "filter/rp_pitch",
    "sens/gyro",
    # "sens/head_pitch",
    # "sens/head_turn",
    "sens/left_wheel_vel",
    "sens/right_wheel_vel",
]
CTRL_MODE = "vel"
ENV_CONFIG = {"ctrl_mode": CTRL_MODE, "lock_head": True, "step_time": 0.01}
USE_BASELINE = True

if __name__ == "__main__":
    BATCH_SIZE = 16

    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
        kwargs=ENV_CONFIG,
    )

    rpenv_p = gym.vector.AsyncVectorEnv(
        [
            lambda: gym.make(
                "RiktigPatrick-v0",
                state_keys=OBS_SPACE,
                render_mode="rgb_array",
                disable_env_checker=True,
            )
            for _ in range(BATCH_SIZE)
        ]
    )

    rpenv_p = TransformObservation(rpenv_p, dict2tensor)

    rpenv_p.reset()

    indim = sum(v.shape[0] for v in rpenv_p.single_observation_space.values())
    actiondim = sum(v.shape[0] for v in rpenv_p.single_action_space.values())

    agent = REINFORCE(indim, actiondim, use_baseline=USE_BASELINE)  # StepAction().ndim)

    RUN_NAME = "rp_test_parallel"
    client = MlflowClient()
    try:
        experiment_id = client.get_experiment_by_name(RUN_NAME).experiment_id
    except:
        experiment_id = client.create_experiment(RUN_NAME)

    MAX_RETURN = 0
    seed = 42
    with mlflow.start_run(experiment_id=experiment_id, run_name=RUN_NAME):
        for episode in range(200001):
            obs, _ = rpenv_p.reset(seed=seed)
            tapes = [Tape(i) for i in range(BATCH_SIZE)]
            full_tapes = []
            ready_ = np.zeros(BATCH_SIZE, dtype=bool)
            while True:
                actions, probs, values = agent.sample_action(obs)

                obs, rewards, terminated, truncated, _ = rpenv_p.step(actions)

                for i, t in enumerate(tapes):
                    t.rewards.append(rewards[i])
                    t.probs.append(probs[i])
                    t.values.append(values[i])
                    if terminated[i]:
                        full_tapes.append(t.build())
                        ready_[i] = True

                        # New tape to tapes list
                        tapes[i] = Tape(i)

                if all(ready_):
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
                    agent.value_net.store()
                MAX_RETURN = mean_return

            log.info(
                f"Episode {episode:<6d} (bs={len(full_tapes)}) --> {mean_return:6.02f} \u00B1 {std_return:5.02f}, min={min(rets):6.02f} max={max(rets):6.02f}, V_loss={val_loss:.02f}"
            )

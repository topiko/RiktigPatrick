from __future__ import annotations
import mlflow
import torch
from torch import nn
from torch.distributions.normal import Normal

import numpy as np

import gymnasium as gym
from gymnasium.envs.registration import register
from gymnasium.wrappers import RecordVideo
from gymnasium.wrappers import RecordEpisodeStatistics
from gymnasium.wrappers import TransformObservation
from gymnasium import ActionWrapper

from mlflow.client import MlflowClient

from riktigpatric.patrick import StepAction


def dict2tensor(obs_d: dict[str, np.ndarray]) -> torch.Tensor:
    obs_arr = np.concatenate(list(obs_d.values()), axis=1)
    return torch.Tensor(obs_arr)


class Policy_Network(nn.Module):
    """Parametrized Policy Network."""

    NETF = "rpnet_p.pth"

    def __init__(self, obs_space_dims: int, action_space_dims: int):
        """Initializes a neural network that estimates the mean and standard deviation
         of a normal distribution from which an action is sampled from.

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """
        super().__init__()

        hidden_space1 = 16  # Nothing special with 16, feel free to change
        hidden_space2 = 32  # Nothing special with 32, feel free to change

        # Shared Network
        self.shared_net = nn.Sequential(
            nn.Linear(obs_space_dims, hidden_space1),
            nn.LeakyReLU(),
            nn.Linear(hidden_space1, hidden_space2),
            nn.LeakyReLU(),
        )

        # Policy Mean specific Linear Layer
        self.policy_mean_net = nn.Sequential(
            nn.Linear(hidden_space2, hidden_space2),
            nn.LeakyReLU(),
            nn.Linear(hidden_space2, action_space_dims),
        )

        # Policy Std Dev specific Linear Layer
        self.policy_stddev_net = nn.Sequential(
            nn.Linear(hidden_space2, action_space_dims)
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

        action_means = self.policy_mean_net(shared_features)
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
            print("FAILED to load the network!")
            return self


class REINFORCE:
    """REINFORCE algorithm."""

    def __init__(self, obs_space_dims: int, action_space_dims: int):
        """Initializes an agent that learns a policy via REINFORCE algorithm [1]
        to solve the task at hand (Inverted Pendulum v4).

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """

        # Hyperparameters
        self.learning_rate = 1e-4  # orig = 1e-4 Learning rate for policy optimization
        self.gamma = 0.9  # Discount factor
        self.eps = 1e-6  # small number for mathematical stability

        self.net = Policy_Network(obs_space_dims, action_space_dims)
        self.optimizer = torch.optim.AdamW(self.net.parameters(), lr=self.learning_rate)

    def sample_action(
        self, obs: torch.Tensor
    ) -> tuple[dict[str, np.ndarray], torch.Tensor]:
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

        return StepAction().from_array(action).to_dict(), probs

    def update(self, tapes: list[Tape]):
        """Updates the policy network's weights."""
        losses = torch.zeros(len(tapes))

        # TODO: should rewards be normalized somehow?
        for i, tape in enumerate(tapes):
            running_g = 0
            gs = []

            # Discounted return (backwards) - [::-1] will return an array in reverse
            for R in tape.rewards[::-1]:
                running_g = R + self.gamma * running_g
                gs.insert(0, running_g)

            deltas = torch.tensor(gs)

            # minimize -1 * prob * reward obtained
            for log_prob, delta in zip(tape.probs, deltas):
                # log(p1*p2*..) = log(p1) + log(p2) + ...
                losses[i] -= log_prob.sum() * delta

        loss = torch.mean(losses)

        # Update the policy network
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()


class Tape:
    def __init__(self, idx: int):
        self.idx = idx
        self.probs = []
        self.rewards = []

    @property
    def ep_return(self) -> float:
        return sum(self.rewards)

    def __len__(self) -> int:
        return len(self.probs)


if __name__ == "__main__":
    BATCH_SIZE = 32

    register(
        id="RiktigPatrick-v0",
        entry_point="sim.envs.rp_env:GymRP",
        max_episode_steps=2000,
    )

    rpenv_p = gym.vector.AsyncVectorEnv(
        [
            lambda: gym.make(
                "RiktigPatrick-v0",
                state_keys=[
                    "sens/gyro",
                    "act/left_wheel",
                    "act/right_wheel",
                    "filter/rp_pitch",
                ],
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

    agent = REINFORCE(indim, actiondim)  # StepAction().ndim)

    RUN_NAME = "rp_test_parallel"
    client = MlflowClient()
    try:
        experiment_id = client.create_experiment(RUN_NAME)
    except:
        experiment_id = client.get_experiment_by_name(RUN_NAME).experiment_id

    returns = []
    MAX_RETURN = 0
    seed = 42
    with mlflow.start_run(experiment_id=experiment_id, run_name=RUN_NAME):
        mean_return = 0
        for episode in range(100001):
            tapes = [Tape(i) for i in range(BATCH_SIZE)]
            full_tapes = []
            obs, _ = rpenv_p.reset(seed=seed)
            ready_ = np.zeros(BATCH_SIZE, dtype=bool)
            for _ in range(100):
                actions, probs = agent.sample_action(obs)

                obs, rewards, terminated, truncated, _ = rpenv_p.step(actions)

                for i, t in enumerate(tapes):
                    t.rewards.append(rewards[i, 0])
                    t.probs.append(probs[i])
                    if terminated[i]:
                        full_tapes.append(t)
                        ready_[i] = True

                if all(ready_):
                    agent.update(full_tapes)
                    rets = [t.ep_return for t in full_tapes]
                    mean_return = np.mean(rets)
                    std_return = np.std(rets)
                    mlflow.log_metrics(
                        {"mean_ret": mean_return, "std_ret": std_return},
                        step=episode,
                    )
                    break

            if episode % 10 == 0:
                if mean_return > (MAX_RETURN + 5):
                    print(f"Best return {mean_return:.02f} -> saving")
                    agent.net.store()
                    MAX_RETURN = mean_return
                    NEW_VID = True
                print(f"Episode {episode:<6d} (bs={BATCH_SIZE}) --> {mean_return:.3f}")

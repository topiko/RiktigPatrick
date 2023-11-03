import numpy as np
import torch
from riktigpatric.patrick import StepAction
from torch.distributions.normal import Normal

from sim.nets import PolicyNetwork, ValueNet
from sim.utils import Tape, dict2tensor


class REINFORCE:
    """REINFORCE algorithm."""

    def __init__(
        self,
        obs_space_dims: int,
        action_space_dims: int,
        model_input: list[str],
        use_baseline: bool = False,
    ):
        """Initializes an agent that learns a policy via REINFORCE algorithm [1]
        to solve the task at hand (Inverted Pendulum v4).

        Args:
            obs_space_dims: Dimension of the observation space
            action_space_dims: Dimension of the action space
        """

        # Hyperparameters
        self.learning_rate = 1e-4  # orig = 1e-4 Learning rate for policy optimization
        self.gamma = 0.99  # Discount factor
        self.eps = 1e-6  # small number for mathematical stability

        self.net = PolicyNetwork(obs_space_dims, action_space_dims)  # .load()
        self.polizy_optimizer = torch.optim.SGD(
            self.net.parameters(), lr=self.learning_rate, momentum=0.0
        )

        self.value_net = ValueNet(obs_space_dims)  # .load()
        self.value_optimizer = torch.optim.AdamW(self.value_net.parameters(), lr=1e-3)
        self.value_loss = torch.nn.MSELoss(reduction="sum")

        self.use_baseline = use_baseline
        self.model_input = model_input

    def sample_action(
        self, obs: torch.Tensor, dt: float
    ) -> tuple[dict[str, np.ndarray], torch.Tensor, torch.Tensor]:
        """Returns an action, conditioned on the policy and observation.

        Args:
            state: Observation from the environment

        Returns:
            action: Action to be performed
        """

        obs_t = dict2tensor({k: v for k, v in obs.items() if k in self.model_input})

        action_means, action_stddevs = self.net(obs_t)

        # create a normal distribution from the predicted
        # mean and standard deviation and sample an action
        distrib = Normal(action_means, action_stddevs + self.eps)

        action = distrib.sample()
        probs = distrib.log_prob(action)

        action = action.numpy()

        value = self.value_net(obs_t)

        return (
            StepAction().from_array(action, dt, obs, lock_head=True).to_dict(),
            probs,
            value,
        )

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

        loss = torch.mean(losses)
        # print(losses[-1], G[0])

        # Update the policy network
        self.polizy_optimizer.zero_grad()
        loss.backward()
        self.polizy_optimizer.step()

        return np.array(rets), val_loss


def rews2returns(
    rewards: np.ndarray,
    discount: float = 1,
) -> torch.Tensor:
    values = np.zeros(len(rewards))
    i = -1
    for R in rewards[::-1]:
        values[i] = R + values[i + 1] * discount
        i -= 1
    return values

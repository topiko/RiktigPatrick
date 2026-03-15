import numpy as np
import torch
from riktigpatric.patrick import StepAction
from torch.distributions.normal import Normal

from sim.nets import PolicyNetwork, ValueNet
from sim.utils import Tape, dict2tensor


class RunningNormalizer:
    def __init__(self, momentum: float = 0.99):
        self.momentum = momentum
        self.mean = 0.0
        self.var = 1.0
        self.count = 1e-4

    def update(self, values: np.ndarray):
        batch_mean = np.mean(values)
        batch_var = np.var(values)
        batch_count = len(values)

        delta = batch_mean - self.mean
        total_count = self.count + batch_count

        self.mean += delta * batch_count / total_count
        self.var = (
            self.momentum * self.var
            + (1 - self.momentum) * (batch_var + delta**2 * self.count / total_count)
        )
        self.count = total_count

    @property
    def std(self) -> float:
        return max(np.sqrt(self.var), 1e-6)


class REINFORCE:
    """REINFORCE algorithm with baseline and reward normalization."""

    def __init__(
        self,
        obs_space_dims: int,
        action_space_dims: int,
        model_input: list[str],
        use_baseline: bool = True,
        init2zeros: bool = False,
        load_net: bool = False,
    ):
        self.learning_rate = 1e-3
        self.gamma = 0.99
        self.eps = 1e-6
        self.entropy_coef = 0.1

        self.net = PolicyNetwork(
            obs_space_dims, action_space_dims, init2zeros=init2zeros
        )
        if load_net:
            self.net = self.net.load()

        self.policy_optimizer = torch.optim.Adam(
            self.net.parameters(), lr=self.learning_rate
        )

        self.value_net = ValueNet(obs_space_dims)
        if load_net:
            self.value_net = self.value_net.load()
        self.value_optimizer = torch.optim.Adam(self.value_net.parameters(), lr=1e-3)
        self.value_loss = torch.nn.MSELoss(reduction="sum")

        self.use_baseline = use_baseline
        self.model_input = model_input
        self.reward_normalizer = RunningNormalizer(momentum=0.99)

    def sample_action(
        self,
        obs: dict,
    ) -> tuple[dict[str, np.ndarray], torch.Tensor, torch.Tensor]:
        obs_t = dict2tensor({k: obs[k] for k in self.model_input})

        action_means, action_stddevs = self.net(obs_t)

        distrib = Normal(action_means, action_stddevs + self.eps)

        action = distrib.sample()
        probs = distrib.log_prob(action)
        entropy = distrib.entropy()

        action = action.numpy()

        value = self.value_net(obs_t)

        return (
            StepAction().from_array(action, lock_head=True).to_dict(),
            probs,
            value,
            entropy,
        )

    def _step_value(self, tapes: list[Tape], returns: np.ndarray) -> float:
        all_values = torch.cat([t.values for t in tapes])
        
        self.value_optimizer.zero_grad()
        value_loss = self.value_loss(all_values, torch.Tensor(returns))
        value_loss.backward()
        self.value_optimizer.step()

        return float(value_loss.detach())

    def update(self, tapes: list[Tape]) -> tuple[np.ndarray, float]:
        all_rewards = np.concatenate([t.rewards for t in tapes])
        self.reward_normalizer.update(all_rewards)

        policy_losses = []
        entropy_losses = []
        returns_list = []

        for tape in tapes:
            G = compute_returns(tape.rewards, self.gamma)
            G = (G - self.reward_normalizer.mean) / self.reward_normalizer.std
            returns_list.append(G)

            baseline = np.zeros_like(G)
            if self.use_baseline:
                baseline = tape.values.detach().numpy()

            advantages = G - baseline

            for log_prob, entropy, advantage in zip(tape.probs, tape.entropies, advantages):
                policy_losses.append(-log_prob.sum() * advantage)
                entropy_losses.append(-entropy.sum() * self.entropy_coef)

        policy_loss = torch.stack(policy_losses).mean()
        entropy_loss = torch.stack(entropy_losses).mean()
        total_policy_loss = policy_loss + entropy_loss

        self.policy_optimizer.zero_grad()
        total_policy_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.net.parameters(), 0.5)
        self.policy_optimizer.step()

        all_returns = np.concatenate(returns_list)
        val_loss = self._step_value(tapes, all_returns)

        returns_np = np.array([t.ep_return for t in tapes])
        return returns_np, val_loss


def compute_returns(rewards: np.ndarray, discount: float) -> np.ndarray:
    returns = np.zeros_like(rewards, dtype=np.float32)
    running_return = 0
    for i in reversed(range(len(rewards))):
        running_return = rewards[i] + discount * running_return
        returns[i] = running_return
    return returns


def compute_value_estimates(
    value_net: "ValueNet",
    history: np.ndarray,
    idx_dict: dict[str, np.ndarray],
    model_input: list[str],
) -> np.ndarray:
    """Compute value estimates for each timestep using the critic.
    
    Args:
        value_net: The value network (critic)
        history: Full episode history array
        idx_dict: Dictionary mapping keys to column indices
        model_input: List of observation keys used as input
    
    Returns:
        Array of value estimates for each timestep
    """
    import torch
    
    value_estimates = []
    for i in range(len(history)):
        obs_dict = {k: history[i, idx_dict[k]] for k in model_input}
        obs_t = torch.concatenate([torch.Tensor(obs_dict[k]).reshape(1, -1) for k in model_input], dim=1)
        with torch.no_grad():
            v = value_net(obs_t).item()
        value_estimates.append(v)
    return np.array(value_estimates)

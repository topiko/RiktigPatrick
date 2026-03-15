import numpy as np
import torch
from riktigpatric.patrick import StepAction
from torch.distributions.normal import Normal

from sim.nets import PolicyNetwork
from sim.sim_config import RL_CONFIG
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
        self.entropy_scale = RL_CONFIG["entropy_scale"]

        # Detect device (CUDA if available)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}", flush=True)

        # Combined actor-critic network
        self.net = PolicyNetwork(
            obs_space_dims, action_space_dims, init2zeros=init2zeros
        )
        self.net = self.net.to(self.device)
        
        if load_net:
            self.net = self.net.load()
            self.net = self.net.to(self.device)

        # Single optimizer for both policy and value heads
        self.optimizer = torch.optim.Adam(self.net.parameters(), lr=self.learning_rate)
        self.value_loss = torch.nn.MSELoss(reduction="mean")

        self.use_baseline = use_baseline
        self.model_input = model_input
        self.reward_normalizer = RunningNormalizer(momentum=0.99)

    def sample_action(
        self,
        obs: dict,
    ) -> tuple[dict[str, np.ndarray], torch.Tensor, torch.Tensor, torch.Tensor]:
        obs_t = dict2tensor({k: obs[k] for k in self.model_input}).to(self.device)

        action_means, action_stddevs, value = self.net(obs_t)

        distrib = Normal(action_means, action_stddevs + self.eps)

        action = distrib.sample()
        probs = distrib.log_prob(action)
        entropy = distrib.entropy()

        action = action.cpu().numpy()

        return (
            StepAction().from_array(action, lock_head=True).to_dict(),
            probs,
            value,
            entropy,
        )

    def update(self, tapes: list[Tape]) -> tuple[np.ndarray, float]:
        all_rewards = np.concatenate([t.rewards for t in tapes])
        self.reward_normalizer.update(all_rewards)

        policy_losses = []
        entropy_losses = []
        value_losses = []
        returns_list = []

        for tape in tapes:
            G = compute_returns(tape.rewards, self.gamma)
            returns_list.append(G)

            baseline = np.zeros_like(G)
            if self.use_baseline:
                baseline = tape.values.detach().cpu().numpy()

            advantages = G - baseline

            # Value loss - move tensors to device
            tape_values = tape.values.squeeze().to(self.device)
            value_losses.append(self.value_loss(tape_values, torch.tensor(G, device=self.device)))

            for log_prob, entropy, advantage in zip(tape.probs, tape.entropies, advantages):
                policy_losses.append(-log_prob.sum() * advantage)
                entropy_losses.append(-entropy.sum() * self.entropy_scale)

        policy_loss = torch.stack(policy_losses).mean()
        entropy_loss = torch.stack(entropy_losses).mean()
        value_loss = torch.stack(value_losses).mean()
        
        entropy_scale = RL_CONFIG["entropy_scale"]
        total_loss = policy_loss + entropy_scale * entropy_loss + value_loss

        self.optimizer.zero_grad()
        total_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.net.parameters(), 0.5)
        self.optimizer.step()

        returns_np = np.array([t.ep_return for t in tapes])
        return returns_np, float(policy_loss.detach()), float(entropy_loss.detach()), float(value_loss.detach())

    def _step_value(self, tapes: list[Tape], returns: np.ndarray) -> float:
        # No longer needed - value is updated in update() now
        return 0.0


def compute_returns(rewards: np.ndarray, discount: float) -> np.ndarray:
    returns = np.zeros_like(rewards, dtype=np.float32)
    running_return = 0
    for i in reversed(range(len(rewards))):
        running_return = rewards[i] + discount * running_return
        returns[i] = running_return
    return returns


def compute_value_estimates(
    policy_net: "PolicyNetwork",
    history: np.ndarray,
    idx_dict: dict[str, np.ndarray],
    model_input: list[str],
) -> np.ndarray:
    """Compute value estimates for each timestep using the critic head.
    
    Args:
        policy_net: The policy network with value head
        history: Full episode history array
        idx_dict: Dictionary mapping keys to column indices (may have _0, _1 suffixes)
        model_input: List of observation keys used as input
    
    Returns:
        Array of value estimates for each timestep
    """
    import torch
    
    device = next(policy_net.parameters()).device
    
    def get_obs_indices(key):
        if key in idx_dict:
            return idx_dict[key]
        # Handle expanded keys like sens/gyro_0, sens/gyro_1
        return np.array([idx_dict[f"{key}_{i}"] for i in range(3)])
    
    value_estimates = []
    for i in range(len(history)):
        obs_parts = []
        for k in model_input:
            indices = get_obs_indices(k)
            if len(indices) == 1:
                obs_parts.append(history[i, indices[0]:indices[0]+1])
            else:
                obs_parts.append(history[i, indices])
        obs_t = torch.concatenate([torch.Tensor(p).reshape(1, -1) for p in obs_parts], dim=1).to(device)
        with torch.no_grad():
            _, _, v = policy_net(obs_t)
            v = v.item()
        value_estimates.append(v)
    return np.array(value_estimates)

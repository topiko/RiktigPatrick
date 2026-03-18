"""
Neural Network Agent for RiktigPatrick

UNIT SYSTEM:
All inputs and outputs use SI units (International System of Units):
- Observations: rad, rad/s (wheel velocities, gyro, etc.)
- Actions: rad/s² (wheel accelerations), rad/s (velocities)
- See rp_env.py for detailed unit documentation

ACTION CONFIGURATION:
Actions are configured with explicit bin values or continuous distributions.
Example:
  actions:
    - act/accelerate_both_wheels:
        type: discrete
        bins: [-50, -25, 0, 25, 50]  # rad/s²
"""

import torch
from torch import nn

from riktigpatric.patrick import Observables


class Agent(nn.Module):
    """Neural network policy for robot control.

    Inputs: Observations in SI units (rad, rad/s, etc.)
    Outputs: Actions in SI units (rad/s², rad/s, etc.)

    Supports multiple action types:
    - discrete: Categorical distribution over explicit bin values
    - continuous: Gaussian distribution (not yet implemented)
    """

    def __init__(
        self,
        inputs: list[dict],
        actions: list[dict],
    ):
        """Initialize agent.

        Args:
            inputs: List of input observable names (dimensions auto-detected from Observable.dim())
                    e.g., ['filter/rp_pitch', 'sens/gyro', ...]
            actions: List of action configs, each with action key, 'type', and parameters
                     e.g., [{'act/accelerate_both_wheels': {'type': 'discrete', 'bins': [-50, -25, 0, 25, 50]}}]
        """
        super().__init__()

        # Parse inputs and convert string keys to Observables
        self.input_keys = []  # Will store Observables enum values
        total_input_size = 0

        for inp in inputs:
            # Input is just a string key (e.g., 'filter/rp_pitch')
            key_str = str(inp)

            # Convert string to Observables enum using from_str()
            obs_key = Observables.from_str(key_str)

            # Get dimension from the Observable itself
            obs_dim = obs_key.dim()

            self.input_keys.append(obs_key)
            total_input_size += obs_dim

        # Parse actions
        self.action_configs = {}
        self.action_bins = {}
        self.action_sizes = {}

        for action_item in actions:
            # Extract action key and config
            action_key = list(action_item.keys())[0]
            action_cfg = action_item[action_key]

            self.action_configs[action_key] = action_cfg

            if action_cfg["type"] == "discrete":
                # Store bins as registered buffer (moves with model to GPU/CPU)
                bins = torch.tensor(action_cfg["bins"], dtype=torch.float32)
                # Use string key for buffer name (can't use Actions enum)
                buffer_name = f"bins_{action_key.replace('/', '_')}"
                self.register_buffer(buffer_name, bins)
                self.action_bins[action_key] = bins
                self.action_sizes[action_key] = len(bins)

            elif action_cfg["type"] == "continuous":
                # For future: continuous actions with Gaussian distribution
                raise NotImplementedError("Continuous actions not yet implemented")

            else:
                raise ValueError(f"Unknown action type: {action_cfg['type']}")

        # Network backbone
        self.model = torch.nn.Sequential(
            torch.nn.Linear(total_input_size, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 32),
            torch.nn.ReLU(),
        )

        # Create action heads based on action sizes
        self.action_heads = torch.nn.ModuleDict(
            {
                action_key.replace("/", "_"): torch.nn.Linear(32, size)
                for action_key, size in self.action_sizes.items()
            }
        )

    def forward(self, x: dict[Observables, torch.Tensor]) -> dict[str, torch.Tensor]:
        """Forward pass through the network.

        Args:
            x: Dict of observations in SI units

        Returns:
            Dict of action logits for each action
        """
        # Concatenate inputs based on input_keys
        input_tensors = []
        for key in self.input_keys:
            input_tensors.append(x[key])
        input_tensor = torch.cat(input_tensors, dim=-1)

        # Pass through backbone
        features = self.model(input_tensor)

        # Generate logits for each action
        action_logits = {}
        for action_key in self.action_configs.keys():
            head_key = action_key.replace("/", "_")
            action_logits[action_key] = self.action_heads[head_key](features)

        return action_logits

    def act(
        self, x: dict[Observables, torch.Tensor]
    ) -> tuple[dict[str, torch.Tensor], torch.Tensor]:
        """Sample actions from the policy.

        Args:
            x: Observations dict with tensors in SI units

        Returns:
            actions: Dict of sampled actions in SI units (keys are strings)
            logp: Log probability of the sampled actions
        """
        action_logits = self.forward(x)

        actions: dict[str, torch.Tensor] = {}
        logp_l = []

        for action_key, logits in action_logits.items():
            action_cfg = self.action_configs[action_key]

            if action_cfg["type"] == "discrete":
                # Sample from categorical distribution
                dist = torch.distributions.Categorical(logits=logits)
                action_idx = dist.sample()
                logp = dist.log_prob(action_idx)

                # Index directly into bins to get action value (SI units)
                bins = self.action_bins[action_key]
                action_value = bins[action_idx]

                actions[action_key] = action_value.unsqueeze(-1)
                # Note: If this action had multiple components (e.g., separate left/right),
                # sum their log probs before appending.
                # Currently each action is single component.
                logp_l.append(logp.unsqueeze(-1))

            elif action_cfg["type"] == "continuous":
                # For future: sample from Gaussian distribution
                # If action has multiple components, sum log probs before appending:
                # logp_total = logp.sum(dim=-1, keepdim=True)
                # logp_l.append(logp_total)
                raise NotImplementedError("Continuous actions not yet implemented")

            else:
                raise ValueError(f"Unknown action type: {action_cfg['type']}")

        # Sum log probabilities across all actions
        logp = torch.cat(logp_l, dim=1).sum(dim=1, keepdim=True)

        return actions, logp

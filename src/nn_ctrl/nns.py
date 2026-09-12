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
    act/accelerate_both_wheels:
      type: discrete
      bins: [-50, -25, 0, 25, 50]  # rad/s²
"""

import torch
from torch import nn

from riktigpatric.patrick import (
    Actions,
    DerivedObs,
    Observable,
    StateVar,
    StateVarKey,
    Target,
)


def _get_obs_decoder(input_: StateVarKey) -> tuple[nn.Module, int]:
    if input_ == Observable.ACC:
        return nn.Linear(3, 16), 16  # 3D acceleration vector -> 16 features
    if input_ == Observable.GYRO:
        return nn.Linear(3, 16), 16  # 3D gyro vector -> 16 features
    if input_ in [Observable.HEAD_PITCH, Observable.HEAD_TURN]:
        return nn.Linear(1, 8), 8  # Single angle -> 8 features
    if input_ in [Observable.LEFT_WHEEL_VEL, Observable.RIGHT_WHEEL_VEL]:
        return nn.Linear(1, 8), 8  # Single velocity -> 8 features
    if input_ in [Observable.RP_PITCH, Observable.TRUE_PITCH]:
        return nn.Linear(1, 8), 8  # Single angle -> 8 features
    if input_ == Observable.OBS_TIME:
        # TODO: sine/cos encoding for time to capture periodicity?
        return nn.Linear(1, 8), 8  # Single time value -> 8 features
    if input_ in (
        DerivedObs.CURRENT_POS, DerivedObs.CURRENT_VEL,
        Target.TARGET_POS, Target.TARGET_VEL,
    ):
        return nn.Linear(1, 4), 4

    raise ValueError(f"Unknown observation key: {input_}")


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
        inputs: list[str],
        actions: dict[str, dict],
        hsize: int = 64,
        n_rnnlayers: int = 1,
    ):
        """Initialize agent.

        Args:
            inputs: State variable names; dimensions are inferred from their keys.
            actions: Mapping from action names to type and distribution parameters.
        """
        super().__init__()
        self.inputs = inputs
        self.actions = actions

        # Parse inputs and convert string keys to Observables

        encoders: dict[str, nn.Module] = {}
        n = 0
        for inp in inputs:
            # Convert string to Observables enum using from_str()

            obs = StateVar.from_str(inp)
            encoders[inp], n_ = _get_obs_decoder(obs)
            n += n_

        self.encoders = nn.ModuleDict(encoders)

        # Network backbone
        self.rnn = nn.GRU(
            input_size=n, hidden_size=hsize, num_layers=n_rnnlayers, batch_first=True
        )
        self.layernorm = nn.LayerNorm(hsize)

        # Parse actions
        self.action_configs: dict[Actions, dict] = {}
        heads: dict[str, nn.Module] = {}
        for act_str, action_d in actions.items():
            # Extract action key and config

            action = Actions.from_str(act_str)  # Convert string to Actions enum
            self.action_configs[action] = {}
            if action_d["type"] == "discrete":
                # Store bins as registered buffer (moves with model to GPU/CPU)
                bin_name_ = f"{act_str}_bins"
                self.register_buffer(
                    bin_name_, torch.tensor(action_d["bins"], dtype=torch.float32)
                )
                self.action_configs[action]["type"] = "discrete"
                self.action_configs[action]["bins_name"] = bin_name_

                heads[act_str] = nn.Linear(
                    hsize, len(getattr(self, bin_name_))
                )  # Output logits for each bin

            elif action_d["type"] == "continuous":
                # For future: continuous actions with Gaussian distribution
                raise NotImplementedError("Continuous actions not yet implemented")

            else:
                raise ValueError(f"Unknown action type: {action_d['type']}")

        self.action_heads = nn.ModuleDict(heads)

        # Value head for critic (if using actor-critic method)
        self.value_head = nn.Sequential(
            nn.Linear(hsize, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
        )

    def forward(
        self, x: dict[StateVarKey, torch.Tensor], h: torch.Tensor | None = None
    ) -> tuple[dict[Actions, torch.Tensor], torch.Tensor, torch.Tensor | None]:
        """Forward pass through the network.

        Args:
            x: Dict of observations in SI units

        Returns:
            Dict of action logits for each action
        """
        # Concatenate inputs based on input_keys
        input_tensors = []
        for key, mod in self.encoders.items():
            # (B, obs_dim) -> (B, feature_dim)
            input_tensors.append(mod(x[StateVar.from_str(key)]))

        # (B, input_size) where input_size = sum of feature_dims from all encoders
        input_tensor = torch.cat(input_tensors, dim=1)

        # (B, input_size) -> (B, hsize)
        if input_tensor.ndim == 2:
            # (T, 1, input_size), single time step
            input_tensor = input_tensor.unsqueeze(1)
        else:
            raise ValueError("Invalid input dims. we expect (B, nvals).")

        x_, h = self.rnn(input_tensor, h)

        # (B, hsize)

        x_ = x_.squeeze(1)
        # (B, hsize) -> (B, hsize)
        x_ = self.layernorm(x_)

        # Generate logits for each action head
        action_logits = {}
        for action_key, mod_ in self.action_heads.items():
            # (B, hsize) -> (B, num_bins) for discrete actions
            action_logits[Actions.from_str(action_key)] = mod_(x_)

        # (B, hsize) -> (B, 1)
        values = self.value_head(x_)

        return action_logits, values, h

    def act(
        self, x: dict[StateVarKey, torch.Tensor], h: torch.Tensor | None = None
    ) -> tuple[
        dict[Actions, torch.Tensor], torch.Tensor, torch.Tensor, torch.Tensor | None
    ]:
        """Sample actions from the policy.

        Args:
            x: Observations dict with tensors in SI units

        Returns:
            actions: Dict of sampled actions in SI units (keys are strings)
            logp: Log probability of the sampled actions
        """
        action_logits, values, h = self.forward(x, h)

        actions: dict[Actions, torch.Tensor] = {}
        logp_l = []

        for action, logits in action_logits.items():
            action_cfg = self.action_configs[action]

            if action_cfg["type"] == "discrete":
                # Sample from categorical distribution
                dist = torch.distributions.Categorical(logits=logits)
                action_idx = dist.sample()
                logp = dist.log_prob(action_idx)

                # Index directly into bins to get action value (SI units)
                action_value = getattr(self, action_cfg["bins_name"])[action_idx]

                actions[action] = action_value.unsqueeze(1)
                # Each action currently has a single component.
                logp_l.append(logp.unsqueeze(1))

            elif action_cfg["type"] == "continuous":
                # For future: sample from Gaussian distribution
                # If action has multiple components, sum log probs before appending:
                # logp_total = logp.sum(dim=-1, keepdim=True)
                # logp_l.append(logp_total)
                raise NotImplementedError("Continuous actions not yet implemented")

            else:
                raise ValueError(f"Unknown action type: {action_cfg['type']}")

        # Add observation time to actions for synchronization check
        if Observable.OBS_TIME in x:
            actions[Actions.TIME] = x[Observable.OBS_TIME]

        # Sum log probabilities across all actions
        logp = torch.cat(logp_l, dim=1).sum(dim=1, keepdim=True)

        return actions, logp, values, h

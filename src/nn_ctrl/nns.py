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

import math

import torch
from torch import nn
from torch.nn import functional as F

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
    if input_ in [
        Observable.LEFT_WHEEL_VEL, Observable.RIGHT_WHEEL_VEL,
        Observable.HEAD_PITCH_VEL, Observable.HEAD_TURN_VEL,
    ]:
        return nn.Linear(1, 8), 8  # Single velocity -> 8 features
    if input_ in [Observable.RP_PITCH, Observable.RP_ROLL, Observable.TRUE_PITCH]:
        return nn.Linear(1, 8), 8  # Single angle -> 8 features
    if input_ == Observable.OBS_TIME:
        # TODO: sine/cos encoding for time to capture periodicity?
        return nn.Linear(1, 8), 8  # Single time value -> 8 features
    if input_ in (
        DerivedObs.CURRENT_POS, DerivedObs.CURRENT_VEL,
        Target.TARGET_POS, Target.TARGET_VEL,
        Target.YAW_RATE, DerivedObs.YAW_RATE,
        DerivedObs.CAMERA_PITCH_WORLD, Target.CAMERA_PITCH_WORLD, Target.HEAD_YAW_NECK,
    ):
        return nn.Linear(1, 4), 4

    raise ValueError(f"Unknown observation key: {input_}")


def continuous_action_settings(config: dict) -> tuple[float, float, float, float]:
    """Validate symmetric action limits and pre-tanh Gaussian standard deviations."""
    limit, initial, minimum, maximum = (
        float(config[key]) for key in ("limit", "initial_std", "min_std", "max_std")
    )
    if (
        not all(math.isfinite(value) for value in (limit, initial, minimum, maximum))
        or not 0 < limit <= torch.finfo(torch.float32).max
        or not 0 < minimum < initial < maximum
    ):
        raise ValueError(
            "Continuous actions need limit > 0 and 0 < min_std < initial_std < max_std"
        )
    return limit, initial, minimum, maximum


class ContinuousHead(nn.Linear):
    """A pre-tanh mean and one learned, smoothly bounded log-standard-deviation."""

    limit: torch.Tensor

    def __init__(self, input_size: int, config: dict):
        limit, initial, minimum, maximum = continuous_action_settings(config)
        super().__init__(input_size, 1)
        self.register_buffer("limit", torch.tensor(limit, dtype=torch.float32))
        self.log_std_min = math.log(minimum)
        self.log_std_max = math.log(maximum)
        fraction = (math.log(initial) - self.log_std_min) / (
            self.log_std_max - self.log_std_min
        )
        self.initial_std_logit = math.log(fraction / (1 - fraction))
        self.std_logit = nn.Parameter(torch.tensor(self.initial_std_logit))
        self.reset_neutral()

    @property
    def std(self) -> torch.Tensor:
        # A smooth bound avoids a hard-clamped parameter becoming stuck outside
        # its permitted range with zero gradient.
        log_std = self.log_std_min + (
            self.log_std_max - self.log_std_min
        ) * self.std_logit.sigmoid()
        return log_std.exp()

    @torch.no_grad()
    def reset_neutral(self):
        self.weight.zero_()
        assert self.bias is not None
        self.bias.zero_()
        self.std_logit.fill_(self.initial_std_logit)

    def sample(self, mean: torch.Tensor, *, deterministic: bool = False):
        distribution = torch.distributions.Normal(mean, self.std)
        # This trainer uses score-function policy gradients, not pathwise gradients.
        # rsample() without detaching would differentiate through the sampled action
        # and cancel the Gaussian mean's score gradient.
        latent = mean.detach() if deterministic else distribution.sample()
        action = self.limit * latent.tanh()
        # Stable log |d(limit*tanh(z))/dz|, including near saturated tanh outputs.
        log_jacobian = self.limit.log() + 2 * (
            math.log(2) - latent - F.softplus(-2 * latent)
        )
        logp = distribution.log_prob(latent) - log_jacobian
        return action, logp


class Agent(nn.Module):
    """Neural network policy for robot control.

    Inputs: Observations in SI units (rad, rad/s, etc.)
    Outputs: Actions in SI units (rad/s², rad/s, etc.)

    Supports multiple action types:
    - discrete: Categorical distribution over explicit bin values
    - continuous: Bounded tanh-Gaussian with a learned standard deviation per head
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
        self.inactive_actions: tuple[Actions, ...] = ()

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
                self.action_configs[action]["type"] = "continuous"
                heads[act_str] = ContinuousHead(hsize, action_d)

            else:
                raise ValueError(f"Unknown action type: {action_d['type']}")

        self.action_heads = nn.ModuleDict(heads)

        # Value head for critic (if using actor-critic method)
        self.value_head = nn.Sequential(
            nn.Linear(hsize, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
        )

    @property
    def device(self) -> torch.device:
        """Actual parameter device, including after .to() or checkpoint loading."""
        return next(self.parameters()).device

    def forward(
        self, x: dict[StateVarKey, torch.Tensor], h: torch.Tensor | None = None
    ) -> tuple[dict[Actions, torch.Tensor], torch.Tensor, torch.Tensor | None]:
        """Forward pass through the network.

        Args:
            x: Dict of observations in SI units

        Returns:
            Action logits or pre-tanh means, value estimates, and hidden state.
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
            # (B, num_bins) for discrete actions; (B, 1) means for continuous actions.
            action_logits[Actions.from_str(action_key)] = mod_(x_)

        # (B, hsize) -> (B, 1)
        values = self.value_head(x_)

        return action_logits, values, h

    def act(
        self, x: dict[StateVarKey, torch.Tensor], h: torch.Tensor | None = None,
        *, deterministic: bool = False,
    ) -> tuple[
        dict[Actions, torch.Tensor], torch.Tensor, torch.Tensor, torch.Tensor | None
    ]:
        """Sample actions, or choose argmax/tanh(mean) for deterministic evaluation.

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
            # Inactive heads issue neutral commands and contribute neither sampling
            # noise nor policy gradients. getattr supports older MLflow models.
            if action in getattr(self, "inactive_actions", ()):
                actions[action] = torch.zeros_like(values)
                continue
            action_cfg = self.action_configs[action]

            if action_cfg["type"] == "discrete":
                # Sample from categorical distribution
                dist = torch.distributions.Categorical(logits=logits)
                action_idx = logits.argmax(dim=-1) if deterministic else dist.sample()
                logp = dist.log_prob(action_idx)

                # Index directly into bins to get action value (SI units)
                action_value = getattr(self, action_cfg["bins_name"])[action_idx]

                actions[action] = action_value.unsqueeze(1)
                # Each action currently has a single component.
                logp_l.append(logp.unsqueeze(1))

            elif action_cfg["type"] == "continuous":
                head = self.action_heads[action.value]
                assert isinstance(head, ContinuousHead)
                actions[action], logp = head.sample(logits, deterministic=deterministic)
                logp_l.append(logp)

            else:
                raise ValueError(f"Unknown action type: {action_cfg['type']}")

        # Add observation time to actions for synchronization check
        if Observable.OBS_TIME in x:
            actions[Actions.TIME] = x[Observable.OBS_TIME]

        # Sum log probabilities across all actions
        logp = (
            torch.cat(logp_l, dim=1).sum(dim=1, keepdim=True)
            if logp_l else torch.zeros_like(values)
        )

        return actions, logp, values, h

    @torch.no_grad()
    def initialize_neutral_head(self, action: Actions, probability: float):
        """Start a newly enabled head near zero, independently of GRU state."""
        head = self.action_heads[action.value]
        if isinstance(head, ContinuousHead):
            head.reset_neutral()
            return
        assert isinstance(head, nn.Linear)
        bins = getattr(self, self.action_configs[action]["bins_name"])
        if not 0 < probability < 1 or len(bins) < 2 or (bins == 0).sum() != 1:
            raise ValueError("Neutral initialization needs one zero bin and 0 < p < 1")
        head.weight.zero_()
        assert head.bias is not None
        head.bias.fill_(math.log((1 - probability) / (len(bins) - 1)))
        head.bias[bins == 0] = math.log(probability)

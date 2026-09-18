# RiktigPatrick

Self-balancing robot with a turning head, trained using reinforcement learning
in MuJoCo. The goal is a controller usable with both simulation and the physical robot.

![rp](rp.jpg)

## Project layout

```text
config/                  Training and simulation configuration
src/
  riktigpatric/          Shared robot state/actions and hardware head control
  controller/            Simulator/real-robot adapter interfaces and factory
  sim/                   Maintained MuJoCo environment and training entry point
  nn_ctrl/               Neural-network policy
  filters/               Shared attitude estimation and quaternion math
  relay/                 Shared hardware wire-format conversions
tests/                   Current simulation regression checks
archive/                 Older trainers, tools, firmware, CAD and experiments
outputs/, plots/, video/ Generated training output
```

### One robot interface, interchangeable backends

The intended design is to keep Patrick's state, action definitions and policy
the same whether observations/actions come from MuJoCo or the physical robot.
Hardware coupling is part of that design: `riktigpatric/`, `controller/` and the
wire-format code in `relay/conversions.py` stay in the active source tree.

Currently, training uses `GymRP` and the shared definitions in
`riktigpatric.patrick` directly. `controller.make_controller()` expresses the
simulator/real-robot adapter boundary, but its real-robot implementation is a stub
and these adapters still need alignment with the current training API and SI
units. The hardware integration remains a future update.

### State-update flow

The simulator owns one shared `State` object. Sensor acquisition returns a
dictionary of SI measurement arrays, including `env/obs_time` in episode-relative
seconds. `State` keeps its filter, odometry and targets internally.

The simulation step explicitly performs:

```python
# After applying the action and advancing physics:
measurements = self._read_sensors()             # Acquisition only
self.state.update(measurements)                # Filter, odometry, targets
terminated = self.terminated
rewards = self._calculate_rewards(self.state, terminated=terminated)
self.state.record_transition(action_time, action, rewards)
observation = self._get_obs(rewards)            # Snapshot plus reward diagnostics
```

`State.update()` copies the measurements and processes them together. There is
no separate "store sensors, then remember to update the filter" call sequence.
It requires a strictly newer timestamp. `State.reset(initial_measurements)`
initializes the state without advancing the filter or recording a transition.

`State.snapshot()` returns owned arrays for policy input. Rewards are added by
the Gymnasium adapter for diagnostics; they are not stored as sensor readings.
Hardware can feed its measurements to the same `State.update()` API, without
providing simulation-only ground-truth pitch or training rewards.

## Maintained path

The active entry point is **`src/sim/train_agent.py`**. Single-environment and
batched training use this same code path.

See [fixes.md](fixes.md) for the cleanup findings and before/after pseudocode.

| File | Purpose |
|------|---------|
| `config/rlrp.yaml` | Hydra configuration, in SI units |
| `config/continuous.yaml` | Bounded continuous-action comparison configuration |
| `src/sim/train_agent.py` | Rollouts, actor–critic training, evaluation |
| `src/sim/envs/rp_env.py` | MuJoCo model and Gymnasium environment |
| `src/sim/utils.py` | Environment creation, batching, episode buffers and returns |
| `src/sim/devices.py` | CPU/CUDA selection and isolated evaluation RNG streams |
| `src/sim/timing.py` | Seeded control-interval jitter on the fixed physics grid |
| `src/sim/checkpoints.py` | Portable training snapshots and validation rollback |
| `src/sim/checkpoint_migrations.py` | Preserve old policies when adding position inputs |
| `src/sim/policy_probe.py` | Sampled conditional policy-change diagnostics |
| `src/sim/curriculum.py` | Stage masks, command sampling and physical promotion gates |
| `src/sim/plot_utils.py` | Episode plots |
| `src/nn_ctrl/nns.py` | Recurrent neural-network agent |
| `src/riktigpatric/patrick.py` | Shared state/action definitions, filter and odometry state |
| `src/riktigpatric/trajectory.py` | Backend-independent position waypoint interpolation |
| `src/riktigpatric/geometry.py` | CAD dimensions and shared camera forward kinematics |
| `src/riktigpatric/servo.py` | Hardware head-servo control |
| `src/controller/` | Backend interface and adapter scaffolding |
| `src/relay/conversions.py` | Shared hardware command and sensor packet format |
| `src/filters/` | Quaternion utilities and Mahony attitude filter |
| `tests/test_active_sim.py` | Active simulation regression checks |

Superseded trainers (`rl_rp.py`, `parallel_rl_rp.py`, `try_policy.py`, `nets.py`),
old executables, embedded deployment code, CAD and setup notes now live under
[`archive/`](archive/README.md), generally preserving their original relative
paths. Only the current source packages are installed; archived material is
excluded from normal lint/type-check discovery.

## Setup

Use **Python 3.12 or newer** and [uv](https://docs.astral.sh/uv/).
Run these commands from the repository root:

```bash
uv sync
```

Dependencies are declared in `pyproject.toml`. uv creates a local `uv.lock`
when resolving them; the lockfile is not currently tracked in this repository.

## Training

```bash
# Default: three-stage curriculum, 16 synchronously stepped environments
uv run python -m sim.train_agent

# One environment, same training loop
uv run python -m sim.train_agent env.n_parallel=1

# Short run without rendering
uv run python -m sim.train_agent env.n_parallel=2 \
  env.max_episode_time=0.2 train.max_iterations=2 logging.plot_freq=0
```

The Hydra configuration is located automatically relative to the script.
`HYDRA_CONFIG_DIR` can override its location. `--cfg job` prints the effective
configuration, and Hydra command-line overrides select individual parameters.

Training runs until interrupted unless `train.max_iterations` is set.
`seed` initializes PyTorch/NumPy and provides the base seed for episode resets.
Each iteration collects one episode per environment, with completed episodes
masked out while the remaining environments finish.

### Compute device and network size

`train.device=auto` (default) selects an available CUDA device, otherwise CPU.
Use `cpu`, `cuda`, or `cuda:N` to choose explicitly. An explicit CUDA request fails
clearly if unavailable; it does not silently fall back to CPU. CUDA indices refer
to devices visible to PyTorch, including any `CUDA_VISIBLE_DEVICES` mapping.

```bash
# Larger recurrent policy and batch on CUDA
uv run python -m sim.train_agent train.device=cuda \
  policy.hsize=256 policy.n_rnnlayers=2 env.n_parallel=64

# Explicit CPU baseline
uv run python -m sim.train_agent train.device=cpu

# Choose a particular visible GPU
uv run python -m sim.train_agent train.device=cuda:1
```

`policy.hsize` defaults to 64 and `policy.n_rnnlayers` to 1. These configure the
GRU and connected heads; observation encoders retain their existing sizes.
Network shapes must match when resuming or restoring a checkpoint. Changing
width/layer count requires a fresh policy or a separately designed weight transfer.

The NN, recurrent state, losses and backward pass run on the selected device.
MuJoCo physics still runs on CPU through `SyncVectorEnv`; larger environment
batches are not automatically distributed across CPU workers or multiple GPUs.
CUDA policy inputs and outputs are packed into one transfer in each direction
per simulation step. Only configured policy inputs (plus the action timestamp)
are sent to the accelerator; CSVs and plots are exported on CPU.

Validation and video evaluation preserve the CPU and selected CUDA RNG streams
and restore the agent's previous train/eval mode. MLflow records the resolved
device and PyTorch/CUDA build versions in `training.*` tags.

Recovery snapshots are kept in host RAM to avoid duplicating model/Adam state in
VRAM. This costs a device-to-host copy per protected update. Benchmark throughput
for the chosen setup; CUDA is not necessarily faster for the small default network.

EGL/video rendering is configured separately from `train.device`. CUDA execution
requires a compatible PyTorch build, driver and accessible GPU.

### Continuous-action comparison

Use the continuous configuration to run the same curriculum with bounded Gaussian
action heads. The base `rlrp.yaml` remains the discrete baseline.

```bash
# Continuous curriculum with sampled and deterministic validation diagnostics
uv run python -m sim.train_agent --config-name continuous logging.mlflow.enabled=true

# Compare sampled and argmax evaluation for a discrete policy
uv run python -m sim.train_agent guard.compare_deterministic=true
```

Each continuous head predicts a pre-tanh mean and learns one state-independent
standard deviation:
```text
z ~ Normal(mean, std)
action = limit * tanh(z)
```
The mean heads start at zero. `initial_std`, `min_std` and `max_std` are configured
per action in **dimensionless pre-tanh units**; log-standard-deviation is smoothly
bounded between the configured limits. Near zero, physical standard deviation is
approximately `limit * std`:

| Head | Physical limit | Initial std | Approximate initial physical std |
|------|----------------|-------------|----------------------------------|
| Common wheel acceleration | ±150 rad/s² | 0.025 | 3.75 rad/s² |
| Right-minus-left wheel velocity | ±4 rad/s | 0.10 | 0.4 rad/s |
| Head pitch and turn | ±1 rad/s each | 0.05 | 0.05 rad/s each |

The acceleration head's `min_std=0.005` allows exploration to approach
approximately 0.75 rad/s² near zero mean.

The action limits match the current discrete baseline. Sampling is local around
the mean, rather than assigning probability to unrelated bins. Samples remain
stochastic at every control step; this does not impose temporal smoothing.

The trainer uses score-function gradients: samples are detached, and log densities
include the stable tanh-and-scale Jacobian. Continuous log densities may be positive
and are not directly comparable to categorical log probabilities. Optimizer,
reward, return and TBPTT settings follow the base configuration.

`guard.compare_deterministic=true` runs a second evaluation on the same cases,
using argmax for discrete heads or `limit*tanh(mean)` for continuous heads. It logs
`validation_deterministic/*`; the original sampled `validation/*` still governs
curriculum promotion and rollback. Both evaluations preserve training RNG state.
This comparison is enabled by default in `continuous.yaml`; it adds a second
validation rollout. `policy/std/act/*` logs learned pre-tanh widths during training.

Continuous and discrete action schemas require their own matching checkpoints.
Start a fresh continuous policy, and use `--config-name continuous` again when
resuming its checkpoint. Both schemes can use the existing stage-separated MLflow
runs. See the [inspection of discrete run b6a33b4d](docs/run_b6a33b4d.md) for the
observed sampling/stability tradeoff that motivates this comparison.

### Recurrent training (TBPTT)

Fresh training uses `train.policy_lr=0.0001`. The loss is
`policy_loss + rl.value_loss_coef * value_loss`, with `rl.value_loss_coef=0.1` to
reduce critic-driven changes to the shared GRU. `losses/value` remains the raw MSE;
`losses/value_weighted` shows its actual contribution to `losses/total`.

`train.kl_probe_every=32` records an observation and detached GRU state every 32
control steps, excluding finished environments. After the optimizer step, the
same observation/state pairs measure conditional policy KL, logged as
`optimization/{policy_kl_mean,policy_kl_max,policy_probe_samples}`. Active Gaussian
and categorical heads are supported; inactive heads contribute no KL. This is
a local diagnostic conditional on cached hidden states, not a full recurrent
history replay or a KL-based rejection rule. It costs an extra forward pass at
probe steps and one batched post-update pass; `0` disables it.

`train.tbptt_steps=124` controls truncated backpropagation through time. At the
default 0.01 s control timestep, each chunk spans up to **1.24 s**. The GRU's
hidden-state values carry forward continuously; only gradient history is detached
at chunk boundaries. Memory is reset at the start of an episode.

Training collects full episodes with gradients enabled, detaching the hidden
state every N rollout steps. It then computes full-episode discounted returns
and performs one backward pass, gradient clip and Adam step per rollout batch.
The critic baseline is detached, losses are normalized by the batch's total valid
transitions, and finished episodes and padded tails contribute no further loss.

Override with `train.tbptt_steps=64`, for example. The value must be a positive
integer; setting it at least as large as the longest episode gives full-episode
BPTT. Chunk size can change when resuming a checkpoint.

This simple approach retains all chunk graphs until the final backward pass, so
activation memory still grows with episode length. If memory becomes a bottleneck,
a future optimization is no-grad collection followed by chunked replay/backward,
freeing each chunk's graph immediately while retaining full-episode return targets.

### Control-interval jitter

MuJoCo integrates at a fixed **2 ms**. `env.step_time=0.01` is the nominal control
period, used for command conversion and as the reward/discount reference interval.
Optional jitter varies how many physics steps pass before the next policy update:

```bash
uv run python -m sim.train_agent --config-name continuous \
  env.step_time_std=0.002 env.min_step_time=0.007
```

The scheduler samples `max(min_step_time, Normal(step_time, step_time_std))`, then
rounds to the physics grid while respecting the minimum. A 7 ms lower bound thus
becomes **8 ms**, with intervals such as 8, 10, 12 and 14 ms. Clipping and rounding
slightly change the realized mean/std. `step_time_std=0` disables jitter (default).
Each environment has an independent, reset-seeded timing stream, separate from
physical randomization; `env.randomize` controls only the latter. Validation and
video evaluation use the same configured timing distribution reproducibly.

Wheel acceleration commands still form velocity targets using the **nominal**
period. Commands are issued before drawing the future scheduling delay, so the
simulator does not compensate using timing information unavailable to the robot.
Actual observation timestamps drive the shared filter, odometry and trajectories.
The policy already receives observation time; no extra policy inputs are required.
This models variable action-hold intervals, not separate sensor or transport delay.

Dense rewards are scaled by `actual_dt / nominal_dt`; a one-off fall cost is not.
Returns use `gamma_dt = rl.discount ** (actual_dt / nominal_dt)`. Thus elapsed
time, rather than the number of policy calls, controls reward accumulation and
discounting. Tracking MAEs weight post-startup samples by their elapsed intervals.

`env.max_episode_time=20.0` sets the duration limit in seconds. The last interval
is shortened to reach that limit, even if this is below the normal jitter minimum.
The optional legacy `env.max_episode_steps` setting (default `null`) limits duration
to `max_episode_steps * step_time`; with jitter it is not a cap on actual decisions.
This allows a full-duration episode even when short intervals need more updates.

CSV traces include `transition/duration`, aligned with the outgoing action.
`episodes/duration/*` and `timing/step_time/{mean,min,max,std}` report realized
timing; episode-length metrics continue to count policy decisions. With jitter,
video frames are resampled onto the nominal playback clock, with the terminal
image retained and duration rounded to a video frame.

### Curriculum (default)

`curriculum.enabled=true` starts in **position holding**, then advances based on fixed-suite
validation. One GRU, critic and four action heads exist throughout; input/output
shapes stay constant across stages.

| Stage | Active action heads | Objectives |
|-------|---------------------|------------|
| `hold_position` | Common wheel acceleration | Hold the starting position with gentle velocity damping |
| `locomotion` | Common acceleration + wheel-speed difference | Balance, forward velocity and yaw rate |
| `full_control` | All four | Locomotion plus horizontal camera elevation and forward neck yaw |

Inactive heads issue exactly zero, without sampling or contributing log-probability
to the policy loss. Their parameters receive no gradient. Corresponding yaw/head
reward components are zeroed **before** the critic's return targets are computed.
The GRU and previously active heads continue learning. At activation, a new discrete
head selects zero with probability `curriculum.neutral_probability=0.9`; its other
bins share the remaining probability. A continuous head instead starts with zero
mean and its configured `initial_std`. Both its mean and exploration parameter
start with fresh optimizer moments. Inactive continuous heads also issue exactly
zero, with no gradient into their mean or standard deviation.

**All curriculum stages allow free leaning** within
`curriculum.pitch_deadband=0.08726646259971647` radians (±5°), then apply a quadratic
penalty. A nonzero torso angle may be needed to place the combined centre of mass
above the wheel contact line. The curve matches the old pitch penalty at the
20° fall-angle reference:

```text
excess = max(0, abs(filtered_body_pitch) - deadband)
pitch_reward = pitch_scale * fall_angle * (excess / (fall_angle - deadband))²
```

With the default scale this costs approximately 0 at 5°, −0.222 at 10° and −2 at
20°, symmetrically for forward/backward lean. True body pitch still controls the
20° episode termination. Deadbands must be nonnegative and below 20°, in radians;
`null` selects the original linear penalty. With curriculum disabled,
`env.pitch_deadband` selects the shaping instead (default `null`).

Training begins with survival, pitch shaping and position holding together;
there is no survival-only warm-up. Steering and head commands start at zero.
The hold stage targets `x=0` relative to the episode origin, using signed fore/aft
wheel odometry, plus gentle damping toward `v=0`. With default scales its tracking
reward is `-abs(x) - 0.5*abs(v)`: `reward/pos=-1` and
`reward/vel=-4` multiplied by `curriculum.hold_velocity_weight=0.125`.
This penalizes sustained drift while discouraging oscillation around the origin.
Both position and velocity are observable throughout. The wheel controller is trained
to stay near the origin from the outset, rather than first learning to accelerate
until reaching its speed limit.

After position hold, each training episode independently samples a command pair from
`curriculum.forward_velocities=[-0.1,0,0.1]` m/s and
`curriculum.yaw_rates=[-0.5,0,0.5]` rad/s. Gaze references remain `[0,0]`.
Validation deterministically covers all pairs, with fixed seeds and small initial
pitch/mass/geometry variations. Training randomization is enabled by default
(`env.randomize=true`), using varying training seeds and fixed validation seeds.

Promotion requires **three consecutive** passing evaluations with **90%** survival
and strict tracking limits, controlled by `curriculum.consecutive_passes` and
`curriculum.survival_fraction`. Evaluations are shared with the guard:
`guard.every=10` updates and `guard.episodes=50` episodes by default.

| Promotion | Survival requirement | Tracking-error limits |
|-----------|----------------------|-----------------------|
| Position hold → Locomotion | ≥90% reach 20 s | Position MAE ≤0.05 m and forward-speed MAE ≤0.05 m/s |
| Locomotion → Full control | ≥90% reach 20 s | Forward-speed MAE ≤0.05 m/s; yaw-rate MAE ≤0.2 rad/s |

These are tunable `curriculum.*` settings, not measured guarantees. MAEs omit the
first `startup_seconds=1.0`, time-weight the remaining samples, and average episodes equally. Short failed episodes
still contribute errors and count against survival; a fall exactly at the required
duration is a failure. Episode limits shorter than a gate prevent promotion.
Initial/resumed evaluations and new-stage baseline evaluations do not advance the
streak. A failed gate or guard rollback clears it. Curriculum evaluation continues
with `guard.enabled=false`, with rollback disabled.

Use `curriculum.enabled=false` to train all configured heads immediately and use
manual references or other tracking modes. The named task-example configurations
already disable the curriculum. Curriculum runs own their references; conflicting
manual target overrides are rejected rather than silently ignored.

### Tracking tasks

With the curriculum disabled, the default task is **zero forward velocity and yaw rate, with horizontal,
forward-facing gaze**:
balance and stop moving, without
requiring a return to the episode's starting position. Select the objective with
`env.tracking_mode`:

| Mode | Policy tracking inputs | Tracking reward |
|------|------------------------|-----------------|
| `velocity` (default) | `target/vel`, `derived/vel` | Absolute velocity error in m/s |
| `position` | `target/pos`, `derived/pos` | Absolute position error in meters |
| `position_velocity` | Both position and velocity pairs | Position error plus velocity error |
| `none` | No tracking inputs | No position/velocity-reference penalty |

All locomotion modes retain balancing and the selected head/yaw objectives. Position and `none` modes
also retain the configured absolute wheel-speed penalty. Velocity and combined modes disable
that penalty, since penalizing motion itself would oppose a nonzero command.

```bash
# Train all heads immediately at zero velocity and yaw rate
uv run python -m sim.train_agent curriculum.enabled=false

# Zero-position holding: also return toward the episode's starting position
uv run python -m sim.train_agent --config-name position_hold

# Disable fore/aft setpoint tracking; retain yaw, head and balancing objectives
uv run python -m sim.train_agent curriculum.enabled=false env.tracking_mode=none

# Request 0.1 m/s forward in all environments and evaluation
uv run python -m sim.train_agent curriculum.enabled=false env.target_vel=0.1

# Three constant training commands: -0.1, 0.0, +0.1 m/s; evaluate at zero velocity
uv run python -m sim.train_agent --config-name velocity_example
```

`train.target_velocities` accepts one signed forward-velocity command per training
environment. `env.target_vel` is the default command and is used for evaluation.
The values persist across steps and resets and may be changed while running.

Positions use signed fore/aft travel distance relative to the episode origin.
The fixed position target remains zero unless overridden:

```bash
# Same fixed target for all training environments and evaluation
uv run python -m sim.train_agent curriculum.enabled=false \
  env.tracking_mode=position env.target_pos=0.2

# Different fixed targets for each training environment
uv run python -m sim.train_agent curriculum.enabled=false \
  env.tracking_mode=position env.n_parallel=3 \
  'train.target_positions=[-0.2,0.0,0.2]'
```

`train.target_positions` must contain exactly one finite value per environment.
In position mode it overrides `env.target_pos` for training; evaluation uses the
environment's default position/trajectory settings.
The rollout's `add_targets()` helper updates both environment state and batched
policy observations. Targets persist across steps and resets; odometry resets
to zero at each episode start.

### Low-level velocity control and an outer controller

The intended control hierarchy is:

```text
position controller or RC command
    -> desired forward velocity in m/s and body yaw rate in rad/s
    -> balancing policy
    -> wheel/head actuator commands
```

For example, an outer position controller could start with:

```text
velocity_command = clamp(Kp * (desired_position - measured_position), speed_limits)
state.target_vel = velocity_command
action = policy(state.snapshot())
```

The low-level velocity policy has no position-target or odometry-position inputs
by default. An RC source can supply the velocity command directly. For batched
simulations, `add_targets(obs, env, target_velocities=[...])` updates both the
shared states and the already-returned policy observation dictionary, without
resetting filters or episode clocks.

Training only at zero velocity establishes balancing/stopping. General command
following needs varied commands and command changes during training. The example
configuration varies constant commands across environments; it does not yet
schedule velocity-command changes within episodes. RC transport and the outer
controller are not implemented here. Body yaw-rate commands use the independent
reference described below.

Changing tracking mode changes the policy input fields, so train a matching
policy. Restoring a model with incompatible inputs raises an error.

### Robot yaw-rate control

`env.yaw_tracking=true` adds a body-yaw tracking objective independently of the
fore/aft and head tasks. `env.target_yaw_rate=0.0` is the default: avoid spinning.
Positive rates turn left about the robot's +Z axis. The shared `State` exposes
`target/yaw_rate` and `derived/yaw_rate`; the latter is the body-frame gyro Z
component, in rad/s. This approximates heading rate near upright; it is not an
absolute heading reference or an exact world-frame heading derivative when tilted.

```bash
# Turn left at 0.5 rad/s while balancing; same reference for evaluation
uv run python -m sim.train_agent curriculum.enabled=false env.target_yaw_rate=0.5

# Train right, straight and left commands; evaluate at the default zero rate
uv run python -m sim.train_agent curriculum.enabled=false env.n_parallel=3 \
  'train.target_yaw_rates=[-0.5,0.0,0.5]'
```

`train.target_yaw_rates` accepts one finite reference per training environment.
It can coexist with forward-velocity or position references and head targets.
For live commands use `state.target_yaw_rate = rate`, or
`add_yaw_targets(obs, env, rates)` for batched environments. Targets persist across
resets and do not generate actuator commands. Training only at zero yaw rate
teaches stopping rotation; command following needs varied training references.

The separate `act/wheel_vel_diff` policy head selects **right minus left wheel
angular velocity**, using nine bins from −4 to +4 wheel rad/s. Its units differ
from the body-yaw reference. For ideal upright rolling, body yaw rate is roughly
`wheel_radius * wheel_velocity_difference / wheel_track`; the policy learns the
mapping using gyro feedback, including actuator dynamics and slip.

The wheel mixer combines the two NN outputs:
```text
mean_target = mean(measured_wheel_velocities) + wheel_acceleration * dt
left_target  = mean_target - wheel_velocity_difference / 2
right_target = mean_target + wheel_velocity_difference / 2
```
It clips common acceleration and mean velocity to their configured limits, then
reduces the requested difference to fit the remaining wheel-speed headroom.
This prioritizes balance when turning and forward-speed demands compete. A zero
difference requests equal wheel speeds; a repeated difference does not accumulate.
The shared implementation is `src/riktigpatric/wheel_control.py`.

`env.yaw_tracking=false` disables its reward and appended policy inputs. To return
to the earlier three-head setup, also remove `act/wheel_vel_diff` from
`policy.actions` in the YAML configuration. The four-head setup adds action and input
weights, so it requires a matching policy rather than an old three-head checkpoint.

### Time-varying position trajectories

Trajectories are lists of `[time_seconds, position_meters]` waypoints. They start
at time zero, interpolate linearly between waypoints and hold the final position.
Times must strictly increase; a single waypoint gives a fixed target.

```bash
# Two environments following different trajectories; evaluation follows the first
uv run python -m sim.train_agent --config-name trajectory_example

# One trajectory shared by all training environments and evaluation
uv run python -m sim.train_agent curriculum.enabled=false \
  env.tracking_mode=position 'env.target_trajectory=[[0,0],[2,0.2],[4,0]]'
```

`train.target_trajectories` accepts one waypoint list per training environment.
Select position mode for position targets/trajectories, and velocity mode for
velocity targets. Choose one of `train.target_positions`,
`train.target_trajectories`, or `train.target_velocities`; combining batch
override types is an error. Each override replaces the corresponding default.
Evaluation uses `env.target_trajectory` when set, otherwise `env.target_pos`.

The shared `State` samples its trajectory from observation time, so the same
target logic is usable with simulated or hardware observations. On each step,
the policy sees the target at time *t* and receives a reward for tracking the
target at *t + dt*. Resetting restarts the trajectory at zero. Replacing a
trajectory mid-episode samples it at the current episode time; setting a fixed
target disables it. The batched `add_targets()` helper handles either target form.

This supplies position references; a policy proven to track trajectories still
requires further work.

### Head/camera tracking

Head tracking is independent of the locomotion mode and enabled by default:

- `env.head_target: [0, 0]` requests **world camera elevation** and **neck-relative
  yaw**, in radians. Zero means horizontal gaze and forward yaw. Positive elevation
  looks up; positive yaw turns left.
- `env.head_trajectory` supplies `[time_seconds, camera_elevation, neck_yaw]`
  waypoints for the default training/evaluation reference.
- `train.head_targets` supplies one fixed angle pair per training environment;
  `train.head_trajectories` supplies one waypoint list per environment. Choose one
  of these two overrides. They coexist with position/velocity overrides.
- `env.head_tracking=false` disables head-reference policy inputs/rewards and
  restores the older neutral-joint-pitch penalty. Configured head action heads
  remain available to the policy.

```bash
# Train full control directly at zero speed, camera horizontal, neck yaw forward
uv run python -m sim.train_agent curriculum.enabled=false

# Slow, opposite head trajectories in two environments
uv run python -m sim.train_agent --config-name head_tracking

# Fixed camera elevation 0.1 rad and neck yaw 0.2 rad
uv run python -m sim.train_agent curriculum.enabled=false 'env.head_target=[0.1,0.2]'

# Locomotion-only reference tracking
uv run python -m sim.train_agent curriculum.enabled=false env.head_tracking=false
```

The shared `State` computes estimated camera elevation from the body attitude
estimate and measured head joints. The rotation chain includes the tilted neck
yaw axis; it does not simply add pitch angles. **The NN chooses active actuator
commands; curriculum-inactive heads issue zero.** There is no inverse-kinematics controller or body-pitch compensation
command applied outside the NN.

With head tracking enabled, the old `abs(head_joint_pitch)` penalty is disabled.
Instead, `reward/camera_pitch` penalizes actual MuJoCo camera elevation error and
`reward/head_yaw` penalizes neck-joint yaw error. The actual camera pose is used
for scoring and plots; the NN receives the estimated pose, not simulation truth.

Head references are sampled at each observation time and restart on episode reset.
`add_head_targets()` refreshes batched references without resetting filters or
changing locomotion targets. Fixed references replace head trajectories. Yaw
interpolation does not wrap because the joint is mechanically limited.

Camera elevation references must be within ±90° and yaw within ±40°; a valid
reference is not necessarily reachable at every body orientation. Start with
level gaze and slow trajectories. Joint/rate limits constrain actuator commands;
the learned policy must coordinate head motion with balancing.

See [geometry and frame conventions](docs/geometry.md) for CAD-derived dimensions,
the confirmed −28°/+50° neck-pitch and ±40° yaw ranges, and modeling assumptions.

### Plots, videos and checkpoints

- `logging.plot_freq`: evaluate and save plots/videos every N iterations,
  including iteration zero; `0` disables evaluation rendering.
- Plots and numeric CSV episode traces go to `plots/`; videos go to `video/`,
  relative to the run's working directory. Directories are created automatically.
- Matching artifacts share a training-iteration name, such as
  `train_iter_000400.mp4`, `train_iter_000400.png` and `train_iter_000400.csv`.
  The number matches the zero-based training iteration, not a recording counter.
- Each evaluation saves and, with MLflow enabled, uploads the matching PNG, CSV
  and MP4. These describe the same evaluation episode, not every training rollout.
- Tracking plots follow the selected mode: position, velocity, or no tracking
  plot for `none`. Inactive reward terms are omitted from plots; CSV traces keep
  all observation and reward fields.
- Head tracking adds target/estimated/actual camera elevation and target/measured
  neck-yaw plots. `env.camera_view` selects `external`, `head`, or `both` (default:
  overview left, head camera right). Paired views use 320×240 pixels each.
- For headless rendering on systems with EGL support, prefix the command with
  `MUJOCO_GL=egl MPLBACKEND=Agg`.
- MLflow is disabled by default. Enable it with
  `logging.mlflow.enabled=true` and set `MLFLOW_TRACKING_URI`; optional credentials
  can be supplied in the environment or a local `.env` file.
- With MLflow enabled, metrics are logged every `logging.mlflow.push_freq`
  iterations and deployment policy models every `logging.save_freq` iterations.
  Resumable training checkpoints are saved locally even without MLflow (see below).
- The full composed configuration, including CLI overrides and resolved `${...}`
  interpolations, is uploaded as **`config/resolved.yaml`** to the parent run and
  every curriculum stage run. Flattened MLflow parameters remain available for
  searching and comparing runs.
- Training metric names are grouped by prefix: `losses/{policy,value,value_weighted,total}`,
  `returns/{mean,min,max,std}`, and `episodes/length/{mean,min,max}`.
- Head evaluations also log `evaluation/head/{camera_pitch_mae,neck_yaw_mae,
  camera_estimation_mae}` in radians, over the post-action observations.
- `policy.restore_id` accepts an MLflow **logged model ID** to restore an agent.
  Use `curriculum.enabled=false`; all heads are enabled and optimizer state and
  iteration count start fresh. Full curriculum resume uses `train.resume_from`.

#### Separate MLflow runs per stage

With curriculum and MLflow enabled, the session is a parent run containing nested
`hold_position`, `locomotion` and `full_control` runs as those stages are reached. Each
child owns its training/validation metrics, policy models, videos, plots and
checkpoints. The parent holds the overall configuration and build/device tags.
Reward curves from different objectives are therefore separate.

Child tags include `curriculum.version`, `curriculum.stage`, `curriculum.stage_index`, inactive actions,
disabled rewards, `training.start_iteration` and `training.resume_from`. A resumed
process starts a new parent session and a **fresh child for the restored stage**,
tagged with its source checkpoint; it does not append to an old stage's curves.
Iterations remain global across the curriculum. A successfully completed child is
closed before the next opens, and an interrupted/failed active child closes with
the session's exception status.

### Training checkpoints and degradation guard

Training checkpoints live in the Hydra run output directory, under `checkpoints/`.
`checkpoints.dir` can override that location. With MLflow enabled, they are also
uploaded as `checkpoints/*` artifacts:

| File | Purpose |
|------|---------|
| `best.pt` | Best fixed-benchmark validation return in the current stage/session |
| `stage_<name>_complete.pt` | Outgoing stage's weights, Adam and curriculum progress |
| `latest.pt` | Initial/periodic, new best, recovered, final or interrupted training state |
| `iteration_XXXXXX.pt` | Periodic archive, every `checkpoints.every` updates (100 by default) |
| `training_failure.txt` | Exception/interrupt traceback if training stops abnormally |

The `.pt` files contain model weights, Adam state, CPU PyTorch/NumPy/Python RNG
states, the selected CUDA RNG state when applicable, configuration and the next
iteration number. Curriculum checkpoints also contain the stage and promotion
streak. They are tensor/data
checkpoints loaded with `weights_only=True`, rather than serialized Python agents.
Writes replace files atomically. Local saves happen before uploads, so an MLflow
upload failure still leaves a local recovery point.

```bash
# Resume a downloaded or local training checkpoint; use an absolute path
uv run python -m sim.train_agent train.resume_from=/path/to/checkpoints/latest.pt

# Resume the strongest saved benchmark policy, including its Adam state
uv run python -m sim.train_agent train.resume_from=/path/to/checkpoints/best.pt

# Resume an older continuous policy with a deliberately smaller optimizer step
uv run python -m sim.train_agent --config-name continuous \
  train.resume_from=/path/to/checkpoints/best.pt train.resume_lr=0.0001
```

`train.resume_from` and `policy.restore_id` are mutually exclusive. The former
restores optimizer LR and progress; the latter loads policy weights with a new
optimizer. `train.resume_lr` explicitly overrides the loaded Adam LR while retaining
its moments; the default `null` preserves the saved LR (including guard reductions).
Changing `train.policy_lr` alone affects fresh optimizers, not resumed ones.
Input order, action specifications and network shapes must match for checkpoint
resume. Keep the same task/seed overrides for reproducible continuation: saved
configuration is included for reference, while the supplied run configuration
remains active. `train.max_iterations` is a total limit, not an additional count.
Match `curriculum.enabled` to the checkpoint: older non-curriculum checkpoints
load with it disabled. Curriculum resume restores the neutral-action mask without
reinitializing learned heads. Changed curriculum criteria/commands or validation
seed/batch/timing settings reset the streak while retaining the stage.

Curriculum state uses schema version 4. Retired `balance` and `stop` stages in older
checkpoints map to `hold_position`, so resuming cannot re-enter the removed stage.
Later stage names are retained. Migration preserves weights and Adam state and
resets the promotion streak for the new curriculum definition.

Training checkpoints from the earlier policy can automatically gain the two
appended position inputs. Existing GRU columns and Adam moments are preserved;
new GRU input columns and their moments start at zero, so the additional inputs
initially have no influence on the controller. New encoders can then learn during
training. This narrow migration requires matching old inputs, action specifications,
hidden size and layer count; unrelated architecture changes are rejected. Full
MLflow model restores still require an exact input/action specification.

Promotion archives `stage_<name>_complete.pt`, enables the next heads, and resets
the guard baseline. New-stage validation establishes its own `best.pt`; guard
rollback cannot cross a stage boundary. Validation progress is saved to `latest.pt`
even when return does not improve.

Checkpoints keep tensor data on CPU and can be loaded on CPU or the selected CUDA
device; Adam moments follow the model's parameters. The saved CUDA RNG stream is
restored to the selected GPU, even if its visible index differs from the source.
CPU loads do not require CUDA and ignore that optional stream. Older CPU-only
checkpoints remain readable; when moving one to CUDA, the GPU stream starts from
the configured seed. Cross-device continuation is supported, but it is not
bitwise-reproducible across CPU/CUDA or different hardware/software versions.

With `guard.enabled=true` (default), training runs a fixed-seed validation batch
before the first update and every `guard.every=10` updates. The batch has
`guard.episodes=50` environments and does not render. Curriculum validation uses
the fixed command suite and randomized initial conditions described above.
Without curriculum, it uses the environment's default targets/trajectory and
randomization settings; training-only target overrides are excluded. Validation preserves training
RNG and policy mode. A resumed run establishes a fresh benchmark baseline.

Rollback requires **one** score more than **20% and 100 return units**
below the best, after the best reaches at least 500. These thresholds are configurable
with `guard.patience`, `drop_fraction`, `absolute_drop` and `min_best_return`.
Rollback restores the best weights and optimizer moments, multiplies the current
learning rate by `guard.lr_factor=0.95` (down to `guard.min_lr`), and continues at
the current iteration and sampling state.

**After a rollback, every subsequent update is checked immediately.** A candidate
must strictly improve the fixed-suite return to be kept. Otherwise the best weights
and Adam moments are restored, keeping the reduced LR unchanged and advancing the
training seeds. With `guard.recovery_attempts=3`, three consecutive rejections stop
training cleanly instead of repeatedly shrinking LR. An improvement clears the
rejection count; per-update checking remains active until the curriculum stage
changes. Extra checks do not accelerate the curriculum's scheduled promotion gates.
Set `guard.recovery_attempts=0` to use the previous periodic rollback behavior.

On exhaustion, `latest.pt` contains the restored best policy and Adam state, current
LR, current RNG and next attempted iteration. `training_stop.json` records the
reason, best/candidate scores and rejected candidate's validation metrics. Both are
uploaded when MLflow is enabled. Resume explicitly with `train.resume_from`; as
with other resumes, this starts a fresh guard baseline. The stop indicates a lack
of accepted updates, not successful training.

Metrics are grouped under `validation/returns/*`, `guard/*` and
`optimization/{gradient_norm,learning_rate}`. Validation step numbers count
completed updates; zero is the initial benchmark. `guard.enabled=false` disables
performance rollback, but local checkpoints and numerical update checks remain.
When `guard/rolled_back=1`, `validation/*` describes the rejected candidate;
`guard/best_return` describes the recovered reference policy's benchmark score.
`guard/{recovering,recovery_failures,stop_requested}` records bounded recovery.
Curriculum also logs `validation/{survival_fraction,position_mae,velocity_mae,yaw_rate_mae}` and
`curriculum/{stage,success_streak}`. Stage indices are 0 (position hold),
1 (locomotion) and 2 (full control).

Non-finite observations, rewards, losses or updates stop training with a recovery
checkpoint; they are not silently retried indefinitely. Hard kills and power loss
cannot run the failure handler, so recovery then uses the most recent saved state.
`latest.pt` is not written after every update by default, and it may be worse than
`best.pt`. A validation guard mitigates policy collapse; it is not a monotonic-
improvement guarantee or a replacement for broader evaluation. A pre-update host
snapshot can still be saved if an accelerator error prevents in-memory rollback.

See the [review of run 2b502b61](docs/run_2b502b61.md) for the observed degradation
and checkpoint gaps that motivated these changes.

### Diagnosing harmful updates

Compare independent rollout-batch gradients and their equal-weight average from
one frozen checkpoint:

```bash
uv run python -m sim.diagnose_updates /path/to/checkpoints/best.pt \
  --batches 4 --learning-rate 0.00018 --output /tmp/update_diagnostics.json
```

The command uses the checkpoint's configuration, restores the same weights and
Adam state before every candidate, and evaluates on the original guard seeds and
a second fixed suite. It writes JSON containing batch seeds, actor/critic shared
gradient norms, gradient cosine similarities, and candidate returns, durations and
tracking errors. Each batch uses distinct action seeds and non-overlapping
environment-seed ranges. One rollout graph is retained at a time. Full-length
diagnostics can take several minutes: four batches require five candidate policies
plus the baseline to be evaluated on both suites.

The default `--loss actor` excludes the new critic gradient; restored Adam moments
still contain historical joint-training gradients. `--fresh-adam` tests without
that history, and `--loss joint` reproduces the normal combined-loss update.
Gradients are averaged **before** clipping and Adam, not by averaging updated
weights. The source checkpoint and MLflow run are not modified.

For controlled reward/credit-horizon comparisons, repeat with either
`--hold-velocity-weight 0` or `--discount 0.999`. These affect gradient collection
only; evaluation retains the original reward so removing a penalty cannot inflate
the reported benchmark score directly. The old critic is retained, so these are
one-step diagnostics, not evidence of convergence under a different objective.

See [the September 18 update diagnosis](docs/update_diagnosis_2026-09-18.md) for
measured gradient, reward and exploration-noise comparisons.

### Reading an episode trace

`plots/train_iter_XXXXXX.csv` contains all observations/targets, actions, rewards,
log-probabilities, value predictions, returns and advantages. Vector observations
have separate columns such as `sens/gyro[0]`, `sens/gyro[1]`, `sens/gyro[2]`.

There are **T+1 rows for T actions**. At row t:

- `env/obs_time` and the observation/target columns describe the state at t.
- `act/*` is the outgoing action taken from that state.
- `transition/reward` is the reward received **after** that action, at t+dt.
- `transition/duration` is that action's realized hold interval in seconds.
- Observation `reward/*` columns are the incoming reward diagnostics at t;
  their reset values are zero placeholders.
- `policy/value`, `policy/log_probability`, `policy/return` and
  `policy/advantage` correspond to the observation/action at t.

The final row preserves the final observation and incoming reward diagnostics.
Its action, transition-reward and policy columns are blank. This format can be
loaded with `pandas.read_csv()` for numerical inspection without a model checkpoint.

## Observations, actions and units

The active simulation and agent use SI units:

| State variable | Meaning | Units / channels |
|----------------|---------|------------------|
| `filter/rp_pitch` | Filtered body pitch | rad, 1 |
| `filter/rp_roll` | Filtered body roll | rad, 1 |
| `simul/rp_pitch` | Ground-truth body pitch, for diagnostics | rad, 1 |
| `sens/left_wheel_vel`, `sens/right_wheel_vel` | Wheel angular velocities | rad/s, 1 each |
| `sens/head_pitch`, `sens/head_turn` | Head joint angles | rad, 1 each |
| `sens/head_pitch_vel`, `sens/head_turn_vel` | Head joint angular velocities | rad/s, 1 each |
| `sens/gyro` | Gyroscope | rad/s, 3 |
| `sens/acc` | Accelerometer | m/s², 3 |
| `env/obs_time` | Episode time | s, 1 |
| `target/pos` | Current position reference | m, 1 |
| `derived/pos` | Wheel odometry | m, 1 |
| `target/vel` | Desired signed forward velocity | m/s, 1 |
| `derived/vel` | Signed forward velocity from wheel odometry | m/s, 1 |
| `target/yaw_rate` | Desired body-Z angular rate, positive left | rad/s, 1 |
| `derived/yaw_rate` | Measured body-Z angular rate from gyro | rad/s, 1 |
| `target/camera_pitch_world` | Desired optical-axis elevation above horizon | rad, 1 |
| `target/head_yaw_neck` | Desired neck-relative yaw | rad, 1 |
| `derived/camera_pitch_world` | Camera elevation from estimated body attitude + joints | rad, 1 |
| `simul/camera_pitch_world` | Actual camera elevation, scoring/diagnostics only | rad, 1 |

`policy.inputs` lists eight common sensor/time fields (12 scalar channels).
`get_policy_inputs()` appends the selected task's two fields: velocity or position
tracking uses 14 scalar input channels, while `none` uses 12. Head tracking adds
six channels (two targets, estimated camera elevation, roll, two joint rates),
giving 20 channels. Yaw-rate tracking adds its target and measured rate, giving
22 channels. Curriculum policies append position reference and odometry, giving
**24 default input channels**. Simulation truth is not added to the NN.
Unused target/state fields remain available in environment observations and plots.
Each policy input has a learned linear encoder. Their outputs feed
a shared 64-unit GRU, layer normalization, action heads and a value head.
Categorical and tanh-Gaussian heads are supported, including mixed configurations.

Default policy actions:
- `act/accelerate_both_wheels`: discrete wheel accelerations in rad/s².
- `act/wheel_vel_diff`: discrete right-minus-left wheel velocity targets in rad/s.
- `act/head_pitch_vel`: discrete head pitch velocities in rad/s.
- `act/head_turn_vel`: discrete neck-relative yaw velocities in rad/s.

Individual wheel velocity commands are also supported by the environment.
Differential steering requires the common acceleration head and cannot be mixed
with individual wheel commands or the legacy unimplemented `accelerate_yaw_turn`.

MuJoCo `intvelocity` actuators integrate commanded velocity into a position
setpoint. Head velocity commands are limited by `env.max_head_vel` (default ±1 rad/s).
Integrated position setpoints and joint bounds use −28°/+50° pitch and ±40° yaw.
Wheel acceleration commands are converted
to velocity commands using the measured wheel velocity and `env.step_time`.

Odometry velocity is mean wheel angular velocity times the nominal wheel radius
(0.05 m), in m/s. Integrating it gives signed travel distance. Neither estimate
corrects for slip or yaw.

## Rewards and episode limits

`env.arena_half_size` controls the rendered floor extent (default 20 m, giving a
40×40 m visual arena). The MuJoCo plane's collision surface is infinite: the robot
cannot fall off its visual edge. Changing this value does not add a position
boundary or change the tracking objective.

Rewards are computed from the newly observed state after each action:

```text
balancing_terms = (
    step_scale
    + fell_scale * fell
    + pitch_reward  # quadratic outside the deadband; null selects legacy linear shaping
)
wheel_penalty = wheel_vel_scale * mean(abs(wheel_velocities))

velocity mode: tracking_terms = velocity_scale * abs(current_vel - target_vel)
position mode: tracking_terms = position_scale * abs(current_pos - target_pos)
                               + wheel_penalty
none mode:     tracking_terms = wheel_penalty
combined mode: tracking_terms = position_scale * abs(current_pos - target_pos)
                               + velocity_weight * velocity_scale * abs(current_vel - target_vel)

yaw tracking:  yaw_terms = yaw_rate_scale * abs(gyro_z - target_yaw_rate)
otherwise:     yaw_terms = 0

head tracking: head_terms = camera_pitch_scale * abs(true_camera_pitch - target_pitch)
                           + head_yaw_scale * abs(neck_yaw - target_neck_yaw)
otherwise:     head_terms = head_pitch_scale * abs(head_joint_pitch)

reward = total_scale * (balancing_terms + tracking_terms + yaw_terms + head_terms)
```

Curriculum stages zero inactive yaw/head components before this final sum.
Position and damped velocity penalties are active from the first training stage.

Scales are the corresponding `reward/*` entries in `config/rlrp.yaml`.
The position and velocity penalties are `reward/pos` and `reward/vel`; only the
selected mode's error terms are active. The default velocity coefficient is -4 per
m/s, matching approximately the former zero-speed penalty for straight travel
with a 0.05 m wheel radius. Pitch uses radians; its default coefficient preserves
the former per-degree penalty strength when linear shaping is selected. Default
deadband shaping preserves that cost at the fall-angle reference.
The default yaw-rate coefficient is −1 per rad/s of error (`reward/yaw_rate`).

Episodes terminate beyond **20° absolute body pitch** and truncate at
`env.max_episode_time` (default 20 seconds), or the legacy nominal duration budget
when `env.max_episode_steps` is set, whichever is shorter. The physics timestep is
0.002 s; `env.step_time` and the episode duration must be positive integer multiples
of it. The default fixed 0.01 s control period gives 2,000 decisions per full episode.

`env.randomize=true` (default) randomizes initial pitch (standard deviation 2°), body/head
masses and wheel diameter. `env.random_scale` controls relative physical parameter
variation. Initial wheel velocity is not randomized.

Older checkpoints saw degree-valued pitch and incorrectly scaled odometry;
retrain them for the corrected observations and reward timing.
The CAD-derived model also changes body/head proportions, neck-pitch sign and
head action/input fields. The current camera-tracking setup needs a fresh policy.

## Checks

```bash
uv run python -m unittest discover -s tests -v
```

These checks cover units, current-state rewards, fixed targets, reset behavior,
trajectory interpolation and timing, unequal episode lengths, reward/return
alignment and gradient flow (including a detached policy baseline). They also
check sensor-packet replay through shared State, acquisition/reward side effects,
explicit recording, and measurement-buffer ownership.
Tracking checks cover task-specific policy inputs, live batched velocity commands,
disabled tracking, and the absence of a conflicting motion penalty in velocity mode.
Head checks compare shared rotations with MuJoCo, test neck-yaw coupling, ensure
only the NN commands actuators, and verify head-reference timing and reward inputs.
Yaw checks cover wheel mixing and saturation, physical turning direction, gyro-based
rewards, live batched references, CSV alignment, and gradients into the new head.
Curriculum checks cover neutral actions and excluded gradients/rewards, promotion
gates, head activation, stage-aware recovery, and separate MLflow child runs.
Continuous-policy checks cover bounds, transformed log densities, score-function
gradients, exploration initialization, mixed heads and paired validation.
CUDA integration checks run when PyTorch detects a CUDA device and otherwise
report skips. They cover rollout/backpropagation, transfers, RNG isolation,
checkpoint migration and rollback on the accelerator.

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
| `src/sim/train_agent.py` | Rollouts, actor–critic training, evaluation |
| `src/sim/envs/rp_env.py` | MuJoCo model and Gymnasium environment |
| `src/sim/utils.py` | Environment creation, batching, episode buffers and returns |
| `src/sim/plot_utils.py` | Episode plots |
| `src/nn_ctrl/nns.py` | Recurrent neural-network agent |
| `src/riktigpatric/patrick.py` | Shared state/action definitions, filter and odometry state |
| `src/riktigpatric/trajectory.py` | Backend-independent position waypoint interpolation |
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
# Default: balance at zero forward velocity, 16 synchronously stepped environments
uv run python -m sim.train_agent

# One environment, same training loop
uv run python -m sim.train_agent env.n_parallel=1

# Short run without rendering
uv run python -m sim.train_agent env.n_parallel=2 \
  env.max_episode_steps=20 train.max_iterations=2 logging.plot_freq=0
```

The Hydra configuration is located automatically relative to the script.
`HYDRA_CONFIG_DIR` can override its location. `--cfg job` prints the effective
configuration, and Hydra command-line overrides select individual parameters.

Training runs until interrupted unless `train.max_iterations` is set.
`seed` initializes PyTorch/NumPy and provides the base seed for episode resets.
Each iteration collects one episode per environment, with completed episodes
masked out while the remaining environments finish.

### Tracking tasks

The default task is **zero forward velocity**: balance and stop moving, without
requiring a return to the episode's starting position. Select the objective with
`env.tracking_mode`:

| Mode | Policy tracking inputs | Tracking reward |
|------|------------------------|-----------------|
| `velocity` (default) | `target/vel`, `derived/vel` | Absolute velocity error in m/s |
| `position` | `target/pos`, `derived/pos` | Absolute position error in meters |
| `none` | No tracking inputs | No position/velocity-reference penalty |

All modes retain the balancing and head-pitch terms. Position and `none` modes
also retain the configured absolute wheel-speed penalty. Velocity mode disables
that penalty, since penalizing motion itself would oppose a nonzero command.

```bash
# Zero-velocity balancing (the default)
uv run python -m sim.train_agent

# Zero-position holding: also return toward the episode's starting position
uv run python -m sim.train_agent --config-name position_hold

# Disable setpoint tracking; retain the other balancing/motion penalties
uv run python -m sim.train_agent env.tracking_mode=none

# Request 0.1 m/s forward in all environments and evaluation
uv run python -m sim.train_agent env.target_vel=0.1

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
uv run python -m sim.train_agent env.tracking_mode=position env.target_pos=0.2

# Different fixed targets for each training environment
uv run python -m sim.train_agent env.tracking_mode=position env.n_parallel=3 \
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
    -> desired forward velocity in m/s
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
controller are not implemented here. Turning would need a separate yaw-rate or
heading command.

Changing tracking mode changes the policy input fields, so train a matching
policy. Restoring a model with incompatible inputs raises an error.

### Time-varying position trajectories

Trajectories are lists of `[time_seconds, position_meters]` waypoints. They start
at time zero, interpolate linearly between waypoints and hold the final position.
Times must strictly increase; a single waypoint gives a fixed target.

```bash
# Two environments following different trajectories; evaluation follows the first
uv run python -m sim.train_agent --config-name trajectory_example

# One trajectory shared by all training environments and evaluation
uv run python -m sim.train_agent \
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

### Plots, videos and checkpoints

- `logging.plot_freq`: evaluate and save plots/videos every N iterations,
  including iteration zero; `0` disables evaluation rendering.
- Plots and numeric CSV episode traces go to `plots/`; videos go to `video/`,
  relative to the run's working directory. Directories are created automatically.
- Each evaluation saves and, with MLflow enabled, uploads the matching PNG, CSV
  and MP4. These describe the same evaluation episode, not every training rollout.
- Tracking plots follow the selected mode: position, velocity, or no tracking
  plot for `none`. Inactive reward terms are omitted from plots; CSV traces keep
  all observation and reward fields.
- For headless rendering on systems with EGL support, prefix the command with
  `MUJOCO_GL=egl MPLBACKEND=Agg`.
- MLflow is disabled by default. Enable it with
  `logging.mlflow.enabled=true` and set `MLFLOW_TRACKING_URI`; optional credentials
  can be supplied in the environment or a local `.env` file.
- With MLflow enabled, metrics are logged every `logging.mlflow.push_freq`
  iterations and models every `logging.save_freq` iterations. Without MLflow,
  model checkpoints are not saved.
- Training metric names are grouped by prefix: `losses/{policy,value,total}`,
  `returns/{mean,min,max,std}`, and `episodes/length/{mean,min,max}`.
- `policy.restore_id` accepts an MLflow **logged model ID** to restore an agent.
  Optimizer state and iteration count start fresh.

### Reading an episode trace

`plots/episode_iter_XXXX.csv` contains all observations/targets, actions, rewards,
log-probabilities, value predictions, returns and advantages. Vector observations
have separate columns such as `sens/gyro[0]`, `sens/gyro[1]`, `sens/gyro[2]`.

There are **T+1 rows for T actions**. At row t:

- `env/obs_time` and the observation/target columns describe the state at t.
- `act/*` is the outgoing action taken from that state.
- `transition/reward` is the reward received **after** that action, at t+dt.
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
| `simul/rp_pitch` | Ground-truth body pitch, for diagnostics | rad, 1 |
| `sens/left_wheel_vel`, `sens/right_wheel_vel` | Wheel angular velocities | rad/s, 1 each |
| `sens/head_pitch`, `sens/head_turn` | Head joint angles | rad, 1 each |
| `sens/gyro` | Gyroscope | rad/s, 3 |
| `sens/acc` | Accelerometer | m/s², 3 |
| `env/obs_time` | Episode time | s, 1 |
| `target/pos` | Current position reference | m, 1 |
| `derived/pos` | Wheel odometry | m, 1 |
| `target/vel` | Desired signed forward velocity | m/s, 1 |
| `derived/vel` | Signed forward velocity from wheel odometry | m/s, 1 |

`policy.inputs` lists eight common sensor/time fields (12 scalar channels).
`get_policy_inputs()` appends the selected task's two fields: velocity or position
tracking therefore uses **14 scalar input channels**, while `none` uses 12.
Unused target/state fields remain available in environment observations and plots.
Each policy input has a learned linear encoder. Their outputs feed
a shared 64-unit GRU, layer normalization, categorical action heads and a value
head. Continuous action distributions are not implemented.

Default policy actions:
- `act/accelerate_both_wheels`: discrete wheel accelerations in rad/s².
- `act/head_pitch_vel`: discrete head pitch velocities in rad/s.

Head turning and individual wheel velocity commands are supported by the
environment but are not selected in the default policy configuration.

MuJoCo `intvelocity` actuators integrate commanded velocity into a position
setpoint. Head velocity commands are limited to ±1 rad/s, with integrated
position setpoints limited to ±1 rad. Wheel acceleration commands are converted
to velocity commands using the measured wheel velocity and `env.step_time`.

Odometry velocity is mean wheel angular velocity times the nominal wheel radius
(0.05 m), in m/s. Integrating it gives signed travel distance. Neither estimate
corrects for slip or yaw.

## Rewards and episode limits

Rewards are computed from the newly observed state after each action:

```text
balancing_terms = (
    step_scale
    + fell_scale * fell
    + pitch_scale * abs(pitch)
    + head_pitch_scale * abs(head_pitch)
)
wheel_penalty = wheel_vel_scale * mean(abs(wheel_velocities))

velocity mode: tracking_terms = velocity_scale * abs(current_vel - target_vel)
position mode: tracking_terms = position_scale * abs(current_pos - target_pos)
                               + wheel_penalty
none mode:     tracking_terms = wheel_penalty

reward = total_scale * (balancing_terms + tracking_terms)
```

Scales are the corresponding `reward/*` entries in `config/rlrp.yaml`.
The position and velocity penalties are `reward/pos` and `reward/vel`; only the
selected mode's error term is active. The default velocity coefficient is -4 per
m/s, matching approximately the former zero-speed penalty for straight travel
with a 0.05 m wheel radius. Pitch uses radians; its default coefficient
preserves approximately the former per-degree penalty strength.

Episodes terminate beyond **20° absolute body pitch** and truncate at 20 seconds
or `env.max_episode_steps` (default 2000), whichever limit is reached first.
The physics timestep is 0.002 s; `env.step_time` must be a positive integer
multiple of it (default 0.01 s).

`env.randomize=true` randomizes initial pitch (standard deviation 2°), body/head
masses and wheel diameter. `env.random_scale` controls relative physical parameter
variation. Initial wheel velocity is not randomized.

Older checkpoints saw degree-valued pitch and incorrectly scaled odometry;
retrain them for the corrected observations and reward timing.

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

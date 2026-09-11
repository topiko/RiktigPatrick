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

## Maintained path

The active entry point is **`src/sim/train_agent.py`**. Single-environment and
batched training use this same code path.

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
# Default: 16 synchronously stepped environments
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

### Position targets

Positions are in meters along the robot's fore/aft travel direction, relative
to the episode origin. The default target is zero (stay near the starting point).

```bash
# Same fixed target for all training environments and evaluation
uv run python -m sim.train_agent env.target_pos=0.2

# Different fixed targets for each training environment
uv run python -m sim.train_agent env.n_parallel=3 \
  'train.target_positions=[-0.2,0.0,0.2]'
```

`train.target_positions` must contain exactly one finite value per environment.
It overrides `env.target_pos` for training; evaluation uses `env.target_pos`.
The rollout's `add_targets()` helper updates both environment state and batched
policy observations. Targets persist across steps and resets; odometry resets
to zero at each episode start.

### Time-varying position trajectories

Trajectories are lists of `[time_seconds, position_meters]` waypoints. They start
at time zero, interpolate linearly between waypoints and hold the final position.
Times must strictly increase; a single waypoint gives a fixed target.

```bash
# Two environments following different trajectories; evaluation follows the first
uv run python -m sim.train_agent --config-name trajectory_example

# One trajectory shared by all training environments and evaluation
uv run python -m sim.train_agent \
  'env.target_trajectory=[[0,0],[2,0.2],[4,0]]'
```

`train.target_trajectories` accepts one waypoint list per training environment.
Use it instead of `train.target_positions`; supplying both is an error.
Either training override replaces the environment's default target settings.
Evaluation uses `env.target_trajectory` when set, otherwise `env.target_pos`.

The shared `State` samples its trajectory from observation time, so the same
target logic is usable with simulated or hardware observations. On each step,
the policy sees the target at time *t* and receives a reward for tracking the
target at *t + dt*. Resetting restarts the trajectory at zero. Replacing a
trajectory mid-episode samples it at the current episode time; setting a fixed
target disables it. The batched `add_targets()` helper handles either target form.

This supplies position references; velocity-target inputs and a policy proven to
track trajectories still require further work.

### Plots, videos and checkpoints

- `logging.plot_freq`: evaluate and save plots/videos every N iterations,
  including iteration zero; `0` disables evaluation rendering.
- Plots go to `plots/`, videos to `video/`, relative to the run's working directory.
  Output directories are created automatically.
- For headless rendering on systems with EGL support, prefix the command with
  `MUJOCO_GL=egl MPLBACKEND=Agg`.
- MLflow is disabled by default. Enable it with
  `logging.mlflow.enabled=true` and set `MLFLOW_TRACKING_URI`; optional credentials
  can be supplied in the environment or a local `.env` file.
- With MLflow enabled, metrics are logged every `logging.mlflow.push_freq`
  iterations and models every `logging.save_freq` iterations. Without MLflow,
  model checkpoints are not saved.
- `policy.restore_id` accepts an MLflow **logged model ID** to restore an agent.
  Optimizer state and iteration count start fresh.

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

The default policy uses all of the above except ground-truth pitch: **14 scalar
input channels**. Each input has a learned linear encoder. Their outputs feed
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

Odometry integrates mean wheel angular velocity times the nominal wheel radius
(0.05 m). It estimates signed travel distance, without correcting for slip or yaw.

## Rewards and episode limits

Rewards are computed from the newly observed state after each action:

```text
reward = total_scale * (
    step_scale
    + fell_scale * fell
    + pitch_scale * abs(pitch)
    + wheel_vel_scale * mean(abs(wheel_velocities))
    + head_pitch_scale * abs(head_pitch)
    + position_scale * abs(current_pos - target_pos)
)
```

Scales are the corresponding `reward/*` entries in `config/rlrp.yaml`.
The position penalty is `reward/pos`. Pitch uses radians; its default coefficient
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
alignment and gradient flow (including a detached policy baseline).

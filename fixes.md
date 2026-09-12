# Simulation cleanup: what was wrong and how it was fixed

This overview explains the changes in commit `05a6261`, covering the maintained
training path, shared robot state, and project reorganization. Sections 16–17 cover
the subsequent state-update refactor and velocity-control task. Examples are pseudocode
rather than literal implementations; historical method names describe the code
at the time of each fix.

The most consequential bugs were incorrect reward alignment, stale-state
rewards, inconsistent units, and gradients flowing through the policy's value
baseline. Together, these could make training optimize something different from
the intended balancing and position-tracking task.

## 1. Episode buffers dropped the first reward and duplicated the last

**Where:** `src/sim/utils.py` — `EpisodeBuffer`; `src/sim/train_agent.py` — `rollout`.

The buffer treated the first reward passed to `add_step()` as an initial dummy
reward. However, the caller had already executed `env.step(action)`: this was
the real reward for the first action. The final reward was then appended again
when finishing the episode.

The intended chronology is exactly:

```text
    reset -> (observation_0, no transition reward)
    action_0 -> (observation_1, reward_1)
    action_1 -> (observation_2, reward_2)
    action_2 -> (observation_3, reward_3, done)
```

The key detail is **when the buffer is called**. The original rollout did this:

```text
    observation_0 = env.reset()
    action_0 = agent(observation_0)
    observation_1, reward_1 = env.step(action_0)
    buffer.add_step(observation_0, action_0, reward_t=reward_1)
```

Despite its name, the `reward_t` argument contains `reward_(t+1)` in this
chronological convention: the reward received after taking `action_t`. No dummy
reward from `reset()` is ever passed to `add_step()`.

```text
Before:
    add_step(observation, action, reward):
        if buffer already contains observations:
            append reward
        append observation and action

    finish(final_observation, final_reward):
        append final_observation
        append final_reward

    Actual transition rewards: [1, 2, 4]
    Stored rewards:            [2, 4, 4]

After:
    add_step(observation, action, reward):
        append observation, action, and reward

    finish(final_observation):
        append final_observation only

    Stored rewards:            [1, 2, 4]
```

**Why it matters:** returns and policy gradients must associate each action with
its own subsequent reward. An episode with `T` actions now has exactly `T`
rewards and `T + 1` observations. Unequal episode lengths remain padded and
masked during batched training.

The storage arrays therefore have these meanings:

```text
    observations = [observation_0, observation_1, observation_2, observation_3]
    actions      = [action_0,      action_1,      action_2]
    rewards      = [reward_1,      reward_2,      reward_3]

    return_for_action_0 = reward_1 + gamma * reward_2 + gamma^2 * reward_3
```

Skipping the first reward would be valid under a different caller contract:
recording `(observation_t, incoming_reward_t, action_t)` **before** executing the
action, with a dummy incoming reward at time zero. That was not how this rollout
called the buffer.

**Double-check:** the original rollout and buffer from the parent of `05a6261`
were executed against the same deterministic three-transition environment as
the revised rollout. `reset()` returned no reward; successive `step()` calls
returned `1`, `2`, and `4`. Both versions stored observations `[0, 1, 2, 3]`
and actions `[0, 1, 2]`, but the original stored rewards `[2, 4, 4]` and the
revised version stored `[1, 2, 4]`. This confirms a caller/buffer contract
mismatch rather than merely a different reward-indexing convention.

## 2. Rewards and filtered pitch described an older state

**Where:** `src/sim/envs/rp_env.py` — `_update_obs`, `_get_reward`, `step`;
`src/riktigpatric/patrick.py` — `State.step`.

The environment read new sensors but computed rewards using the previously
stored observations. It also published filtered pitch before advancing the
filter. Consequently, the observation returned after an action combined data
from different times.

```text
Before:
    advance physics to t + dt
    read new sensors
    reward = reward_from(old stored state)
    publish sensors with old filtered pitch
    advance filter and odometry

After:
    advance physics to t + dt
    store new sensor measurements
    advance filter and odometry
    update the target for t + dt
    reward = reward_from(updated state and target)
    record and return that state and reward
```

**Why it matters:** the agent now receives feedback about the result of the
action it just took. With trajectories, the observation at `t` contains the
reference at `t`, and the next reward measures tracking error at `t + dt`.

## 3. The policy loss could train its own value baseline

**Where:** `src/sim/utils.py` — `get_advantages`.

The policy uses advantage estimates to weight sampled actions. The value
estimate is a baseline, but it remained attached to the gradient computation.

```text
Before:
    advantage = return - value_prediction
    policy_loss = -log_probability * advantage
    backward(policy_loss)  # Also changes the baseline through this expression

After:
    advantage = return - stop_gradient(value_prediction)
    policy_loss = -log_probability * advantage
    value_loss = (value_prediction - return)^2
    backward(policy_loss + value_loss)
```

**Why it matters:** the policy should improve action probabilities, rather than
reduce its loss by changing the baseline. The value loss still trains the
critic. The shared network backbone can still receive both policy and value
gradients; only the unwanted path through the baseline is detached.

## 4. Position was accumulated wheel rotation, not distance

**Where:** `src/riktigpatric/patrick.py` — `State.step`.

Wheel sensors return angular velocity in rad/s. Integrating this alone gives
wheel rotation, whereas `derived/pos` was intended to represent meters.

```text
Before:
    position += mean(left_wheel_omega, right_wheel_omega) * dt

After:
    position += mean(left_wheel_omega, right_wheel_omega) * wheel_radius * dt
```

For example, `2 rad/s` for one second with a `0.05 m` wheel radius corresponds
to `0.1 m` of nominal travel, not `2 m`.

The radius is now an explicit input to the shared `State`. The simulation uses
the nominal radius, `0.05 m`. This remains wheel odometry: it does not correct
for slipping, yaw, or differences between nominal and randomized wheel size.

## 5. Pitch observations contradicted the documented SI units

**Where:** `src/riktigpatric/patrick.py` — `State.euler`;
`src/sim/envs/rp_env.py`; `config/rlrp.yaml`.

The README and policy documentation described radians, but both filtered pitch
and diagnostic ground-truth pitch were supplied in degrees.

```text
Before:
    filtered_pitch = filter.euler_in_degrees
    true_pitch = quaternion_to_pitch_in_radians * 180 / pi

After:
    filtered_pitch = degrees_to_radians(filter.euler_in_degrees)
    true_pitch = quaternion_to_pitch_in_radians
    fallen = abs(true_pitch) > degrees_to_radians(20)
```

The Mahony filter's existing degree-valued public interface was preserved;
conversion happens at the shared state boundary.

The pitch reward coefficient was also converted:

```text
    -0.1 per degree ≈ -5.73 per radian
```

This preserves approximately the former pitch-penalty strength rather than
silently weakening it by a factor of about 57. Old policies still need
retraining for the corrected inputs and reward timing.

## 6. Fixed targets were disconnected from environment state

**Where:** `src/sim/train_agent.py` — `add_targets`;
`src/sim/utils.py` — `SingleEnvWrapper`;
`src/riktigpatric/patrick.py` and `src/sim/envs/rp_env.py` — target properties.

Several problems overlapped:

- The target-setting call in the rollout was commented out.
- The helper wrote `env.target_pos`, but observations read `state.targets`.
- `State.step()` unconditionally overwrote the target with zero.
- The injected observation had shape `(1,)` instead of `(number_of_envs, 1)`.
- Gymnasium distributes lists/tuples passed to `set_attr`; an ndarray would
  instead be broadcast as one object to every environment.

```text
Before:
    env.set_attr("target_pos", array_of_targets)  # Wrong distribution semantics
    obs["target/pos"] = [0]                     # Wrong batch shape
    ...
    State.step():
        state.targets["target/pos"] = [0]        # Erases the requested target

After:
    validate one finite, float32-representable target per environment
    env.set_attr("target_pos", list_of_targets)
    # Each environment's property updates its shared State.
    obs["target/pos"] = targets.reshape(number_of_envs, 1)
    # Fixed targets survive steps and resets.
```

The single-environment wrapper now offers matching `set_attr()` behavior and
forwards reset options. The same rollout code works with a batch of one or many.

## 7. Supplying a position target did not reward tracking it

**Where:** `src/sim/envs/rp_env.py` — `_get_reward`; `config/rlrp.yaml`.

The target was a policy input, but the reward only covered survival, pitch,
wheel velocity, and head pitch. There was no direct incentive to reach a
requested position.

```text
Before:
    reward = balancing_and_motion_terms

After:
    position_reward = position_scale * abs(current_position - target_position)
    reward = balancing_and_motion_terms + position_reward
```

`reward/pos` defaults to `-1.0`, penalizing absolute position error in meters.
This establishes a tracking objective; the coefficient may still need tuning.

## 8. Observation snapshots could change after being returned

**Where:** `src/sim/envs/rp_env.py` — `_get_obs` and `truncated`.

Observation dictionaries exposed arrays owned by mutable state. In particular,
odometry was updated in place, so a previously returned observation could change
when the next action was executed. This could corrupt the observation/action
pair saved by the single-environment rollout.

```text
Before:
    obs["derived/pos"] = state.position_array
    state.position_array += movement  # Also changes the old observation

After:
    obs[key] = copy_as_float32(state_value)
    state.position_array += movement  # Old observation remains a snapshot
```

Returned arrays now match the observation space's float32 dtype. Arbitrary
±100 bounds were removed from observation channels that could exceed them.
The truncation flag is now a scalar boolean rather than a one-element array.

## 9. Floating-point comparisons could add an extra physics step

**Where:** `src/sim/envs/rp_env.py` — initialization and `step`.

Repeated floating-point addition can leave the simulated time fractionally
below a desired endpoint. The old loop could then execute one extra substep.

```text
Before:
    end_time = current_time + control_dt
    while current_time < end_time:
        physics.step()

After:
    substeps = round(control_dt / physics_dt)
    validate that control_dt is a positive multiple of physics_dt
    repeat substeps times:
        physics.step()
```

At the defaults, each `0.01 s` control step now consistently executes five
`0.002 s` physics steps. This keeps action integration, timestamps, and target
sampling aligned.

## 10. Head actuator position limits were mistaken for velocity limits

**Where:** `src/sim/envs/rp_env.py` — `MujocoRP` and `_apply_action`.

MuJoCo's `intvelocity` actuator integrates a velocity command into a position
setpoint. Its `actrange` bounds that integrated position, not the input velocity.
The old code labelled `actrange` as rad/s and lacked an explicit head control
range.

```text
Before:
    head.actrange = [-1, 1]    # Incorrectly described as a velocity limit

After:
    head.ctrlrange = [-1, 1]   # Velocity command limit, rad/s
    head.ctrllimited = true
    head.actrange = [-1, 1]    # Integrated position setpoint limit, rad
```

Wheel acceleration handling was also consolidated into one helper:

```text
    acceleration = clamp(requested_acceleration, acceleration_limits)
    commanded_velocity = clamp(measured_velocity + acceleration * dt,
                               velocity_limits)
```

This removes duplicated left/right/both-wheel code and enforces the configured
acceleration bound.

## 11. Evaluation changed subsequent training randomness

**Where:** `src/sim/train_agent.py` — `main` and `evaluate_and_plot`.

The configured base seed was not being applied consistently. Evaluation also
sampled from the same PyTorch random-number stream used for training, so changing
the plot frequency changed later training actions.

```text
Before:
    training_actions = sample(global_rng)
    evaluation_actions = sample(global_rng)
    next_training_actions = sample(global_rng)  # Evaluation advanced the stream

After:
    initialize training RNGs from config.seed
    training_reset_seed = config.seed + iteration

    save training RNG state
    seed evaluation RNG separately
    run evaluation
    restore training RNG state
```

Two otherwise identical short CLI runs, with evaluation enabled and disabled,
produced identical training metrics after this change. This checks isolation
within that setup, not universal reproducibility across machines.

## 12. Startup, output, and lifecycle handling needed repair

**Where:** `src/sim/train_agent.py`, `src/sim/plot_utils.py`,
`src/sim/utils.py`, `pyproject.toml`, and `README.md`.

### Configuration and dependencies

Hydra looked for `config/` relative to the training script, although the actual
directory lives at the repository root.

```text
Before: config_directory = "config"
After:  config_directory = repository_root_relative_to_script / "config"
```

The environment-variable override remains available. Setup instructions now
use the project's Python ≥3.12 requirement and declared dependencies, including
`python-dotenv` and minimum Gymnasium/MLflow versions needed by the active APIs.

Other stale README descriptions were corrected: the active agent already used
a GRU with per-input encoders and 14 scalar input channels, rather than the
documented older architecture. The documented default actions and randomization
now match the code. Unused YAML options, such as a separate `value_lr` and
`entropy_scale`, were removed so they no longer suggest behavior the active
training loop does not implement.

### Output and controlled runs

```text
Before:
    create video environment unconditionally
    train forever
    save_plot(path)  # Assumes its parent directory exists

After:
    create video environment only if plot_freq > 0
    train until max_iterations, or until interrupted if unset
    create plot parent directory before saving
```

`env.max_episode_steps` is configurable through the factory. Environment
registration is performed once rather than repeatedly replacing the same ID.

### Cleanup and model terminology

Environment/wrapper cleanup is registered with an `ExitStack`. MLflow runs use
their context manager so exiting the training scope closes the run and can
record failure on an exception. The restore message now correctly calls
`policy.restore_id` a **logged model ID**, rather than a run ID.

## 13. Optional state-history recording was broken

**Where:** `src/riktigpatric/patrick.py` — history and serialization methods.

Recording referenced a nonexistent derived-state attribute, started array
concatenation with `None`, and called `get_state_arr()` with unsupported
arguments. Recording also needed to occur after the current reward was available.

```text
Before:
    state_dict.update(nonexistent_derived_state)
    array = concatenate(None, first_values)
    indices = get_state_arr(unsupported_arguments)

After:
    state_dict = observations + actions + rewards + derived_values + targets
    array = concatenate(list_of_value_arrays)
    indices = get_state_arr().indices
    record after updating the current state and reward
```

Stored history now includes current timestamps, rewards, odometry, and targets.
Type annotations for episode/state dictionaries and network module keys were
also corrected to match their actual contents.

## 14. New capability: shared, batched position trajectories

**Where:** `src/riktigpatric/trajectory.py`, shared `State`, `GymRP`,
`add_targets`, and `config/trajectory_example.yaml`.

Previously there was only a position-target placeholder. The new trajectory
object interpolates time/position waypoints without depending on MuJoCo or
Gymnasium.

```text
    waypoints = [(0 seconds, 0 meters),
                 (2 seconds, 0.2 meters),
                 (4 seconds, 0 meters)]

    target_at(1 second) = 0.1 meters
    target_at(3 seconds) = 0.1 meters
    target_at(5 seconds) = 0 meters  # Hold final position

    State.step():
        target_position = trajectory.position_at(observation_time)

    State.reset():
        restart trajectory at time zero
```

Each environment can receive a different trajectory in a single batch update.
All waypoint lists are validated before any environment is changed. Times must
start at zero and increase strictly; values must be finite and positions must
fit the observation dtype.

Replacing a trajectory samples it at the current episode time. Setting a fixed
target disables the trajectory. The helper refreshes the policy's current
target observation immediately after either change.

The important architectural choice is that **shared robot state owns target
evaluation**. Simulation supplies timestamps and sensor measurements today;
hardware can supply them through the same state contract later.

## 15. Legacy code obscured the active path and leaked into imports

**Where:** `archive/`, `pyproject.toml`, and `src/relay/conversions.py`.

Old trainers, scripts, firmware resources, experiments, and CAD files were
mixed with the maintained simulation. They were moved into `archive/`, generally
preserving their old relative paths. Package discovery is restricted to the
relevant source packages, and archived code is excluded from normal lint/type
discovery.

One dependency required more than moving files: the shared wire-format module
imported its packet decoder from a keyboard CLI. Importing that CLI performed
hostname lookup even when only the simulator or shared state was needed.

```text
Before:
    robot state -> wire conversions -> keyboard CLI -> hostname lookup

After:
    robot state -> wire conversions, including packet decoder
    archived keyboard CLI -> shared packet decoder
```

The robot definitions, hardware head/servo code, backend interfaces, and wire
protocol remain visible in `src/`. The reorganization preserves the goal of
using the same robot/control model with either hardware or simulation.

## 16. Separate acquisition, state processing, rewards, and recording

The timing fix in section 2 originally left a structural problem: `_update_obs()`
still read sensors, advanced shared state, calculated rewards, and triggered
recording. Its broad responsibilities made the required order hard to see.

The follow-up refactor makes that sequence explicit in `GymRP.step()`:

```text
Before this refactor (timing already fixed):
    advance physics
    _update_obs():
        state.update_obs(read_sensors())
        state.step()                       # Filter, odometry and targets
        calculate and attach rewards
        state.update_rewards(rewards)      # Also records history

After:
    advance physics
    measurements = _read_sensors()          # Acquisition only
    state.update(measurements)              # One complete shared-state update
    rewards = _calculate_rewards(state, terminated)
    state.record_transition(action_time, action, rewards)
    return _get_obs(rewards)                 # Export only
```

There is still **one shared State object**. Its filter and odometry stay inside
it; callers pass in a measurement packet rather than moving separate state
objects between the robot and simulator. Packet arrays are copied so a hardware
reader can safely reuse its buffers.

Specific boundaries now enforced by the API and tests:

- `_read_sensors()` reads measurements without modifying shared state. It does
  not supply filtered pitch or rewards.
- `State.update(measurements)` replaces the old `update_obs()`/`step()` pair.
  It advances filtering, odometry and targets together and rejects non-increasing
  timestamps before changing state.
- `_calculate_rewards(state, terminated=...)` explicitly reads the supplied
  processed state; it neither acquires sensors nor advances the filter.
- `State.snapshot()` exports copied sensor/processed-state arrays. Available
  fields need not include simulation-only true pitch or reward channels.
- The Gymnasium observation exporter attaches reward diagnostics, preserving the
  training/plotting interface while keeping rewards out of sensor storage.
- `record_transition()` explicitly captures the incoming action, current state,
  and resulting reward. It does not mutate the caller's action or advance state.
- `State.reset(initial_measurements)` initializes an observation without a filter
  integration interval, action, or transition reward. It clears previous history.

Additional tests replay sensor packets into an independent shared State without
ground-truth pitch, verify that reads/reward calculation have no state-update
side effects, check explicit recording and action ownership, and reject repeated
timestamps without partially changing state.

The full suite passes **16 tests** after this refactor. A direct comparison with
the pre-refactor implementation produced exactly matching observations, rewards,
termination flags and histories over **80 transitions**, including randomized
resets. A batched trajectory training run also completed with plots and videos.

## 17. Velocity commands as the default low-level control objective

The default task now balances at `target_vel = 0 m/s`. Position tracking remains
available through `env.tracking_mode=position` (or `--config-name position_hold`),
and `env.tracking_mode=none` disables both reference-tracking rewards.

Shared State exposes `target/vel` and `derived/vel`. The velocity estimate is
mean wheel angular velocity times nominal wheel radius. A velocity setpoint can
be changed without resetting the filter or episode, including different commands
for each simulation via `train.target_velocities` or the batched target helper.

```text
position controller or RC -> desired velocity -> balancing policy -> actuators

velocity_reward = velocity_scale * abs(measured_velocity - target_velocity)
```

Velocity mode disables the old absolute wheel-speed penalty, which would oppose
a nonzero command. Position mode retains it; `none` also retains this general
motion penalty while removing setpoint tracking. The trainer supplies only the
selected task's tracking inputs to the policy, so a velocity policy does not
receive absolute position by default.

The suite now contains 21 tests, including live batched command updates, task
input selection, actual NN command inputs, and reward isolation. RC velocity following needs
training with varied commands and command changes, plus the hardware integration.

## Verification and remaining work

Verification performed for the commit:

- **12 regression tests**, including units, reward alignment, detached baseline,
  snapshot behavior, history, trajectories, resets, and unequal episode lengths.
- Short single/batched CLI training runs, including trajectory configuration.
- Plot/video generation and matching training metrics with evaluation on/off.
- Local MLflow model save, successful run closure, and restored training.
- Active simulation/policy/filter/trajectory lint and type checks.
- Package-content review and preservation checks for the archived tracked files.

Remaining work is explicitly separate from these fixes:

- The real-robot adapter is still a stub; the adapter layer needs alignment with
  the current training API and SI units. Hardware interchangeability is a design
  goal, not yet an end-to-end verified feature.
- Existing hardware-code type diagnostics remain, including 20 in `RPHead` when
  checking the whole shared `patrick.py` file.
- Position/velocity references and rewards now work, but reliable learned
  command or trajectory tracking has not been demonstrated.
- Odometry is nominal-radius wheel integration, not corrected physical position.

Run the regression suite with:

```bash
uv run python -m unittest discover -s tests -v
```

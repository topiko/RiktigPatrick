# RiktigPatrick

Self-balancing robot with a turning head, trained via reinforcement learning in MuJoCo simulation, with a backend-agnostic controller that can be swapped between simulator and physical robot.

![rp](rp.jpg)

## Goal

Train a balancing robot using RL to balance upright and respond to disturbances. The controller should be:
- **Backend-agnostic**: Swappable between MuJoCo simulation and physical robot
- **Robust**: Handle initial state randomization and perturbations

## Project Structure

```
RiktigPatrick/
├── config/
│   └── rlrp.yaml          # Hydra configuration (SI units)
├── src/sim/               # RL training in MuJoCo
│   ├── train_agent.py     # Main training script (uses Hydra config)
│   ├── rl_rp.py           # Sequential RL training (legacy)
│   ├── parallel_rl_rp.py  # Parallel RL training (legacy)
│   ├── try_policy.py      # Test trained policy (legacy)
│   ├── algos.py           # REINFORCE algorithm (legacy)
│   ├── nets.py            # Neural network architectures (legacy)
│   ├── utils.py           # Environment registration
│   └── envs/rp_env.py     # MuJoCo environment (SI units)
├── src/nn_ctrl/
│   └── nns.py             # Neural network agent (SI units)
├── src/filters/           # Sensor filtering
│   ├── qutils.py          # Quaternion utilities
│   └── mahony.py          # AHRS filter
├── notes/                 # Setup notes
└── pyproject.toml         # Package config
```

## Setup

```bash
cd src/
uv venv --python 3.10 ../.venv
source ../.venv/bin/activate
uv pip install gymnasium mujoco dm-control torch numpy mlflow pandas
```

## Run Training

### New Training (Recommended - uses Hydra config, SI units)

```bash
cd src/
export PYTHONPATH="$PWD:$PYTHONPATH"

# Run training with Hydra configuration
python sim/train_agent.py
```

Configuration is in `config/rlrp.yaml`. All values use SI units.

### Legacy Training (older scripts)

```bash
cd src/
export PYTHONPATH="$PWD:$PYTHONPATH"

# Set mlflow credentials (or source .env file)
export MLFLOW_TRACKING_URI=https://ml.twohands.dev
export MLFLOW_TRACKING_USERNAME=topiko
export MLFLOW_TRACKING_PASSWORD='your-password'

# Parallel training
python sim/parallel_rl_rp.py

# Sequential training
python sim/rl_rp.py
```

Note: Legacy scripts use deprecated `sim_config.py` and may need updates.

## Test Policy

```bash
python try_policy.py --policy REINFORCE
python try_policy.py --policy pid
```

## Configuration

### New Training Configuration

Edit `config/rlrp.yaml` (all values in SI units):

| Section | Parameter | Description | Units |
|---------|-----------|-------------|-------|
| env | step_time | Simulation timestep | seconds |
| env | randomize | Enable initial state randomization | boolean |
| env | n_parallel | Number of parallel environments | - |
| env | max_wheel_vel | Maximum wheel velocity | rad/s |
| env | max_wheel_acc | Maximum wheel acceleration | rad/s² |
| train | policy_lr | Policy learning rate | - |
| policy | actions | List of action types | - |
| policy | act_map | Action space bins per action | - |
| reward | pitch_coef | Penalty coefficient for pitch deviation | - |
| reward | yaw_coef | Penalty coefficient for yaw deviation | - |

### Legacy Configuration

Edit `src/sim/config.yaml` (for legacy scripts):

| Section | Parameter | Description |
|---------|-----------|-------------|
| training | nrollouts | Number of parallel environments |
| training | max_episodes | Total training episodes |
| rl | learning_rate | Optimizer learning rate |
| rl | gamma | Discount factor |

## Reward Function

The reward encourages the robot to balance upright while minimizing energy:

```
reward = step_reward - pitch_coef * |pitch| - yaw_coef * |yaw| - action_coef * |action|
```

The episode terminates when |pitch| > 20 degrees.

## State Space

### New Training (`train_agent.py`)
Model input (4 dims, all SI units):
- `filter/rp_pitch` - Filtered pitch angle (rad)
- `sens/left_wheel_vel` - Left wheel velocity (rad/s)
- `sens/right_wheel_vel` - Right wheel velocity (rad/s)
- `env/obs_time` - Simulation time (s)

### Legacy Training
Model input (7 dims):
- Filtered pitch (from AHRS)
- Gyroscope (3-axis)
- Left wheel velocity
- Right wheel velocity
- Time (normalized)

## Units and Conversions

**The system uses SI units (International System of Units) throughout:**

### Physical Quantities
- **Angles**: radians (rad)
- **Angular velocities**: radians per second (rad/s)
- **Angular accelerations**: radians per second squared (rad/s²)
- **Time**: seconds (s)
- **Linear accelerations**: meters per second squared (m/s²)

### Observations (from MuJoCo sensors)
All observations are in SI units:
- `sens/left_wheel_vel`: rad/s (wheel angular velocity)
- `sens/right_wheel_vel`: rad/s (wheel angular velocity)
- `sens/gyro`: rad/s (gyroscope, 3-axis)
- `filter/rp_pitch`: rad (filtered pitch angle)
- `sens/acc`: m/s² (accelerometer, 3-axis)
- `env/obs_time`: s (simulation time)

### Actions (to MuJoCo actuators)
All actions are in SI units:
- `act/accelerate_both_wheels`: rad/s² (wheel acceleration)
- `act/left_wheel_vel`, `act/right_wheel_vel`: rad/s (wheel target velocity)
- `act/head_pitch_vel`, `act/head_turn_vel`: rad/s (head velocity, ±1 rad/s max)

**Note**: All actuators use **velocity control** (`intvelocity` type in MuJoCo).
Wheel accelerations are integrated to velocities before sending to actuators.

### Configuration Parameters
All parameters in `config/rlrp.yaml` use SI units:
- `max_wheel_vel: 10.0` - rad/s (maximum wheel velocity)
- `max_wheel_acc: 50.0` - rad/s² (maximum wheel acceleration)
- `step_time: 0.01` - seconds (simulation timestep)

### Normalization Scales
The `obs_scales` in config are in SI units (placeholders, need tuning):
- `filter/rp_pitch: 0.35` - rad (~±20° range)
- `sens/gyro: 10.5` - rad/s (~±600°/s range)
- `sens/left_wheel_vel: 10.0` - rad/s (max wheel velocity)
- `sens/right_wheel_vel: 10.0` - rad/s (max wheel velocity)

### Conversion Constants
For reference only (defined in `rp_env.py`, not used in main training):
- `RAD2DEG = 180/π` - Convert rad/s → deg/s (for display)
- `RAD2REV = 1/(2π)` - Convert rad/s → rev/s (for display)

**Note**: The new training code (`train_agent.py`) uses only SI units end-to-end. Legacy training scripts may use different unit conventions.

## Randomization

When `randomize: true`:
- Initial pitch: Gaussian (μ=0, σ=2°)
- Initial wheel velocity: Gaussian (μ=0, σ=1 rad/s) - both wheels same velocity

This helps the policy generalize to different starting conditions.

## Known Issues

- Training tends to plateau around ~50 return - may need hyperparameter tuning

## Architecture

The network uses **separate encoders** for policy and value (no shared weights):
- **Policy encoder**: 7 → 32 → 16 (Tanh activations)
- **Value encoder**: 7 → 64 → 32 → 16 (Tanh activations, larger capacity)
- **Policy heads**: Mean (Sigmoid) and StdDev (Softplus)
- **Value head**: 16 → 1 (linear)

This separation prevents the value function from being constrained by the policy's representation needs, which was causing the value head to output constant values with the previous shared encoder architecture.

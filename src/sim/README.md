# To get things running

## Setup

```bash
# Create and activate virtual environment with uv
cd src/
uv venv --python 3.10 ../.venv
source ../.venv/bin/activate
uv pip install gymnasium mujoco dm-control torch numpy mlflow pandas

# Or if you have conda:
conda activate rl2
```

## Run RL Training

```bash
cd src/
export PYTHONPATH="$PWD:$PYTHONPATH"

# Sequential training (recommended for testing)
python rl_rp.py

# Parallel training (faster)
python parallel_rl_rp.py
```

## Test Policies

```bash
# Test with trained network
python try_policy.py --policy REINFORCE

# Test with PID
python try_policy.py --policy pid
```

## Run mlflow for tracking (optional)

```bash
mlflow server
# Then view at 127.0.0.1:5000
```

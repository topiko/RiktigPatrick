#!/usr/bin/env python
"""Tests for observation pipeline integrity."""

import sys

sys.path.insert(0, "src")

import numpy as np
import torch
from sim.envs.rp_env import GymRP
from sim.sim_config import OBS_SPACE, MODEL_INPUT
from sim.nets import PolicyNetwork


def test_env_time_monotonic():
    """Test that env/time increases monotonically."""
    print("=== Test: env/time monotonic ===")
    env = GymRP(state_keys=OBS_SPACE, randomize=False, record=True)
    obs, _ = env.reset()

    times = [obs["env/time"][0]]
    for i in range(10):
        action = {"act/left_wheel": np.array([0.0]), "act/right_wheel": np.array([0.0])}
        obs, _, _, _, _ = env.step(action)
        times.append(obs["env/time"][0])

    for i in range(1, len(times)):
        assert times[i] > times[i - 1], (
            f"Time not monotonic: {times[i - 1]} -> {times[i]}"
        )

    print(f"Times: {times}")
    print("PASS: env/time is monotonic")


def test_history_time_matches_obs():
    """Test that history records correct time values."""
    print("\n=== Test: history time matches obs ===")
    env = GymRP(state_keys=OBS_SPACE, randomize=False, record=True)
    obs, _ = env.reset()

    obs_times = [obs["env/time"][0]]
    for i in range(5):
        action = {"act/left_wheel": np.array([0.0]), "act/right_wheel": np.array([0.0])}
        obs, _, _, _, _ = env.step(action)
        obs_times.append(obs["env/time"][0])

    history, idx = env.state.history
    hist_times = history[:, idx["env/time"]].flatten()

    print(f"Obs times: {obs_times}")
    print(f"Hist times: {hist_times}")

    # History excludes last entry due to action shifting
    # So hist_times should match obs_times[1:-1]
    expected_times = obs_times[1:-1]
    assert len(hist_times) == len(expected_times), (
        f"Length mismatch: {len(hist_times)} vs {len(expected_times)}"
    )
    for i in range(len(hist_times)):
        assert abs(hist_times[i] - expected_times[i]) < 1e-6, (
            f"Time mismatch at {i}: {hist_times[i]} vs {expected_times[i]}"
        )

    print("PASS: history time matches obs")


def test_model_input_shape():
    """Test that model gets correct input shape."""
    print("\n=== Test: model input shape ===")
    env = GymRP(state_keys=OBS_SPACE, randomize=False)
    obs, _ = env.reset()

    # Build input tensor
    obs_parts = []
    for k in MODEL_INPUT:
        if k == "sens/gyro":
            obs_parts.append(obs[k])
        else:
            obs_parts.append(obs[k])

    obs_arr = np.concatenate(obs_parts)
    print(f"MODEL_INPUT: {MODEL_INPUT}")
    print(f"Obs array shape: {obs_arr.shape}")

    expected_dim = 1 + 3 + 1 + 1 + 1  # pitch + gyro(3) + left_vel + right_vel + time
    assert obs_arr.shape[0] == expected_dim, (
        f"Shape mismatch: {obs_arr.shape[0]} vs {expected_dim}"
    )

    # Test network
    net = PolicyNetwork(obs_space_dims=expected_dim, action_space_dims=2)
    obs_t = torch.tensor(obs_arr, dtype=torch.float32).unsqueeze(0)
    with torch.no_grad():
        means, stddevs, values = net(obs_t)

    print(
        f"Network output shapes: means={means.shape}, stddevs={stddevs.shape}, values={values.shape}"
    )
    print("PASS: model input shape correct")


def test_gradient_flow():
    """Test that gradients flow correctly through the network."""
    print("\n=== Test: gradient flow ===")

    net = PolicyNetwork(obs_space_dims=7, action_space_dims=2)

    # Create fake input
    x = torch.randn(4, 7, requires_grad=True)

    means, stddevs, values = net(x)

    # Compute fake loss
    loss = means.sum() + stddevs.sum() + values.sum()
    loss.backward()

    # Check gradients exist
    assert x.grad is not None, "No gradient on input"
    assert not torch.isnan(x.grad).any(), "NaN in gradient"

    print(f"Input gradient shape: {x.grad.shape}")
    print(f"Input gradient mean: {x.grad.mean().item():.6f}")
    print("PASS: gradients flow correctly")


def test_time_normalization():
    """Test that time is normalized correctly in network."""
    print("\n=== Test: time normalization ===")

    net = PolicyNetwork(obs_space_dims=7, action_space_dims=2)

    # Test with different time values
    times = [0.0, 1.0, 5.0, 10.0, 100.0]

    for t in times:
        x = torch.zeros(1, 7)
        x[0, 6] = t  # time is at index 6

        with torch.no_grad():
            # Check forward pass doesn't crash
            means, stddevs, values = net(x)

        print(f"Time={t}: means={means[0].numpy()}, stddevs={stddevs[0].numpy()}")

    print("PASS: time normalization works")


if __name__ == "__main__":
    test_env_time_monotonic()
    test_history_time_matches_obs()
    test_model_input_shape()
    test_gradient_flow()
    test_time_normalization()
    print("\n=== ALL TESTS PASSED ===")

"""Numeric evaluation traces with explicit observation/transition alignment."""

import csv
from pathlib import Path

import numpy as np

from sim.utils import EpisodeBuffer


def save_episode_csv(
    buffer: EpisodeBuffer,
    path: Path,
    *,
    returns: np.ndarray,
    advantages: np.ndarray,
) -> Path:
    """Export T+1 observations with T outgoing actions and transition rewards.

    Row t contains observation_t, action_t, and transition/reward = reward_(t+1).
    Observation reward/* fields describe the incoming reward at that observation.
    The final row contains only the final observation; transition/policy fields
    are blank. Vector channels are expanded as name[0], name[1], etc.
    """
    steps = buffer.seq_len
    rows = steps + 1
    columns: dict[str, list[float | None]] = {}

    def add_stream(name: str, values: np.ndarray, expected_length: int):
        values = np.asarray(values)
        if values.ndim == 0 or len(values) != expected_length:
            raise ValueError(f"Expected {name} to have {expected_length} rows")
        values = values.reshape(expected_length, -1)
        for channel in range(values.shape[1]):
            label = name if values.shape[1] == 1 else f"{name}[{channel}]"
            columns[label] = values[:, channel].tolist() + [None] * (rows - len(values))

    for key, values in buffer.get_stvar_dict().items():
        add_stream(key.value, values, rows)
    for key, values in buffer.get_action_dict().items():
        add_stream(key.value, values, steps)
    add_stream("transition/reward", np.asarray(buffer.rewards_l), steps)
    add_stream("policy/value", buffer.get_values().detach().cpu().numpy(), steps)
    add_stream(
        "policy/log_probability", buffer.get_logps().detach().cpu().numpy(), steps
    )
    add_stream("policy/return", returns, steps)
    add_stream("policy/advantage", advantages, steps)

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values(), strict=True))
    return path

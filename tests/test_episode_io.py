"""Check exported state/action/reward alignment, including the final state."""

import csv
import tempfile
import unittest
from pathlib import Path

import numpy as np
import torch

from riktigpatric.patrick import Actions, DerivedObs, Observable, Target
from sim.episode_io import save_episode_csv
from sim.utils import EpisodeBuffer, get_advantages, get_returns


def episode(rewards: list[float]) -> EpisodeBuffer:
    def observation(time, incoming_reward):
        return {
            Observable.OBS_TIME: np.array([float(time)]),
            Observable.GYRO: np.array([time, 2 * time, 3 * time]),
            Observable.REWARD_TOTAL: np.array([incoming_reward]),
            Target.TARGET_VEL: np.array([0.2]),
            DerivedObs.CURRENT_VEL: np.array([0.1 * time]),
        }

    buffer = EpisodeBuffer()
    for time, reward in enumerate(rewards):
        buffer.add_step(
            observation(time, rewards[time - 1] if time else 0.0),
            {
                Actions.TIME: np.array([float(time)]),
                Actions.ACC_BOTH_WHEELS: np.array([10.0 + time]),
            },
            reward,
            torch.tensor(-0.1, requires_grad=True),
            torch.tensor(time + 0.5, requires_grad=True),
        )
    buffer.finish(observation(len(rewards), rewards[-1]))
    return buffer


class EpisodeExportTests(unittest.TestCase):
    def test_csv_preserves_observations_and_outgoing_transition_alignment(self):
        for rewards in ([7.0], [1.0, 2.0, 4.0]):
            with self.subTest(rewards=rewards), tempfile.TemporaryDirectory() as folder:
                buffer = episode(rewards)
                returns = get_returns(buffer.get_rewards(), discount=0.5)
                advantages = get_advantages(returns, buffer.get_values())
                path = save_episode_csv(
                    buffer, Path(folder) / "nested" / "episode.csv",
                    returns=returns.numpy(), advantages=advantages.numpy(),
                )
                with path.open(newline="", encoding="utf-8") as stream:
                    rows = list(csv.DictReader(stream))

                self.assertEqual(len(rows), len(rewards) + 1)
                for time, row in enumerate(rows):
                    self.assertEqual(float(row["env/obs_time"]), time)
                    self.assertEqual(float(row["sens/gyro[2]"]), 3 * time)
                    self.assertEqual(float(row["target/vel"]), 0.2)
                    incoming = rewards[time - 1] if time else 0.0
                    self.assertEqual(float(row["reward/total"]), incoming)
                    if time < len(rewards):
                        self.assertEqual(float(row["act/time"]), time)
                        self.assertEqual(
                            float(row["act/accelerate_both_wheels"]), 10 + time
                        )
                        self.assertEqual(float(row["transition/reward"]), rewards[time])
                        self.assertEqual(float(row["policy/value"]), time + 0.5)
                        self.assertAlmostEqual(
                            float(row["policy/log_probability"]), -0.1
                        )
                        self.assertEqual(
                            float(row["policy/return"]), returns[time].item()
                        )
                        self.assertEqual(
                            float(row["policy/advantage"]), advantages[time].item()
                        )
                    else:
                        for name, value in row.items():
                            if name.startswith(("act/", "transition/", "policy/")):
                                self.assertEqual(value, "")

    def test_csv_rejects_state_length_diagnostics_instead_of_shifting_them(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "episode.csv"
            with self.assertRaises(ValueError):
                save_episode_csv(
                    episode([1.0, 2.0]), path,
                    returns=np.zeros(3), advantages=np.zeros(2),
                )
            self.assertFalse(path.exists())


if __name__ == "__main__":
    unittest.main()

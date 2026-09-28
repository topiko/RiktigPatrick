"""Tracking-input changes require fresh training checkpoints."""

import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import torch
from hydra import compose, initialize_config_dir

from nn_ctrl.nns import Agent
from riktigpatric.patrick import DerivedObs, Target
from sim.checkpoints import capture_state, load_checkpoint, save_checkpoint
from sim.curriculum import Curriculum
from sim.train_agent import get_policy_inputs, make_agent


def config():
    with initialize_config_dir(
        config_dir=str(Path(__file__).resolve().parents[1] / "config"),
        version_base=None,
    ):
        cfg = compose(config_name="continuous")
    cfg.train.device = "cpu"
    cfg.policy.hsize = 4
    cfg.policy.n_rnnlayers = 2
    return cfg


class PositionInputMigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def legacy_snapshot(self, cfg):
        inputs = []
        for key in get_policy_inputs(cfg):
            if key == DerivedObs.VELOCITY_ERROR.value:
                inputs.append(DerivedObs.CURRENT_VEL.value)
            elif key != DerivedObs.POSITION_ERROR.value:
                inputs.append(key)
        inputs.extend((Target.TARGET_POS.value, DerivedObs.CURRENT_POS.value))
        old = Agent(inputs, cfg.policy.actions, hsize=4, n_rnnlayers=2)
        optimizer = torch.optim.Adam(old.parameters(), lr=0.001)
        curriculum = Curriculum(cfg)
        return capture_state(old, optimizer, 1600, curriculum)

    def test_legacy_raw_position_checkpoint_is_rejected(self):
        cfg = config()
        snapshot = self.legacy_snapshot(cfg)
        agent = make_agent(cfg)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        curriculum = Curriculum(cfg)
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "legacy.pt", snapshot, {})
            with self.assertRaisesRegex(ValueError, "inputs/actions"):
                load_checkpoint(path, agent, optimizer, curriculum)

    def test_invalid_resume_lr_is_rejected_for_current_checkpoint(self):
        cfg = config()
        agent = make_agent(cfg)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        curriculum = Curriculum(cfg)
        snapshot = capture_state(agent, optimizer, 1600, curriculum)
        before = deepcopy(agent.state_dict())
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "current.pt", snapshot, {})
            for rate in (0, -1, float("nan"), float("inf")):
                with self.assertRaisesRegex(ValueError, "learning rate"):
                    load_checkpoint(
                        path, agent, optimizer, curriculum, learning_rate=rate
                    )
        torch.testing.assert_close(agent.state_dict(), before, rtol=0, atol=0)


if __name__ == "__main__":
    unittest.main()

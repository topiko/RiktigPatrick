"""Adding position observations must preserve the existing controller and Adam state."""

import tempfile
import unittest
from copy import deepcopy
from pathlib import Path

import torch
from hydra import compose, initialize_config_dir

from nn_ctrl.nns import Agent, ContinuousHead
from riktigpatric.patrick import StateVar, StateVarKey
from sim.checkpoint_migrations import INPUT_WEIGHT, POSITION_INPUTS
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

    def old_policy(self, cfg):
        inputs = get_policy_inputs(cfg)
        self.assertEqual(inputs[-2:], POSITION_INPUTS)
        old = Agent(inputs[:-2], cfg.policy.actions, hsize=4, n_rnnlayers=2)
        with torch.no_grad():
            for head in old.action_heads.values():
                assert isinstance(head, ContinuousHead) and head.bias is not None
                head.weight.normal_(std=0.1)
                head.bias.fill_(0.05)
                head.std_logit.fill_(0.2)
        optimizer = torch.optim.Adam(old.parameters(), lr=0.001)
        # Prime distinct, nonzero moments for every old parameter.
        loss = torch.stack([
            (i + 1) * p.square().sum() for i, p in enumerate(old.parameters())
        ]).sum()
        loss.backward()
        optimizer.step()
        optimizer.zero_grad(set_to_none=True)
        curriculum = Curriculum(cfg)
        snapshot = capture_state(old, optimizer, 1600, curriculum)
        snapshot["curriculum"].update(version=2, stage="balance", success_streak=2)
        return old, optimizer, snapshot

    def test_controller_and_adam_survive_added_inputs_and_lr_override(self):
        torch.manual_seed(4)
        cfg = config()
        old, old_optimizer, snapshot = self.old_policy(cfg)
        new = make_agent(cfg)
        optimizer = torch.optim.Adam(new.parameters(), lr=cfg.train.policy_lr)
        curriculum = Curriculum(cfg)
        original = deepcopy(snapshot)
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "old.pt", snapshot, {})
            self.assertEqual(load_checkpoint(path, new, optimizer, curriculum,
                                            learning_rate=0.0001), 1600)
            self.assertEqual(optimizer.param_groups[0]["lr"], 0.0001)
            self.assertEqual(curriculum.stage, "balance")
            self.assertEqual(curriculum.success_streak, 0)
            obs: dict[StateVarKey, torch.Tensor] = {
                StateVar.from_str(key): torch.randn(3, StateVar.from_str(key).dim())
                for key in new.inputs
            }
            old_h = new_h = None
            for _ in range(4):
                old_outputs, old_values, old_h = old.forward(obs, old_h)
                new_outputs, new_values, new_h = new.forward(obs, new_h)
                torch.testing.assert_close(new_outputs, old_outputs)
                torch.testing.assert_close(new_values, old_values)
                torch.testing.assert_close(new_h, old_h)
            old_width = old.rnn.input_size
            new_params = dict(new.named_parameters())
            for name, parameter in old.named_parameters():
                for key, expected in old_optimizer.state[parameter].items():
                    actual = optimizer.state[new_params[name]][key]
                    if name == INPUT_WEIGHT and expected.shape == parameter.shape:
                        torch.testing.assert_close(actual[:, :old_width], expected,
                                                   rtol=0, atol=0)
                        self.assertEqual(torch.count_nonzero(actual[:, old_width:]), 0)
                    else:
                        torch.testing.assert_close(actual, expected, rtol=0, atol=0)
            for key in POSITION_INPUTS:
                for parameter in new.encoders[key].parameters():
                    self.assertNotIn(parameter, optimizer.state)
            # New columns can learn immediately; the encoders are not also zeroed.
            new_values.sum().backward()
            gradient = new.rnn.weight_ih_l0.grad
            assert isinstance(gradient, torch.Tensor)
            self.assertGreater(gradient[:, old_width:].abs().sum(), 0)
            # A second save/resume uses the normal strict path, not another expansion.
            state = capture_state(new, optimizer, 1600, curriculum)
            path = save_checkpoint(Path(folder) / "expanded.pt", state, {})
            load_checkpoint(path, new, optimizer, curriculum)
            torch.testing.assert_close(new.state_dict(), state["model"], rtol=0, atol=0)
        torch.testing.assert_close(snapshot["model"], original["model"], rtol=0, atol=0)
        torch.testing.assert_close(
            snapshot["optimizer"], original["optimizer"], rtol=0, atol=0
        )

    def test_unrelated_shape_changes_and_invalid_resume_lr_are_rejected(self):
        cfg = config()
        _, _, snapshot = self.old_policy(cfg)
        cfg.policy.hsize = 8
        new = make_agent(cfg)
        optimizer = torch.optim.Adam(new.parameters())
        curriculum = Curriculum(cfg)
        before = deepcopy(new.state_dict())
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "old.pt", snapshot, {})
            with self.assertRaisesRegex(ValueError, "shape"):
                load_checkpoint(path, new, optimizer, curriculum)
            for rate in (0, -1, float("nan"), float("inf")):
                with self.assertRaisesRegex(ValueError, "learning rate"):
                    load_checkpoint(
                        path, new, optimizer, curriculum, learning_rate=rate
                    )
        torch.testing.assert_close(new.state_dict(), before, rtol=0, atol=0)


if __name__ == "__main__":
    unittest.main()

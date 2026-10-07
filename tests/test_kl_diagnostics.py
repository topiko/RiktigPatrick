"""Rejection analysis must preserve live weights, optimizer, and random streams."""

import json
import random
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import torch
from omegaconf import OmegaConf
from test_policy_probe import agent as mixed_agent
from test_tbptt import make_agent, scripted_env
from test_training_safety import config

from sim.checkpoints import capture_state, restore_state
from sim.kl_diagnostics import KLDiagnostics, _leave_out_gradient, publish_kl_report
from sim.kl_guard import KLGuardSettings
from sim.optimizers import make_optimizer
from sim.train_agent import _log_training_update, training_update


class KLDiagnosticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def config(self, *, diagnostics, backtracks=8, folder="unused"):
        cfg = config(folder)
        cfg.train.tbptt_steps = 2
        cfg.kl_guard = OmegaConf.structured(KLGuardSettings(
            enabled=True, diagnostics=diagnostics, loo_top_k=2,
            max_kl=1e-5, max_backtracks=backtracks,
        ))
        return cfg

    def assert_training_states_equal(self, actual, expected):
        torch.testing.assert_close(actual["model"], expected["model"], rtol=0, atol=0)
        self.assertEqual(actual["optimizer"]["param_groups"],
                         expected["optimizer"]["param_groups"])
        torch.testing.assert_close(actual["optimizer"]["state"],
                                   expected["optimizer"]["state"], rtol=0, atol=0)
        torch.testing.assert_close(actual["torch_rng"], expected["torch_rng"])
        for key in ("numpy_rng", "python_rng", "next_iteration", "curriculum"):
            self.assertEqual(actual[key], expected[key])

    def test_analysis_preserves_training_and_checks_original_batch(self):
        for backtracks in (0, 8):
            torch.manual_seed(71)
            agent = make_agent()
            optimizer = make_optimizer(agent, 0.03, 0.005)
            sum(p.square().sum() for p in agent.parameters()).backward()
            optimizer.step()
            before = capture_state(agent, optimizer, 3)
            with self.assertLogs(level="INFO"):
                baseline = training_update(
                    self.config(diagnostics=False, backtracks=backtracks),
                    scripted_env(), agent, optimizer, 3,
                )
            expected = capture_state(agent, optimizer, 4)
            expected_grads = [None if p.grad is None else p.grad.clone()
                              for p in agent.parameters()]
            restore_state(agent, optimizer, before)
            reports = []
            env = scripted_env()
            with self.assertLogs(level="INFO"), patch.object(
                env, "reset", wraps=env.reset
            ) as reset:
                metrics = training_update(
                    self.config(diagnostics=True, backtracks=backtracks),
                    env, agent, optimizer, 3, reports=reports,
                )
            reset.assert_called_once()
            self.assertGreater(metrics["kl_guard/initial_kl_mean"], 1e-5)
            self.assertEqual(metrics["kl_guard/accepted"], bool(backtracks))
            self.assertEqual(
                metrics["kl_guard/accepted"], baseline["kl_guard/accepted"]
            )
            self.assertEqual(metrics["kl_diagnostics/failed"], 0)
            self.assertLess(
                metrics["kl_diagnostics/gradient_reconstruction_error"], 1e-3
            )
            self.assert_training_states_equal(
                capture_state(agent, optimizer, 4), expected
            )
            for parameter, gradient in zip(agent.parameters(), expected_grads):
                torch.testing.assert_close(parameter.grad, gradient)
            self.assertEqual(len(reports), 1)
            report = reports[0]
            self.assertEqual(report["iteration"], 3)
            self.assertEqual([e["steps"] for e in report["episodes"]], [1, 3, 7])
            self.assertEqual([e["environment_seed"] for e in report["episodes"]],
                             [45, 46, 47])
            self.assertAlmostEqual(sum(e["policy_norm_share"]
                                       for e in report["episodes"]), 1)
            self.assertEqual(len(report["leave_one_out"]), 2)
            self.assertFalse(report["trials"][0]["policy_adam_reset"])
            for trial in report["trials"][1:]:
                self.assertTrue(trial["policy_adam_reset"])
            self.assertEqual(metrics["kl_guard/policy_adam_reset"], bool(backtracks))
            self.assertIn("original Adam", report["leave_one_out_optimizer_state"])
            for omitted in report["leave_one_out"]:
                self.assertEqual(omitted["samples"], 11)
                self.assertEqual(len(omitted["episodes"]), 3)
            json.dumps(report, allow_nan=False)

    def test_saturated_continuous_actions_are_recreated_without_inverse_tanh(self):
        torch.manual_seed(9)
        agent = mixed_agent()
        with torch.no_grad():
            agent.action_heads["act/accelerate_both_wheels"].bias.fill_(9)
        optimizer = make_optimizer(agent, 0.03, 0.005)
        reports = []
        with self.assertLogs(level="INFO"):
            metrics = training_update(
                self.config(diagnostics=True, backtracks=0), scripted_env(),
                agent, optimizer, 0, reports=reports,
            )
        self.assertEqual(metrics["kl_diagnostics/failed"], 0)
        self.assertLess(metrics["kl_diagnostics/gradient_reconstruction_error"], 1e-3)
        self.assertEqual(len(reports), 1)

    def test_analysis_failure_is_reported_without_changing_training(self):
        agent = make_agent()
        optimizer = make_optimizer(agent, 0.03, 0.005)
        before = capture_state(agent, optimizer, 0)
        with self.assertLogs(level="INFO"):
            training_update(self.config(diagnostics=False, backtracks=0),
                            scripted_env(), agent, optimizer, 0)
        expected = capture_state(agent, optimizer, 1)
        restore_state(agent, optimizer, before)
        reports = []

        def broken(_agent):
            torch.rand(5)
            np.random.rand(5)
            random.random()
            raise RuntimeError("diagnostic failure")

        with self.assertLogs(level="INFO"), patch.object(
            KLDiagnostics, "_episode_gradients", side_effect=broken
        ):
            metrics = training_update(
                self.config(diagnostics=True, backtracks=0), scripted_env(),
                agent, optimizer, 0, reports=reports,
            )
        self.assert_training_states_equal(capture_state(agent, optimizer, 1), expected)
        self.assertEqual(metrics["kl_diagnostics/failed"], 1)
        self.assertIn("diagnostic failure", reports[0]["error"])

    def test_leave_out_renormalizes_to_remaining_transition_count(self):
        result = _leave_out_gradient([torch.tensor([3.0]), None],
                                    [torch.tensor([1.0]), None], 10, 4)
        torch.testing.assert_close(result[0], torch.tensor([10/3]))
        self.assertIsNone(result[1])
        with self.assertRaises(ValueError):
            _leave_out_gradient([], [], 4, 4)

    def test_mlflow_receives_rejection_metrics_between_regular_logging_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config(diagnostics=True, backtracks=0,
                              folder=str(Path(directory) / "checkpoints"))
            agent = make_agent()
            optimizer = make_optimizer(agent, 0.03, 0.005)
            reports = []
            with self.assertLogs(level="INFO"):
                metrics = training_update(cfg, scripted_env(), agent, optimizer, 3,
                                          reports=reports)
            cfg.logging.mlflow.enabled = True
            cfg.logging.mlflow.push_freq = 10
            with (
                patch("sim.kl_diagnostics.mlflow.log_metrics") as log_metrics,
                patch("sim.kl_diagnostics.mlflow.log_artifact") as log_artifact,
            ):
                _log_training_update(cfg, 3, metrics, reports)
            log_metrics.assert_called_once_with(reports[0]["metrics"], step=3)
            path = Path(directory) / "kl_diagnostics/iteration_000003.json"
            log_artifact.assert_called_once_with(
                str(path), artifact_path="diagnostics/kl"
            )
            report = json.loads(path.read_text())
            self.assertEqual(len(report["episodes"]), 3)
            self.assertTrue(report["trials"][0]["rejected"])
            cfg.logging.mlflow.push_freq = 3
            with (
                patch("sim.kl_diagnostics.mlflow.log_metrics") as log_metrics,
                patch("sim.kl_diagnostics.mlflow.log_artifact"),
            ):
                _log_training_update(cfg, 3, metrics, reports)
            log_metrics.assert_called_once_with(metrics, step=3)

    def test_upload_failure_keeps_local_report(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg = config(str(Path(directory) / "checkpoints"))
            cfg.logging.mlflow.enabled = True
            report = {"iteration": 3, "metrics": {"kl_diagnostics/failed": 0.0}}
            with self.assertLogs(level="ERROR"), patch(
                "sim.kl_diagnostics.mlflow.log_artifact",
                side_effect=RuntimeError("offline"),
            ):
                publish_kl_report(cfg, report)
            path = Path(directory) / "kl_diagnostics/iteration_000003.json"
            self.assertEqual(json.loads(path.read_text()), report)


if __name__ == "__main__":
    unittest.main()

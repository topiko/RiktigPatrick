"""Device selection, portable checkpoints, and conditional CUDA integration."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import PropertyMock, patch

import numpy as np
import torch
from omegaconf import DictConfig, OmegaConf

from nn_ctrl.nns import Agent
from riktigpatric.patrick import Actions, Observable
from sim.checkpoints import (
    PolicyGuard,
    capture_state,
    cpu_snapshot,
    load_checkpoint,
    restore_state,
    save_checkpoint,
)
from sim.curriculum import Curriculum
from sim.devices import evaluation_rng, resolve_device, seed_torch
from sim.train_agent import make_agent, training_update, validate_policy
from sim.utils import (
    EpisodeBuffer,
    ebufs2batchd,
    npd2tensord,
    register_and_make_env,
    tensord2npd,
)


def config() -> DictConfig:
    cfg = OmegaConf.load(Path(__file__).resolve().parents[1] / "config/rlrp.yaml")
    assert isinstance(cfg, DictConfig)
    cfg.env.n_parallel = 2
    cfg.env.max_episode_steps = 3
    cfg.train.tbptt_steps = 2
    return cfg


def tiny_agent(device="cpu"):
    return Agent(
        [Observable.OBS_TIME.value],
        {Actions.ACC_BOTH_WHEELS.value: {
            "type": "discrete", "bins": [-1.0, 0.0, 1.0]
        }}, hsize=8, n_rnnlayers=2,
    ).to(device)


def prime_optimizer(agent):
    optimizer = torch.optim.Adam(agent.parameters(), lr=0.01)
    torch.stack([p.square().sum() for p in agent.parameters()]).sum().backward()
    optimizer.step()
    optimizer.zero_grad(set_to_none=True)
    return optimizer


class DeviceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_cpu_fallback_and_explicit_unavailable_cuda(self):
        with patch("torch.cuda.is_available", return_value=False):
            self.assertEqual(resolve_device("auto"), torch.device("cpu"))
            self.assertEqual(resolve_device("cpu"), torch.device("cpu"))
            with self.assertRaises(RuntimeError):
                resolve_device("cuda")
        with self.assertRaises(ValueError):
            resolve_device("mps")

    def test_cuda_selection_normalizes_and_validates_device_index(self):
        with (
            patch("torch.cuda.is_available", return_value=True),
            patch("torch.cuda.device_count", return_value=2),
            patch("torch.cuda.current_device", return_value=1),
        ):
            self.assertEqual(resolve_device("auto"), torch.device("cuda:1"))
            self.assertEqual(resolve_device("cuda:0"), torch.device("cuda:0"))
            with self.assertRaises(ValueError):
                resolve_device("cuda:2")

    def test_network_size_is_configurable(self):
        cfg = config()
        cfg.train.device = "cpu"
        cfg.policy.hsize = 96
        cfg.policy.n_rnnlayers = 2
        agent = make_agent(cfg)
        self.assertEqual(agent.device, torch.device("cpu"))
        self.assertEqual(agent.rnn.hidden_size, 96)
        self.assertEqual(agent.rnn.num_layers, 2)
        env = register_and_make_env(cfg)
        self.addCleanup(env.close)
        obs, _ = env.reset(seed=1)
        _, logp, value, hidden = agent.act(npd2tensord(obs, device=agent.device))
        self.assertEqual(tuple(logp.shape), (2, 1))
        self.assertEqual(tuple(value.shape), (2, 1))
        assert hidden is not None
        self.assertEqual(tuple(hidden.shape), (2, 2, 96))

    def test_cpu_conversions_detach_actions_and_preserve_shapes(self):
        arrays = {
            "vector": np.arange(6).reshape(2, 3), "scalar": np.array([[1.], [2.]])
        }
        tensors = npd2tensord(arrays, device="cpu")
        for tensor in tensors.values():
            self.assertEqual(tensor.dtype, torch.float32)
            tensor.requires_grad_()
        result = tensord2npd(tensors)
        for key in arrays:
            np.testing.assert_array_equal(result[key], arrays[key])
        self.assertTrue(all(tensor.requires_grad for tensor in tensors.values()))

    def test_snapshot_is_independent_and_preserves_state_dict_metadata(self):
        agent = tiny_agent()
        original = agent.state_dict()
        copied = cpu_snapshot(original)
        self.assertEqual(copied._metadata, original._metadata)
        key = next(iter(copied))
        copied[key].fill_(123)
        self.assertFalse(torch.equal(copied[key], original[key]))
        tensor = torch.ones(2)
        aliases = cpu_snapshot({"first": tensor, "nested": [(tensor,)]})
        self.assertIs(aliases["first"], aliases["nested"][0][0])
        self.assertIsNot(aliases["first"], tensor)

    def test_network_shape_mismatch_is_rejected_before_weights_change(self):
        agent = tiny_agent()
        optimizer = prime_optimizer(agent)
        snapshot = capture_state(agent, optimizer, 0)
        different = Agent(agent.inputs, agent.actions, hsize=16, n_rnnlayers=2)
        before = cpu_snapshot(different.state_dict())
        with self.assertRaises(ValueError):
            restore_state(different, torch.optim.Adam(different.parameters()), snapshot)
        for key, value in different.state_dict().items():
            torch.testing.assert_close(value, before[key], rtol=0, atol=0)

    def test_cpu_load_ignores_saved_cuda_rng_without_accessing_cuda(self):
        agent = tiny_agent()
        optimizer = prime_optimizer(agent)
        snapshot = capture_state(agent, optimizer, 3)
        snapshot["device"] = "cuda:1"
        snapshot["cuda_rng"] = torch.ones(16, dtype=torch.uint8)
        with patch("torch.cuda.set_rng_state") as set_cuda_rng:
            self.assertEqual(restore_state(agent, optimizer, snapshot), 3)
            set_cuda_rng.assert_not_called()
        # Older CPU-only checkpoints remain supported.
        del snapshot["cuda_rng"]
        del snapshot["device"]
        self.assertEqual(restore_state(agent, optimizer, snapshot), 3)

    def test_rng_routes_to_selected_gpu_using_host_rng_tensor(self):
        agent = tiny_agent()
        optimizer = prime_optimizer(agent)
        rng = torch.ones(16, dtype=torch.uint8)
        with (
            patch.object(Agent, "device", new_callable=PropertyMock,
                         return_value=torch.device("cuda:1")),
            patch("torch.cuda.get_rng_state", return_value=rng) as get_rng,
            patch("torch.cuda.set_rng_state") as set_rng,
        ):
            snapshot = capture_state(agent, optimizer, 0)
            get_rng.assert_called_once_with(torch.device("cuda:1"))
            restore_state(agent, optimizer, snapshot)
            self.assertEqual(set_rng.call_args.args[1], torch.device("cuda:1"))
            torch.testing.assert_close(set_rng.call_args.args[0], rng)

    def test_cpu_evaluation_does_not_touch_cuda_generators(self):
        before = torch.get_rng_state().clone()
        with patch("torch.cuda.manual_seed_all") as seed_all:
            with evaluation_rng(torch.device("cpu"), 123):
                torch.rand(10)
            seed_all.assert_not_called()
        torch.testing.assert_close(torch.get_rng_state(), before, rtol=0, atol=0)


@unittest.skipUnless(torch.cuda.is_available(), "CUDA device unavailable")
class CUDAIntegrationTests(unittest.TestCase):
    def test_curriculum_masks_and_activation_work_on_cuda(self):
        cfg = config()
        cfg.train.device = "cuda"
        agent = make_agent(cfg)
        curriculum = Curriculum(cfg)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        env = register_and_make_env(cfg)
        self.addCleanup(env.close)
        for _ in range(3):
            metrics = training_update(cfg, env, agent, optimizer, 0, curriculum)
            self.assertTrue(all(np.isfinite(v) for v in metrics.values()))
            for action in curriculum.inactive_actions:
                self.assertTrue(all(
                    p.grad is None
                    for p in agent.action_heads[action.value].parameters()
                ))
            state = capture_state(agent, optimizer, 1, curriculum)
            restore_state(agent, optimizer, state, curriculum=curriculum)
            curriculum.advance(agent, optimizer)

    def test_cuda_rollout_update_and_evaluation_rng(self):
        cfg = config()
        cfg.train.device = "cuda:1" if torch.cuda.device_count() > 1 else "cuda:0"
        cfg.policy.hsize = 16
        cfg.policy.n_rnnlayers = 2
        agent = make_agent(cfg)
        self.assertTrue(all(p.is_cuda for p in agent.parameters()))
        self.assertTrue(all(b.is_cuda for b in agent.buffers()))
        env = register_and_make_env(cfg)
        self.addCleanup(env.close)
        optimizer = torch.optim.Adam(agent.parameters(), lr=cfg.train.policy_lr)
        with patch.object(env, "step", wraps=env.step) as step:
            metrics = training_update(cfg, env, agent, optimizer, 0)
            self.assertTrue(all(isinstance(a, np.ndarray)
                                for a in step.call_args.args[0].values()))
        self.assertTrue(all(np.isfinite(v) for v in metrics.values()))
        cpu_rng = torch.get_rng_state().clone()
        cuda_rng = torch.cuda.get_rng_state(agent.device).clone()
        other_rng = (
            torch.cuda.get_rng_state(0).clone() if agent.device.index != 0 else None
        )
        first = validate_policy(cfg, env, agent)
        second = validate_policy(cfg, env, agent)
        self.assertEqual(first, second)
        torch.testing.assert_close(torch.get_rng_state(), cpu_rng, rtol=0, atol=0)
        torch.testing.assert_close(torch.cuda.get_rng_state(agent.device), cuda_rng)
        if other_rng is not None:
            torch.testing.assert_close(torch.cuda.get_rng_state(0), other_rng)
        self.assertTrue(agent.training)

    def test_gpu_checkpoint_round_trip_cpu_portability_and_guard_rollback(self):
        device = resolve_device("cuda")
        seed_torch(42, device)
        agent = tiny_agent(device)
        optimizer = prime_optimizer(agent)
        snapshot = capture_state(agent, optimizer, 7)
        self.assertTrue(all(v.device.type == "cpu" for v in snapshot["model"].values()))
        expected = torch.rand(4, device=device)
        with tempfile.TemporaryDirectory() as folder:
            path = save_checkpoint(Path(folder) / "gpu.pt", snapshot, {})
            self.assertEqual(load_checkpoint(path, agent, optimizer), 7)
            torch.testing.assert_close(torch.rand(4, device=device), expected)
            for parameter, state in optimizer.state.items():
                self.assertEqual(state["exp_avg"].device, parameter.device)
                self.assertEqual(state["step"].device.type, "cpu")
            cpu_agent = tiny_agent()
            cpu_optimizer = torch.optim.Adam(cpu_agent.parameters())
            rng = torch.cuda.get_rng_state(device).clone()
            self.assertEqual(load_checkpoint(path, cpu_agent, cpu_optimizer), 7)
            torch.testing.assert_close(torch.cuda.get_rng_state(device), rng)
            for key, value in cpu_agent.state_dict().items():
                torch.testing.assert_close(value, snapshot["model"][key])

        guard = PolicyGuard(agent, optimizer)
        guard.observe(100, 7)
        with torch.no_grad():
            next(agent.parameters()).add_(10)
        rng = torch.cuda.get_rng_state(device).clone()
        guard.observe(0, 8)
        self.assertEqual(guard.observe(0, 9), "rollback")
        self.assertEqual(agent.device, device)
        self.assertEqual(optimizer.param_groups[0]["lr"], 0.005)
        torch.testing.assert_close(torch.cuda.get_rng_state(device), rng)

    def test_cpu_checkpoint_can_resume_on_cuda_with_seeded_gpu_stream(self):
        cpu_agent = tiny_agent()
        snapshot = capture_state(cpu_agent, prime_optimizer(cpu_agent), 3)
        device = resolve_device("cuda")
        agent = tiny_agent(device)
        optimizer = torch.optim.Adam(agent.parameters())
        rng = torch.cuda.get_rng_state(device).clone()
        restore_state(agent, optimizer, snapshot)
        torch.testing.assert_close(torch.cuda.get_rng_state(device), rng)
        for parameter, state in optimizer.state.items():
            self.assertEqual(state["exp_avg"].device, parameter.device)

    def test_packed_transfers_and_unequal_padding_preserve_cuda_gradients(self):
        device = resolve_device("cuda")
        arrays = {"vector": np.arange(6).reshape(2, 3), "scalar": np.ones((2, 1))}
        tensors = npd2tensord(arrays, device=device)
        for tensor in tensors.values():
            self.assertEqual(tensor.device, device)
            tensor.requires_grad_()
        exported = tensord2npd(tensors)
        for key in arrays:
            np.testing.assert_array_equal(exported[key], arrays[key])

        buffers, leaves = [], []
        for length in (2, 4):
            buffer = EpisodeBuffer()
            for time in range(length):
                value = torch.tensor(0.5, device=device, requires_grad=True)
                logp = torch.tensor(-0.5, device=device, requires_grad=True)
                leaves.extend([value, logp])
                buffer.add_step(
                    {Observable.OBS_TIME: np.array([float(time)])},
                    {Actions.TIME: np.array([float(time)])}, 1.0, logp, value,
                )
            buffer.finish({Observable.OBS_TIME: np.array([float(length)])})
            buffers.append(buffer)
        logps, rewards, values, lengths, mask = ebufs2batchd(buffers)
        for tensor in (logps, rewards, values, mask):
            self.assertEqual(tensor.device, device)
        np.testing.assert_array_equal(lengths, [2, 4])
        ((logps + values) * mask).sum().backward()
        for leaf in leaves:
            torch.testing.assert_close(leaf.grad, torch.ones_like(leaf))


if __name__ == "__main__":
    unittest.main()

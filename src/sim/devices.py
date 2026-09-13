"""Training device selection and isolated CPU/CUDA random-number streams."""

from contextlib import contextmanager

import torch


def resolve_device(requested: str | torch.device) -> torch.device:
    """Auto-select CUDA when available; explicit CUDA requests never fall back."""
    if requested == "auto":
        requested = "cuda" if torch.cuda.is_available() else "cpu"
    device = torch.device(requested)
    if device.type == "cpu":
        return torch.device("cpu")
    if device.type != "cuda":
        raise ValueError("Training device must be auto, cpu, cuda, or cuda:N")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA requested, but PyTorch cannot access a CUDA device")
    index = torch.cuda.current_device() if device.index is None else device.index
    if not 0 <= index < torch.cuda.device_count():
        raise ValueError(f"CUDA device index {index} is unavailable")
    return torch.device("cuda", index)


def seed_torch(seed: int, device: torch.device) -> None:
    """Seed CPU and the selected GPU without reseeding unrelated CUDA devices."""
    torch.random.default_generator.manual_seed(seed)
    if device.type == "cuda":
        with torch.cuda.device(device):
            torch.cuda.manual_seed(seed)


@contextmanager
def evaluation_rng(device: torch.device, seed: int):
    """Restore both training generators after validation/video action sampling."""
    devices = []
    if device.type == "cuda":
        index = (
            device.index if device.index is not None else torch.cuda.current_device()
        )
        devices = [index]
    with torch.random.fork_rng(devices=devices):
        seed_torch(seed, device)
        yield

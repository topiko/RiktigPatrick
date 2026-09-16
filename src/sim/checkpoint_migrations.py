"""Narrow, behavior-preserving migration for the added position observations."""

from copy import deepcopy

import torch

from nn_ctrl.nns import Agent
from riktigpatric.patrick import DerivedObs, Target

POSITION_INPUTS = [Target.TARGET_POS.value, DerivedObs.CURRENT_POS.value]
INPUT_WEIGHT = "rnn.weight_ih_l0"


def _pad_columns(value: torch.Tensor, shape: torch.Size) -> torch.Tensor:
    padded = value.new_zeros(shape)
    padded[:, :value.shape[1]] = value
    return padded


def add_position_inputs(agent: Agent, optimizer, state: dict, policy: dict) -> dict:
    """Extend only the known appended position inputs; all other mismatches stay errors.

    Old GRU columns/Adam moments are copied exactly, new columns/moments start at
    zero, and new encoders retain their initialization. Zeroing both encoders and
    GRU columns would prevent the new inputs from ever receiving a gradient.
    """
    previous = state["policy"]
    if (
        policy["actions"] != previous["actions"]
        or policy["inputs"] != [*previous["inputs"], *POSITION_INPUTS]
    ):
        return state
    expected = agent.state_dict()
    new_keys = {
        f"encoders.{name}.{part}"
        for name in POSITION_INPUTS for part in ("weight", "bias")
    }
    old_model = state["model"]
    if old_model.keys() != expected.keys() - new_keys:
        raise ValueError("Unexpected model keys for position-input migration")
    added_width = sum(
        expected[f"encoders.{key}.bias"].numel() for key in POSITION_INPUTS
    )
    for key, value in old_model.items():
        shape = expected[key].shape
        if key == INPUT_WEIGHT:
            shape = (shape[0], shape[1] - added_width)
        if not isinstance(value, torch.Tensor) or value.shape != shape:
            raise ValueError("Checkpoint shape differs; match policy.hsize/n_rnnlayers")

    result = deepcopy(state)
    model = result["model"]
    for key in new_keys:
        model[key] = expected[key].detach().to(device="cpu", copy=True)
    model[INPUT_WEIGHT] = _pad_columns(
        model[INPUT_WEIGHT], expected[INPUT_WEIGHT].shape
    )
    result["policy"] = policy
    _migrate_adam(agent, optimizer, result, old_model)
    return result


def _migrate_adam(agent, optimizer, result: dict, old_model: dict):
    saved = result["optimizer"]
    current = optimizer.state_dict()
    if len(saved["param_groups"]) != 1 or len(current["param_groups"]) != 1:
        raise ValueError("Position-input migration requires a single Adam group")
    names = list(dict(agent.named_parameters()))
    # state_dict order captures the original module/parameter order, even if the
    # current action-head dictionary has been reordered.
    old_names = [name for name in old_model if name in names]
    old_ids = saved["param_groups"][0]["params"]
    if len(old_names) != len(old_ids):
        raise ValueError("Checkpoint optimizer parameters do not match the saved model")
    new_ids = dict(zip(names, current["param_groups"][0]["params"], strict=True))
    id_names = dict(zip(old_ids, old_names, strict=True))
    remapped = {}
    for old_id, slots in saved["state"].items():
        name = id_names[old_id]
        if name == INPUT_WEIGHT:
            for key, value in slots.items():
                if (
                    isinstance(value, torch.Tensor)
                    and value.shape == old_model[name].shape
                ):
                    slots[key] = _pad_columns(value, result["model"][name].shape)
        remapped[new_ids[name]] = slots
    saved["state"] = remapped
    saved["param_groups"][0]["params"] = current["param_groups"][0]["params"]
    if "param_names" in saved["param_groups"][0]:
        saved["param_groups"][0]["param_names"] = names

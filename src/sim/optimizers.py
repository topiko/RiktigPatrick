"""Independent actor/critic Adam rates, including legacy checkpoint group loading."""

import math
from copy import deepcopy

import torch


def make_optimizer(agent, policy_lr: float, critic_lr: float | None = None):
    critic_lr = policy_lr if critic_lr is None else critic_lr
    for name, rate in (("policy", policy_lr), ("critic", critic_lr)):
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError(f"{name} learning rate must be positive and finite")
    policy = [p for name, p in agent.named_parameters()
              if not name.startswith("value_head.")]
    return torch.optim.Adam([
        {"name": "policy", "params": policy, "lr": policy_lr},
        {"name": "critic", "params": list(agent.value_head.parameters()),
         "lr": critic_lr},
    ])


def restore_optimizer_state(optimizer, saved):
    """Split the old single group in model parameter order, preserving Adam moments."""
    saved = deepcopy(saved)
    current_groups = optimizer.state_dict()["param_groups"]
    old_groups = saved["param_groups"]
    if len(old_groups) == 1 and [g.get("name") for g in current_groups] == [
        "policy", "critic"
    ]:
        old = old_groups[0]
        new_ids = [p for group in current_groups for p in group["params"]]
        if len(old["params"]) != len(new_ids):
            raise ValueError("Checkpoint optimizer parameter count differs")
        id_map = dict(zip(old["params"], new_ids, strict=True))
        saved["state"] = {id_map[key]: value for key, value in saved["state"].items()}
        saved["param_groups"] = []
        offset = 0
        for current in current_groups:
            group = {**old, "name": current["name"], "params": current["params"]}
            if "param_names" in old:
                end = offset + len(group["params"])
                group["param_names"] = old["param_names"][offset:end]
            offset += len(group["params"])
            saved["param_groups"].append(group)
    optimizer.load_state_dict(saved)

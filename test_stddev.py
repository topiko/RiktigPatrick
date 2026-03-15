#!/usr/bin/env python
import sys
sys.path.insert(0, 'src')

import torch
from sim.nets import PolicyNetwork

for init2zeros in [False, True]:
    net = PolicyNetwork(
        obs_space_dims=7,
        action_space_dims=2,
        init2zeros=init2zeros,
    )

    print(f"\n=== init2zeros={init2zeros} ===")
    x = torch.randn(10, 7)

    with torch.no_grad():
        means, stddevs, values = net(x)

    print(f"Stddevs min: {stddevs.min():.4f}")
    print(f"Stddevs max: {stddevs.max():.4f}")
    print(f"Stddevs mean: {stddevs.mean():.4f}")

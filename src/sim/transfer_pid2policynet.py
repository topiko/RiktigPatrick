import numpy as np
import torch
from torch.utils.data import Dataset

from sim.nets import PolicyNetwork
from sim.PIDPolicy import PIDPolicy
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.try_policy import run_episode
from sim.utils import actiondim, model_indim, register_and_make_env


def train_on_episode(X_train, y_train, policy_net: PolicyNetwork):
    loss = torch.nn.MSELoss()
    optim = torch.optim.Adam(policy_net.parameters(), lr=0.00001)

    losses = 0

    ds = Ds(X_train, y_train)

    for X, y in ds:
        optim.zero_grad()
        ymean, ystd = policy_net(X)
        loss_ = loss(torch.hstack([ymean, ystd]), y)

        loss_.backward()
        optim.step()

        losses += loss_.item()

    return losses


class Ds(Dataset):
    def __init__(self, X, y, batch_size: int = 64, shuffle: bool = True):
        self.X = X
        self.y = y
        self._batch_size = batch_size

        if shuffle:
            idx = np.random.permutation(len(self.X))
            self.X = self.X[idx]
            self.y = self.y[idx]

    def __getitem__(self, idx):
        slice_ = slice(idx * self._batch_size, (idx + 1) * self._batch_size)
        if idx * self._batch_size > len(self.X):
            raise IndexError

        return self.X[slice_], self.y[slice_]

    def __len__(self):
        return len(self.X) // self._batch_size


def Xyfromhistory(
    history: np.ndarray,
    idx_d: dict[str, np.ndarray],
    model_input: list[str],
    actions_space: list[str],
) -> tuple[torch.Tensor, torch.Tensor]:
    history = torch.Tensor(history)
    X = torch.hstack(
        [history[:, idxs] for k, idxs in idx_d.items() if k in model_input]
    )

    y_mean = torch.hstack(
        [history[:, idxs] for k, idxs in idx_d.items() if k in actions_space]
    )
    y_std = torch.ones_like(y_mean) * 0.001
    y = torch.hstack([y_mean, y_std])

    return X, y


if __name__ == "__main__":
    ENV_CONFIG["record"] = True
    rpenv = register_and_make_env(ENV_CONFIG, OBS_SPACE)

    rpenv.reset()

    pid_agent = PIDPolicy(10, 0, 0, ENV_CONFIG["step_time"])

    indim = model_indim(rpenv, MODEL_INPUT)
    actiondim = actiondim(rpenv)
    action_space = list(rpenv.action_space.keys())
    policy_net = PolicyNetwork(indim, actiondim)

    i = 0
    while True:
        run_episode(pid_agent, rpenv, i, nrollouts=1)

        history, idx_d = rpenv.state.history

        Xtrain, ytrain = Xyfromhistory(history, idx_d, MODEL_INPUT, action_space)
        loss = train_on_episode(Xtrain, ytrain, policy_net)
        i += 1
        print(loss)
        if loss < 0.1:
            policy_net.store()

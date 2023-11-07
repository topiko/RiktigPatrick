import matplotlib.pyplot as plt
import numpy as np
import torch
from torch.utils.data import Dataset

from sim.custom_policies import NetPolicy, PIDPolicy
from sim.nets import PolicyNetwork
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.try_policy import PlotGroups, plot_state_history, run_episode
from sim.utils import actiondim, model_indim, register_and_make_env


def train_on_episode(X_train, y_train, policy_net: PolicyNetwork, nepochs: int = 2):
    loss = torch.nn.MSELoss(reduction="sum")
    optim = torch.optim.Adam(policy_net.parameters(), lr=0.001)

    _nepochs = 0
    while True:
        ds = Ds(X_train, y_train)
        losses = 0
        for X, y in ds:
            optim.zero_grad()
            ymean, ystd = policy_net(X)
            loss_ = loss(torch.hstack([ymean, ystd]), y)

            loss_.backward()
            optim.step()

            losses += loss_.item()

        if (losses < 10) | (_nepochs > nepochs):
            return losses
        _nepochs += 1


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
    action_space: list[str],
) -> tuple[torch.Tensor, torch.Tensor]:
    history = torch.Tensor(history)

    X = torch.hstack([history[:, idx_d[k]] for k in MODEL_INPUT])

    y_mean = torch.hstack([history[:, idx_d[k]] for k in action_space])
    y_std = torch.ones_like(y_mean) * 0.0001
    y = torch.hstack([y_mean, y_std])

    return X, y


def check_output(
    history: np.ndarray,
    idx_d: dict[str, np.ndarray],
    action_space: list[str],
    policy_net: PolicyNetwork,
):
    X, y = Xyfromhistory(history, idx_d, MODEL_INPUT, action_space)
    ymean, ystd = policy_net(X)

    plg = PlotGroups()
    plg.wheel_left += ("act/left_wheel_net",)
    plg.wheel_right += ("act/right_wheel_net",)

    X, y = Xyfromhistory(history, idx_d, MODEL_INPUT, action_space)
    ymean = policy_net(X)[0].detach().numpy()
    ymean[1:] = ymean[:-1]
    ymean[0] = 0

    history = np.hstack([history, ymean])
    ncols = history.shape[1]
    idx_d["act/left_wheel_net"] = np.arange(ncols - 2, ncols - 1)
    idx_d["act/right_wheel_net"] = np.arange(ncols - 1, ncols)
    plot_state_history(history, idx_d, plg)


if __name__ == "__main__":
    ENV_CONFIG["record"] = True
    ENV_CONFIG["randomize"] = True
    rpenv = register_and_make_env(ENV_CONFIG, OBS_SPACE)

    rpenv.reset()

    pid_agent = PIDPolicy(10, 0, 0, ENV_CONFIG["step_time"])

    indim = model_indim(rpenv, MODEL_INPUT)
    actiondim = actiondim(rpenv)
    action_space = list(rpenv.action_space.keys())
    policy_net = PolicyNetwork(indim, actiondim).load()

    i = 0
    mean_loss = 100
    alpha = 0.3
    minloss = 100
    nrollouts = 10
    while minloss > 0.1:
        xtrains = []
        ytrains = []
        for _ in range(nrollouts):
            run_episode(pid_agent, rpenv, np.random.randint(0, 1e6), nrollouts=1)

            history, idx_d = rpenv.state.history

            Xtrain, ytrain = Xyfromhistory(history, idx_d, MODEL_INPUT, action_space)
            xtrains.append(Xtrain)
            ytrains.append(ytrain)

        Xtrain = torch.vstack(xtrains)
        ytrain = torch.vstack(ytrains)
        loss = train_on_episode(Xtrain, ytrain, policy_net)
        i += 1
        if (mean_loss := alpha * loss + (1 - alpha) * mean_loss) < minloss:
            minloss = mean_loss
            print("Net saved.")
            policy_net.store()
        print(f"{mean_loss:7.3f} | {loss:7.3f}")

    agent = pid_agent  # NetPolicy()  # pid_agent  # PolicyNetwork()
    rpenv.reset()
    run_episode(agent, rpenv, i, nrollouts=1)
    history, idx_d = rpenv.state.history
    check_output(history, idx_d, action_space, policy_net)

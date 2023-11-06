import matplotlib.pyplot as plt
import numpy as np
import torch
from torch.utils.data import Dataset

from sim.nets import PolicyNetwork
from sim.PIDPolicy import PIDPolicy
from sim.sim_config import ENV_CONFIG, MODEL_INPUT, OBS_SPACE
from sim.try_policy import PlotGroups, plot_state_history, run_episode
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

    plt.plot(ymean[:, 0], label="left")
    plt.plot(ymean[:, 1], label="right")
    plt.legend()
    plt.show()

    history = np.hstack([history, ymean])
    ncols = history.shape[1]
    idx_d["act/left_wheel_net"] = np.arange(ncols - 2, ncols - 1)
    idx_d["act/right_wheel_net"] = np.arange(ncols - 1, ncols)
    plot_state_history(history, idx_d, plg)


if __name__ == "__main__":
    ENV_CONFIG["record"] = True
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
    while True:
        run_episode(pid_agent, rpenv, i, nrollouts=1)

        history, idx_d = rpenv.state.history

        Xtrain, ytrain = Xyfromhistory(history, idx_d, MODEL_INPUT, action_space)
        loss = train_on_episode(Xtrain, ytrain, policy_net)
        i += 1
        print(f"{mean_loss:7.3f} | {loss:7.3f}")
        if (mean_loss := alpha * loss + (1 - alpha) * mean_loss) < 10:
            print("Net saved.")
            policy_net.store()
            break

    check_output(history, idx_d, action_space, policy_net)

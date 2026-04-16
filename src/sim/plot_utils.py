"""Plotting utilities for FinishedEpisode data visualization."""

import os

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from riktigpatric.patrick import Actions, StateVarKey
from sim.utils import Episode, MiscKeys


def plot_stream(
    eps: Episode,
    xkey: StateVarKey | Actions,
    ykey: StateVarKey | Actions | MiscKeys,
    ax: Axes,
) -> Axes:
    """Plot a stream of data from an episode."""

    x = eps.get_data(xkey)
    y = eps.get_data(ykey)

    ax.plot(x, y, "-|", lw=1, label=ykey)
    ax.set_ylabel(ykey.value)

    ax.spines.top.set_visible(False)
    ax.spines.right.set_visible(False)

    return ax


def plot_episode(
    eps: Episode,
    keys: list[
        tuple[StateVarKey | Actions, tuple[StateVarKey | Actions | MiscKeys, ...]]
    ],
    figw: float = 22,
    rowh: float = 3,
    save_path: os.PathLike | None = None,
) -> Figure:
    nrows = len(keys)

    fig, axes = plt.subplots(
        nrows=nrows, ncols=1, figsize=(figw, rowh * nrows), sharex=True
    )
    if nrows == 1:
        axes = [axes]  # Ensure axes is always a list for consistency

    for ax, (xkey, ykeys) in zip(axes, keys):
        if not ykeys:
            continue
        for ykey in ykeys:
            plot_stream(eps, xkey, ykey, ax)

        ax.set_title(
            f"{ykey.value} vs {xkey.value}",
            fontsize=10,
            fontweight="bold",
            loc="left",
        )
        ax.legend(frameon=False, loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time [s]")

    if save_path is not None:
        fig.savefig(save_path, dpi=230, bbox_inches="tight")

    return fig

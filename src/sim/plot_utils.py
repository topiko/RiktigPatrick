"""Plotting utilities for Episode data visualization."""

import matplotlib.pyplot as plt

from riktigpatric.patrick import Actions, Observables
from sim.utils import Episode


def plot_stream(
    eps: Episode, xkey: Observables | Actions, ykey: Observables | Actions, ax: plt.Axes
) -> plt.Axes:
    """Plot a stream of data from an episode."""

    x = eps.get_data(xkey)
    y = eps.get_data(ykey)

    ax.plot(x, y, "-|", lw=1, label=ykey)
    ax.set_xlabel(xkey.value)
    ax.set_ylabel(ykey.value)

    ax.spines.top.set_visible(False)
    ax.spines.right.set_visible(False)

    return ax


def plot_episode(
    eps: Episode,
    keys: list[tuple[Observables | Actions, tuple[Observables | Actions, ...]]],
    figw: float = 12,
    rowh: float = 2,
) -> plt.Figure:
    nrows = len(keys)

    fig, axes = plt.subplots(
        nrows=nrows, ncols=1, figsize=(figw, rowh * nrows), sharex=True
    )
    if nrows == 1:
        axes = [axes]  # Ensure axes is always a list for consistency

    for ax, (xkey, ykeys) in zip(axes, keys):
        for ykey in ykeys:
            plot_stream(eps, xkey, ykey, ax)

        ax.set_xlabel(xkey.value)
        ax.legend()

    return fig

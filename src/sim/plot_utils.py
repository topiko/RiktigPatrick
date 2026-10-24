"""Plotting utilities for FinishedEpisode data visualization."""

import os
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from riktigpatric.patrick import Actions, DerivedObs, Observable, StateVarKey, Target
from sim.utils import Episode, MiscKeys

STREAM_UNITS = {
    **dict.fromkeys((Observable.OBS_TIME, Actions.TIME), "s"),
    **dict.fromkeys((Target.TARGET_POS, DerivedObs.CURRENT_POS), "m"),
    **dict.fromkeys((Target.TARGET_VEL, DerivedObs.CURRENT_VEL), "m/s"),
    Observable.ACC: "m/s²",
    **dict.fromkeys((
        Observable.RP_PITCH, Observable.RP_ROLL, Observable.TRUE_PITCH,
        Observable.HEAD_PITCH, Observable.HEAD_TURN, Observable.TRUE_CAMERA_PITCH,
        Target.CAMERA_PITCH_WORLD, Target.HEAD_YAW_NECK, DerivedObs.CAMERA_PITCH_WORLD,
    ), "rad"),
    **dict.fromkeys((
        Observable.GYRO, Observable.LEFT_WHEEL_VEL, Observable.RIGHT_WHEEL_VEL,
        Observable.HEAD_PITCH_VEL, Observable.HEAD_TURN_VEL,
        Target.YAW_RATE, DerivedObs.YAW_RATE,
        Actions.VEL_LEFT_WHEEL, Actions.VEL_RIGHT_WHEEL, Actions.VEL_WHEEL_DIFF,
        Actions.VEL_HEAD_PITCH, Actions.VEL_HEAD_TURN,
    ), "rad/s"),
    **dict.fromkeys((
        Actions.ACC_LEFT_WHEEL, Actions.ACC_RIGHT_WHEEL, Actions.ACC_BOTH_WHEELS,
        Actions.ACC_HEAD_PITCH, Actions.ACC_HEAD_TURN, Actions.ACC_YAW_TURN,
    ), "rad/s²"),
    **dict.fromkeys((
        DerivedObs.POSITION_ERROR, DerivedObs.VELOCITY_ERROR, MiscKeys.LOGPS,
    ), "dimensionless"),
    **dict.fromkeys((
        MiscKeys.REWARDS, MiscKeys.VALUE_ESTIM, MiscKeys.RETURNS, MiscKeys.ADVANTAGES,
        *(key for key in Observable if key.value.startswith("reward/")),
    ), "reward units"),
}


def stream_label(key: StateVarKey | Actions | MiscKeys) -> str:
    return f"{key.value} [{STREAM_UNITS[key]}]"


def plot_stream(
    eps: Episode,
    xkey: StateVarKey | Actions,
    ykey: StateVarKey | Actions | MiscKeys,
    ax: Axes,
) -> Axes:
    """Plot a stream of data from an episode."""

    x = eps.get_data(xkey)
    y = eps.get_data(ykey)

    ax.plot(x, y, "-|", lw=1, label=stream_label(ykey))
    ax.set_ylabel(stream_label(ykey))

    ax.spines.top.set_visible(False)
    ax.spines.right.set_visible(False)

    return ax


def plot_episode(
    eps: Episode,
    keys: list[
        tuple[StateVarKey | Actions, tuple[StateVarKey | Actions | MiscKeys, ...]]
        | tuple[
            StateVarKey | Actions,
            tuple[StateVarKey | Actions | MiscKeys, ...],
            tuple[StateVarKey | Actions | MiscKeys, ...],
        ]
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

    for ax, row in zip(axes, keys):
        xkey, primary = row[:2]
        secondary = row[2] if len(row) == 3 else ()
        if not primary and not secondary:
            continue
        for ykey in primary:
            plot_stream(eps, xkey, ykey, ax)
        handles, labels = ax.get_legend_handles_labels()
        if secondary:
            twin = ax.twinx()
            twin.spines.top.set_visible(False)
            for ykey in secondary:
                plot_stream(eps, xkey, ykey, twin)
            twin_handles, twin_labels = twin.get_legend_handles_labels()
            handles += twin_handles
            labels += twin_labels
        title_key = (primary or secondary)[-1]
        ax.set_title(
            f"{title_key.value} vs {xkey.value}",
            fontsize=10,
            fontweight="bold",
            loc="left",
        )
        ax.legend(handles, labels, frameon=False, loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Time [s]")

    if save_path is not None:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=230, bbox_inches="tight")

    return fig

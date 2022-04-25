import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

plt.style.use('monitor/pltstyle.mplstyle')
FIGS = plt.rcParams["figure.figsize"]

if __name__ == '__main__':
    df = pd.read_hdf('data/rp.hdf5')

    times = (df.rptime.values - df.rptime.min())*1000
    dts = np.diff(times)

    # Comunication times:
    # =======================
    title = f'max/avg/min={dts.max():.1f}/{dts.mean():.1f}/{dts.min():.1f} ms. std = {dts.std():.1f} ms.'
    fig, ax = plt.subplots()
    bins = np.logspace(1, np.log10(1000), 100, base=10)
    ax.hist(dts, bins=bins, color='black', alpha=.6)
    ax.set_title(title)

    ax.semilogx()
    ax.semilogy()
    ax.set_ylabel('Count')
    ax.set_xlabel('rptime diff [ms]')
    fig.subplots_adjust(bottom=.15)
    plt.show()
    # =======================


    # Servos+otors+Orietation
    # =======================
    fig, (axphi, axtheta, axmot, axrpy) = plt.subplots(4,1, sharex=True, figsize=(FIGS[0], FIGS[1]*2))

    l_d = {'color':'black', 'lw':1.2}
    for ax, lab in zip((axphi, axtheta), ('phi', 'theta')):
        target_col = f'head_{lab}_target_angle'
        value_col = f'head_{lab}_angle'

        ax.plot(times, df.loc[:, value_col], '-x', **l_d, label='Value')
        ax.plot(times, df.loc[:, target_col], '-.o', **l_d, label='Target')
        ax.set_title(f'head_{lab}')
        ax.set_ylabel('deg')
        ax.legend()


    for lab, ls in zip(('roll', 'pitch', 'yaw'), ('-.', '-', '--')):
        axrpy.plot(times, df.loc[:, lab].values, ls, **l_d, label=lab)


    axrpy.legend()
    axrpy.set_ylabel('deg')
    axrpy.set_xlabel('Time [ms]')
    plt.show()
    # =======================

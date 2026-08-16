import matplotlib.pyplot as plt
import numpy as np
import os


def plot_results(
    time,
    state,
    input_signal,
    u_max
):

    fig, ax = plt.subplots(
        3,
        1,
        figsize=(9, 8),
        sharex=True
    )

    # Angle
    ax[0].plot(
        time,
        np.rad2deg(state[:, 0])
    )

    ax[0].axhline(
        0,
        color="black",
        linestyle="--"
    )

    ax[0].set_ylabel("theta [deg]")
    ax[0].grid()

    # Angular velocity
    ax[1].plot(
        time,
        state[:, 1]
    )

    ax[1].axhline(
        0,
        color="black",
        linestyle="--"
    )

    ax[1].set_ylabel("omega [rad/s]")
    ax[1].grid()

    # Control input
    ax[2].step(
        time[:-1],
        input_signal,
        where="post"
    )

    ax[2].axhline(
        u_max,
        color="red",
        linestyle="--"
    )

    ax[2].axhline(
        -u_max,
        color="red",
        linestyle="--"
    )

    ax[2].set_ylabel("u [Nm]")
    ax[2].set_xlabel("time [s]")
    ax[2].grid()

    plt.tight_layout()

    os.makedirs("plots", exist_ok=True)
    plt.savefig("plots/results.png", dpi=150)

    plt.show()


def plot_comparison(
    time,
    x_kf,
    u_kf,
    x_raw,
    u_raw,
    u_max
):

    fig, ax = plt.subplots(
        3,
        1,
        figsize=(9, 8),
        sharex=True
    )

    # Angle
    ax[0].plot(
        time,
        np.rad2deg(x_kf[:, 0]),
        label="with KF"
    )

    ax[0].plot(
        time,
        np.rad2deg(x_raw[:, 0]),
        label="no filter",
        alpha=0.7
    )

    ax[0].axhline(
        0,
        color="black",
        linestyle="--"
    )

    ax[0].set_ylabel("theta [deg]")
    ax[0].legend()
    ax[0].grid()

    # Angular velocity
    ax[1].plot(
        time,
        x_kf[:, 1],
        label="with KF"
    )

    ax[1].plot(
        time,
        x_raw[:, 1],
        label="no filter",
        alpha=0.7
    )

    ax[1].axhline(
        0,
        color="black",
        linestyle="--"
    )

    ax[1].set_ylabel("omega [rad/s]")
    ax[1].legend()
    ax[1].grid()

    # Control input
    ax[2].step(
        time[:-1],
        u_kf,
        where="post",
        label="with KF"
    )

    ax[2].step(
        time[:-1],
        u_raw,
        where="post",
        label="no filter",
        alpha=0.7
    )

    ax[2].axhline(
        u_max,
        color="red",
        linestyle="--"
    )

    ax[2].axhline(
        -u_max,
        color="red",
        linestyle="--"
    )

    ax[2].set_ylabel("u [Nm]")
    ax[2].set_xlabel("time [s]")
    ax[2].legend()
    ax[2].grid()

    plt.tight_layout()

    os.makedirs("plots", exist_ok=True)
    plt.savefig("plots/comparison.png", dpi=150)

    plt.show()
"""
Helper functions for visualization optimization results.
"""

import pandas as pd
import numpy as np
import grama as gr
import matplotlib.pyplot as plt


def plot_trajectory(
    state_sequence,
    deployment_state,
    waypt,
    label,
):
    """
    Given a sequence of states, plot the trajectory.
    """
    x_vec = [s[0] for s in state_sequence]
    y_vec = [s[1] for s in state_sequence]
    th_vec = [s[2] for s in state_sequence]

    # plot data
    plt.plot(
        deployment_state[0],
        deployment_state[1],
        "*",
        markersize=20,
        label="deployment",
        color="red",
        zorder=1,
    )
    plt.plot(
        waypt[0],
        waypt[1],
        "*",
        markersize=20,
        label="waypoint",
        color="lime",
        zorder=1,
    )
    plt.plot(x_vec, y_vec, label=None)  # line
    plt.scatter(x_vec, y_vec, c=np.linspace(0, 1, len(x_vec)))  # dots
    plt.quiver(
        x_vec,
        y_vec,
        np.cos(th_vec),
        np.sin(th_vec),
        headwidth=1,
        headlength=1,
        zorder=0,
    )  # headings

    # label data
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title(f"Deterministic Optimal Trajectory w/ Wind Heading of {label}")
    plt.legend(loc="upper left")


def plot_control_vector(control_vec, total_time, timesteps, sharex=True, r=3):
    # grab sequential control vectors
    lin_vels = [round(control_vec[f"v{k}"], r) for k in range(timesteps)]
    ang_vels = [round(control_vec[f"w{k}"], r) for k in range(timesteps)]
    tspan = np.linspace(1, total_time, timesteps)

    # set up plotspace
    fig, (ax1, ax2) = plt.subplots(1, 2)

    # plot control vectors
    ax1.scatter(tspan, lin_vels, c=np.linspace(0, 1, len(tspan)))
    ax1.set_ylabel("ASV Linear Velocity (m/s)")
    ax2.scatter(tspan, ang_vels, c=np.linspace(0, 1, len(tspan)))
    ax2.set_ylabel("ASV Angular Velocity (rad/s)")

    # layout
    fig.supxlabel("Time (s)")
    fig.suptitle("ASV Control Vector Over Time")
    fig.tight_layout()


def plot_state_vector(
    state_sequence, deployment_state, total_time, timesteps, sharex=True, r=3
):
    x_vec = [round(s[0], r) for s in state_sequence]
    y_vec = [round(s[1], r) for s in state_sequence]
    th_vec = [round(s[2], r) for s in state_sequence]

    # timespan
    tspan = np.linspace(1, total_time, timesteps + 1)

    # set up plotspace
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3)

    # plot control vectors
    ax1.scatter(tspan, x_vec, c=np.linspace(0, 1, len(tspan)))
    ax1.set_ylabel("ASV X Position (m)")
    ax2.scatter(tspan, y_vec, c=np.linspace(0, 1, len(tspan)))
    ax2.set_ylabel("ASV Y Position (m)")
    ax3.scatter(tspan, th_vec, c=np.linspace(0, 1, len(tspan)))
    ax3.set_ylabel("ASV Heading (rad)")

    # layout
    fig.supxlabel("Time (s)")
    fig.suptitle("ASV State Vector Over Time")
    fig.tight_layout()


def plot_probabilistic_trajectories(
    control_vec, md_prob, deployment_state, waypt, n_samples=5, seed=42
):
    """
    Sample n_samples random variable vectors from the probabilistic model,
    simulate trajectories using the given (fixed) control vector, and plot them.
    """
    # --- sample random variables from the probabilistic model ---
    # eval_sample draws n_samples rows of the RV marginals only;
    rv_samples = gr.eval_sample(
        md_prob,
        df_det=control_vec,
        n=n_samples,
        seed=seed,
        comm=False,
    )

    # --- plot fixed landmarks ---
    fig, ax = plt.subplots()
    ax.plot(
        deployment_state[0],
        deployment_state[1],
        "*",
        markersize=20,
        label="deployment",
        color="red",
        zorder=3,
    )
    ax.plot(
        waypt[0], waypt[1], "*", markersize=20, label="waypoint", color="lime", zorder=3
    )

    # --- simulate and plot one trajectory per sample row ---
    cmap = plt.get_cmap("cool")
    for i, (_, row) in enumerate(rv_samples.iterrows()):
        state_sequence = prob_state_transition(deployment_state, row)
        x_vec = [s[0] for s in state_sequence]
        y_vec = [s[1] for s in state_sequence]
        th_vec = [s[2] for s in state_sequence]

        # color = cmap(i / max(n_samples - 1, 1))
        ax.plot(x_vec, y_vec, alpha=0.7, label=f"sample {i+1}")
        ax.scatter(
            x_vec, y_vec, c=np.linspace(0, 1, len(x_vec)), cmap="cool", s=20, zorder=2
        )
        ax.quiver(
            x_vec,
            y_vec,
            np.cos(th_vec),
            np.sin(th_vec),
            headwidth=1,
            headlength=1,
            alpha=0.4,
            zorder=1,
        )

    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title(f"Probabilistic Trajectories (n={n_samples} samples)")
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.show()

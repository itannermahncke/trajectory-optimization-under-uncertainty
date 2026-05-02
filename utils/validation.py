"""
My hacky solution of running several instances of the deterministic model with different wind directions.
There is surely a better method of doing this than what follows, but my simple for-loop method of rerunning
the model and altering the D_ANG constant seemed to cause some backend py-grama issues.
"""

import pandas as pd
import numpy as np
import grama as gr
import math

# --- USEFUL CODE FROM MAIN FILE ---

# constants
DT = 0.1  # sec
total_time = 5  # sec
timesteps = int(total_time / DT)
tol = 1e-2

# parameters
deployment_state = (0.0, 0.0, 0.0)  # x, y, theta
waypt = (4.0, 4.0)  # x, y

# constraint values
V_MAX = 3.0  # m/s
W_MAX = 3.0  # rad/s

# deterministic disturbance values
DET = False
D_MAG = 0.5
D_ANG = math.pi / 2


def det_state_transition(
    init_state: tuple[float, float, float],
    input_vec: pd.Series,
):
    """
    Helper function to perform state transitions from an initial state using a control vector and random variables.

    This is a deterministic function that does not account for sampled uncertainty values.
    """
    # grab initial state
    states = [init_state]

    # grab sequential control vectors
    lin_vels = [float(input_vec[f"v{k}"]) for k in range(timesteps)]
    ang_vels = [float(input_vec[f"w{k}"]) for k in range(timesteps)]

    # iterate through control vector
    for vk, wk in zip(lin_vels, ang_vels):
        # state variables
        xk, yk, thk = states[-1]

        # disturbance in xy
        d_ang_flip = D_ANG + math.pi
        d_ang_wrap = np.arctan2(np.sin(d_ang_flip), np.cos(d_ang_flip))
        d_x = D_MAG * math.cos(d_ang_wrap)
        d_y = D_MAG * math.sin(d_ang_wrap)

        # kinematics
        xk1 = xk + DT * (vk * math.cos(thk) + d_x)
        yk1 = yk + DT * (vk * math.sin(thk) + d_y)
        thk1 = thk + DT * (wk)
        thk1 = np.arctan2(np.sin(thk1), np.cos(thk1))  # wrap to [-pi, pi]

        # iterate state
        states.append((xk1, yk1, thk1))

    # return final state sequence
    return states


# primary objective functions
def D_total_distance(input_vec):
    """
    Vectorized function to calculate the D (cumulative distance from waypoint across time) column of a scenario row.
    """
    # sum distance
    sum_distance = 0

    state_sequence = det_state_transition(deployment_state, input_vec)

    # iterate through state sequence
    for s in state_sequence:
        # state variables
        xk, yk, _ = s

        # add to sum distance
        sum_distance += np.sqrt((xk - waypt[0]) ** 2 + (yk - waypt[1]) ** 2)

    # return raw distance from state to waypoint
    return sum_distance


def J_final_energy(input_vec):
    """
    Vectorized function to calculate the J (final energy) column of a scenario row.
    """
    # relative energy consumption
    relative_energy = 0.0

    # grab sequential control vectors
    lin_vels = [float(input_vec[f"v{k}"]) for k in range(timesteps)]
    ang_vels = [float(input_vec[f"w{k}"]) for k in range(timesteps)]

    # square all velocities
    for vel in lin_vels + ang_vels:
        relative_energy += vel**2

    # return total relative energy
    return relative_energy


# grama variable vectors
controls = [
    f"w{int(k/2)}" if k % 2 == 0 else f"v{int(k/2)}" for k in range(timesteps * 2)
]  # lin vel, ang vel
objectives = ["D", "J"]  # cumulative distance, energy consumed
states = [f"x{k}" for k in range(timesteps)]  # state (x, y, theta)


def model_trajectory(df_in: pd.DataFrame):
    """
    DataFrame-based method of extracting objective function values from a given control input sequence.

    Args:
        df_in: A DataFrame in which each column represents a decision variable or a random variable.

    Returns:
        df_out: A DataFrame in which each column represents an objective function value.
    """
    # establish the out dataframe
    df_out = pd.DataFrame()

    # establish vectorized functions for objectives
    df_out["D"] = df_in.apply(D_total_distance, axis="columns")
    df_out["J"] = df_in.apply(J_final_energy, axis="columns")

    # return
    return df_out


# --- USEFUL CODE FROM MAIN FILE ---


def perform_validation(test_angles):
    """
    Validate the deterministic model by demonstrating its performance under a given set of test disturbance angles.
    """
    opt_controls = []

    for d_ang in test_angles:
        # set the angle
        D_ANG = d_ang

        # redefine the model
        md_asv_det = (
            # name model
            gr.Model("ASV Motion Kinematics")
            # state transition kinematics
            >> gr.cp_vec_function(
                fun=model_trajectory,
                var=controls,
                out=objectives,
            )
            # constrain linear velocity commands
            >> gr.cp_bounds(**{f"v{k}": (-0.0, +V_MAX) for k in range(timesteps)})
            # constrain angular velocity commands
            >> gr.cp_bounds(**{f"w{k}": (-W_MAX, +W_MAX) for k in range(timesteps)})
        )

        # unconstrained optimization on primary objective
        df_single = md_asv_det >> gr.ev_min(
            out_min="D",
            n_restart=3,
        )
        opt_val_D = df_single["D"].min()

        # lexicographic method on secondary objective
        df_multi: pd.DataFrame = (
            md_asv_det
            >> gr.cp_vec_function(
                fun=lambda df: gr.df_make(
                    Dtol_leq=df["D"] - opt_val_D - tol,
                ),
                var=["D"],
                out=["Dtol_leq"],
            )
            >> gr.ev_min(
                out_min="J",
                n_restart=3,
                out_leq=["Dtol_leq"],
            )
        )
        opt_controls.append(df_multi[controls].loc[df_multi["D"].idxmin()])

    return opt_controls

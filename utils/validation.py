"""
My hacky solution of running several instances of the deterministic model with different wind directions.
This is necessary because the way that py-grama parallelizes operations in the background means that my attempts
to run several trials with varying wind directions would get conflated results, even when each trial succeeded when
running independently.
"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import grama as gr
import math

import utils.visualization as viz

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

# grama variable vectors
controls = [
    f"w{int(k/2)}" if k % 2 == 0 else f"v{int(k/2)}" for k in range(timesteps * 2)
]  # lin vel, ang vel
objectives = ["D", "J"]  # cumulative distance, energy consumed
states = [f"x{k}" for k in range(timesteps)]  # state (x, y, theta)


# primary objective functions
def D_total_distance(input_vec):
    sum_distance = 0
    state_sequence = state_transition(deployment_state, input_vec)
    for s in state_sequence:
        xk, yk, _ = s
        sum_distance += np.sqrt((xk - waypt[0]) ** 2 + (yk - waypt[1]) ** 2)
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


def model_trajectory(df_in):
    df_out = pd.DataFrame()
    df_out["D"] = df_in.apply(D_total_distance, axis="columns")
    df_out["J"] = df_in.apply(J_final_energy, axis="columns")
    return df_out


# --- END USEFUL CODE FROM MAIN FILE ---


def state_transition(init_state, input_vec):
    """
    Slightly modified state transition function that expects disturbance heading as a decision variable.
    """
    # state sequencing and control variables
    states = [init_state]
    lin_vels = [float(input_vec[f"v{k}"]) for k in range(timesteps)]
    ang_vels = [float(input_vec[f"w{k}"]) for k in range(timesteps)]

    # for each timestep
    for vk, wk in zip(lin_vels, ang_vels):
        # extract state
        xk, yk, thk = states[-1]

        # extract disturbance
        d_ang = input_vec["dth"]
        d_x = D_MAG * math.cos(d_ang)
        d_y = D_MAG * math.sin(d_ang)

        # kinematics
        xk1 = xk + DT * (vk * math.cos(thk) + d_x)
        yk1 = yk + DT * (vk * math.sin(thk) + d_y)
        thk1 = np.arctan2(np.sin(thk + DT * wk), np.cos(thk + DT * wk))

        # save state
        states.append((xk1, yk1, thk1))
    return states


def perform_validation(test_angles):
    """
    Validate the deterministic model by demonstrating its performance under a given set of test disturbance angles.

    Each test angle is associated with a new model identical to the deterministic model but with disturbance angle as a constant decision variable.
    """
    opt_controls = []

    for d_ang in test_angles:
        # redefine the model
        md_asv_det = (
            # name model
            gr.Model("ASV Motion Kinematics")
            # state transition kinematics
            >> gr.cp_vec_function(
                fun=model_trajectory,
                var=controls + ["dth"],
                out=objectives,
            )
            # constrain linear velocity commands
            >> gr.cp_bounds(**{f"v{k}": (-0.0, +V_MAX) for k in range(timesteps)})
            # constrain angular velocity commands
            >> gr.cp_bounds(**{f"w{k}": (-W_MAX, +W_MAX) for k in range(timesteps)})
            # constrain d_ang
            >> gr.cp_bounds(dth=(d_ang, d_ang))
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
        opt_controls.append(df_multi[controls + ["dth"]].loc[df_multi["D"].idxmin()])

    plot_validation_results(test_angles, opt_controls)


def plot_validation_results(test_angles, opt_controls):
    """
    Plot results.
    """
    for k, (ang, vec) in enumerate(zip(test_angles, opt_controls)):
        viz.plot_trajectory(
            state_transition(deployment_state, vec),
            deployment_state,
            waypt,
            round(ang, 3),
        )
        plt.show()

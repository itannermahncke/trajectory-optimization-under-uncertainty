"""
Helper functions for verifying the deterministic model.
"""

import math
import numpy as np
import pandas as pd

# --- helpers ---

# constants
DT = 0.1  # sec
total_time = 5  # sec
timesteps = int(total_time / DT)
tol = 1e-2
D_ANG = 0


def make_input_vec(v=0.0, w=0.0):
    """Create a uniform control input Series for all timesteps."""
    data = {}
    for k in range(timesteps):
        data[f"v{k}"] = v
        data[f"w{k}"] = w
    return pd.Series(data)


def states_close(s1, s2, atol=1e-6):
    """Check two state tuples are approximately equal."""
    return all(abs(a - b) < atol for a, b in zip(s1, s2))


def report(name, passed):
    status = "PASS" if passed else "FAIL"
    print(f"[{status}] {name}")
    return passed


def run_all_tests(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    tests = [
        test_output_length,
        test_initial_state_preserved,
        test_zero_inputs_zero_disturbance_no_motion,
        test_pure_linear_velocity_heading_zero,
        test_pure_angular_velocity_no_translation,
        test_heading_wraps_correctly,
        test_disturbance_direction_from_east_pushes_west,
        test_disturbance_direction_from_north_pushes_south,
        test_disturbance_magnitude_scales_displacement,
        test_non_default_initial_state,
    ]
    results = [t(det_state_transition) for t in tests]
    passed = sum(results)
    print(f"\n{passed}/{len(results)} tests passed")


# --- tests ---


def test_output_length(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (0.0, 0.0, 0.0)
    result = det_state_transition(init, make_input_vec(v=0.0, w=0.0))
    return report("output length is timesteps+1", len(result) == timesteps + 1)


def test_initial_state_preserved(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (1.5, -2.3, 0.7)
    result = det_state_transition(init, make_input_vec(v=0.0, w=0.0))
    return report("initial state preserved as first element", result[0] == init)


def test_zero_inputs_zero_disturbance_no_motion(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (0.0, 0.0, 0.0)
    result = det_state_transition(init, make_input_vec(v=0.0, w=0.0))
    final = result[-1]
    passed = abs(final[0]) < 1e-9 and abs(final[1]) < 1e-9 and abs(final[2]) < 1e-9
    return report("zero inputs + zero disturbance = no motion", passed)


def test_pure_linear_velocity_heading_zero(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (0.0, 0.0, 0.0)
    v = 1.0
    result = det_state_transition(init, make_input_vec(v=v, w=0.0))
    final = result[-1]
    expected_x = v * DT * timesteps
    passed = (
        abs(final[0] - expected_x) < 1e-6
        and abs(final[1]) < 1e-6
        and abs(final[2]) < 1e-6
    )
    return report("pure linear velocity at heading=0 moves only in +x", passed)


def test_pure_angular_velocity_no_translation(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (2.0, 3.0, 0.0)
    result = det_state_transition(init, make_input_vec(v=0.0, w=1.0))
    passed = all(
        abs(s[0] - init[0]) < 1e-6 and abs(s[1] - init[1]) < 1e-6 for s in result
    )
    return report("zero linear velocity + angular velocity = rotation only", passed)


def test_heading_wraps_correctly(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (0.0, 0.0, 0.0)
    result = det_state_transition(init, make_input_vec(v=0.0, w=3.0))
    passed = all(-math.pi <= s[2] <= math.pi for s in result)
    return report("heading wraps within [-pi, pi]", passed)


def test_disturbance_direction_from_east_pushes_west(det_state_transition):
    init = (0.0, 0.0, 0.0)
    result = det_state_transition(
        init, make_input_vec(v=0.0, w=0.0), d_mag=1.0, d_ang=0.0
    )
    final = result[-1]
    passed = final[0] < -1e-6 and abs(final[1]) < 1e-6
    return report("D_ANG=0 (wind from east) pushes ASV in -x", passed)


def test_disturbance_direction_from_north_pushes_south(det_state_transition):
    init = (0.0, 0.0, 0.0)
    result = det_state_transition(
        init, make_input_vec(v=0.0, w=0.0), d_mag=1.0, d_ang=math.pi / 2
    )
    final = result[-1]
    passed = abs(final[0]) < 1e-6 and final[1] < -1e-6
    return report("D_ANG=pi/2 (wind from north) pushes ASV in -y", passed)


def test_disturbance_magnitude_scales_displacement(det_state_transition):
    inp = make_input_vec(v=0.0, w=0.0)
    init = (0.0, 0.0, 0.0)
    x1 = det_state_transition(init, inp, d_mag=1.0, d_ang=0.0)[-1][0]
    x2 = det_state_transition(init, inp, d_mag=2.0, d_ang=0.0)[-1][0]
    passed = abs(x2 - 2 * x1) < 1e-6
    return report("disturbance displacement scales linearly with D_MAG", passed)


def test_non_default_initial_state(det_state_transition):
    global D_ANG, D_MAG
    D_ANG, D_MAG = 0.0, 0.0
    init = (5.0, -3.0, math.pi / 2)
    v = 1.0
    result = det_state_transition(init, make_input_vec(v=v, w=0.0))
    final = result[-1]
    expected_y = init[1] + v * DT * timesteps
    passed = (
        abs(final[0] - init[0]) < 1e-6
        and abs(final[1] - expected_y) < 1e-6
        and abs(final[2] - math.pi / 2) < 1e-6
    )
    return report("non-origin initial state with heading=pi/2 moves in +y", passed)

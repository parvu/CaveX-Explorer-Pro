"""Pure-Python port of cavex_sic_slam's dynamics_model.hpp/.cpp
(include/cavex_sic_slam/dynamics_model.hpp) -- same BlueROV2 thruster
geometry and quadratic-drag constants, verified against this repo's own
bluerov2_sim/model.sdf (its Hydrodynamics plugin's xUabsU/yVabsV/zWabsW
match exactly: -33.732/-54.16/-73.225). No ROS/GTSAM dependency, so it can
be unit-tested standalone, same convention as the C++ original.

Ported rather than reused directly because sic_slam_graph_backend.py is a
pure-Python ISAM2 node (python3-gtsam bindings), not a C++ node that could
link cavex_sic_slam's compiled library -- see current_factor.py for the
GTSAM CustomFactor that uses this.
"""
import numpy as np

# Thrust-direction unit vectors (body frame), thrusters 1-6, matching
# defaultBlueRov2Geometry() exactly. Position is omitted -- CurrentFactor
# only needs linear velocity, no torque terms (same as the C++ original).
KD = 0.70710678118654752
THRUSTER_DIRECTIONS = np.array([
    [-KD, -KD, 0.0],
    [-KD, KD, 0.0],
    [KD, -KD, 0.0],
    [KD, KD, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, 0.0, -1.0],
])

# Diagonal quadratic-drag coefficients (N per (m/s)^2), always negative.
DRAG_COEFFICIENTS = np.array([-33.732, -54.16, -73.225])


def body_force(thrust_n):
    """Sum of thrust_n[i] * direction[i] over all six thrusters."""
    thrust_n = np.asarray(thrust_n, dtype=float)
    return thrust_n @ THRUSTER_DIRECTIONS


def quasi_steady_velocity(force):
    """Per-axis solve of force + drag*v*|v| == 0: v = sign(F)*sqrt(|F|/|coef|)."""
    mag = np.abs(DRAG_COEFFICIENTS)
    v = np.sqrt(np.abs(force) / np.where(mag < 1e-12, 1.0, mag))
    v = np.where(mag < 1e-12, 0.0, v)
    return np.where(force >= 0.0, v, -v)


def predict_body_velocity(thrust_n):
    """Convenience composition: body_force() then quasi_steady_velocity()."""
    return quasi_steady_velocity(body_force(thrust_n))


if __name__ == "__main__":
    # ponytail: minimal self-check, mirrors the C++ test_current_factor.cpp
    # spirit (not a line-for-line port of it).
    zero = predict_body_velocity([0.0] * 6)
    assert np.allclose(zero, 0.0), zero

    # Pure +X thrust (thrusters 3,4 point +X-ish, 1,2 point -X-ish per the
    # vectored layout): driving 3&4 forward should give a positive-X result.
    v = predict_body_velocity([0.0, 0.0, 10.0, 10.0, 0.0, 0.0])
    assert v[0] > 0.0, v
    assert np.isclose(v[1], 0.0, atol=1e-9), v

    # Pure -Z thrust (vertical thrusters 5,6 point -Z): should give
    # positive net -Z force -> quasi-steady velocity solves to negative Z
    # (force>=0 branch flips sign only for force<0; -Z thrust is negative
    # force on Z axis since direction is (0,0,-1)*positive thrust).
    v = predict_body_velocity([0.0, 0.0, 0.0, 0.0, 10.0, 10.0])
    assert v[2] < 0.0, v

    print("current_dynamics self-check OK")

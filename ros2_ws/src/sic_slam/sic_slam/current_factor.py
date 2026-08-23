"""GTSAM CurrentFactor for sic_slam's Point3-only dead-reckoning graph.

Mirrors cavex_sic_slam's real CurrentFactor (include/cavex_sic_slam/
current_factor.hpp) -- same observability mechanism (ground velocity minus
predicted through-water velocity reveals current) and the same
current_dynamics.py physics -- but adapted to this graph's actual state,
which is Point3 POSITIONS only (no Pose3 rotation, no separate velocity
variable, unlike cavex_sic_slam's Pose3+Velocity+CombinedImuFactor graph).
A literal port isn't possible here for that reason; this is the analogous
factor for a displacement-based graph instead of a velocity-based one:

    residual = (X_cur - X_prev) - (v_pred_through_water + C) * dt

i.e. ground-frame displacement over dt equals through-water displacement
plus the current's own displacement contribution. No rotation term is
needed because this graph's dead-reckoning already has none (raw IMU
accel is integrated directly, not rotated into a world frame -- a
pre-existing simplification of this prototype, not something this factor
changes).

Built with gtsam.CustomFactor (Python-side factor definition, no C++
compilation needed) rather than the C++ NoiseModelFactor3 the original
uses, since sic_slam_graph_backend.py is a pure python3-gtsam node.
"""
import gtsam
import numpy as np

from sic_slam.current_dynamics import predict_body_velocity


def make_current_factor(prev_key, cur_key, current_key, thrust_n, dt, noise_model):
    """Returns a gtsam.CustomFactor. thrust_n is captured now (a constant
    for this factor instance, like the C++ original's constructor-time
    v_pred_ -- current thrust is a measurement, not an optimization
    variable)."""
    v_pred = predict_body_velocity(thrust_n)

    def error_func(this, values, jacobians):
        x_prev = values.atPoint3(prev_key)
        x_cur = values.atPoint3(cur_key)
        c = values.atVector(current_key)

        residual = (x_cur - x_prev) - (v_pred + c) * dt

        if jacobians is not None:
            jacobians[0] = -np.eye(3)
            jacobians[1] = np.eye(3)
            jacobians[2] = -np.eye(3) * dt

        return residual

    return gtsam.CustomFactor(
        noise_model, [prev_key, cur_key, current_key], error_func)

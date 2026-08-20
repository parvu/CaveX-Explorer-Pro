# Copyright 2026 CaveX Explorer Pro
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Pure control math for Drift/Current Suppression (DCS).

No ROS imports here -- this module is directly unit-testable and is the
only place the control logic lives; dcs_controller.py is a thin ROS
wrapper around it.
"""
import math


def rotate_world_to_body(vx, vy, yaw):
    """Rotate a world-frame (vx, vy) into the body frame at heading yaw."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    body_x = c * vx + s * vy
    body_y = -s * vx + c * vy
    return body_x, body_y


def confidence_from_covariance(cov_trace, threshold):
    """
    Map current-estimate covariance trace to a [0, 1] confidence.

    0 at or above threshold (estimate too uncertain to trust), 1 at zero
    covariance, linear in between. Before the graph has observed enough
    motion to separate current from drift, this stays near zero and DCS
    degrades to plain feedback instead of confidently steering into a bad
    estimate.
    """
    if cov_trace >= threshold:
        return 0.0
    if cov_trace <= 0.0:
        return 1.0
    return 1.0 - (cov_trace / threshold)


def feed_forward(
        desired_body_vx, desired_body_vy,
        current_world_vx, current_world_vy,
        yaw, confidence):
    """
    Compensate the desired body velocity for estimated current.

    Rotates the current estimate into the body frame and subtracts it
    (scaled by confidence) from the desired body velocity, so the
    commanded through-water velocity produces the desired over-ground
    velocity.
    """
    current_body_x, current_body_y = rotate_world_to_body(
        current_world_vx, current_world_vy, yaw)
    ff_vx = desired_body_vx - confidence * current_body_x
    ff_vy = desired_body_vy - confidence * current_body_y
    return ff_vx, ff_vy


class PiController:
    """A minimal PI controller with anti-windup clamping."""

    def __init__(self, kp, ki, i_max):
        self._kp = kp
        self._ki = ki
        self._i_max = i_max
        self._integral = 0.0

    def update(self, error, dt):
        self._integral += error * dt
        self._integral = max(-self._i_max, min(self._i_max, self._integral))
        return self._kp * error + self._ki * self._integral

    def reset(self):
        self._integral = 0.0


def cross_track_error(actual_body_vx, actual_body_vy, desired_body_vx, desired_body_vy):
    """
    Signed lateral (cross-track) velocity error, body frame.

    Positive desired heading is along (desired_body_vx, desired_body_vy);
    the cross-track component is the actual velocity's projection onto the
    left-hand perpendicular of that heading, which is what a lateral PI
    correction needs to null out.
    """
    heading_norm = math.hypot(desired_body_vx, desired_body_vy)
    if heading_norm < 1e-6:
        return 0.0
    perp_x = -desired_body_vy / heading_norm
    perp_y = desired_body_vx / heading_norm
    return actual_body_vx * perp_x + actual_body_vy * perp_y


def saturate(vx, vy, max_speed_mps):
    """Clamp (vx, vy) to max_speed_mps, reporting the shortfall magnitude."""
    speed = math.hypot(vx, vy)
    if speed <= max_speed_mps or speed < 1e-9:
        return vx, vy, 0.0
    scale = max_speed_mps / speed
    return vx * scale, vy * scale, speed - max_speed_mps

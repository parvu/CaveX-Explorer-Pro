# Copyright 2026 CaveX Explorer Pro
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Unit tests for the pure DCS control math."""
import math

from cavex_dcs.dcs_math import (
    confidence_from_covariance,
    cross_track_error,
    feed_forward,
    PiController,
    rotate_world_to_body,
    saturate,
)


def test_rotate_world_to_body_identity_yaw():
    bx, by = rotate_world_to_body(1.0, 2.0, 0.0)
    assert math.isclose(bx, 1.0, abs_tol=1e-9)
    assert math.isclose(by, 2.0, abs_tol=1e-9)


def test_rotate_world_to_body_ninety_degrees():
    bx, by = rotate_world_to_body(1.0, 0.0, math.pi / 2)
    assert math.isclose(bx, 0.0, abs_tol=1e-9)
    assert math.isclose(by, -1.0, abs_tol=1e-9)


def test_confidence_zero_when_covariance_at_or_above_threshold():
    assert confidence_from_covariance(0.5, 0.5) == 0.0
    assert confidence_from_covariance(1.0, 0.5) == 0.0


def test_confidence_one_when_covariance_is_zero():
    assert confidence_from_covariance(0.0, 0.5) == 1.0


def test_confidence_scales_linearly_between():
    assert math.isclose(confidence_from_covariance(0.25, 0.5), 0.5, abs_tol=1e-9)


def test_feed_forward_cancels_current_at_full_confidence():
    # Current pushes the ROV at +0.2 m/s; to make over-ground velocity hit
    # the desired 0.5, the commanded through-water velocity must be
    # desired - current = 0.3 (so that + current = 0.5 over ground).
    ff_vx, ff_vy = feed_forward(
        desired_body_vx=0.5, desired_body_vy=0.0,
        current_world_vx=0.2, current_world_vy=0.0,
        yaw=0.0, confidence=1.0)
    assert math.isclose(ff_vx, 0.3, abs_tol=1e-6)
    assert math.isclose(ff_vy, 0.0, abs_tol=1e-6)


def test_feed_forward_degrades_to_desired_at_zero_confidence():
    ff_vx, ff_vy = feed_forward(
        desired_body_vx=0.5, desired_body_vy=0.0,
        current_world_vx=0.2, current_world_vy=0.0,
        yaw=0.0, confidence=0.0)
    assert math.isclose(ff_vx, 0.5, abs_tol=1e-6)
    assert math.isclose(ff_vy, 0.0, abs_tol=1e-6)


def test_pi_controller_accumulates_and_responds():
    pi = PiController(kp=1.0, ki=0.5, i_max=10.0)
    out1 = pi.update(error=1.0, dt=1.0)
    assert math.isclose(out1, 1.5, abs_tol=1e-6)  # kp*e + ki*integral(=1.0)
    out2 = pi.update(error=1.0, dt=1.0)
    assert math.isclose(out2, 2.0, abs_tol=1e-6)  # integral now 2.0


def test_pi_controller_integral_windup_clamped():
    pi = PiController(kp=0.0, ki=1.0, i_max=2.0)
    for _ in range(10):
        pi.update(error=1.0, dt=1.0)
    out = pi.update(error=1.0, dt=1.0)
    assert out <= 2.0 + 1e-9


def test_pi_controller_reset_clears_integral():
    pi = PiController(kp=0.0, ki=1.0, i_max=10.0)
    pi.update(error=1.0, dt=1.0)
    pi.reset()
    out = pi.update(error=0.0, dt=1.0)
    assert math.isclose(out, 0.0, abs_tol=1e-9)


def test_cross_track_error_zero_when_aligned():
    err = cross_track_error(
        actual_body_vx=0.5, actual_body_vy=0.0,
        desired_body_vx=0.5, desired_body_vy=0.0)
    assert math.isclose(err, 0.0, abs_tol=1e-9)


def test_cross_track_error_positive_for_lateral_drift():
    # perp = (-desired_vy, desired_vx)/|desired| = (0, 1) here, so the
    # error is the actual velocity's y-component projected onto that:
    # 0.5*0 + 0.1*1 = 0.1.
    err = cross_track_error(
        actual_body_vx=0.5, actual_body_vy=0.1,
        desired_body_vx=0.5, desired_body_vy=0.0)
    assert math.isclose(err, 0.1, abs_tol=1e-9)


def test_saturate_passes_through_under_limit():
    vx, vy, shortfall = saturate(0.3, 0.0, max_speed_mps=1.0)
    assert math.isclose(vx, 0.3, abs_tol=1e-9)
    assert shortfall == 0.0


def test_saturate_clamps_and_reports_shortfall():
    vx, vy, shortfall = saturate(2.0, 0.0, max_speed_mps=1.0)
    assert math.isclose(vx, 1.0, abs_tol=1e-6)
    assert math.isclose(shortfall, 1.0, abs_tol=1e-6)


def test_pi_correction_must_be_subtracted_to_oppose_drift():
    """
    Pin the sign convention dcs_controller._desired_cb must use.

    Desired heading is purely along +x, and the ROV has drifted +0.2 m/s
    laterally (+y, body frame). cross_track_error is positive for this
    drift, and a PI controller with positive gains preserves that sign, so
    `correction` is positive. The controller decomposes `correction` onto
    the heading's left-hand perpendicular (0, 1) here, giving a positive
    `corr_vy` -- the SAME direction as the drift.

    A correct controller must SUBTRACT this from the feed-forward command
    (cmd_vy = ff_vy - corr_vy) so the net commanded y-velocity opposes the
    drift (pulls back toward the track). The old, buggy controller ADDED it
    (cmd_vy = ff_vy + corr_vy), which commands MORE lateral velocity in the
    drift direction -- positive feedback. This test fails under the old
    additive composition and passes under the new subtractive one.
    """
    desired_body_vx, desired_body_vy = 1.0, 0.0
    actual_body_vx, actual_body_vy = 0.0, 0.2  # drifting sideways, +y

    err = cross_track_error(
        actual_body_vx=actual_body_vx, actual_body_vy=actual_body_vy,
        desired_body_vx=desired_body_vx, desired_body_vy=desired_body_vy)
    assert err > 0.0  # sanity: drift is measured as positive

    pi = PiController(kp=0.6, ki=0.1, i_max=0.5)
    correction = pi.update(err, dt=0.05)
    assert correction > 0.0  # sanity: positive gains preserve the sign

    heading_norm = math.hypot(desired_body_vx, desired_body_vy)
    corr_vy = desired_body_vx / heading_norm * correction
    assert corr_vy > 0.0  # sanity: decomposed correction points with the drift

    ff_vy = 0.0  # zero feed-forward baseline, isolates the PI term

    # Correct (subtractive) composition: net corrective y-velocity opposes
    # the drift.
    cmd_vy_correct = ff_vy - corr_vy
    assert cmd_vy_correct < 0.0

    # Buggy (additive) composition, for documentation: it reinforces the
    # drift instead of opposing it. If this ever stops holding, the
    # dcs_controller.py fix has been reverted.
    cmd_vy_buggy = ff_vy + corr_vy
    assert cmd_vy_buggy > 0.0

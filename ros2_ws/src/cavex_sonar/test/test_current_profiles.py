"""
Unit tests for the current profile generator in current_field_node.py.

Pure maths, no ROS: the profile functions are deliberately importable without
a running ROS graph so they can be tested directly.
"""

from cavex_sonar.current_field_node import current_at

import pytest


def test_constant_profile_is_time_invariant():
    p = {'vx': 0.3, 'vy': -0.1, 'vz': 0.0}
    assert current_at('constant', 0.0, p) == pytest.approx((0.3, -0.1, 0.0))
    assert current_at('constant', 123.4, p) == pytest.approx((0.3, -0.1, 0.0))


def test_step_profile_is_zero_before_the_step_and_full_after():
    p = {'vx': 0.4, 'vy': 0.0, 'vz': 0.0, 'step_time': 10.0}
    assert current_at('step', 9.9, p) == pytest.approx((0.0, 0.0, 0.0))
    assert current_at('step', 10.1, p) == pytest.approx((0.4, 0.0, 0.0))


def test_sinusoidal_profile_oscillates_within_amplitude():
    p = {'vx': 0.5, 'vy': 0.0, 'vz': 0.0, 'period_s': 20.0}
    for t in [i * 0.5 for i in range(80)]:
        x, _, _ = current_at('sinusoidal', t, p)
        assert -0.5 - 1e-9 <= x <= 0.5 + 1e-9


def test_sinusoidal_profile_completes_one_cycle_per_period():
    p = {'vx': 0.5, 'vy': 0.0, 'vz': 0.0, 'period_s': 20.0}
    assert current_at('sinusoidal', 0.0, p)[0] == pytest.approx(
        current_at('sinusoidal', 20.0, p)[0], abs=1e-9)


def test_unknown_profile_raises_rather_than_silently_returning_zero():
    # A typo in a launch argument must fail loudly, not quietly disable the
    # disturbance and invalidate an entire evaluation run.
    with pytest.raises(ValueError):
        current_at('sinusiodal', 1.0, {'vx': 0.5})

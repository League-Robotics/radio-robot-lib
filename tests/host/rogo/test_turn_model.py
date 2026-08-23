"""tests/host/rogo/test_turn_model.py -- `rogo.turn_model.compute_turn`
as a pure function: no transport, no config file, no Session -- just the
ported rotation math (ticket 003's own Testing plan: "the rotation-model
math as a pure function (no transport needed)").

Expected values below were computed independently in Python (the same
formula this module documents: `theta = radians((angle - offset) /
gain)`, `target_arc = |theta| * trackwidth / (2 * slip)`, `duration_ms =
round(1000 * target_arc / speed)`) rather than by calling
`compute_turn()` twice, so a bug in the implementation can't cancel
itself out against an equally-buggy expectation.
"""

from __future__ import annotations

import pytest

from rogo import turn_model


# ---------------------------------------------------------------------------
# The plain kinematic estimate (no gain/offset correction, slip=1.0) --
# elite's own "no motor model" fallback branch.
# ---------------------------------------------------------------------------

def test_positive_angle_turns_right_forward_left_backward():
    # sign convention (module docstring): positive angle -> CCW -> right
    # wheel forward, left wheel backward.
    cmd_l, cmd_r, duration_ms = turn_model.compute_turn(90, 200, 115, 1.0)
    assert (cmd_l, cmd_r) == (-200, 200)
    assert duration_ms == 452


def test_negative_angle_flips_the_wheel_signs_same_magnitude_duration():
    cmd_l, cmd_r, duration_ms = turn_model.compute_turn(-90, 200, 115, 1.0)
    assert (cmd_l, cmd_r) == (200, -200)
    assert duration_ms == 452  # same |angle| -> same duration


def test_zero_angle_is_a_zero_duration_no_op():
    cmd_l, cmd_r, duration_ms = turn_model.compute_turn(0, 200, 115, 1.0)
    assert duration_ms == 0
    assert (cmd_l, cmd_r) == (-200, 200)  # sign(0) resolves to the CCW branch


def test_larger_trackwidth_takes_longer():
    _, _, short = turn_model.compute_turn(90, 200, 100, 1.0)
    _, _, long_ = turn_model.compute_turn(90, 200, 200, 1.0)
    assert long_ > short


def test_full_rotation_360_degrees():
    _, _, duration_ms = turn_model.compute_turn(360, 200, 115, 1.0)
    assert duration_ms == 1806


# ---------------------------------------------------------------------------
# rotational_slip fallback -- calibration data absent (None) falls back
# to a no-slip (1.0) linear estimate, matching elite's own
# `slip_val = float(cfg_slip) if cfg_slip is not None else 1.0`.
# ---------------------------------------------------------------------------

def test_none_slip_falls_back_to_no_slip_estimate():
    with_none = turn_model.compute_turn(90, 200, 115, None)
    with_explicit_one = turn_model.compute_turn(90, 200, 115, 1.0)
    assert with_none == with_explicit_one


def test_slip_less_than_one_increases_commanded_duration():
    # Slip < 1.0 means the robot under-rotates for a given arc -- the
    # model compensates by commanding a LONGER duration for the same
    # angle, matching _turn_command's own `target_arc / (2 * slip)`.
    _, _, no_slip = turn_model.compute_turn(90, 200, 115, 1.0)
    _, _, with_slip = turn_model.compute_turn(90, 200, 115, 0.75)
    assert with_slip == 602
    assert with_slip > no_slip


# ---------------------------------------------------------------------------
# Optional linear correction (gain/offset) -- elite's own
# `rotation_gain`/`rotation_offset_deg`, staged (unused) in
# config/robots/tovez.json today.
# ---------------------------------------------------------------------------

def test_gain_and_offset_shift_a_small_angle_noticeably():
    _, _, uncorrected = turn_model.compute_turn(15, 200, 115, 1.0)
    _, _, corrected = turn_model.compute_turn(
        15, 200, 115, 1.0, gain=1.061, offset_deg=-5.54)
    assert uncorrected == 75
    assert corrected == 97


def test_default_gain_and_offset_are_the_identity():
    default = turn_model.compute_turn(90, 200, 115, 1.0)
    explicit_identity = turn_model.compute_turn(
        90, 200, 115, 1.0, gain=1.0, offset_deg=0.0)
    assert default == explicit_identity


# ---------------------------------------------------------------------------
# Error handling -- a caller with no trackwidth, a non-positive speed, or
# a zero gain gets a clear ValueError, not a crash deep inside math.radians
# or a silent division by zero.
# ---------------------------------------------------------------------------

def test_missing_trackwidth_raises_value_error():
    with pytest.raises(ValueError):
        turn_model.compute_turn(90, 200, None, 1.0)


def test_zero_trackwidth_raises_value_error():
    with pytest.raises(ValueError):
        turn_model.compute_turn(90, 200, 0, 1.0)


def test_negative_trackwidth_raises_value_error():
    with pytest.raises(ValueError):
        turn_model.compute_turn(90, 200, -10, 1.0)


def test_zero_speed_raises_value_error():
    with pytest.raises(ValueError):
        turn_model.compute_turn(90, 0, 115, 1.0)


def test_negative_speed_raises_value_error():
    with pytest.raises(ValueError):
        turn_model.compute_turn(90, -200, 115, 1.0)


def test_zero_gain_raises_value_error():
    with pytest.raises(ValueError):
        turn_model.compute_turn(90, 200, 115, 1.0, gain=0.0)

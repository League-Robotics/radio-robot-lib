"""turn_model.py -- the in-place rotation math `rogo turn` needs, ported
from `radio-robot-elite/src/host/robot_radio/io/cli.py`'s
`_turn_command` (ticket 003's own Implementation Plan).

**What ported, what did not.** `_turn_command` resolves wheelbase/slip
from one of two sources (a firmware-calibrated `data/robot_calibration.
json`, or a fallback onto the active robot's own `geometry.trackwidth`/
`calibration.rotational_slip`) and then picks one of three duration
models depending on what that source provides: a bivariate-polynomial
motor model (needs `numpy` and per-robot calibration trial data this
repo has none of), a legacy linear-inverse model (`arc_efficiency`/
`startup_loss_mm`, also calibration data this repo doesn't have), or a
plain no-motor-model kinematic estimate. Only the **fallback source**
(`trackwidth`/`rotational_slip`, exactly what `rogo.config.RobotConfig`
already loads) and the **plain kinematic estimate** branch transfer --
sprint.md's Implementation Plan is explicit that the polynomial model
does not ("no ... polynomial motor model this repo has no calibration
data for"). The linear gain/offset correction (elite's own
`rotation_gain(_neg)`/`rotation_offset_deg(_neg)`, present today in
`config/robots/tovez.json`'s `geometry` group even though elite's own
firmware never consumed them) is exposed here as this function's
optional `gain`/`offset_deg` parameters rather than read from config
directly -- `rogo.config.RobotConfig` doesn't carry those fields yet
(no current caller before this ticket), so a future ticket that wants
`rogo turn` to read them from config wires them through these same
parameters rather than this module growing a second config-reading path.

**No camera, no I/O.** Elite's own `_turn_command` returns `(cmd_l,
cmd_r, duration)` with a final sign flip explained there as correcting
for `aprilcam`'s image-space (Y-down) yaw convention -- pure scaffolding
around a camera this repo has no concept of (sprint.md's Decision 3).
What that flip actually DOES, independent of why elite wanted it, is
fix a sign relationship between "positive commanded angle" and "which
wheel goes which direction" -- a property of the rotation math itself,
not of any vision system. This module keeps that same relationship
(`left = -sign * speed`, `right = +sign * speed` for `sign = 1` when
the compensated angle is non-negative) as `rogo`'s own convention:
positive `angle_deg` means the RIGHT wheel drives forward and the LEFT
wheel drives backward -- a counterclockwise rotation viewed from above
for a normally-mounted differential-drive robot (matches elite's own
CLI help text, "positive degrees = world CCW").
"""

from __future__ import annotations

import math


def compute_turn(
    angle_deg: float,
    speed_mm_s: float,
    trackwidth_mm: float | None,
    rotational_slip: float | None,
    gain: float = 1.0,
    offset_deg: float = 0.0,
) -> tuple[int, int, int]:
    """Compute `(cmd_l, cmd_r, duration_ms)` for one `WHEELS_V` call that
    rotates the robot in place by `angle_deg` (signed; positive = CCW,
    per this module's own docstring) at wheel-speed magnitude
    `speed_mm_s`.

    `rotational_slip=None` -- calibration data absent -- falls back to
    `1.0` (no slip), exactly `_turn_command`'s own fallback
    (`slip_val = float(cfg_slip) if cfg_slip is not None else 1.0`):
    the caller gets a linear estimate rather than a hard failure, at the
    documented cost of the ~10-20% first-try error elite's own
    docstring warns about.

    `gain`/`offset_deg` apply the optional linear correction
    `compensated = (angle_deg - offset_deg) / gain` BEFORE the rotation
    math runs -- both default to the identity (`gain=1.0`,
    `offset_deg=0.0`), so a caller with no correction data gets exactly
    the uncorrected kinematic estimate.

    Raises `ValueError` if `trackwidth_mm` is `None` or not positive,
    if `speed_mm_s` is not positive (this is a commanded SPEED
    MAGNITUDE -- direction comes from the sign of `angle_deg`, not from
    `speed_mm_s`), or if `gain` is zero (division by zero in the
    correction step).
    """
    if trackwidth_mm is None or trackwidth_mm <= 0:
        raise ValueError(
            f"compute_turn requires a positive trackwidth_mm, got {trackwidth_mm!r}")
    if speed_mm_s <= 0:
        raise ValueError(
            f"compute_turn requires a positive speed_mm_s, got {speed_mm_s!r}")
    if gain == 0:
        raise ValueError("compute_turn requires a nonzero gain")

    slip = rotational_slip if rotational_slip is not None else 1.0

    compensated_deg = (angle_deg - offset_deg) / gain
    theta = math.radians(compensated_deg)
    sign = 1 if theta >= 0 else -1
    target_arc_mm = abs(theta) * trackwidth_mm / (2.0 * slip)  # mm of arc per wheel
    duration_ms = int(round(1000.0 * target_arc_mm / speed_mm_s))

    cmd_l = int(round(-sign * speed_mm_s))
    cmd_r = int(round(sign * speed_mm_s))
    return cmd_l, cmd_r, duration_ms

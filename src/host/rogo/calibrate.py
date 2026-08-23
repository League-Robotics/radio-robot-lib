"""calibrate.py -- the manual/tape-measure multi-trial calibration flow
`rogo calibrate turns`/`rogo calibrate distance` run, ported from
`radio-robot-elite/src/host/robot_radio/io/calibrate.py`'s
`cmd_calibrate_turns`/`cmd_calibrate_distance` -- the non-`--auto`
branches only, per sprint.md's Design Rationale Decision 4. Owns trial
sequencing, prompts, and residual computation (sprint.md's Architecture
Step 3, this module's own row); delegates motion to `robot_v6.motion`
and the turn model (`rogo.turn_model`, ticket 003) and persistence to
`rogo.config` (ticket 002).

**What ported, what did not.** Elite's own manual mode still degraded
into an OTOS/aprilcam-daemon best-effort path even with `--auto` off
(`_make_proto_cfg`, the adaptive `OA`/`OL` firmware-register pushes, the
bivariate motor model) -- none of that transfers (sprint.md's
Implementation Plan: "drop the bivariate-polynomial motor-model and
OTOS-linear-scale pieces, which depend on calibration data this repo has
no equivalent source for"). What DOES port is the shape of elite's own
residual/slip arithmetic: `new_scale = current_scale * (ground_truth /
internal_estimate)`. Here, "internal estimate" is simply the TARGET this
module itself commanded (the turn/drive model's own prediction assuming
today's slip/scale is correct), and "ground truth" is the operator's
tape-measured/protractor-read actual value -- no camera, no OTOS, no
firmware register, exactly the "encoder telemetry [replaced by open-loop
timing] + operator input" flow sprint.md's Step 1 confirms is fully
self-contained.

**Physical derivation, turns.** `turn_model.compute_turn(target_deg,
speed, trackwidth, slip)` computes a wheel-arc length (and so a
duration) such that, IF `slip` were exactly correct, the robot would
rotate exactly `target_deg`. Given the physical relationship
`actual_theta = slip_true * ideal_kinematic_theta` (motion-api.md#2.1:
"rotational_slip is the measured ratio of actual rotation to ideal"),
substituting the commanded wheel-arc through both the model's own
formula and the true physical response shows
`slip_true = slip_used * (actual_deg / target_deg)` -- i.e. exactly
elite's own "current * (ground_truth / estimate)" shape, with
`target_deg` standing in for elite's OTOS-stream estimate. The same
derivation, substituting a straight-line kinematic distance
(`speed * duration`) for the arc-length term, gives an analogous
`distance_scale` correction for `rogo calibrate distance` -- see
`compute_distance_trial()` below.

**Multi-trial statistics, batched rather than adaptive.** Elite's own
`cmd_calibrate_turns` re-pushed an updated OTOS register live after
EVERY trial (a genuine closed-loop firmware adaptation this repo has no
equivalent register for) but then saved the batch MEAN ratio for
`cmd_calibrate_distance`'s otos_linear_scale regardless. This module
always uses the batch approach for both commands (`compute_calibration`
below): every trial in a run commands using the SAME starting slip/scale
(loaded once, before the first trial), and the final saved value is the
starting value times the mean of all trials' ratios. This is simpler,
order-independent, and matches classical multi-trial calibration
practice; it is a deliberate simplification of elite's own DUAL
approach (adaptive push + batch save), not an oversight.

**Sane-range rejection (ticket's own AC #4, echoing motion-api#2.1's own
"do NOT bend trackwidth" caution).** Neither `SLIP_SANE_RANGE` nor
`DISTANCE_SCALE_SANE_RANGE` come from a firmware register domain (unlike
elite's own OTOS-register clamp, `[0.872, 1.127]`, an 8-bit fixed-point
hardware limit that has no equivalent here -- `WHEELS_V`'s wheel speeds
are ordinary floats, not a scaled register write). `(0.5, 1.5)` is this
module's own engineering judgment for "physically plausible, not a
data-entry mistake" -- a robot that appears to rotate/travel under half
or over one-and-a-half times its commanded amount is far more likely a
misread tape measure, a wrong-units entry, or a trial run against the
wrong target than a real calibration fact, and `compute_calibration()`
refuses to persist such a value rather than silently writing it.
"""

from __future__ import annotations

import dataclasses
from typing import Callable, Sequence

from robot_v6 import motion
from robot_v6.reliability import Session

from . import config, turn_model
from .config import RobotConfig

_DEFAULT_TIMEOUT = 3.0  # [s] -- matches rogo.cli's own _DEFAULT_TIMEOUT

# Manual/tape-measure calibration needs a target angle a human can
# actually verify without a camera or a firmware closed loop -- elite's
# own 360-degree full-spin target (its own TN closed-loop firmware
# command, camera/OTOS-read) is not human-measurable by eye or tape
# (a full spin returns visually to the start orientation). A quarter
# turn is the natural manual analog: mark a reference line, spin, read
# the result off a protractor or a framing square against the mark --
# this module's own deliberate deviation from elite's default, not a
# straight port of the number 360.
DEFAULT_TURN_TARGET_DEG = 90.0
DEFAULT_TURN_TRIALS = 6  # matches elite's own cmd_calibrate_turns default
DEFAULT_TURN_SPEED_MM_S = 200.0  # matches rogo.cli's own turn-command default

DEFAULT_DISTANCE_TARGET_MM = 400.0  # elite's own default (40 cm)
DEFAULT_DISTANCE_TRIALS = 3  # elite's own default, and its own "need >= 3" floor
DEFAULT_DISTANCE_SPEED_MM_S = 200.0

_MIN_TRIALS = 3  # elite's own "need >= 3 trials to compute statistics" floor

# See module docstring's own "Sane-range rejection" section.
SLIP_SANE_RANGE: tuple[float, float] = (0.5, 1.5)
DISTANCE_SCALE_SANE_RANGE: tuple[float, float] = (0.5, 1.5)


# ---------------------------------------------------------------------------
# Pure residual computation -- no session, no I/O. This is the "run N
# trials, collect a measured value per trial, compute a result" core
# ticket 007's MCP tool needs without a terminal prompt: it takes an
# explicit list of already-known measured values, not `input()`.
# ---------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class TrialSample:
    """One trial's target (what this module commanded, assuming the
    starting slip/scale was correct) and measured (what the operator
    reported) value, same units (degrees for turns, mm for distance)."""

    target: float
    measured: float

    @property
    def ratio(self) -> float:
        return self.measured / self.target


@dataclasses.dataclass(frozen=True)
class CalibrationResult:
    """The outcome of `compute_calibration()`. `updated_value` is `None`
    in exactly two cases, distinguished by `rejected_reason`: fewer than
    `min_trials` usable samples were recorded (`rejected_reason is
    None` -- there was nothing to reject, just not enough data), or a
    value WAS computed but fell outside `sane_range`
    (`rejected_reason` explains why, per AC #4)."""

    samples: tuple[TrialSample, ...]
    starting_value: float
    mean_ratio: float | None
    updated_value: float | None
    rejected_reason: str | None


def compute_calibration(
    current_value: float | None,
    target: float,
    measured_values: Sequence[float],
    sane_range: tuple[float, float],
    min_trials: int = _MIN_TRIALS,
) -> CalibrationResult:
    """Compute an updated slip/scale from `measured_values` (one per
    trial, all against the same `target`), starting from
    `current_value` (`None` -- never calibrated -- falls back to `1.0`,
    the identity, exactly `turn_model.compute_turn()`'s own fallback).
    Non-positive measurements are dropped rather than raising (a
    caller's own input validation should normally prevent them, but this
    function stays defensive rather than trusting every caller).
    """
    starting = current_value if current_value is not None else 1.0
    samples = tuple(
        TrialSample(target=target, measured=m) for m in measured_values if m > 0)
    if len(samples) < min_trials:
        return CalibrationResult(
            samples=samples, starting_value=starting,
            mean_ratio=None, updated_value=None, rejected_reason=None)

    mean_ratio = sum(s.ratio for s in samples) / len(samples)
    updated = starting * mean_ratio
    lo, hi = sane_range
    if not (lo <= updated <= hi):
        return CalibrationResult(
            samples=samples, starting_value=starting, mean_ratio=mean_ratio,
            updated_value=None,
            rejected_reason=(
                f"computed value {updated:.4f} is outside the sane range "
                f"[{lo}, {hi}] -- not saved"),
        )
    return CalibrationResult(
        samples=samples, starting_value=starting, mean_ratio=mean_ratio,
        updated_value=updated, rejected_reason=None)


# ---------------------------------------------------------------------------
# compute_distance_trial() -- the straight-line analog of
# turn_model.compute_turn(), simple enough (no trigonometry, no
# gain/offset correction) to keep here rather than a whole new
# `distance_model` module. Pure function, its own unit tests.
# ---------------------------------------------------------------------------

def compute_distance_trial(
    distance_mm: float, speed_mm_s: float, distance_scale: float | None,
) -> tuple[int, int, int]:
    """Compute `(cmd_l, cmd_r, duration_ms)` for one `WHEELS_V` call that
    drives the robot straight `distance_mm` at wheel-speed magnitude
    `speed_mm_s`. `distance_scale=None` -- no calibration data -- falls
    back to `1.0` (no correction), the identity, exactly
    `turn_model.compute_turn()`'s own `rotational_slip=None` fallback.

    Physical model (see module docstring): `actual_mm = distance_scale *
    (speed_mm_s * duration_s)`, so achieving `distance_mm` needs
    `duration_s = distance_mm / (distance_scale * speed_mm_s)`.

    Raises `ValueError` if `distance_mm`/`speed_mm_s` is not positive.
    """
    if distance_mm <= 0:
        raise ValueError(
            f"compute_distance_trial requires a positive distance_mm, got {distance_mm!r}")
    if speed_mm_s <= 0:
        raise ValueError(
            f"compute_distance_trial requires a positive speed_mm_s, got {speed_mm_s!r}")
    scale = distance_scale if distance_scale is not None else 1.0
    if scale <= 0:
        scale = 1.0  # a corrupt/negative stored scale must not divide the wrong way
    duration_ms = int(round(1000.0 * distance_mm / (scale * speed_mm_s)))
    cmd = int(round(speed_mm_s))
    return cmd, cmd, duration_ms


# ---------------------------------------------------------------------------
# Interactive trial loop, shared by both commands -- prompt to start,
# drive, prompt for the measured value. `input_fn`/`print_fn` are
# injected (never called as bare `input()`/`print()`), the same
# testability pattern `rogo.cli._cmd_drive_stream()`'s own `sleep=`
# parameter already establishes in this package: a test drives this
# exact function with scripted stdin against a REAL session (e.g.
# tools/sim), with no TTY involved at all. Defaulting to `None` rather
# than binding `input`/`print` directly as the parameter default (see
# `_cmd_drive_stream()`'s own docstring for why) means a caller that
# does NOT override them still gets whichever `input`/`print` currently
# resolve to, looked up fresh on every call -- not whatever they were
# when this module was first imported.
# ---------------------------------------------------------------------------

def _run_interactive_trials(
    session: Session,
    trials: int,
    spin_label: str,
    measured_prompt: str,
    drive_one: Callable[[], tuple[int, int]],  # () -> (seq_id, duration_ms)
    *,
    input_fn: Callable[[str], str] | None = None,
    print_fn: Callable[[str], None] | None = None,
    timeout: float = _DEFAULT_TIMEOUT,
) -> list[float]:
    _input = input_fn if input_fn is not None else input
    _print = print_fn if print_fn is not None else print
    measured: list[float] = []
    for i in range(1, trials + 1):
        _print(f"Trial {i}/{trials}: aim robot, press Enter to {spin_label}. "
               f"Type 'q' to finish early.")
        try:
            raw = _input("")
        except EOFError:
            break
        if raw.strip().lower() == "q":
            break

        seq_id, duration_ms = drive_one()
        acked = session.wait_for_ack(seq_id, timeout=timeout)
        if acked:
            session.wait_for_done(seq_id, timeout=duration_ms / 1000.0 + timeout)
        else:
            _print(f"  warning: (#{seq_id}) not acked within {timeout}s")

        try:
            raw_measured = _input(measured_prompt)
        except EOFError:
            break
        raw_measured = raw_measured.strip()
        if raw_measured.lower() in ("skip", "s", ""):
            _print("  Skipped.")
            continue
        try:
            value = float(raw_measured)
        except ValueError:
            _print(f"  Invalid {raw_measured!r} -- skipped.")
            continue
        if value <= 0:
            _print("  Measured value must be > 0 -- skipped.")
            continue
        measured.append(value)
        _print(f"  Recorded sample {len(measured)}.")
    return measured


def _report_and_save(
    cfg: RobotConfig,
    field_name: str,
    result: CalibrationResult,
    apply_fn: Callable[[RobotConfig, float], RobotConfig],
    *,
    input_fn: Callable[[str], str],
    print_fn: Callable[[str], None],
) -> int:
    """Print the trial table/statistics, then -- only if a value was
    actually computed -- prompt to save it via `rogo.config`. Declining
    (or a rejected/insufficient result) leaves `cfg.path` untouched
    (AC #3). Returns a process exit code."""
    print_fn(f"\n{'=' * 60}")
    if not result.samples:
        print_fn("No usable trials recorded -- nothing to compute.")
        return 1

    print_fn(f"{'#':>3}  {'target':>10}  {'measured':>10}  {'ratio':>8}")
    for i, sample in enumerate(result.samples, 1):
        print_fn(f"{i:>3}  {sample.target:>10.2f}  {sample.measured:>10.2f}  "
                  f"{sample.ratio:>8.4f}")

    if result.mean_ratio is None:
        print_fn(f"\nNeed >= {_MIN_TRIALS} trials, got {len(result.samples)} "
                  f"-- not enough data.")
        return 1

    print_fn(f"\nMean ratio (measured/target): {result.mean_ratio:.4f}")
    print_fn(f"Current {field_name}: {result.starting_value:.6f}")

    if result.updated_value is None:
        print_fn(f"error: {result.rejected_reason}")
        return 1

    print_fn(f"Computed {field_name}: {result.updated_value:.6f}")
    try:
        raw = input_fn(f"Save {field_name}={result.updated_value:.6f} to {cfg.path}? [Y/n] ")
    except EOFError:
        raw = ""
    if raw.strip().lower() in ("n", "no"):
        print_fn("Skipped -- no changes saved.")
        return 0

    new_cfg = apply_fn(cfg, result.updated_value)
    config.save_robot_config(new_cfg)
    print_fn(f"Saved to {new_cfg.path}")
    return 0


# ---------------------------------------------------------------------------
# rogo calibrate turns
# ---------------------------------------------------------------------------

def calibrate_turns(
    session: Session,
    cfg: RobotConfig,
    trials: int = DEFAULT_TURN_TRIALS,
    speed_mm_s: float = DEFAULT_TURN_SPEED_MM_S,
    target_deg: float = DEFAULT_TURN_TARGET_DEG,
    *,
    input_fn: Callable[[str], str] | None = None,
    print_fn: Callable[[str], None] | None = None,
    timeout: float = _DEFAULT_TIMEOUT,
) -> int:
    """`rogo calibrate turns` -- run up to `trials` manual trials (prompt
    -> spin `target_deg` via `WHEELS_V` -> prompt for the operator's
    measured degrees), compute an updated `rotational_slip`, and prompt
    to save via `rogo.config` (AC's own flow, sprint.md SUC-003)."""
    _input = input_fn if input_fn is not None else input
    _print = print_fn if print_fn is not None else print

    if cfg.trackwidth_mm is None:
        _print("error: active robot config has no geometry.trackwidth -- "
               "can't compute a turn")
        return 1

    _print(f"Turn calibration: {trials} trials, target={target_deg:+.0f} deg, "
           f"speed={speed_mm_s:g} mm/s")
    _print(f"Current rotational_slip: "
           f"{cfg.rotational_slip if cfg.rotational_slip is not None else '(uncalibrated)'}")

    def _drive_one() -> tuple[int, int]:
        cmd_l, cmd_r, duration_ms = turn_model.compute_turn(
            target_deg, speed_mm_s, cfg.trackwidth_mm, cfg.rotational_slip)
        seq_id = motion.wheels_v(session, cmd_l, cmd_r, duration_ms)
        return seq_id, duration_ms

    measured = _run_interactive_trials(
        session, trials, "spin", "  Measured degrees turned (or 'skip'): ",
        _drive_one, input_fn=_input, print_fn=_print, timeout=timeout)

    result = compute_calibration(cfg.rotational_slip, target_deg, measured, SLIP_SANE_RANGE)
    return _report_and_save(
        cfg, "rotational_slip", result,
        lambda c, v: dataclasses.replace(c, rotational_slip=v),
        input_fn=_input, print_fn=_print)


# ---------------------------------------------------------------------------
# rogo calibrate distance
# ---------------------------------------------------------------------------

def calibrate_distance(
    session: Session,
    cfg: RobotConfig,
    trials: int = DEFAULT_DISTANCE_TRIALS,
    distance_mm: float = DEFAULT_DISTANCE_TARGET_MM,
    speed_mm_s: float = DEFAULT_DISTANCE_SPEED_MM_S,
    *,
    input_fn: Callable[[str], str] | None = None,
    print_fn: Callable[[str], None] | None = None,
    timeout: float = _DEFAULT_TIMEOUT,
) -> int:
    """`rogo calibrate distance` -- the straight-line equivalent of
    `calibrate_turns()`: run up to `trials` manual trials (prompt ->
    drive `distance_mm` via `WHEELS_V` -> prompt for the operator's
    measured distance), compute an updated `distance_scale`, and prompt
    to save via `rogo.config`."""
    _input = input_fn if input_fn is not None else input
    _print = print_fn if print_fn is not None else print

    _print(f"Distance calibration: {trials} trials, target={distance_mm:g} mm, "
           f"speed={speed_mm_s:g} mm/s")
    _print(f"Current distance_scale: "
           f"{cfg.distance_scale if cfg.distance_scale is not None else '(uncalibrated)'}")

    def _drive_one() -> tuple[int, int]:
        left, right, duration_ms = compute_distance_trial(
            distance_mm, speed_mm_s, cfg.distance_scale)
        seq_id = motion.wheels_v(session, left, right, duration_ms)
        return seq_id, duration_ms

    measured = _run_interactive_trials(
        session, trials, "drive", "  Measured distance traveled in mm (or 'skip'): ",
        _drive_one, input_fn=_input, print_fn=_print, timeout=timeout)

    result = compute_calibration(cfg.distance_scale, distance_mm, measured,
                                  DISTANCE_SCALE_SANE_RANGE)
    return _report_and_save(
        cfg, "distance_scale", result,
        lambda c, v: dataclasses.replace(c, distance_scale=v),
        input_fn=_input, print_fn=_print)

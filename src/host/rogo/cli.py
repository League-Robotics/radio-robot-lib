"""cli.py -- `rogo`'s argparse entry point. Parses arguments and
dispatches to `rogo.connection`/`rogo.config`/`rogo.turn_model`/
`robot_v6.motion`; this module stays a thin router (sprint.md's
Architecture Step 3: "it routes, it does not implement").

Ticket 002 wired exactly two subcommands -- `hello` and `stop` -- as
"the simplest possible smoke test of the whole stack end to end." Ticket
003 added `drive` and `turn` to the SAME command table `hello`/`stop`
already live in -- there never was a separate "stub" table to graduate
out of; `build_parser()` has always been the real one. Ticket 004 (this
module's current state) adds `goto` (one `GO_TO_R` call, robot-frame --
sprint.md's Design Rationale Decision 3: the aprilcam camera closed loop
from elite does NOT port, and `go_to_w`'s world-frame variant stays
unavailable until a pose source exists, specification.md#13) and
`config get`/`config set` (pure `GET`/`SET` wire delegation,
protocol.md#7, via `robot_v6.motion`'s wrappers). Later tickets extend
this module's subcommand table (`calibrate`/`repl`/`mcp`) without
changing this shape.

**The stakeholder's soft-warning decision (sprint 001
stakeholder_approval gate), binding for `drive --mm`:** a command that
reaches `DiffDriveAdapter`'s `kUnknown` planner gap (no planner behind
`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W` today, protocol.md#5) is
a SOFT WARNING, not a hard error -- the wire verb genuinely was sent and
genuinely was acked (protocol.md#8.9: a merits rejection still ADVANCES
the sequence), so the CLI reports the `err` outcome plainly and still
exits 0. This module surfaces that via `_await_ack_and_err()` +
`_print_soft_warning()` below, used by `drive --mm`'s `WHEELS_X` path.
`drive --ms`/`stream`/`turn` all go over `WHEELS_V`, `DiffDriveAdapter`'s
one real verb (protocol.md#5) -- no soft-warning path applies to them.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from robot_v6 import motion
from robot_v6.codec import Reply
from robot_v6.reliability import Session
from robot_v6.transport import TransportClosed

from . import config, connection, turn_model

_DEFAULT_TIMEOUT = 3.0  # [s] -- generous for a local subprocess/socket/serial hop
_DEFAULT_TURN_SPEED_MM_S = 200.0  # matches elite's own `p_turn --speed` default
_ERR_UNKNOWN = "1"  # protocol.md's own resultCode() table: kUnknown -> 1

# `drive stream`/bare-mode WHEELS_V keepalive shape: a wheel-velocity
# HOLD (motion-api.md#3.2) is only as safe as its own lease -- too short
# and a slightly-late resend lets the robot coast to a stop between
# keepalives; too long and Ctrl-C takes that long to actually halt the
# robot if the final STOP send is somehow lost. `_STREAM_LEASE_MULTIPLE`
# gives a resend a few cycles of slack; `_WHEELS_V_MAX_DURATION_MS` is
# protocol.md#5's own "WHEELS_V's 5000 ms ceiling," never exceeded
# regardless of how sparse `--resend` asks to be.
_STREAM_LEASE_FLOOR_MS = 500
_STREAM_LEASE_MULTIPLE = 3
_WHEELS_V_MAX_DURATION_MS = 5000

# `goto` -- GO_TO_R's own field types (protocol_handler.cpp: x/y/speed/
# arrive are parseInt32'd, timeout is parseUint32'd -- ALL FIVE are
# integer wire fields, not floats; ticket 003's own int-typed-wire-field
# lesson applies here identically) drive argparse's `type=int` choice
# for every one of `goto`'s numeric arguments below.
_DEFAULT_GOTO_SPEED_MM_S = 200  # matches _DEFAULT_TURN_SPEED_MM_S's own
# magnitude (elite's own turn-in-place default), kept as a plain int
# here since GO_TO_R's `speed` field is int32-parsed firmware-side --
# unlike `_DEFAULT_TURN_SPEED_MM_S`, which stays a float because
# `turn_model.compute_turn()` does its own physics in float and rounds
# to int only at the very end.
_DEFAULT_GOTO_ARRIVE_MM = 0  # motion-api.md#3.5: "0 takes the
# configured default" (10mm on tovez) -- the CLI's own default just
# forwards that same "let the adapter decide" behavior rather than
# guessing a distance of its own.

# `config get` -- how long to keep draining `get` reply lines after the
# ack before deciding a bare GET's field dump is finished. A named GET
# gets at most one line so this rarely matters for it, but a bare GET's
# line count varies by adapter (DiffDriveAdapter exposes 15 wire fields,
# protocol.md#7) and isn't knowable ahead of time from the wire alone --
# unlike `_await_ack_and_err()`'s single same-id `err`, there is no
# fixed count to wait for.
_GET_DRAIN_IDLE = 0.3  # [s]


def _pump_until(session: Session, predicate, timeout: float = _DEFAULT_TIMEOUT):
    """Poll `session.pump()` until `predicate(replies_so_far)` is true
    or `timeout` seconds elapse, returning everything collected either
    way. A small local helper rather than something `reliability.py`
    itself provides: `wait_for_ack()`/`wait_for_done()` there track one
    specific numeric condition, not an arbitrary reply-verb predicate
    like "have we seen a `device` line yet" (`hello`'s own need)."""
    deadline = time.monotonic() + timeout
    collected = []
    while time.monotonic() < deadline:
        collected.extend(session.pump(0.2))
        if predicate(collected):
            break
    return collected


def _await_ack_and_err(
    session: Session, seq_id: int, timeout: float = _DEFAULT_TIMEOUT
) -> tuple[bool, Reply | None]:
    """Pump until `seq_id` is retired by a cumulative ack (or `timeout`
    elapses), then drain a short extra grace window for a same-id `err`
    reply that a merits rejection emits ALONGSIDE the ack, not instead
    of it (protocol.md#8.9: `dispatch()` writes `ack` then, if the
    adapter itself rejects the call, `err <code> #<id>` immediately
    after, in the same handler call -- so in practice both lines are
    already sitting in the same read chunk by the time the ack is
    observed; the extra `pump()` below is a small safety margin, not
    load-bearing). Returns `(acked, err_reply_or_None)`."""
    replies = _pump_until(session, lambda rs: session.highest_acked >= seq_id, timeout=timeout)
    acked = session.highest_acked >= seq_id
    if acked:
        replies = replies + session.pump(0.2)
    err = next((r for r in replies if r.verb == "err" and r.id == seq_id), None)
    return acked, err


def _print_soft_warning(verb: str, seq_id: int, err: Reply) -> None:
    """STAKEHOLDER DECISION (sprint 001 stakeholder_approval gate, see
    module docstring): print the adapter's rejection plainly and let the
    caller still return 0 -- the command ran (was acked) and the wire
    verb really was sent; there is no crash and no false "success"
    silence either."""
    code = err.fields[0] if err.fields else "?"
    detail = "no planner for this verb (kUnknown/ERR_UNKNOWN)" if code == _ERR_UNKNOWN \
        else f"err {code}"
    print(
        f"warning: {verb} (#{seq_id}) was acked and sent, but the adapter "
        f"rejected it on merit: {detail}",
        file=sys.stderr,
    )
    print(f"{verb} sent (#{seq_id}); adapter reports {detail}")


def _await_ack_and_get_lines(
    session: Session, seq_id: int, timeout: float = _DEFAULT_TIMEOUT
) -> tuple[bool, list[Reply]]:
    """Pump until `seq_id` is retired by a cumulative ack, then drain the
    `get` reply lines `execGet()` writes AFTER that ack (dispatch()
    sends the ack unconditionally before running the verb's own
    executor -- protocol.md#8.2): zero lines for an unknown name
    (protocol.md#7's own "unknown name -> no get line, but the command
    is still acked" rule), one line for a named `GET`, or one line per
    field the adapter reports for a bare `GET`.

    `get` replies carry no `#<id>` of their own
    (`protocol_handler.cpp`'s `execGet()` formats `"get %s %s\\n"` with
    no trailing id token -- unlike `err`, which S8.6 requires to end
    every line with one), so they cannot be correlated to `seq_id` the
    way `_await_ack_and_err()` correlates a same-id `err`. This relies
    instead on `rogo config get` only ever having one `GET` outstanding
    at a time, and drains for `_GET_DRAIN_IDLE` seconds of silence
    (rather than a fixed one-shot grace pump) since a bare GET's own
    line count isn't known ahead of time. Returns `(acked,
    get_replies)`.
    """
    replies = _pump_until(session, lambda rs: session.highest_acked >= seq_id, timeout=timeout)
    acked = session.highest_acked >= seq_id
    if not acked:
        return False, []
    get_replies = [r for r in replies if r.verb == "get"]
    deadline = time.monotonic() + _GET_DRAIN_IDLE
    while time.monotonic() < deadline:
        new_replies = session.pump(0.1)
        new_get = [r for r in new_replies if r.verb == "get"]
        if new_get:
            get_replies.extend(new_get)
            deadline = time.monotonic() + _GET_DRAIN_IDLE  # more data seen -- extend the window
    return True, get_replies


def _print_config_set_error(name: str, err: Reply) -> None:
    """`SET`'s own merits-rejection path (protocol.md#7): the handler
    holds no field table, so an unrecognized `name` comes back as
    `err 1 #<id>` (`ERR_UNKNOWN`), layered on the in-order ack every
    `SET` gets regardless (protocol.md#8.2) -- the exact same wire code
    `goto`'s own `kUnknown` planner gap uses. Deliberately NOT routed
    through `_print_soft_warning()`, though: that helper's wording ("no
    planner for this verb") describes a capability this ADAPTER merely
    lacks, which is true of `goto` on `DiffDriveAdapter` but not of a
    mistyped config field name -- that is a genuine caller error, with
    no adapter anywhere that would accept it, so `cmd_config_set()`
    treats this as a hard error (nonzero exit) rather than a warning."""
    code = err.fields[0] if err.fields else "?"
    if code == _ERR_UNKNOWN:
        print(f"error: SET {name} rejected -- no such config field: {name!r}",
              file=sys.stderr)
    else:
        print(f"error: SET {name} rejected -- err {code}", file=sys.stderr)


def cmd_hello(args: argparse.Namespace) -> int:
    """Send `HELLO`, print the resulting `device` banner. The
    simplest possible round trip: no sequencing, no motion, just proof
    the target is alive and answering (per protocol.md#8.3, `HELLO`'s
    reply is byte-identical to the unsolicited boot banner already
    emitted on connect, so this also verifies re-sending it works)."""
    conn = connection.resolve(args)
    try:
        conn.session.send_unsequenced("HELLO")
        replies = _pump_until(
            conn.session, lambda rs: any(r.verb == "device" for r in rs))
        banner = next((r for r in replies if r.verb == "device"), None)
        if banner is None:
            print("no device banner received", file=sys.stderr)
            return 1
        fields = list(banner.fields) + ["?"] * max(0, 4 - len(banner.fields))
        role, common_name, name, serial = fields[:4]
        print(f"role={role} common_name={common_name} name={name} serial={serial}")
        return 0
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def cmd_stop(args: argparse.Namespace) -> int:
    """Send the sequenced `STOP` command and report whether it was
    acked -- `rogo`'s other half of ticket 002's smoke test, exercising
    `robot_v6.motion` and the reliability layer's ack path rather than
    `hello`'s unsequenced one."""
    conn = connection.resolve(args)
    try:
        seq_id = motion.stop(conn.session)
        acked = conn.session.wait_for_ack(seq_id, timeout=_DEFAULT_TIMEOUT)
        if acked:
            print(f"STOP acked (#{seq_id})")
            return 0
        print(f"STOP sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# drive -- bare mode, --ms, and stream all over WHEELS_V; --mm over
# WHEELS_X (sprint.md's SUC-001, this ticket's own description).
# ---------------------------------------------------------------------------

def _wheels_x_fields(left: int, right: int, mm: int) -> tuple[int, int, int, int]:
    """Map `drive <L> <R> --mm <N>`'s speed-shaped positionals onto
    `WHEELS_X`'s own distance-shaped wire fields (`left right cruise
    timeout #id`, motion-api.md#3.1). The two verbs disagree about which
    pair of fields is PER-WHEEL and which is SHARED: `WHEELS_X` shares
    one `cruise` speed across both wheels and gives each wheel its own
    commanded DISTANCE, while `drive`'s `<L> <R>` positionals are
    per-wheel SPEEDS in every other mode -- this is the one place that
    reshaping happens. All four fields must stay whole numbers: the wire
    handler decodes `WHEELS_X`'s left/right/cruise/timeout with
    `parseInt32`/`parseUint32` (protocol_handler.cpp), not
    `parseFloatField` -- a value with a decimal point ("100.0") is a
    DECODE FAILURE (protocol.md#8.9), not a rejected-but-acked call, so
    this function only ever returns `int`s.

    `cruise` is the dominant wheel's speed ceiling (motion-api.md's own
    framing of `cruise` as "a ceiling, not a hold"). Each wheel's signed
    distance is `mm` in THAT wheel's own commanded direction: for an
    in-place turn (`left`/`right` opposite signs), this sends each wheel
    the same arc length in opposite directions -- exactly what a bounded
    turn needs. For a straight/curved drive (same sign, unequal
    magnitude), giving both wheels the same distance is a
    simplification rather than a kinematic model this repo has no
    calibration data to justify -- and this verb is `kUnknown` on
    `DiffDriveAdapter` today regardless (sprint.md's Out of Scope), so
    the CLI's job here is a well-formed wire call, not exact kinematics.

    `timeout_ms` is a generous backstop (motion-api.md#3.1's own
    "bounded ... by the required timeout backstop"): 3x the naive
    constant-cruise ETA, floored at 1000ms, so a real planner has slack
    to decelerate without tripping its own safety cutoff.

    Raises `ValueError` if both `left` and `right` are zero (no cruise
    speed to move at all).
    """
    cruise = max(abs(left), abs(right))
    if cruise <= 0:
        raise ValueError("drive --mm requires a nonzero left or right speed")
    left_distance = mm if left >= 0 else -mm
    right_distance = mm if right >= 0 else -mm
    eta_ms = 1000.0 * abs(mm) / cruise
    timeout_ms = max(1000, int(round(eta_ms * _STREAM_LEASE_MULTIPLE)))
    return int(left_distance), int(right_distance), int(cruise), timeout_ms


def _cmd_drive_ms(session: Session, left: int, right: int, duration_ms: int) -> int:
    """`drive <L> <R> --ms <N>` -- one `WHEELS_V` call, reporting the
    ack and (since `DiffDriveAdapter` gives this verb real kinematic
    effect, protocol.md#5) the completion outcome too."""
    seq_id = motion.wheels_v(session, left, right, duration_ms)
    acked, err = _await_ack_and_err(session, seq_id)
    if not acked:
        print(f"WHEELS_V sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    if err is not None:
        _print_soft_warning("WHEELS_V", seq_id, err)
        return 0
    done = session.wait_for_done(seq_id, timeout=duration_ms / 1000.0 + _DEFAULT_TIMEOUT)
    if done is None:
        print(f"WHEELS_V acked (#{seq_id}) but never completed within timeout",
              file=sys.stderr)
        return 1
    print(f"WHEELS_V acked (#{seq_id}), done reason={done.reason}")
    return 0


def _cmd_drive_mm(session: Session, left: int, right: int, mm: int) -> int:
    """`drive <L> <R> --mm <N>` -- one `WHEELS_X` call, surfacing
    whatever the connected adapter actually reports: a soft warning
    (exit 0) on `DiffDriveAdapter`'s documented `kUnknown` gap, or the
    normal ack/completion outcome on an adapter that DOES implement it
    (e.g. `tools/sim`'s `FakeMotionAdapter`, UC-002)."""
    try:
        left_d, right_d, cruise, timeout_ms = _wheels_x_fields(left, right, mm)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    seq_id = motion.wheels_x(session, left_d, right_d, cruise, timeout_ms)
    acked, err = _await_ack_and_err(session, seq_id)
    if not acked:
        print(f"WHEELS_X sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    if err is not None:
        _print_soft_warning("WHEELS_X", seq_id, err)
        return 0
    done = session.wait_for_done(seq_id, timeout=timeout_ms / 1000.0 + _DEFAULT_TIMEOUT)
    if done is None:
        print(f"WHEELS_X acked (#{seq_id}) but never completed within timeout",
              file=sys.stderr)
        return 1
    print(f"WHEELS_X acked (#{seq_id}), done reason={done.reason}")
    return 0


def _cmd_drive_stream(
    session: Session, left: int, right: int, resend_ms: int, *, sleep=None
) -> int:
    """`drive <L> <R> stream [--resend MS]` (and bare `drive <L> <R>`
    with neither `--ms`/`--mm`/`stream` given -- see `cmd_drive()`):
    re-issue `WHEELS_V` at `resend_ms` cadence until Ctrl-C, matching
    `reliability.py`'s documented "current reading always overrides the
    previous one" semantics for `WHEELS_V` on `DiffDriveAdapter` (no
    queue, no completion event -- each new call just replaces the held
    velocity), then send the sequenced `STOP`.

    `sleep`, when given, replaces the pacing call -- a test can pass one
    that raises `KeyboardInterrupt` after a bounded number of calls
    instead of needing a real Ctrl-C, by calling this function directly
    (NOT by monkeypatching the global `time.sleep`: that name is shared
    process-wide, including by `subprocess.Popen.wait()`'s own internal
    polling loop, which a `--sim` connection's teardown calls -- a test
    that patched it globally found its fake `KeyboardInterrupt` firing
    inside that unrelated polling loop too). Defaults to `None` rather
    than binding `time.sleep` directly as the parameter default so a
    caller that does NOT pass `sleep` still gets whatever `time.sleep`
    currently resolves to, looked up fresh on every call via the
    `_sleep` local below.
    """
    _sleep = sleep if sleep is not None else time.sleep
    lease_ms = min(
        _WHEELS_V_MAX_DURATION_MS,
        max(_STREAM_LEASE_FLOOR_MS, resend_ms * _STREAM_LEASE_MULTIPLE),
    )
    resend_s = resend_ms / 1000.0
    print(f"streaming WHEELS_V {left} {right} (resend every {resend_ms}ms, "
          f"lease {lease_ms}ms) -- Ctrl-C to stop")
    try:
        while True:
            motion.wheels_v(session, left, right, lease_ms)
            session.pump(0.0)  # drain what's available; never blocks the cadence
            _sleep(resend_s)
    except KeyboardInterrupt:
        print()  # move past a bare ^C already echoed to the terminal
    seq_id = motion.stop(session)
    acked = session.wait_for_ack(seq_id, timeout=_DEFAULT_TIMEOUT)
    if acked:
        print(f"STOP acked (#{seq_id})")
        return 0
    print(f"STOP sent (#{seq_id}) but not acked within {_DEFAULT_TIMEOUT}s",
          file=sys.stderr)
    return 1


def cmd_drive(args: argparse.Namespace) -> int:
    """`rogo drive <L> <R> [--ms N | --mm N | stream] [--resend MS]` --
    dispatch to the three (bare mode folds into `stream`'s own loop, per
    `_cmd_drive_stream()`'s own docstring) shapes this ticket implements."""
    stream_kw = getattr(args, "stream_kw", None)
    if stream_kw is not None and stream_kw != "stream":
        print(f"error: unexpected positional argument {stream_kw!r} -- "
              "did you mean 'stream'?", file=sys.stderr)
        return 2
    if stream_kw == "stream" and (args.ms is not None or args.mm is not None):
        print("error: 'stream' is mutually exclusive with --ms and --mm", file=sys.stderr)
        return 2
    if args.ms is not None and args.mm is not None:
        print("error: --ms and --mm are mutually exclusive", file=sys.stderr)
        return 2
    if args.resend <= 0:
        print(f"error: --resend must be > 0, got {args.resend}", file=sys.stderr)
        return 2

    conn = connection.resolve(args)
    try:
        if args.mm is not None:
            return _cmd_drive_mm(conn.session, args.left, args.right, args.mm)
        if args.ms is not None:
            return _cmd_drive_ms(conn.session, args.left, args.right, args.ms)
        # Neither --ms nor --mm: literal 'stream' or bare `drive <L> <R>`
        # both mean "hold this velocity until told otherwise" -- the same
        # WHEELS_V-keepalive loop either way.
        return _cmd_drive_stream(conn.session, args.left, args.right, args.resend)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# turn -- the ported rotation model (turn_model.compute_turn), one
# WHEELS_V call (sprint.md's SUC-001).
# ---------------------------------------------------------------------------

def cmd_turn(args: argparse.Namespace) -> int:
    """`rogo turn <degrees> [--speed]` -- compute `(cmd_l, cmd_r,
    duration_ms)` from the active robot's `trackwidth`/`rotational_slip`
    (falling back to a no-slip estimate when `rotational_slip` is
    absent, per `turn_model.compute_turn()`'s own docstring) and issue
    one `WHEELS_V` call."""
    if args.speed <= 0:
        print(f"error: --speed must be > 0, got {args.speed}", file=sys.stderr)
        return 2

    cfg = config.load_active_robot()
    if cfg is None or cfg.trackwidth_mm is None:
        print(
            "error: no active robot config with geometry.trackwidth found "
            "(config/robots/active_robot.json) -- can't compute a turn",
            file=sys.stderr,
        )
        return 1

    try:
        cmd_l, cmd_r, duration_ms = turn_model.compute_turn(
            args.degrees, args.speed, cfg.trackwidth_mm, cfg.rotational_slip)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    conn = connection.resolve(args)
    try:
        seq_id = motion.wheels_v(conn.session, cmd_l, cmd_r, duration_ms)
        acked, err = _await_ack_and_err(conn.session, seq_id)
        if not acked:
            print(f"WHEELS_V sent (#{seq_id}) but not acked within "
                  f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
            return 1
        if err is not None:
            _print_soft_warning("WHEELS_V", seq_id, err)
            return 0
        done = conn.session.wait_for_done(
            seq_id, timeout=duration_ms / 1000.0 + _DEFAULT_TIMEOUT)
        if done is None:
            print(f"WHEELS_V acked (#{seq_id}) but never completed within timeout",
                  file=sys.stderr)
            return 1
        print(f"turn {args.degrees:+.1f}deg -> WHEELS_V {cmd_l} {cmd_r} {duration_ms} "
              f"(#{seq_id}), done reason={done.reason}")
        return 0
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# goto -- one GO_TO_R call, robot-frame (sprint.md's Design Rationale
# Decision 3: elite's aprilcam-camera closed loop does NOT port; the
# world-frame go_to_w variant stays unavailable until a pose source
# exists, specification.md#13).
# ---------------------------------------------------------------------------

def _goto_default_timeout_ms(x: int, y: int, speed_mm_s: int) -> int:
    """A generous timeout backstop when `--timeout` is not given,
    following `_wheels_x_fields()`'s own ETA-based approach
    (motion-api.md#3.1's "bounded ... by the required timeout backstop"
    logic, applied here to `GO_TO_R`'s own required `timeout` field):
    `_STREAM_LEASE_MULTIPLE` (3x) the straight-line ETA at the commanded
    cruise speed, floored at 1000ms so a very short hop still gets a
    livable window. `GO_TO_R`'s actual path length (motion-api.md#3.5's
    own arc-length formula) is usually somewhat longer than the
    straight-line chord used here, which is exactly why a MULTIPLE, not
    the bare ETA, is the floor."""
    distance_mm = math.hypot(x, y)
    eta_ms = 1000.0 * distance_mm / speed_mm_s
    return max(1000, int(round(eta_ms * _STREAM_LEASE_MULTIPLE)))


def cmd_goto(args: argparse.Namespace) -> int:
    """`rogo goto <x> <y> [--speed] [--arrive] [--timeout]` -- one
    `GO_TO_R` call. Reports the adapter's ACTUAL reply via the same
    `_await_ack_and_err()`/`_print_soft_warning()` path `drive --mm`/
    `turn` already use for `DiffDriveAdapter`'s documented `kUnknown`
    planner gap (UC-002/UC-003) -- never a false "arrived" claim: an
    ack only means the call was ACCEPTED, and `done reason=...`
    (`wait_for_done()`'s own outcome, printed verbatim) is the only
    arrival signal this function ever manufactures."""
    if args.speed <= 0:
        print(f"error: --speed must be > 0, got {args.speed}", file=sys.stderr)
        return 2
    timeout_ms = (
        args.timeout if args.timeout is not None
        else _goto_default_timeout_ms(args.x, args.y, args.speed)
    )
    if timeout_ms <= 0:
        print(f"error: --timeout must be > 0, got {timeout_ms}", file=sys.stderr)
        return 2

    conn = connection.resolve(args)
    try:
        seq_id = motion.go_to_r(
            conn.session, args.x, args.y, args.speed, args.arrive, timeout_ms)
        acked, err = _await_ack_and_err(conn.session, seq_id)
        if not acked:
            print(f"GO_TO_R sent (#{seq_id}) but not acked within "
                  f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
            return 1
        if err is not None:
            _print_soft_warning("GO_TO_R", seq_id, err)
            return 0
        done = conn.session.wait_for_done(
            seq_id, timeout=timeout_ms / 1000.0 + _DEFAULT_TIMEOUT)
        if done is None:
            print(f"GO_TO_R acked (#{seq_id}) but never completed within timeout",
                  file=sys.stderr)
            return 1
        print(f"goto ({args.x}, {args.y}) -> GO_TO_R {args.x} {args.y} {args.speed} "
              f"{args.arrive} {timeout_ms} (#{seq_id}), done reason={done.reason}")
        return 0
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# config get/set -- pure GET/SET wire delegation (protocol.md#7): this
# library stores no field table of its own, so `robot_v6.motion.get`/
# `.set` are thin wrappers and `rogo.cli` is thinner still.
# ---------------------------------------------------------------------------

def cmd_config_get(args: argparse.Namespace) -> int:
    """`rogo config get [name]` -- bare `GET` lists every field the
    adapter reports (protocol.md#6/#7: one `get` line per field); a
    `name` asks for just that one. An unknown name gets NO `get` line
    at all, though the command is still acked (protocol.md#7's own
    "unknown name -> no get line" rule) -- reported here as a clear
    error rather than a silent, field-less "success"."""
    conn = connection.resolve(args)
    try:
        seq_id = motion.get(conn.session, args.name)
        acked, get_replies = _await_ack_and_get_lines(conn.session, seq_id)
        if not acked:
            print(f"GET sent (#{seq_id}) but not acked within "
                  f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
            return 1
        if not get_replies:
            if args.name is not None:
                print(f"error: no such config field: {args.name!r}", file=sys.stderr)
                return 1
            print("(adapter reports no config fields)")
            return 0
        for reply in get_replies:
            name, value = reply.fields[0], reply.fields[1]
            print(f"{name}={value}")
        return 0
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def cmd_config_set(args: argparse.Namespace) -> int:
    """`rogo config set <name> <value>` -- `SET`'s own delegation
    (protocol.md#7). An unknown `name` is a genuine caller mistake, not
    a capability gap, so it is reported as a hard error (nonzero exit)
    rather than `goto`'s soft-warning treatment of the same wire error
    code -- see `_print_config_set_error()`'s own docstring for why."""
    conn = connection.resolve(args)
    try:
        seq_id = motion.set(conn.session, args.name, args.value)
        acked, err = _await_ack_and_err(conn.session, seq_id)
        if not acked:
            print(f"SET sent (#{seq_id}) but not acked within "
                  f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
            return 1
        if err is not None:
            _print_config_set_error(args.name, err)
            return 1
        print(f"SET {args.name}={args.value} acked (#{seq_id})")
        return 0
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rogo",
        description="Command-line control for a protocol-v6 robot, relay, "
                    "or tools/sim.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_hello = sub.add_parser(
        "hello", help="Probe the target: send HELLO, print the device banner")
    connection.add_target_arguments(p_hello)
    p_hello.set_defaults(func=cmd_hello)

    p_stop = sub.add_parser(
        "stop", help="Send the sequenced STOP command")
    connection.add_target_arguments(p_stop)
    p_stop.set_defaults(func=cmd_stop)

    p_drive = sub.add_parser(
        "drive",
        help="Drive: rogo drive <L> <R> [--ms N | --mm N | stream] "
             "[--resend MS]. Bare mode (no flags) behaves like stream.")
    connection.add_target_arguments(p_drive)
    p_drive.add_argument("left", type=int, help="left wheel speed (mm/s)")
    p_drive.add_argument("right", type=int, help="right wheel speed (mm/s)")
    p_drive.add_argument(
        "stream_kw", nargs="?", default=None, metavar="stream",
        help="literal 'stream': re-issue WHEELS_V at --resend cadence "
             "until Ctrl-C, then STOP")
    p_drive.add_argument(
        "--ms", type=int, default=None, help="duration in ms -- one WHEELS_V call")
    p_drive.add_argument(
        "--mm", type=int, default=None, help="distance in mm -- one WHEELS_X call")
    p_drive.add_argument(
        "--resend", type=int, default=150,
        help="stream/bare mode's WHEELS_V resend cadence in ms (default: 150)")
    p_drive.set_defaults(func=cmd_drive)

    p_turn = sub.add_parser(
        "turn",
        help="Turn in place N degrees (positive = CCW) using the active "
             "robot's rotation model")
    connection.add_target_arguments(p_turn)
    p_turn.add_argument("degrees", type=float, help="angle in degrees, signed (CCW = +)")
    p_turn.add_argument(
        "--speed", type=float, default=_DEFAULT_TURN_SPEED_MM_S,
        help=f"wheel speed magnitude in mm/s (default: {_DEFAULT_TURN_SPEED_MM_S:g})")
    p_turn.set_defaults(func=cmd_turn)

    p_goto = sub.add_parser(
        "goto",
        help="Drive to a robot-frame point via GO_TO_R: "
             "rogo goto <x> <y> [--speed] [--arrive] [--timeout]")
    connection.add_target_arguments(p_goto)
    # x/y/speed/arrive/timeout are all int-parsed wire fields
    # (parseInt32/parseUint32, protocol_handler.cpp) -- see this
    # module's own int-typed-wire-field constants above.
    p_goto.add_argument("x", type=int, help="target x, robot frame, forward (mm)")
    p_goto.add_argument("y", type=int, help="target y, robot frame, left (mm)")
    p_goto.add_argument(
        "--speed", type=int, default=_DEFAULT_GOTO_SPEED_MM_S,
        help=f"cruise speed magnitude in mm/s (default: {_DEFAULT_GOTO_SPEED_MM_S})")
    p_goto.add_argument(
        "--arrive", type=int, default=_DEFAULT_GOTO_ARRIVE_MM,
        help="arrival tolerance in mm; 0 takes the adapter's configured "
             f"default (motion-api.md#3.5) (default: {_DEFAULT_GOTO_ARRIVE_MM})")
    p_goto.add_argument(
        "--timeout", type=int, default=None,
        help="timeout backstop in ms (default: computed from distance/speed)")
    p_goto.set_defaults(func=cmd_goto)

    p_config = sub.add_parser(
        "config", help="Read/write the adapter's config fields (GET/SET, protocol.md#7)")
    config_sub = p_config.add_subparsers(dest="config_command", required=True)

    p_config_get = config_sub.add_parser(
        "get", help="GET [name] -- one field, or every field the adapter reports")
    connection.add_target_arguments(p_config_get)
    p_config_get.add_argument(
        "name", nargs="?", default=None,
        help="field name, e.g. wheel_control.pid_kp (default: list every field)")
    p_config_get.set_defaults(func=cmd_config_get)

    p_config_set = config_sub.add_parser("set", help="SET <name> <value>")
    connection.add_target_arguments(p_config_set)
    p_config_set.add_argument("name", help="field name, e.g. wheel_control.pid_kp")
    # The wire's config value is parseFloatField'd, not parseInt32'd
    # (protocol.md#7.2: "config values are the one place floats appear
    # on the wire") -- the one numeric argument in this whole module
    # that stays float rather than int, unlike goto's five.
    p_config_set.add_argument("value", type=float, help="new value for the field")
    p_config_set.set_defaults(func=cmd_config_set)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except connection.TargetError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except connection.SimBinaryError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 3
    except OSError as exc:
        print(f"error: could not reach target: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

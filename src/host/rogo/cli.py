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
protocol.md#7, via `robot_v6.motion`'s wrappers). Ticket 005 added
`calibrate turns`/`calibrate distance` (delegating trial sequencing to
`rogo.calibrate`). Ticket 006 (this module's current state) adds `repl`
-- each `cmd_*()` above is split into a non-session "validate/prepare"
step and a session-only `_run_*()`/`_dispatch_*()` body precisely so
`rogo.repl`'s per-line dispatch (`_dispatch_repl_line()` below) can
reuse the SAME per-verb logic against one already-open `Session`,
without `cmd_*()`'s own per-invocation `connection.resolve()`/`close()`
pair running once per repl line. Later tickets extend this module's
subcommand table (`mcp`) without changing this shape.

Sprint 002 ticket 001 adds the top-level `--agent` flag: `main()` scans
raw `argv` for it BEFORE `parser.parse_args()` ever runs, since
`build_parser()`'s subparsers are `required=True` and would otherwise
reject a bare `rogo --agent` invocation with no subcommand -- see
`main()`'s own comment. `agent_manual.MANUAL` is this module's only new
import; no other subcommand's behavior changes.

Sprint 003 ticket 009 wires this module into the new daemon subsystem
(sprint.md's Architecture Step 3, this module's own row), three
additive changes: (1) a `serve` subcommand (`cmd_serve()`) that imports
`rogo.daemon` and starts its server loop, injecting `daemon_client.
build_session_dispatch_table()` -- the SAME generic Session-RPC table
`daemon_client.py`'s own client half (`ClientConnection`/
`_RemoteSession`, ticket 008) speaks, so a daemon started this way is
wire-compatible with every dispatch body below with NO per-verb table
of its own needed here; (2) every one-shot `cmd_*()`'s own
`connection.resolve(args)` call site is replaced with `daemon_client.
get_connection(args, spawn=False)` -- auto-detect only, falling back to
`connection.resolve()` UNCHANGED when no daemon is found (zero
regression for a caller that never runs `rogo serve`); (3) `cmd_repl()`
resolves through `daemon_client.get_connection(args, spawn=True)`
instead -- auto-spawn-if-absent, since a repl session is itself a
long-lived tool like `rogo serve`/`rogo mcp`. This module still never
imports `rogo.daemon_client`'s or `rogo.daemon`'s internals beyond
their own public surface, and `rogo.daemon`/`rogo.daemon_client` never
import this module back -- the one-directional edge sprint.md's
architecture review specifically verified (see `daemon.py`'s own module
docstring, "Injection, not import" section).

Sprint 003 ticket 010 completes this for `cmd_mcp()`, the one
long-lived subcommand ticket 009 deliberately left untouched: `cmd_mcp()`
now resolves through `daemon_client.get_connection(args, spawn=True)`
too -- the identical auto-spawn-if-absent call `cmd_repl()` already
makes -- rather than `connection.resolve()` directly, so an MCP session
shares a daemon with any concurrent one-shot command (or another
long-lived session) instead of holding the robot/relay/sim connection
exclusively for itself. `rogo.mcp_server`'s own tool bodies need no
changes for this: the `Session` `daemon_client.ClientConnection.session`
hands back presents the identical call surface a direct connection's
`.session` already does (see `rogo.mcp_server`'s own module docstring).

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
import signal
import sys
import time
from pathlib import Path

from robot_v6 import motion
from robot_v6.codec import Reply
from robot_v6.reliability import Session
from robot_v6.transport import TransportClosed

from . import (
    agent_manual, calibrate, config, connection, daemon, daemon_client, mcp_server, repl,
    turn_model,
)

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


def _run_hello(session: Session) -> int:
    """`hello`'s session-only body -- shared by `cmd_hello()` (resolves
    its own throwaway connection, direct-CLI use) and `rogo.repl`'s
    per-line dispatch (reuses the repl's own already-open `Session`,
    ticket 006: "holds one Session open for the whole repl lifetime")."""
    session.send_unsequenced("HELLO")
    replies = _pump_until(
        session, lambda rs: any(r.verb == "device" for r in rs))
    banner = next((r for r in replies if r.verb == "device"), None)
    if banner is None:
        print("no device banner received", file=sys.stderr)
        return 1
    fields = list(banner.fields) + ["?"] * max(0, 4 - len(banner.fields))
    role, common_name, name, serial = fields[:4]
    print(f"role={role} common_name={common_name} name={name} serial={serial}")
    return 0


def cmd_hello(args: argparse.Namespace) -> int:
    """Send `HELLO`, print the resulting `device` banner. The
    simplest possible round trip: no sequencing, no motion, just proof
    the target is alive and answering (per protocol.md#8.3, `HELLO`'s
    reply is byte-identical to the unsolicited boot banner already
    emitted on connect, so this also verifies re-sending it works).

    Resolves through `daemon_client.get_connection(args, spawn=False)`
    (ticket 009) -- routes through an already-running `rogo serve`
    daemon for the resolved target when one exists, else falls back to
    `connection.resolve()` unchanged (module docstring)."""
    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _run_hello(conn.session)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def _run_stop(session: Session) -> int:
    """`stop`'s session-only body -- see `_run_hello()`'s own docstring
    for why this split exists (shared by `cmd_stop()` and
    `rogo.repl`)."""
    seq_id = motion.stop(session)
    acked = session.wait_for_ack(seq_id, timeout=_DEFAULT_TIMEOUT)
    if acked:
        print(f"STOP acked (#{seq_id})")
        return 0
    print(f"STOP sent (#{seq_id}) but not acked within "
          f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
    return 1


def cmd_stop(args: argparse.Namespace) -> int:
    """Send the sequenced `STOP` command and report whether it was
    acked -- `rogo`'s other half of ticket 002's smoke test, exercising
    `robot_v6.motion` and the reliability layer's ack path rather than
    `hello`'s unsequenced one. See `cmd_hello()`'s own docstring for
    why this resolves through `daemon_client.get_connection()` now."""
    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _run_stop(conn.session)
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


def _validate_drive_args(args: argparse.Namespace) -> int | None:
    """`drive`'s own argument validation, fails fast before ever
    resolving a target -- extracted out of `cmd_drive()` so `rogo.repl`'s
    per-line dispatch can run the identical checks against a line's own
    parsed `Namespace` without resolving (or needing) a connection of its
    own. Returns an exit code on failure, `None` when the arguments are
    well-formed."""
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
    return None


def _dispatch_drive_mode(session: Session, args: argparse.Namespace) -> int:
    """`drive`'s session-only body -- picks the `--mm`/`--ms`/stream
    shape and runs it against an already-open `session`. Assumes
    `_validate_drive_args(args)` already returned `None` (both
    `cmd_drive()` and `rogo.repl`'s dispatch call it first)."""
    if args.mm is not None:
        return _cmd_drive_mm(session, args.left, args.right, args.mm)
    if args.ms is not None:
        return _cmd_drive_ms(session, args.left, args.right, args.ms)
    # Neither --ms nor --mm: literal 'stream' or bare `drive <L> <R>`
    # both mean "hold this velocity until told otherwise" -- the same
    # WHEELS_V-keepalive loop either way.
    return _cmd_drive_stream(session, args.left, args.right, args.resend)


def cmd_drive(args: argparse.Namespace) -> int:
    """`rogo drive <L> <R> [--ms N | --mm N | stream] [--resend MS]` --
    dispatch to the three (bare mode folds into `stream`'s own loop, per
    `_cmd_drive_stream()`'s own docstring) shapes this ticket implements."""
    error = _validate_drive_args(args)
    if error is not None:
        return error

    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _dispatch_drive_mode(conn.session, args)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# turn -- the ported rotation model (turn_model.compute_turn), one
# WHEELS_V call (sprint.md's SUC-001).
# ---------------------------------------------------------------------------

def _prepare_turn(degrees: float, speed: float) -> tuple[int, tuple[int, int, int] | None]:
    """`turn`'s non-session preparation -- validate `speed`, load the
    active robot config, and run `turn_model.compute_turn()` -- shared
    by `cmd_turn()` (direct CLI) and `rogo.repl`'s per-line dispatch, so
    a repl line reloads the active config exactly like a fresh `rogo
    turn` invocation would (matching direct-CLI behavior 1:1 rather than
    caching the config for the whole repl lifetime). Returns `(exit_code,
    None)` on any failure (the message is already printed), or `(0,
    (cmd_l, cmd_r, duration_ms))` on success."""
    if speed <= 0:
        print(f"error: --speed must be > 0, got {speed}", file=sys.stderr)
        return 2, None

    cfg = config.load_active_robot()
    if cfg is None or cfg.trackwidth_mm is None:
        print(
            "error: no active robot config with geometry.trackwidth found "
            "(config/robots/active_robot.json) -- can't compute a turn",
            file=sys.stderr,
        )
        return 1, None

    try:
        cmd_l, cmd_r, duration_ms = turn_model.compute_turn(
            degrees, speed, cfg.trackwidth_mm, cfg.rotational_slip)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2, None

    return 0, (cmd_l, cmd_r, duration_ms)


def _run_turn(session: Session, degrees: float, cmd_l: int, cmd_r: int, duration_ms: int) -> int:
    """`turn`'s session-only body -- one `WHEELS_V` call using the
    `(cmd_l, cmd_r, duration_ms)` `_prepare_turn()` already computed.
    See `_run_hello()`'s own docstring for why this split exists."""
    seq_id = motion.wheels_v(session, cmd_l, cmd_r, duration_ms)
    acked, err = _await_ack_and_err(session, seq_id)
    if not acked:
        print(f"WHEELS_V sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    if err is not None:
        _print_soft_warning("WHEELS_V", seq_id, err)
        return 0
    done = session.wait_for_done(
        seq_id, timeout=duration_ms / 1000.0 + _DEFAULT_TIMEOUT)
    if done is None:
        print(f"WHEELS_V acked (#{seq_id}) but never completed within timeout",
              file=sys.stderr)
        return 1
    print(f"turn {degrees:+.1f}deg -> WHEELS_V {cmd_l} {cmd_r} {duration_ms} "
          f"(#{seq_id}), done reason={done.reason}")
    return 0


def cmd_turn(args: argparse.Namespace) -> int:
    """`rogo turn <degrees> [--speed]` -- compute `(cmd_l, cmd_r,
    duration_ms)` from the active robot's `trackwidth`/`rotational_slip`
    (falling back to a no-slip estimate when `rotational_slip` is
    absent, per `turn_model.compute_turn()`'s own docstring) and issue
    one `WHEELS_V` call."""
    exit_code, params = _prepare_turn(args.degrees, args.speed)
    if params is None:
        return exit_code
    cmd_l, cmd_r, duration_ms = params

    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _run_turn(conn.session, args.degrees, cmd_l, cmd_r, duration_ms)
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


def _prepare_goto(x: int, y: int, speed: int, timeout: int | None) -> tuple[int, int | None]:
    """`goto`'s non-session preparation -- validate `speed`, compute the
    default timeout backstop when `timeout` is not given, and validate
    that. Shared by `cmd_goto()` and `rogo.repl`'s per-line dispatch;
    see `_prepare_turn()`'s own docstring for the shape of this split.
    Returns `(exit_code, None)` on failure, `(0, timeout_ms)` on
    success."""
    if speed <= 0:
        print(f"error: --speed must be > 0, got {speed}", file=sys.stderr)
        return 2, None
    timeout_ms = timeout if timeout is not None else _goto_default_timeout_ms(x, y, speed)
    if timeout_ms <= 0:
        print(f"error: --timeout must be > 0, got {timeout_ms}", file=sys.stderr)
        return 2, None
    return 0, timeout_ms


def _run_goto(session: Session, x: int, y: int, speed: int, arrive: int, timeout_ms: int) -> int:
    """`goto`'s session-only body -- one `GO_TO_R` call, using the
    `timeout_ms` `_prepare_goto()` already resolved. Reports the
    adapter's ACTUAL reply via the same `_await_ack_and_err()`/
    `_print_soft_warning()` path `drive --mm`/`turn` already use for
    `DiffDriveAdapter`'s documented `kUnknown` planner gap (UC-002/
    UC-003) -- never a false "arrived" claim: an ack only means the call
    was ACCEPTED, and `done reason=...` (`wait_for_done()`'s own
    outcome, printed verbatim) is the only arrival signal this function
    ever manufactures."""
    seq_id = motion.go_to_r(session, x, y, speed, arrive, timeout_ms)
    acked, err = _await_ack_and_err(session, seq_id)
    if not acked:
        print(f"GO_TO_R sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    if err is not None:
        _print_soft_warning("GO_TO_R", seq_id, err)
        return 0
    done = session.wait_for_done(
        seq_id, timeout=timeout_ms / 1000.0 + _DEFAULT_TIMEOUT)
    if done is None:
        print(f"GO_TO_R acked (#{seq_id}) but never completed within timeout",
              file=sys.stderr)
        return 1
    print(f"goto ({x}, {y}) -> GO_TO_R {x} {y} {speed} "
          f"{arrive} {timeout_ms} (#{seq_id}), done reason={done.reason}")
    return 0


def cmd_goto(args: argparse.Namespace) -> int:
    """`rogo goto <x> <y> [--speed] [--arrive] [--timeout]` -- one
    `GO_TO_R` call. See `_run_goto()`'s own docstring for the reporting
    contract."""
    exit_code, timeout_ms = _prepare_goto(args.x, args.y, args.speed, args.timeout)
    if timeout_ms is None:
        return exit_code

    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _run_goto(conn.session, args.x, args.y, args.speed, args.arrive, timeout_ms)
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

def _run_config_get(session: Session, name: str | None) -> int:
    """`config get [name]`'s session-only body -- see `_run_hello()`'s
    own docstring for why this split exists. Bare `GET` lists every
    field the adapter reports (protocol.md#6/#7: one `get` line per
    field); a `name` asks for just that one. An unknown name gets NO
    `get` line at all, though the command is still acked (protocol.md#7's
    own "unknown name -> no get line" rule) -- reported here as a clear
    error rather than a silent, field-less "success"."""
    seq_id = motion.get(session, name)
    acked, get_replies = _await_ack_and_get_lines(session, seq_id)
    if not acked:
        print(f"GET sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    if not get_replies:
        if name is not None:
            print(f"error: no such config field: {name!r}", file=sys.stderr)
            return 1
        print("(adapter reports no config fields)")
        return 0
    for reply in get_replies:
        field_name, value = reply.fields[0], reply.fields[1]
        print(f"{field_name}={value}")
    return 0


def cmd_config_get(args: argparse.Namespace) -> int:
    """`rogo config get [name]` -- see `_run_config_get()`'s own
    docstring for the reporting contract."""
    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _run_config_get(conn.session, args.name)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def _run_config_set(session: Session, name: str, value: float) -> int:
    """`config set <name> <value>`'s session-only body -- see
    `_run_hello()`'s own docstring for why this split exists. An unknown
    `name` is a genuine caller mistake, not a capability gap, so it is
    reported as a hard error (nonzero exit) rather than `goto`'s
    soft-warning treatment of the same wire error code -- see
    `_print_config_set_error()`'s own docstring for why."""
    seq_id = motion.set(session, name, value)
    acked, err = _await_ack_and_err(session, seq_id)
    if not acked:
        print(f"SET sent (#{seq_id}) but not acked within "
              f"{_DEFAULT_TIMEOUT}s", file=sys.stderr)
        return 1
    if err is not None:
        _print_config_set_error(name, err)
        return 1
    print(f"SET {name}={value} acked (#{seq_id})")
    return 0


def cmd_config_set(args: argparse.Namespace) -> int:
    """`rogo config set <name> <value>` -- `SET`'s own delegation
    (protocol.md#7). See `_run_config_set()`'s own docstring for the
    reporting contract."""
    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return _run_config_set(conn.session, args.name, args.value)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# repl -- ticket 006: run commands over one persistent connection from an
# argument list, piped stdin, or an interactive prompt. `rogo.repl` owns
# only the command LOOP (read a line, parse it, decide whether to keep
# going); the actual per-verb dispatch below lives here, in `rogo.cli`,
# since it needs direct access to this module's own private per-verb
# helpers (`_run_hello`, `_validate_drive_args`/`_dispatch_drive_mode`,
# `_prepare_turn`/`_run_turn`, `_prepare_goto`/`_run_goto`,
# `_run_config_get`/`_run_config_set`) -- reusing the SAME
# `build_parser()` subparsers tickets 002/003/004 already built, per this
# ticket's own Description ("a much smaller command loop reusing the
# same rogo.cli per-subcommand argument parsers ... not a
# reimplementation"). `rogo.repl` is handed this function as a callback
# rather than importing `rogo.cli` itself, which would create a circular
# import (`cli.cmd_repl()` needs `repl.run()`).
# ---------------------------------------------------------------------------

def _dispatch_repl_line(session: Session, args: argparse.Namespace) -> int:
    """Route one already-parsed repl line to the session-only body the
    matching top-level `cmd_*()` uses after `connection.resolve()` --
    but against the repl's own already-open `session`, never resolving
    or closing a connection of its own (ticket 006: "holds one Session
    open for the whole repl lifetime and drains replies/telemetry
    between commands").

    `calibrate` is deliberately NOT dispatchable from inside a repl
    line: its own multi-trial flow (`rogo.calibrate.calibrate_turns`/
    `calibrate_distance`) is an interactive wizard spanning several
    prompts around ONE drive, not a single self-contained command --
    this ticket's own scope is reusing the parsers "already built by
    tickets 003/004" (drive/turn/goto/config), plus ticket 002's
    hello/stop. A nested `repl` (or `mcp`, once it exists) inside a repl
    line makes no sense either. Both fall through to the same
    unsupported-command message below."""
    if args.command == "hello":
        return _run_hello(session)
    if args.command == "stop":
        return _run_stop(session)
    if args.command == "drive":
        error = _validate_drive_args(args)
        if error is not None:
            return error
        return _dispatch_drive_mode(session, args)
    if args.command == "turn":
        exit_code, params = _prepare_turn(args.degrees, args.speed)
        if params is None:
            return exit_code
        cmd_l, cmd_r, duration_ms = params
        return _run_turn(session, args.degrees, cmd_l, cmd_r, duration_ms)
    if args.command == "goto":
        exit_code, timeout_ms = _prepare_goto(args.x, args.y, args.speed, args.timeout)
        if timeout_ms is None:
            return exit_code
        return _run_goto(session, args.x, args.y, args.speed, args.arrive, timeout_ms)
    if args.command == "config":
        if args.config_command == "get":
            return _run_config_get(session, args.name)
        if args.config_command == "set":
            return _run_config_set(session, args.name, args.value)
    print(f"error: {args.command!r} is not supported inside 'rogo repl' -- "
          "run it as its own separate rogo command instead", file=sys.stderr)
    return 2


def cmd_repl(args: argparse.Namespace) -> int:
    """`rogo repl [COMMAND ...]` -- resolve ONE connection for the whole
    repl lifetime (this ticket's own AC #1: "runs both commands over one
    connection"), then hand it to `rogo.repl.run()` along with a fresh
    `build_parser()` (for per-line parsing) and `_dispatch_repl_line`
    (for per-line dispatch). See `rogo.repl`'s own module docstring for
    the three input modes.

    Resolves through `daemon_client.get_connection(args, spawn=True)`
    (ticket 009) rather than `connection.resolve()` directly -- a repl
    session is itself a long-lived tool, so it AUTO-SPAWNS a daemon for
    the resolved target when none is already running (always preferring
    an already-running one first, module docstring), rather than only
    auto-detecting one the way the one-shot `cmd_*()`s above do. Either
    way the object handed to `repl.run()` presents the identical
    `Session` surface a direct connection's own `.session` does, so
    nothing below this call site changes."""
    try:
        conn = daemon_client.get_connection(args, spawn=True)
    except daemon_client.RobotNameRequiredError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except daemon_client.DaemonUnavailableError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    try:
        return repl.run(conn.session, args.commands, build_parser(), _dispatch_repl_line)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# calibrate turns/distance -- the manual/tape-measure trial sequence
# (sprint.md's Design Rationale Decision 4: `--auto` camera mode does not
# port). This module stays a thin router even here: all trial
# sequencing, prompting, and residual/save logic lives in
# `rogo.calibrate` (its own module docstring); `cli.py`'s job is just
# resolving the connection/config and translating the returned exit code.
# ---------------------------------------------------------------------------

def cmd_calibrate_turns(args: argparse.Namespace) -> int:
    """`rogo calibrate turns [--speed] [--trials N]` -- see
    `rogo.calibrate.calibrate_turns()` for the full flow. See
    `cmd_hello()`'s own docstring for why this resolves through
    `daemon_client.get_connection()` now."""
    if args.speed <= 0:
        print(f"error: --speed must be > 0, got {args.speed}", file=sys.stderr)
        return 2
    if args.trials <= 0:
        print(f"error: --trials must be > 0, got {args.trials}", file=sys.stderr)
        return 2

    cfg = config.load_active_robot()
    if cfg is None:
        print(
            "error: no active robot config found "
            "(config/robots/active_robot.json) -- can't calibrate",
            file=sys.stderr,
        )
        return 1

    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return calibrate.calibrate_turns(conn.session, cfg, args.trials, args.speed)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


def cmd_calibrate_distance(args: argparse.Namespace) -> int:
    """`rogo calibrate distance [--distance] [--speed] [--trials N]` --
    see `rogo.calibrate.calibrate_distance()` for the full flow. See
    `cmd_hello()`'s own docstring for why this resolves through
    `daemon_client.get_connection()` now."""
    if args.speed <= 0:
        print(f"error: --speed must be > 0, got {args.speed}", file=sys.stderr)
        return 2
    if args.distance <= 0:
        print(f"error: --distance must be > 0, got {args.distance}", file=sys.stderr)
        return 2
    if args.trials <= 0:
        print(f"error: --trials must be > 0, got {args.trials}", file=sys.stderr)
        return 2

    cfg = config.load_active_robot()
    if cfg is None:
        print(
            "error: no active robot config found "
            "(config/robots/active_robot.json) -- can't calibrate",
            file=sys.stderr,
        )
        return 1

    conn = daemon_client.get_connection(args, spawn=False)
    try:
        return calibrate.calibrate_distance(
            conn.session, cfg, args.trials, args.distance, args.speed)
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# mcp -- ticket 007: expose drive/turn/goto/config/calibrate as MCP
# tools over ONE connection resolved for the server's whole lifetime
# (same resolution as every other subcommand above). All tool-dispatch
# logic lives in `rogo.mcp_server` (its own module docstring explains
# why that module does NOT import this one back -- it would be
# circular, since THIS function needs `rogo.mcp_server.serve()`); this
# function's own job is exactly `cmd_repl()`'s shape: resolve, hand off,
# translate the handoff's own exceptions to an exit code, always close.
# Ticket 010 makes that "exactly cmd_repl()'s shape" literal for
# resolution too: `cmd_mcp()` now calls `daemon_client.get_connection(
# args, spawn=True)`, the identical auto-spawn-if-absent call `cmd_repl()`
# already makes, rather than `connection.resolve()` -- an MCP session is
# a long-lived tool exactly like `repl`, so it shares a daemon with any
# concurrent one-shot command instead of holding the connection alone.
# ---------------------------------------------------------------------------

def cmd_mcp(args: argparse.Namespace) -> int:
    """`rogo mcp [--sim|--connect|--port] [--listen HOST:PORT
    [--allow-remote]]` -- resolve ONE connection for the MCP server's
    whole lifetime (sprint.md SUC-005's own Main Flow: "same resolution
    as SUC-001"), then hand it to `rogo.mcp_server.serve()`. Defaults to
    stdio transport for the MCP wire itself -- a separate pipe from the
    resolved ROBOT connection above -- per this ticket's own binding
    security requirement (`rogo.mcp_server`'s own module docstring,
    Security section): no `--listen` means no network surface at all;
    `--listen` alone only reaches loopback; a non-loopback host needs
    `--allow-remote` too. Validates `--listen`/`--allow-remote` BEFORE
    resolving the robot/relay/sim connection, so a malformed or
    disallowed flag fails fast rather than after paying for a
    (possibly slow, possibly sim-rebuilding) connection first.

    Resolves through `daemon_client.get_connection(args, spawn=True)`
    (ticket 010) rather than `connection.resolve()` directly -- exactly
    `cmd_repl()`'s own shape (see that function's own docstring): an MCP
    session is itself a long-lived tool, so it prefers an already-
    running `rogo serve` daemon for the resolved target, or auto-spawns
    one when none is running, rather than holding the serial/sim
    connection exclusively for itself. This is what lets a concurrent
    one-shot command (or another `rogo mcp`/`rogo repl` session) reach
    the same robot without contention while this server is running
    (SUC-002's own multi-client acceptance criterion) -- `rogo mcp`
    itself no longer calls `connection.resolve()` at all."""
    try:
        mcp_server.resolve_listen_target(args.listen, args.allow_remote)
    except mcp_server.ListenTargetError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    try:
        conn = daemon_client.get_connection(args, spawn=True)
    except daemon_client.RobotNameRequiredError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except daemon_client.DaemonUnavailableError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    try:
        return mcp_server.serve(conn.session, listen=args.listen, allow_remote=args.allow_remote)
    except mcp_server.ListenTargetError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()


# ---------------------------------------------------------------------------
# serve -- ticket 009: `cmd_serve()` imports `rogo.daemon` and starts its
# server loop against a resolved target, injecting `daemon_client.
# build_session_dispatch_table()` -- the SAME generic Session-RPC table
# `daemon_client.py`'s own client half already speaks (its own module
# docstring: "Public so a future cmd_serve() (ticket 009) can reuse it
# verbatim rather than reimplementing this mapping"). This IS this
# module's own per-verb dispatch reused by injection, one layer down:
# every entry in that table forwards straight to the SAME `Session`
# methods (`send`/`pump`/`wait_for_ack`/...) `_run_hello()`/
# `_dispatch_drive_mode()`/etc. already call directly against a direct
# connection -- so no new, separate "drive"/"turn"/"goto"-keyed table is
# needed here, and a daemon started by a user's own `rogo serve` is
# wire-compatible with `daemon_client.get_connection()`'s auto-detect/
# auto-spawn clients (`cmd_*()`s above, `cmd_repl()`) no matter which of
# the two started it.
# ---------------------------------------------------------------------------

def _with_serve_activity_tracking(
    table: daemon.DispatchTable, last_activity: list[float],
) -> daemon.DispatchTable:
    """Wrap every handler in `table` so calling it stamps
    `last_activity[0]` with the current time -- `_wait_until_stopped()`'s
    own `--idle-timeout` watchdog reads this same list to decide when to
    self-terminate (used when THIS subcommand is itself what
    `daemon_client.default_spawn_argv()` spawns, ticket 008/009's own
    reconciliation: an auto-spawned worker has no interactive user to
    Ctrl-C it, so it needs to notice idleness itself). Duplicated, in
    miniature, from `daemon_client`'s own private identically-shaped
    helper rather than imported -- that helper is a private
    implementation detail of `daemon_client.run_daemon_worker()`'s own
    idle-timeout logic, and this module has no other reason to depend on
    it; see `daemon.py`'s own `_force_line_buffered()` for the identical
    duplicate-rather-than-couple precedent already established in this
    package."""
    def _wrap(fn):
        def _wrapped(session, params, abort):
            last_activity[0] = time.monotonic()
            return fn(session, params, abort)
        return _wrapped
    return {verb: _wrap(fn) for verb, fn in table.items()}


def _raise_keyboard_interrupt(signum: int, frame: object) -> None:
    del signum, frame
    raise KeyboardInterrupt


def _wait_until_stopped(
    idle_timeout: float | None, last_activity: list[float], *, sleep=None,
) -> None:
    """Block the calling thread until Ctrl-C, SIGTERM, or -- when
    `idle_timeout` is truthy -- until that many seconds elapse with no
    dispatched request (`last_activity[0]`, stamped by
    `_with_serve_activity_tracking()`'s wrapped handlers). SIGTERM is
    mapped onto the same `KeyboardInterrupt` path Ctrl-C (SIGINT)
    already takes by default, so a daemon started non-interactively
    (an auto-spawned worker, an init system) shuts down the same clean
    way a user's own Ctrl-C would; `signal.signal()` only works from the
    main thread, so registering it is a best-effort no-op when called
    from anywhere else (a test driving `cmd_serve()` off the main
    thread) -- the idle-timeout half of this function still applies
    either way. `sleep`, when given, replaces the pacing call -- see
    `_cmd_drive_stream()`'s own docstring for why (a test injects a fake
    that raises after a bounded number of calls instead of a real
    Ctrl-C)."""
    _sleep = sleep if sleep is not None else time.sleep
    previous_sigterm = None
    try:
        previous_sigterm = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    except ValueError:
        pass  # not the main thread -- SIGTERM handling unavailable here
    try:
        while True:
            if idle_timeout and time.monotonic() - last_activity[0] >= idle_timeout:
                return
            _sleep(0.2)
    except KeyboardInterrupt:
        print()  # move past a bare ^C already echoed to the terminal
    finally:
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)


def cmd_serve(args: argparse.Namespace) -> int:
    """`rogo serve [--sim|--connect|--port] [--name NAME]
    [--socket-dir DIR] [--idle-timeout SECONDS] [--stdio-pipe]` -- start
    a daemon (`rogo.daemon`) holding ONE connection open for the whole
    process's lifetime, injecting `daemon_client.
    build_session_dispatch_table()` (see this section's own header
    comment for why no separate per-verb table is needed).

    `--stdio-pipe` serves the framed protocol over this process's own
    stdin/stdout instead of a Unix domain socket (tests/embedding --
    delegates to `daemon.run_stdio_pipe_from_args()`, ticket 007's own
    boot function) and returns once stdin hits EOF. The default is a
    named Unix domain socket (production): the name is resolved via
    `daemon.resolve_robot_name()` (`--name` overrides; else HELLO; else
    the fixed `"sim"` default for a `--sim` target with no HELLO
    answer), `--socket-dir` overrides the well-known socket directory
    (`daemon.default_socket_dir()`), and the server runs until Ctrl-C/
    SIGTERM -- or, when `--idle-timeout` is given, until that many
    seconds elapse with no dispatched request (`_wait_until_stopped()`;
    used by `daemon_client.default_spawn_argv()` when this subcommand
    is itself what an auto-spawned `rogo repl`/`rogo mcp` boots).

    The Unix-socket branch's own `DaemonServer` is constructed with
    `is_estop=daemon_client.is_estop_request` (ticket 011's own fix --
    see that function's own module-level header comment in
    `daemon_client.py`): without it, an `ESTOP` sent through this
    subcommand's own generic session-RPC dispatch table would never be
    classified as estop-priority at all, silently defeating the issue's
    safety carry-over in production. The `--stdio-pipe` branch does not
    need the same override -- that transport serves exactly one client,
    strictly sequentially (`daemon.run_stdio_pipe()`'s own module
    docstring), so there is never a second, concurrent client's request
    for an estop to preempt."""
    dispatch_table = daemon_client.build_session_dispatch_table()

    if args.stdio_pipe:
        try:
            daemon.run_stdio_pipe_from_args(args, dispatch_table)
        except TransportClosed as exc:
            print(f"error: connection closed: {exc}", file=sys.stderr)
            return 1
        return 0

    conn = connection.resolve(args)
    try:
        try:
            name = daemon.resolve_robot_name(
                conn.session, override=args.name, sim=bool(getattr(args, "sim", False)))
        except daemon.RobotNameError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        socket_dir = Path(args.socket_dir) if args.socket_dir else None
        socket_path = daemon.socket_path_for_name(name, socket_dir=socket_dir)

        table = dispatch_table
        last_activity = [time.monotonic()]
        if args.idle_timeout:
            table = _with_serve_activity_tracking(dispatch_table, last_activity)

        server = daemon.DaemonServer(conn, table, is_estop=daemon_client.is_estop_request)
        server.start()
        try:
            listener = daemon.UnixSocketListener(server, socket_path)
            listener.start()
            try:
                print(f"rogo serve: {name!r} listening at {socket_path} -- Ctrl-C to stop")
                _wait_until_stopped(args.idle_timeout, last_activity)
            finally:
                listener.stop()
        finally:
            server.stop()
    except TransportClosed as exc:
        print(f"error: connection closed: {exc}", file=sys.stderr)
        return 1
    finally:
        conn.transport.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rogo",
        description="Command-line control for a protocol-v6 robot, relay, "
                    "or tools/sim.",
    )
    # Top-level flag, checked in main() BEFORE the subparsers' own
    # `required=True` would otherwise reject a bare `rogo --agent`
    # invocation (see main()'s own comment) -- registered here anyway so
    # `rogo --help` documents it and this parser stays the single source
    # of truth the pinning test (tests/host/rogo/test_agent_manual.py)
    # introspects.
    parser.add_argument(
        "--agent", action="store_true",
        help="print the complete agent-oriented Markdown manual (every "
             "subcommand, every option, units, exit codes) and exit 0 -- "
             "resolves no target and requires no subcommand",
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

    p_repl = sub.add_parser(
        "repl",
        help="Run one or more commands over a single persistent connection: "
             "an argument list, piped stdin, or an interactive prompt")
    connection.add_target_arguments(p_repl)
    p_repl.add_argument(
        "commands", nargs="*", metavar="COMMAND",
        help="one or more quoted command strings, e.g. "
             "'drive 100 100 --ms 200' 'stop'. With none given, read "
             "commands from stdin (piped) or an interactive prompt (tty).")
    p_repl.set_defaults(func=cmd_repl)

    p_calibrate = sub.add_parser(
        "calibrate",
        help="Run a manual, tape-measure-verified calibration trial sequence")
    calibrate_sub = p_calibrate.add_subparsers(dest="calibrate_command", required=True)

    p_cal_turns = calibrate_sub.add_parser(
        "turns",
        help="Calibrate rotational_slip: rogo calibrate turns [--speed] [--trials N]")
    connection.add_target_arguments(p_cal_turns)
    p_cal_turns.add_argument(
        "--speed", type=float, default=calibrate.DEFAULT_TURN_SPEED_MM_S,
        help=f"wheel speed magnitude in mm/s (default: {calibrate.DEFAULT_TURN_SPEED_MM_S:g})")
    p_cal_turns.add_argument(
        "--trials", type=int, default=calibrate.DEFAULT_TURN_TRIALS,
        help=f"number of manual trials (default: {calibrate.DEFAULT_TURN_TRIALS})")
    p_cal_turns.set_defaults(func=cmd_calibrate_turns)

    p_cal_distance = calibrate_sub.add_parser(
        "distance",
        help="Calibrate distance_scale: rogo calibrate distance "
             "[--distance] [--speed] [--trials N]")
    connection.add_target_arguments(p_cal_distance)
    p_cal_distance.add_argument(
        "--distance", type=float, default=calibrate.DEFAULT_DISTANCE_TARGET_MM,
        help=f"target distance per trial in mm "
             f"(default: {calibrate.DEFAULT_DISTANCE_TARGET_MM:g})")
    p_cal_distance.add_argument(
        "--speed", type=float, default=calibrate.DEFAULT_DISTANCE_SPEED_MM_S,
        help=f"wheel speed magnitude in mm/s "
             f"(default: {calibrate.DEFAULT_DISTANCE_SPEED_MM_S:g})")
    p_cal_distance.add_argument(
        "--trials", type=int, default=calibrate.DEFAULT_DISTANCE_TRIALS,
        help=f"number of manual trials (default: {calibrate.DEFAULT_DISTANCE_TRIALS})")
    p_cal_distance.set_defaults(func=cmd_calibrate_distance)

    p_mcp = sub.add_parser(
        "mcp",
        help="Start an MCP server exposing drive/turn/goto/config/"
             "calibrate_turns as tools (stdio transport by default -- "
             "see rogo.mcp_server for --listen's binding rules)")
    connection.add_target_arguments(p_mcp)
    p_mcp.add_argument(
        "--listen", metavar="HOST:PORT", default=None,
        help="serve the MCP protocol over TCP instead of stdio, bound "
             "to HOST:PORT (loopback only unless --allow-remote is "
             "also given)")
    p_mcp.add_argument(
        "--allow-remote", action="store_true",
        help="required alongside --listen to bind to a non-loopback host")
    p_mcp.set_defaults(func=cmd_mcp)

    p_serve = sub.add_parser(
        "serve",
        help="Start a daemon holding one connection open for multiple "
             "clients/sessions: rogo serve [--sim|--connect|--port] "
             "[--name NAME] [--socket-dir DIR] [--idle-timeout SECONDS] "
             "[--stdio-pipe]")
    connection.add_target_arguments(p_serve)
    p_serve.add_argument(
        "--name", default=None,
        help="robot name override for the Unix-socket file (<name>.sock) "
             "-- default: resolved via HELLO, or 'sim' for a --sim target "
             "with no HELLO answer")
    p_serve.add_argument(
        "--socket-dir", default=None,
        help="override the well-known socket directory (default: "
             "daemon.default_socket_dir())")
    p_serve.add_argument(
        "--idle-timeout", type=float, default=None,
        help="self-terminate after this many idle seconds with no "
             "dispatched request (default: never -- run until Ctrl-C/"
             "SIGTERM); used by an auto-spawned rogo repl/rogo mcp worker")
    p_serve.add_argument(
        "--stdio-pipe", action="store_true",
        help="serve the framed daemon protocol over this process's own "
             "stdin/stdout instead of a Unix domain socket -- for tests "
             "and embedding (see rogo.daemon.run_stdio_pipe)")
    p_serve.set_defaults(func=cmd_serve)

    return parser


def main(argv: list[str] | None = None) -> int:
    # `--agent` must work with no subcommand given, but build_parser()'s
    # own subparsers are `required=True` (a bare `rogo` with nothing else
    # is a usage error, per this module's own longstanding behavior) --
    # `parser.parse_args()` would enforce that requirement and reject
    # `rogo --agent` before `args.agent` could ever be checked. Scan the
    # raw argv for the flag FIRST, before requiring or resolving a
    # subcommand/target at all, rather than relaxing `required=True` on
    # the shared parser other callers (rogo.repl's own per-line parsing)
    # rely on staying strict.
    resolved_argv = sys.argv[1:] if argv is None else argv
    if "--agent" in resolved_argv:
        print(agent_manual.MANUAL)
        return 0

    parser = build_parser()
    args = parser.parse_args(resolved_argv)
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

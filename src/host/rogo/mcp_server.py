"""mcp_server.py -- `rogo mcp`: expose motion/config/calibration
operations as MCP tools (sprint.md's Architecture Step 3,
`rogo.mcp_server`'s own row; SUC-005). Adapts
`radio-robot-elite/src/host/robot_radio/io/robot_mcp.py`'s
tool-definition SHELL -- one process-lifetime server exposing named
tools, each a thin translation from tool arguments to an underlying
call, no business logic of its own -- to THIS repo's stack: no
`SerialConnection`/`Nezha`/`Navigator`, just the same
`robot_v6.motion`/`rogo.config`/`rogo.calibrate` calls `rogo.cli`'s own
subcommands already use (sprint.md's Architecture diagram: `MCP
-->|delegates to| MOTION`/`CFG`/`CAL`, with no edge to `rogo.cli` at
all).

**Why this module does NOT import `rogo.cli`.** `rogo.cli`'s own
`_run_*`/`_prepare_*`/`_dispatch_*` session-only bodies (ticket 006's
refactor) are built around `print()` plus a process exit code -- the
right shape for a terminal, wrong for an MCP tool call, which must
return STRUCTURED data (an MCP client needs `{"acked": true,
"done_reason": "..."}`, not an integer) and must never write to real
stdout while `--stdio` transport is active (a stray `print()` would
land on the same fd the JSON-RPC wire itself uses, corrupting it).
`rogo.repl`'s own module docstring already establishes this codebase's
answer to "a second caller wants the CLI's per-verb logic without its
printing": don't import `cli.py` (it would also be circular here --
`cli.py` must import THIS module to wire the `mcp` subcommand); write
this module's own thin translation layer instead, calling the exact
same underlying primitives (`robot_v6.motion.*`, `Session.pump()`/
`.wait_for_done()`, `rogo.config.*`, `rogo.calibrate.compute_calibration()`)
`rogo.cli` calls -- so there is no duplicated BUSINESS logic (verb
encoding, config parsing, calibration math each still live in exactly
one place); only the reporting shape differs, same as `rogo.repl`
differing from `rogo.cli` only in how a result reaches the user.

**Ticket 010: `session` may now be daemon-routed, not just direct --
and this module never has to know or care which.** `rogo.cli`'s own
`cmd_mcp()` resolves this module's `session` through `daemon_client.
get_connection(args, spawn=True)` -- the identical auto-spawn-if-absent
policy `cmd_repl()` already used (ticket 009) -- rather than `rogo.
connection.resolve()` directly: an MCP session is itself a long-lived
tool, so it prefers an already-running `rogo serve` daemon for the
resolved target, or spawns one, letting a concurrent one-shot command
(or another long-lived session) reach the same robot without contention
(SUC-002's own multi-client requirement) instead of holding the
serial/sim connection exclusively for itself. `session` is therefore no
longer guaranteed to be a direct `rogo.connection.Connection.session`
-- it may equally be a `daemon_client.ClientConnection.session` (a
`_RemoteSession` proxying one `session_send`/`session_pump`/... RPC per
call to the daemon's own held `Session`). This is exactly why this
module's decision, above, to keep its own thin translation layer rather
than import `cli.py`'s print-based dispatch bodies pays off a second
time: `_RemoteSession` presents the IDENTICAL `Session` surface a
direct connection's does (`send`/`send_unsequenced`/`pump`/
`highest_acked`/`wait_for_ack`/`wait_for_done`, `daemon_client.py`'s
own module docstring), so this module's wire-glue helpers (`_pump_until()`/
`_await_ack_and_err()`/`_await_ack_and_get_lines()` below) needed NO
changes at all for this ticket and are NOT retired by it -- they are
what makes `session`'s origin (direct vs. daemon-routed) invisible to
every `_tool_*()` body below, the same way `rogo.cli`'s own per-verb
dispatch bodies stayed unchanged across the identical daemon-routing
swap (ticket 009's own AC #3).

**STAKEHOLDER DECISION (sprint 001 stakeholder_approval gate,
binding):** kUnknown outcomes are soft WARNINGS, not tool errors -- an
acked call the adapter then rejects on merit (protocol.md#8.9) is
reported IN the tool's own result (a `warning`/`error` key), never
raised as an MCP tool error, mirroring `rogo.cli`'s own
`_print_soft_warning()`/`_print_config_set_error()` treatment of the
exact same wire condition. Only a genuine transport-level failure --
`TransportClosed`, or an ack/done that never arrives within a bounded
wait -- is raised (as `UnreachableTargetError`) and so surfaces as an
MCP tool error (this ticket's own AC #3): every wait in this module is
bounded (mirrors `rogo.cli`'s own `_DEFAULT_TIMEOUT`), so a dead target
reports an error promptly rather than hanging a client forever.

**Security (binding -- this ticket's own Description, sprint.md's
Migration Concerns security note, echoing `protocol.md#6.3`'s
`RUN`-allowlist caution and `wifi-link#11`'s no-authentication-at-this-
layer caution): `rogo mcp` is a new EXTERNAL control surface --
whatever it exposes is remotely callable by anything that can reach
it.** `serve()` below defaults to `stdio` transport for the MCP
server's OWN wire to its client -- a separate pipe entirely from the
ROBOT connection `--sim`/`--connect`/`--port` already resolved via
`rogo.connection` -- which has NO network surface at all, a strictly
stronger guarantee than "binds to 127.0.0.1". Passing `--listen
HOST:PORT` opts into a TCP transport instead; `_resolve_listen_target()`
enforces this ticket's own binding rule on THAT opt-in: a loopback host
is allowed outright, any other host additionally requires the paired
`--allow-remote` flag (Description: "Any non-localhost bind must be an
explicit opt-in flag") -- two explicit, named, auditable opt-ins
stacked on top of a safe-by-default (no network at all) starting point.
"""

from __future__ import annotations

import dataclasses
import math
import time
from typing import Any

from mcp.server.mcpserver import MCPServer

from robot_v6 import motion
from robot_v6.reliability import Session
from robot_v6.transport import TransportClosed

from . import calibrate, config, turn_model

_DEFAULT_TIMEOUT = 3.0  # [s] -- matches rogo.cli's own _DEFAULT_TIMEOUT
_ERR_UNKNOWN = "1"  # protocol.md's own resultCode() table: kUnknown -> 1
_GET_DRAIN_IDLE = 0.3  # [s] -- matches rogo.cli's own _run_config_get() constant

_DEFAULT_TURN_SPEED_MM_S = 200.0  # matches rogo.cli's own turn-command default
_DEFAULT_GOTO_SPEED_MM_S = 200  # matches rogo.cli's own goto-command default
_DEFAULT_GOTO_ARRIVE_MM = 0  # motion-api.md#3.5: "0 takes the configured default"
_GOTO_TIMEOUT_MULTIPLE = 3  # matches rogo.cli's own _STREAM_LEASE_MULTIPLE reuse

# This ticket's own binding security requirement (module docstring,
# Security section): a `--listen` host outside this set additionally
# requires `--allow-remote`.
LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})


class UnreachableTargetError(RuntimeError):
    """Raised by a tool function when the resolved robot/relay/sim
    connection is genuinely gone (`TransportClosed`) or an ack/done
    never arrives within this module's own bounded wait. The MCP
    framework's own `call_tool` request handler
    (`mcp.server.mcpserver.server`) wraps every tool invocation in a
    try/except that turns any raised exception into
    `CallToolResult(is_error=True, ...)` -- so raising this here is
    exactly what surfaces "unreachable target" through the MCP error
    channel rather than hanging a client (this ticket's own AC #3),
    with no bespoke error-channel plumbing needed in this module."""


class ListenTargetError(ValueError):
    """Raised by `_resolve_listen_target()` when `--listen` is
    malformed, or names a non-loopback host without the paired
    `--allow-remote` opt-in (module docstring's Security section)."""


# ---------------------------------------------------------------------------
# Wire-glue helpers -- deliberately this module's OWN copies of the
# shape `rogo.cli`'s `_pump_until()`/`_await_ack_and_err()`/
# `_await_ack_and_get_lines()` implement (see module docstring for why
# this isn't an import): the underlying `Session`/`robot_v6.motion`
# calls are identical, so this is reporting-shape duplication (dict vs.
# print+exit-code), not business-logic duplication.
# ---------------------------------------------------------------------------

def _pump_until(session: Session, predicate, timeout: float = _DEFAULT_TIMEOUT) -> list:
    """Poll `session.pump()` until `predicate(replies_so_far)` is true
    or `timeout` elapses, returning everything collected either way.
    Needed instead of `session.wait_for_ack()`/`wait_for_done()` for
    the ack+err combo below specifically because those methods pump
    internally without returning what they read -- a same-chunk `err`
    (or `get`) reply arriving alongside the ack it rides with would be
    silently lost if this used `wait_for_ack()` directly (protocol.md
    #8.9: the ack is written unconditionally before the verb's own
    executor runs, so both lines are typically already in the same
    read)."""
    deadline = time.monotonic() + timeout
    collected: list = []
    while time.monotonic() < deadline:
        collected.extend(session.pump(0.2))
        if predicate(collected):
            break
    return collected


def _await_ack_and_err(session: Session, seq_id: int, timeout: float = _DEFAULT_TIMEOUT):
    """Pump until `seq_id` is retired by a cumulative ack (or `timeout`
    elapses), then drain a short extra grace window for a same-id `err`
    reply a merits rejection emits alongside the ack (protocol.md#8.9).
    Returns `(acked, err_reply_or_None)`. Raises `TransportClosed` --
    callers convert that to `UnreachableTargetError`, not this helper,
    so every call site controls its own error message."""
    replies = _pump_until(session, lambda rs: session.highest_acked >= seq_id, timeout=timeout)
    acked = session.highest_acked >= seq_id
    if acked:
        replies = replies + session.pump(0.2)
    err = next((r for r in replies if r.verb == "err" and r.id == seq_id), None)
    return acked, err


def _await_ack_and_get_lines(session: Session, seq_id: int, timeout: float = _DEFAULT_TIMEOUT):
    """`GET`'s own variant of `_await_ack_and_err()`: pump until acked,
    then drain `get` reply lines for `_GET_DRAIN_IDLE` seconds of
    silence (a bare `GET`'s line count is not known ahead of time).
    Returns `(acked, get_replies)`."""
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
            deadline = time.monotonic() + _GET_DRAIN_IDLE
    return True, get_replies


def _report_motion_outcome(
    session: Session, verb: str, seq_id: int, done_timeout: float
) -> dict[str, Any]:
    """Shared ack/warning/done reporting for `drive`/`turn`/`goto` --
    the one wire dance those three tools all run after sending their
    own verb: await the ack (raising `UnreachableTargetError` if it
    never comes), report a same-id `err` as a soft WARNING (stakeholder
    decision, module docstring) rather than raising, then await
    completion the same bounded way."""
    try:
        acked, err = _await_ack_and_err(session, seq_id)
    except TransportClosed as exc:
        raise UnreachableTargetError(
            f"connection closed while awaiting {verb} ack (#{seq_id}): {exc}") from exc
    if not acked:
        raise UnreachableTargetError(
            f"{verb} sent (#{seq_id}) but not acked within {_DEFAULT_TIMEOUT}s")
    if err is not None:
        code = err.fields[0] if err.fields else "?"
        detail = ("no planner for this verb (kUnknown/ERR_UNKNOWN)"
                   if code == _ERR_UNKNOWN else f"err {code}")
        return {"acked": True, "seq_id": seq_id, "warning": detail}
    try:
        done = session.wait_for_done(seq_id, timeout=done_timeout)
    except TransportClosed as exc:
        raise UnreachableTargetError(
            f"connection closed while awaiting {verb} completion (#{seq_id}): {exc}") from exc
    if done is None:
        raise UnreachableTargetError(
            f"{verb} acked (#{seq_id}) but never completed within timeout")
    return {"acked": True, "seq_id": seq_id, "done_reason": done.reason}


def _goto_default_timeout_ms(x: int, y: int, speed_mm_s: int) -> int:
    """Mirrors `rogo.cli`'s own `_goto_default_timeout_ms()`: a generous
    ETA-based backstop when a caller doesn't supply one -- small enough
    arithmetic (not verb encoding, config parsing, or calibration math)
    that duplicating it here is a UX convenience, not the business-logic
    duplication this module otherwise avoids."""
    distance_mm = math.hypot(x, y)
    eta_ms = 1000.0 * distance_mm / speed_mm_s
    return max(1000, int(round(eta_ms * _GOTO_TIMEOUT_MULTIPLE)))


# ---------------------------------------------------------------------------
# Tool bodies -- one per MCP tool `build_server()` registers below.
# Each is a thin translation from already-validated arguments to the
# underlying `robot_v6.motion`/`rogo.config`/`rogo.calibrate` call --
# no session/wire logic beyond what `_report_motion_outcome()`/
# `_await_ack_and_get_lines()` above already centralize.
# ---------------------------------------------------------------------------

def _tool_hello(session: Session) -> dict[str, Any]:
    session.send_unsequenced("HELLO")
    try:
        replies = _pump_until(session, lambda rs: any(r.verb == "device" for r in rs))
    except TransportClosed as exc:
        raise UnreachableTargetError(
            f"connection closed while awaiting HELLO reply: {exc}") from exc
    banner = next((r for r in replies if r.verb == "device"), None)
    if banner is None:
        raise UnreachableTargetError("no device banner received within timeout")
    fields = list(banner.fields) + ["?"] * max(0, 4 - len(banner.fields))
    role, common_name, name, serial = fields[:4]
    return {"role": role, "common_name": common_name, "name": name, "serial": serial}


def _tool_stop(session: Session) -> dict[str, Any]:
    seq_id = motion.stop(session)
    try:
        acked = session.wait_for_ack(seq_id, timeout=_DEFAULT_TIMEOUT)
    except TransportClosed as exc:
        raise UnreachableTargetError(
            f"connection closed while awaiting STOP ack (#{seq_id}): {exc}") from exc
    if not acked:
        raise UnreachableTargetError(
            f"STOP sent (#{seq_id}) but not acked within {_DEFAULT_TIMEOUT}s")
    return {"acked": True, "seq_id": seq_id}


def _tool_drive(session: Session, left: int, right: int, ms: int) -> dict[str, Any]:
    """One `WHEELS_V` call -- `DiffDriveAdapter`'s one verb with real
    kinematic effect today (protocol.md#5). `drive --mm`/`stream`'s own
    `WHEELS_X`/keepalive-loop shapes (`rogo.cli`'s `_cmd_drive_mm`/
    `_cmd_drive_stream`) are deliberately not exposed here: `--mm` would
    duplicate `rogo.cli`'s own private `_wheels_x_fields()` reshape
    (real business logic, not wire glue) for a verb that's `kUnknown` on
    this adapter regardless, and `stream`'s whole point is a human
    holding Ctrl-C -- neither fits an MCP tool call's one-shot
    request/response shape. A future ticket can add them if an MCP
    client ever needs `WHEELS_X`/streaming specifically."""
    seq_id = motion.wheels_v(session, left, right, ms)
    return _report_motion_outcome(session, "WHEELS_V", seq_id, ms / 1000.0 + _DEFAULT_TIMEOUT)


def _tool_turn(session: Session, degrees: float, speed: float) -> dict[str, Any]:
    if speed <= 0:
        raise ValueError(f"speed must be > 0, got {speed}")
    cfg = config.load_active_robot()
    if cfg is None or cfg.trackwidth_mm is None:
        return {"error": "no active robot config with geometry.trackwidth found "
                          "(config/robots/active_robot.json) -- can't compute a turn"}
    try:
        cmd_l, cmd_r, duration_ms = turn_model.compute_turn(
            degrees, speed, cfg.trackwidth_mm, cfg.rotational_slip)
    except ValueError as exc:
        return {"error": str(exc)}
    seq_id = motion.wheels_v(session, cmd_l, cmd_r, duration_ms)
    result = _report_motion_outcome(
        session, "WHEELS_V", seq_id, duration_ms / 1000.0 + _DEFAULT_TIMEOUT)
    result.update({"degrees": degrees, "cmd_l": cmd_l, "cmd_r": cmd_r, "duration_ms": duration_ms})
    return result


def _tool_goto(
    session: Session, x: int, y: int, speed: int, arrive: int, timeout_ms: int | None
) -> dict[str, Any]:
    if speed <= 0:
        raise ValueError(f"speed must be > 0, got {speed}")
    resolved_timeout_ms = timeout_ms if timeout_ms is not None else _goto_default_timeout_ms(x, y, speed)
    if resolved_timeout_ms <= 0:
        raise ValueError(f"timeout_ms must be > 0, got {resolved_timeout_ms}")
    seq_id = motion.go_to_r(session, x, y, speed, arrive, resolved_timeout_ms)
    result = _report_motion_outcome(
        session, "GO_TO_R", seq_id, resolved_timeout_ms / 1000.0 + _DEFAULT_TIMEOUT)
    result.update({"x": x, "y": y, "speed": speed, "arrive": arrive, "timeout_ms": resolved_timeout_ms})
    return result


def _tool_config_get(session: Session, name: str | None) -> dict[str, Any]:
    seq_id = motion.get(session, name)
    try:
        acked, get_replies = _await_ack_and_get_lines(session, seq_id)
    except TransportClosed as exc:
        raise UnreachableTargetError(
            f"connection closed while awaiting GET reply (#{seq_id}): {exc}") from exc
    if not acked:
        raise UnreachableTargetError(
            f"GET sent (#{seq_id}) but not acked within {_DEFAULT_TIMEOUT}s")
    if not get_replies:
        if name is not None:
            return {"error": f"no such config field: {name!r}"}
        return {"fields": {}}
    fields = {reply.fields[0]: reply.fields[1] for reply in get_replies}
    return {"fields": fields}


def _tool_config_set(session: Session, name: str, value: float) -> dict[str, Any]:
    seq_id = motion.set(session, name, value)
    try:
        acked, err = _await_ack_and_err(session, seq_id)
    except TransportClosed as exc:
        raise UnreachableTargetError(
            f"connection closed while awaiting SET ack (#{seq_id}): {exc}") from exc
    if not acked:
        raise UnreachableTargetError(
            f"SET sent (#{seq_id}) but not acked within {_DEFAULT_TIMEOUT}s")
    if err is not None:
        code = err.fields[0] if err.fields else "?"
        if code == _ERR_UNKNOWN:
            return {"acked": True, "seq_id": seq_id,
                     "error": f"no such config field: {name!r}"}
        return {"acked": True, "seq_id": seq_id, "error": f"err {code}"}
    return {"acked": True, "seq_id": seq_id, "name": name, "value": value}


def _tool_calibrate_turns(
    measured_degrees: list[float], target_deg: float, save: bool
) -> dict[str, Any]:
    """This ticket's own AC #4: a non-interactive `calibrate_turns` tool
    calling ticket 005's non-interactive trial-loop CORE
    (`rogo.calibrate.compute_calibration()` -- see that function's own
    module-docstring section, "Pure residual computation ... this is
    the core ticket 007's MCP tool needs without a terminal prompt")
    rather than `rogo.calibrate.calibrate_turns()`'s own `input()`-based
    prompt loop, which an MCP client cannot answer.

    `measured_degrees` IS the explicit trial-count/measured-values shape
    the AC asks for: its own length is the trial count, one already-
    measured result per trial -- each trial having been driven
    separately (e.g. via this same server's own `turn` tool, at
    `target_deg`/an agreed speed) and measured externally (tape
    measure/protractor), since a single MCP tool call cannot pause
    mid-call for a human measurement the way `input()` can. No trial
    driving or duplicated trial-sequencing logic happens in this
    function at all -- it is a pure compute-and-optionally-persist
    step, exactly `rogo.calibrate.calibrate_turns()`'s own
    `compute_calibration()` + `_report_and_save()` tail end, minus the
    `input()`-based Y/n prompt (`save` replaces it as an explicit
    argument)."""
    cfg = config.load_active_robot()
    if cfg is None:
        return {"error": "no active robot config found "
                          "(config/robots/active_robot.json) -- can't calibrate"}
    result = calibrate.compute_calibration(
        cfg.rotational_slip, target_deg, measured_degrees, calibrate.SLIP_SANE_RANGE)
    response: dict[str, Any] = {
        "trials_used": len(result.samples),
        "starting_value": result.starting_value,
        "mean_ratio": result.mean_ratio,
        "updated_value": result.updated_value,
        "rejected_reason": result.rejected_reason,
        "saved": False,
    }
    if result.updated_value is not None and save:
        new_cfg = dataclasses.replace(cfg, rotational_slip=result.updated_value)
        config.save_robot_config(new_cfg)
        response["saved"] = True
        response["config_path"] = str(new_cfg.path)
    return response


# ---------------------------------------------------------------------------
# Server assembly -- one `MCPServer` per resolved connection, built
# fresh by `serve()` for each `rogo mcp` invocation (SUC-005's own Main
# Flow: the target is resolved once, "same resolution as SUC-001", not
# per tool call the way elite's own "connect"/"disconnect" tools work).
# ---------------------------------------------------------------------------

def build_server(session: Session) -> MCPServer:
    """Build the MCP server around one already-resolved `session`. Each
    registered tool is a thin closure over `session` calling straight
    into one of the `_tool_*()` bodies above -- no state of its own
    beyond that."""
    server = MCPServer(
        "rogo",
        description="Control a protocol-v6 robot, relay, or tools/sim: "
                     "drive/turn/goto motion, config get/set, and "
                     "non-interactive turn calibration.",
    )

    @server.tool()
    def hello() -> dict[str, Any]:
        """Probe the target: send HELLO, return the device banner."""
        return _tool_hello(session)

    @server.tool()
    def stop() -> dict[str, Any]:
        """Send the sequenced STOP command; report whether it was acked."""
        return _tool_stop(session)

    @server.tool()
    def drive(left: int, right: int, ms: int) -> dict[str, Any]:
        """Drive both wheels via WHEELS_V: left/right speeds in mm/s,
        held for ms milliseconds. Returns the ack/completion outcome."""
        return _tool_drive(session, left, right, ms)

    @server.tool()
    def turn(degrees: float, speed: float = _DEFAULT_TURN_SPEED_MM_S) -> dict[str, Any]:
        """Turn in place `degrees` (positive = CCW) at wheel-speed
        magnitude `speed` mm/s, using the active robot's rotation
        model (trackwidth/rotational_slip from config/robots/)."""
        return _tool_turn(session, degrees, speed)

    @server.tool()
    def goto(
        x: int, y: int, speed: int = _DEFAULT_GOTO_SPEED_MM_S,
        arrive: int = _DEFAULT_GOTO_ARRIVE_MM, timeout_ms: int | None = None,
    ) -> dict[str, Any]:
        """Drive to a robot-frame point (x forward, y left, mm) via
        GO_TO_R. `timeout_ms` defaults to a generous ETA-based backstop
        when omitted. Reports the adapter's actual reply, including the
        documented kUnknown planner gap on DiffDriveAdapter today."""
        return _tool_goto(session, x, y, speed, arrive, timeout_ms)

    @server.tool(name="config_get")
    def config_get(name: str | None = None) -> dict[str, Any]:
        """Read one named config field, or every field the adapter
        reports if `name` is omitted (protocol.md#7's GET)."""
        return _tool_config_get(session, name)

    @server.tool(name="config_set")
    def config_set(name: str, value: float) -> dict[str, Any]:
        """Write one config field (protocol.md#7's SET)."""
        return _tool_config_set(session, name, value)

    @server.tool(name="calibrate_turns")
    def calibrate_turns_tool(
        measured_degrees: list[float],
        target_deg: float = calibrate.DEFAULT_TURN_TARGET_DEG,
        save: bool = False,
    ) -> dict[str, Any]:
        """Non-interactive rotational_slip calibration: compute an
        updated value from already-measured trial results (one entry
        per trial -- drive each trial separately, e.g. via the `turn`
        tool, and measure it externally first). Does not drive the
        robot itself. Set `save=true` to persist a valid result to the
        active robot's config file."""
        return _tool_calibrate_turns(measured_degrees, target_deg, save)

    return server


def resolve_listen_target(listen: str | None, allow_remote: bool) -> tuple[str, int] | None:
    """`None` when `--listen` was not given -- `serve()`'s default,
    stdio transport, no network surface at all (module docstring's
    Security section). Otherwise parses `HOST:PORT` (same colon-split
    convention `rogo.connection._split_host_port()` uses for
    `--connect`) and enforces this ticket's own binding rule: a
    loopback `HOST` is always allowed; any other host additionally
    requires `allow_remote=True`, else raises `ListenTargetError`.

    Public (no leading underscore) rather than this module's other
    internal helpers: `rogo.cli.cmd_mcp()` calls this too, to validate
    `--listen`/`--allow-remote` BEFORE `connection.resolve()` spends
    time/resources reaching a robot/relay/sim just to then fail on a
    malformed or disallowed flag."""
    if listen is None:
        return None
    host, sep, port_text = listen.rpartition(":")
    if not sep or not host or not port_text.isdigit():
        raise ListenTargetError(f"--listen expects HOST:PORT, got {listen!r}")
    port = int(port_text)
    if host not in LOOPBACK_HOSTS and not allow_remote:
        raise ListenTargetError(
            f"--listen {listen!r} names a non-loopback host ({host!r}) -- "
            "pass --allow-remote to bind anywhere other than localhost "
            "(rogo mcp is a new external control surface with no "
            "authentication at this layer, sprint.md's own Migration "
            "Concerns security note)"
        )
    return host, port


def serve(session: Session, *, listen: str | None = None, allow_remote: bool = False) -> int:
    """Build the tool server around `session` and run it: `stdio`
    transport by default (no `--listen`), or `streamable-http` bound to
    `--listen`'s `HOST:PORT` once `_resolve_listen_target()` has cleared
    it. Blocks for the server's whole lifetime (a stdio client
    disconnecting, or Ctrl-C for a `--listen` server); always returns 0
    on a clean exit -- `rogo.cli.cmd_mcp()` is what maps a
    `ListenTargetError`/`TransportClosed` raised around THIS call to a
    nonzero process exit code.

    `session` is whatever `cmd_mcp()`'s own `daemon_client.get_connection(
    args, spawn=True)` resolved (ticket 010) -- a direct `rogo.connection.
    Connection.session` or a daemon-routed `_RemoteSession` proxy; this
    function (and every `_tool_*()` body it builds tools around) does not
    need to know which (module docstring's own "session may now be
    daemon-routed" section)."""
    target = resolve_listen_target(listen, allow_remote)
    server = build_server(session)
    if target is None:
        server.run("stdio")
    else:
        host, port = target
        server.run("streamable-http", host=host, port=port)
    return 0

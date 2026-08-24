"""daemon_client.py -- the find/spawn/direct-connect policy (sprint.md's
Architecture Step 3, `rogo.daemon_client`'s own row; ticket 008's own
Description): the ONLY module that decides whether a caller talks to an
already-running `rogo serve` daemon, a freshly spawned one, or falls
back to today's direct connection (`rogo.connection.resolve()`,
unchanged). Returns an object presenting the same call surface
`rogo.connection.resolve()`'s `Connection` already does, so `cli.py`'s
existing per-verb dispatch bodies (`_run_hello`, `_dispatch_drive_mode`,
`_run_turn`, ...) need no changes beyond how they obtain their
connection -- this ticket's own AC #3.

---- Two policies, chosen by the caller (`get_connection(..., spawn=)`)
----

- **Auto-detect-only** (`spawn=False`) -- used by `cli.py`'s one-shot
  subcommands (ticket 009): look for a running daemon by robot name; if
  found, connect to it; if not, fall back to `rogo.connection.resolve()`
  UNCHANGED -- zero regression for a caller that never runs `rogo
  serve`. Never spawns a process of its own (sprint.md's Design
  Rationale: "a fire-and-forget one-shot command has no natural moment
  to ever stop [a spawned] daemon again").
- **Auto-spawn-if-absent** (`spawn=True`) -- used by `cli.py`'s
  `cmd_repl()` and `mcp_server.py` (tickets 009/010), both themselves
  long-lived session tools: same lookup; if not found, spawn a daemon as
  a real OS subprocess (`subprocess.Popen`, not a Python import of
  `daemon.py` -- it must outlive this call), wait for it to become
  reachable within a bounded timeout, then connect. Raises
  `DaemonUnavailableError` (not a hang) if it never becomes reachable.

---- The client-side connection is a generic Session-RPC proxy, not a
per-CLI-verb one ----

`cli.py`'s dispatch bodies (`_run_hello`, `_run_stop`,
`_dispatch_drive_mode`, ...) are built entirely on
`robot_v6.reliability.Session`'s own public surface: `send()`,
`send_unsequenced()`, `pump()`, `highest_acked`, `wait_for_ack()`,
`wait_for_done()` (see `rogo.cli`'s own module-level helpers and
`robot_v6.motion`'s six operations, all of which bottom out in
`session.send()`). For those bodies to run UNCHANGED against a
daemon-proxied connection (this ticket's own AC #3), `ClientConnection`
below hands them a `_RemoteSession` that implements exactly that same
surface -- each call translating into ONE framed `daemon_protocol`
request ("session_send", "session_pump", "session_wait_for_ack", ...)
against a GENERIC dispatch table (`build_session_dispatch_table()`)
that, server-side, forwards straight to the SAME method on the REAL
`Session` the daemon holds, and maps the result back into the same
Python shapes (`robot_v6.codec.Reply`, `robot_v6.reliability.DoneEvent`).
This is deliberately NOT a table of `cli.py`-level verbs ("drive",
"turn", "goto", ...): a per-CLI-verb table would require `cli.py`'s own
dispatch bodies to be rewritten to call it, which contradicts this
ticket's own "no changes beyond how it obtains its connection" AC, and
it would also require importing `cli.py`'s per-verb logic into whatever
gets spawned -- which does not exist as a wired `rogo serve` subcommand
yet (ticket 009's job; see "Spawning a real daemon with no `rogo serve`
subcommand yet" below). Remoting `Session`'s own primitives instead is
independent of `cli.py` entirely, and is exactly what a generic
"connection-shaped client interface" (sprint.md Step 2) means in
practice.

---- Spawning: `default_spawn_argv()` boots the real `serve` subcommand
(ticket 009 reconciliation) ----

Ticket 008 (this module's original state) had no `serve` subcommand to
spawn yet, so `default_spawn_argv()` booted THIS module itself as a
bootable worker (`python -m rogo.daemon_client --sim|--connect|--port
... --name <name> --idle-timeout <seconds>`, see `run_daemon_worker()`
below) -- a real, but standalone, boot path with no dependency on the
CLI router module. Ticket 009 built that `serve` subcommand
(`rogo.cli`'s own `cmd_serve()`), and reconciled `default_spawn_argv()`
to boot THAT instead: `python -m rogo.cli serve --sim|--connect|--port
... --name <name> --idle-timeout <seconds> [--socket-dir <dir>]` --
robust whether or not the `rogo` console script is on `PATH`, since
`-m` resolves the module directly. `cmd_serve()`'s own Unix-socket
branch binds a `daemon.UnixSocketListener` at
`daemon.socket_path_for_name(name)`, injects `build_session_dispatch_
table()` (below) -- the SAME table this module's own worker already
used -- and self-terminates after `idle_timeout` idle seconds exactly
like `run_daemon_worker()` does (`cmd_serve()`'s own `--idle-timeout`
flag, mirroring this module's idle-tracking approach in miniature,
documented at its own call site). `spawn_argv=` on `get_connection()`
stays fully overridable (a caller, or a test, may still boot anything
it likes in place of the default). `run_daemon_worker()`/`__main__`
below remain a still-functional, dependency-light ALTERNATIVE boot path
-- useful directly, or embedded, with no `argparse`/CLI-router
involved at all -- but are no longer what `default_spawn_argv()` itself
targets; either way, THIS module's own wire vocabulary ("session_send",
...) is what a `ClientConnection` speaks, so nothing about the CLIENT
half ever needed to change across this reconciliation.

---- Idle-timeout self-termination (sprint.md Step 7's own open
question; this ticket's own decision) ----

An auto-spawned worker tracks the timestamp of its own most-recently
DISPATCHED request (not literally "zero open sockets" -- `UnixSocketListener`
exposes no public connected-client count to observe that directly, and
"no request served in `idle_timeout` seconds" is what a caller actually
experiences as "nobody is using this daemon" in practice) and exits its
own serve loop once that idle interval elapses, running its normal
`listener.stop()`/`server.stop()`/`conn.transport.close()` teardown
before the process exits. Default: `DEFAULT_IDLE_TIMEOUT_S` (5 minutes)
-- long enough not to flap during an ordinary interactive `rogo repl`/
`rogo mcp` session, short enough not to accumulate indefinitely across a
classroom's many auto-spawned daemons (sprint.md's own stated risk).
Overridable per the sprint's own requirement ("make it overridable via
flag/env var"): the `idle_timeout` parameter of `get_connection()`, the
worker's own `--idle-timeout` flag, or the `ROGO_DAEMON_IDLE_TIMEOUT`
environment variable (`IDLE_TIMEOUT_ENV_VAR`), in that precedence order.

---- Client-side robot-name resolution is deliberately narrower than
the daemon's own `resolve_robot_name()` ----

`daemon.resolve_robot_name()` is a SERVER-side operation: a daemon that
already holds a connection uses it to name ITS OWN socket, via a live
`HELLO` round trip when no override is given. A CLIENT must decide
whether a daemon is even worth looking for BEFORE paying for a
connection of its own -- sending `HELLO` first to learn a name would
defeat the entire point for a real serial port (the exact
open/close-resets-the-robot problem this whole daemon exists to avoid).
`resolve_client_name()` below therefore recognizes only the two tiers
that need no live connection: an explicit override (or `args.name`, for
a caller whose own parser added such a flag) wins immediately; a
`--sim` target with neither falls back to the same fixed default
`daemon.resolve_robot_name()` uses (`"sim"`). It returns `None` --  not
an error -- when neither applies (e.g. `--port`/`--connect` with no
`--name`): `get_connection()`'s auto-detect policy then falls straight
to direct-connect (nothing to look up), and its auto-spawn policy raises
`RobotNameRequiredError` (spawning with no name to bind a socket under
makes no sense).

---- Module boundary discipline ----

This module never imports `rogo.cli`, directly or indirectly -- it is a
library several callers (`cli.py`, `mcp_server.py`, tickets 009/010)
depend on, not the other way around (this ticket's own AC #5).
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import itertools
import os
import select
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence

from robot_v6 import codec
from robot_v6.reliability import DoneEvent, PendingBufferFull
from robot_v6.transport import Transport, TransportClosed

from . import connection, daemon
from . import daemon_protocol as dp

# ---------------------------------------------------------------------------
# Tunables -- see module docstring's "Idle-timeout self-termination" section
# for the idle-timeout ones specifically.
# ---------------------------------------------------------------------------

DEFAULT_IDLE_TIMEOUT_S = 300.0  # [s] 5 minutes -- this ticket's own decision.
IDLE_TIMEOUT_ENV_VAR = "ROGO_DAEMON_IDLE_TIMEOUT"
DEFAULT_SPAWN_TIMEOUT_S = 10.0  # [s] bounded wait for a spawned daemon's
# socket to become connectable -- generous for a local subprocess start
# (tools/sim, once already built, starts in well under a second), still
# bounded so an auto-spawn caller never hangs indefinitely.
DEFAULT_FIND_TIMEOUT_S = 1.0  # [s] connect+probe timeout when checking
# whether a daemon is already listening -- a local Unix-socket connect is
# near-instant either way (refused/missing immediately, accepted
# immediately); this bounds the PROBE round trip, not the connect itself.

_SIM_DEFAULT_NAME = "sim"  # matches daemon.resolve_robot_name()'s own
# default_sim_name -- see resolve_client_name()'s own docstring for why
# this module never calls that function itself.

_RPC_MARGIN_S = 3.0  # [s] -- wire-level slack added on top of a remoted
# Session call's own server-side timeout, so the CLIENT never gives up on
# a reply before the DAEMON itself would have (the daemon's one worker
# thread blocks dispatching wait_for_ack()/wait_for_done() for up to that
# long -- see daemon.py's own "single wire-owner thread" note).


class DaemonUnavailableError(RuntimeError):
    """Raised by `_DaemonWireClient.call()` when no reply for a given
    request arrives within its own timeout, and by `get_connection()`
    when an auto-spawned daemon never becomes reachable within
    `spawn_timeout` -- either way, "the daemon did not answer in time,"
    never a hang."""


class RobotNameRequiredError(RuntimeError):
    """Raised by `get_connection(..., spawn=True)` when no robot name
    could be determined without opening a connection (see
    `resolve_client_name()`'s own docstring) -- there is nothing to name
    a spawned daemon's socket after."""


# ---------------------------------------------------------------------------
# Client-side connection object -- the same shape as
# `rogo.connection.Connection` (`.transport`, `.session`), so callers do
# not need to branch on which kind of connection they got.
# ---------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class ClientConnection:
    """Returned by `find_daemon()`/`get_connection()` when a request was
    routed through a daemon. `.transport.close()` closes only THIS
    client's own socket to the daemon -- the daemon process (and its own
    held robot connection) is unaffected, exactly like closing one of
    several concurrent `UnixSocketListener` client connections does
    server-side."""

    transport: Transport
    session: "_RemoteSession"


class _UnixSocketTransport(Transport):
    """A client connection to a `UnixSocketListener`'s socket path --
    the client-side mirror of `SocketTransport` (TCP), just `AF_UNIX`
    instead of `AF_INET`. `robot_v6.transport` has no Unix-socket
    transport of its own (nothing on the ROBOT-facing wire ever needs
    one); this one speaks `daemon_protocol`'s line-oriented framing over
    it via the SAME `Transport.send_line()`/`read_lines()` base every
    other transport in this codebase already uses."""

    def __init__(self, socket_path: Path, *, connect_timeout: float) -> None:
        super().__init__()
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(connect_timeout)
        try:
            sock.connect(str(socket_path))
        except OSError:
            sock.close()
            raise
        sock.settimeout(None)  # blocking; _read_chunk uses select() for pacing
        self._sock = sock

    def _read_chunk(self, timeout: float | None) -> bytes:
        ready, _, _ = select.select([self._sock], [], [], timeout)
        if not ready:
            return b""
        data = self._sock.recv(4096)
        if data == b"":
            raise TransportClosed("unix socket closed by daemon")
        return data

    def _write_bytes(self, data: bytes) -> None:
        self._sock.sendall(data)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


_KNOWN_EXCEPTION_TYPES: dict[str, type[Exception]] = {
    "ValueError": ValueError,
    "TypeError": TypeError,
    "PendingBufferFull": PendingBufferFull,
}
# `DaemonServer._execute()`'s own contract (daemon.py): a dispatch body's
# raised exception becomes a failed Reply with `type` set to the
# exception's class name. This is the reverse mapping, used by
# `_DaemonWireClient._unwrap()` so a remoted Session call raises the SAME
# kind of exception a direct call would have -- an unrecognized type name
# (any exception this table does not know about) becomes a plain
# `RuntimeError` carrying the daemon's own message, rather than silently
# swallowing the distinction.


class _DaemonWireClient:
    """Sends one framed `daemon_protocol.Request` per `call()` and
    blocks for ITS OWN correlated reply (matched by id -- the wire's only
    pairing guarantee, same as `daemon_test_helpers.ForkedDaemon`'s own
    client, tests/host/rogo/daemon_test_helpers.py). A reply whose
    `error` is set is re-raised as a Python exception (see
    `_KNOWN_EXCEPTION_TYPES` above) rather than returned."""

    def __init__(self, transport: Transport) -> None:
        self._transport = transport
        self._id_counter = itertools.count(1)

    def call(self, verb: str, params: dict, *, timeout: float | None) -> object:
        request_id = next(self._id_counter)
        self._transport.send_line(
            dp.encode_request(dp.Request(id=request_id, verb=verb, params=params)))
        deadline = None if timeout is None else time.monotonic() + timeout
        poll = 0.05
        while True:
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                raise DaemonUnavailableError(
                    f"no reply from daemon for {verb!r} (id={request_id}) "
                    f"within {timeout}s"
                )
            wait = poll if remaining is None else min(poll, remaining)
            for line in self._transport.read_lines(timeout=wait):
                reply = dp.decode_reply(line)
                if reply.id == request_id:
                    return self._unwrap(reply)
                # A reply for a different id should not occur -- this
                # client always waits for its own call's reply before
                # issuing the next one (no pipelining across calls) --
                # but ignoring it rather than raising keeps this loop
                # robust if it ever did.

    @staticmethod
    def _unwrap(reply: dp.Reply) -> object:
        if reply.error is not None:
            exc_type = _KNOWN_EXCEPTION_TYPES.get(reply.error.type, RuntimeError)
            raise exc_type(reply.error.message)
        return reply.result

    def close(self) -> None:
        self._transport.close()


class _RemoteSession:
    """Client-side stand-in for `robot_v6.reliability.Session`, handed
    back as `ClientConnection.session` -- implements exactly the subset
    of `Session`'s public surface `rogo.cli`'s existing dispatch bodies
    call (`send`/`send_unsequenced`/`pump`/`highest_acked`/
    `wait_for_ack`/`wait_for_done`), each one translating into ONE
    `daemon_protocol` request/reply round trip against the generic
    session-RPC dispatch table `build_session_dispatch_table()` builds
    server-side. See module docstring's "generic Session-RPC proxy"
    section for the full rationale."""

    def __init__(self, wire: _DaemonWireClient) -> None:
        self._wire = wire

    def send(self, verb: str, *fields: object) -> int:
        return self._wire.call(
            "session_send", {"wire_verb": verb, "wire_fields": list(fields)},
            timeout=_RPC_MARGIN_S,
        )

    def send_unsequenced(self, verb: str, *fields: object) -> None:
        self._wire.call(
            "session_send_unsequenced",
            {"wire_verb": verb, "wire_fields": list(fields)},
            timeout=_RPC_MARGIN_S,
        )

    def pump(self, timeout: float | None = 0.0) -> list[codec.Reply]:
        wire_timeout = None if timeout is None else timeout + _RPC_MARGIN_S
        result = self._wire.call("session_pump", {"timeout": timeout}, timeout=wire_timeout)
        return [codec.Reply(verb=r["verb"], fields=tuple(r["fields"]), id=r["id"]) for r in result]

    @property
    def highest_acked(self) -> int:
        return self._wire.call("session_highest_acked", {}, timeout=_RPC_MARGIN_S)

    def wait_for_ack(self, seq_id: int, timeout: float | None = 5.0) -> bool:
        wire_timeout = None if timeout is None else timeout + _RPC_MARGIN_S
        return self._wire.call(
            "session_wait_for_ack", {"seq_id": seq_id, "timeout": timeout},
            timeout=wire_timeout,
        )

    def wait_for_done(self, seq_id: int, timeout: float | None = 5.0) -> DoneEvent | None:
        wire_timeout = None if timeout is None else timeout + _RPC_MARGIN_S
        result = self._wire.call(
            "session_wait_for_done", {"seq_id": seq_id, "timeout": timeout},
            timeout=wire_timeout,
        )
        if result is None:
            return None
        return DoneEvent(id=result["id"], reason=result["reason"])


# ---------------------------------------------------------------------------
# The generic session-RPC dispatch table -- server side of `_RemoteSession`
# above. Reusable by a future `cmd_serve()` (ticket 009); built and used
# directly by this module's own worker (`run_daemon_worker()` below).
#
# `_dispatch_session_wait_for_ack`/`_dispatch_session_wait_for_done` are
# the two handlers here that can genuinely block for a long time (a
# `drive`/`turn`/`goto`'s own multi-second completion wait, `cli.py`'s
# `_cmd_drive_ms()` and friends) -- exactly the "one client's long
# `drive`" case the issue's own safety carry-over names. Ticket 011's own
# end-to-end pass against the REAL `rogo serve` wiring found both of
# them discarding their own `abort` argument (`del abort`) entirely: the
# estop-priority queue could set `abort` all day and neither handler
# would ever notice, so an ESTOP from another client would still sit
# behind a long drive's full natural timeout, not preempt it. Both are
# now poll loops -- bounded `_WAIT_POLL_S`-sized calls into the
# underlying (abort-unaware) `Session.wait_for_ack()`/`wait_for_done()`,
# rechecking `abort` between each -- so an estop is felt within one poll
# interval instead of only once the original call's own full timeout
# elapses. `_dispatch_session_send`/`_dispatch_session_send_unsequenced`
# (non-blocking) and `_dispatch_session_highest_acked` (no I/O) need no
# such change; `_dispatch_session_pump()`'s own `timeout` is always a
# short, caller-controlled value in every call site this codebase makes
# (`rogo.cli`'s own `_pump_until()` polls in 0.2s steps) rather than a
# multi-second completion wait, so it is left as a plain pass-through.
# ---------------------------------------------------------------------------

_WAIT_POLL_S = 0.1  # [s] -- how often the two abort-aware wait dispatch
# bodies below recheck `abort` between short polls of the underlying
# (Session-level, abort-unaware) blocking call. Small enough that an
# estop's own preemption is felt promptly (daemon.py's own
# "Estop-priority queue" safety requirement); large enough not to turn
# a long wait into a pump() busy-loop.


def _reply_to_json(reply: codec.Reply) -> dict:
    return {"verb": reply.verb, "fields": list(reply.fields), "id": reply.id}


def _dispatch_session_send(session, params, abort):  # noqa: ANN001 -- DispatchFn shape
    del abort
    return session.send(params["wire_verb"], *params.get("wire_fields", []))


def _dispatch_session_send_unsequenced(session, params, abort):
    del abort
    session.send_unsequenced(params["wire_verb"], *params.get("wire_fields", []))
    return None


def _dispatch_session_pump(session, params, abort):
    del abort
    replies = session.pump(params.get("timeout", 0.0))
    return [_reply_to_json(r) for r in replies]


def _dispatch_session_highest_acked(session, params, abort):
    del params, abort
    return session.highest_acked


def _dispatch_session_wait_for_ack(session, params, abort):
    seq_id = params["seq_id"]
    timeout = params.get("timeout", 5.0)
    deadline = None if timeout is None else time.monotonic() + timeout
    while True:
        if abort.is_set():
            return False
        remaining = None if deadline is None else deadline - time.monotonic()
        if remaining is not None and remaining <= 0:
            return False
        step = _WAIT_POLL_S if remaining is None else min(_WAIT_POLL_S, remaining)
        if session.wait_for_ack(seq_id, timeout=step):
            return True


def _dispatch_session_wait_for_done(session, params, abort):
    seq_id = params["seq_id"]
    timeout = params.get("timeout", 5.0)
    deadline = None if timeout is None else time.monotonic() + timeout
    while True:
        if abort.is_set():
            return None
        remaining = None if deadline is None else deadline - time.monotonic()
        if remaining is not None and remaining <= 0:
            return None
        step = _WAIT_POLL_S if remaining is None else min(_WAIT_POLL_S, remaining)
        done = session.wait_for_done(seq_id, timeout=step)
        if done is not None:
            return {"id": done.id, "reason": done.reason}


def build_session_dispatch_table() -> daemon.DispatchTable:
    """The generic Session-RPC verb table -- one entry per `_RemoteSession`
    method, each forwarding straight to the identically-named method on
    the REAL `Session` a `DaemonServer` holds. Public so a future
    `cmd_serve()` (ticket 009) can reuse it verbatim rather than
    reimplementing this mapping."""
    return {
        "session_send": _dispatch_session_send,
        "session_send_unsequenced": _dispatch_session_send_unsequenced,
        "session_pump": _dispatch_session_pump,
        "session_highest_acked": _dispatch_session_highest_acked,
        "session_wait_for_ack": _dispatch_session_wait_for_ack,
        "session_wait_for_done": _dispatch_session_wait_for_done,
    }


# ---------------------------------------------------------------------------
# is_estop_request() -- `rogo serve`'s own `DaemonServer(..., is_estop=...)`
# classifier (`cli.cmd_serve()`). See `daemon.py`'s own `DEFAULT_ESTOP_VERBS`
# / `is_estop` docstring sections for why `DaemonServer`'s plain
# `estop_verbs` membership check (against `request.verb` alone) can never
# recognize a real `ESTOP` sent through this module's generic session-RPC
# table: every `rogo.cli` dispatch body's own wire verb (`STOP`, `WHEELS_V`,
# `ESTOP`, ...) travels as a `wire_verb` PARAMETER of a
# `session_send`/`session_send_unsequenced` REQUEST, never as that
# request's own top-level `verb` -- so `request.verb` is always one of
# THIS table's six generic RPC names, never `"estop"`/`"halt"`, no matter
# what a client actually sent. This function looks one level deeper, at
# the wrapped `wire_verb`, so the issue's own safety carry-over ("an
# estop/halt request from ANY client jumps to the front of the work
# queue") holds for a REAL `rogo serve` daemon, not just for a test's own
# directly-named fake dispatch table.
# ---------------------------------------------------------------------------

_ESTOP_WIRE_VERBS = frozenset({"ESTOP"})
# protocol.md#8.3's own unsequenced "panic path" verb
# (`robot_v6.motion.estop()`'s own docstring: "the panic path, not a
# general-purpose halt"). `STOP` (`robot_v6.motion.stop()`) is a
# PLANNED, sequenced stop -- deliberately excluded here, matching
# motion.py's own documented distinction between the two.


def is_estop_request(request: dp.Request) -> bool:
    """True when `request` is a `session_send`/`session_send_unsequenced`
    RPC call whose wrapped `wire_verb` is itself safety-critical
    (`ESTOP`) -- see this section's own header comment for the full
    rationale. Every other request (including `session_send_unsequenced`
    calls for any OTHER unsequenced verb, e.g. `HELLO`/`PING`) is not
    estop-priority."""
    if request.verb not in ("session_send", "session_send_unsequenced"):
        return False
    return request.params.get("wire_verb") in _ESTOP_WIRE_VERBS


def _with_activity_tracking(
    table: daemon.DispatchTable, last_activity: list[float],
) -> daemon.DispatchTable:
    """Wrap every handler in `table` so calling it stamps
    `last_activity[0]` with the current time -- `run_daemon_worker()`'s
    own idle-timeout watchdog reads this same list to decide when to
    stop (see module docstring's "Idle-timeout self-termination"
    section). A one-element list, not a plain float, purely so the
    wrapped closures below can mutate it without a `nonlocal` per
    handler."""
    def _wrap(fn):
        def _wrapped(session, params, abort):
            last_activity[0] = time.monotonic()
            return fn(session, params, abort)
        return _wrapped
    return {verb: _wrap(fn) for verb, fn in table.items()}


# ---------------------------------------------------------------------------
# Client-side name resolution -- see module docstring's own section.
# ---------------------------------------------------------------------------

def resolve_client_name(args: argparse.Namespace, *, override: str | None = None) -> str | None:
    """Determine the robot name to look up/spawn a daemon under, WITHOUT
    ever opening a connection. See module docstring for the full
    rationale. `override` wins immediately; else `args.name` (if the
    caller's own parser happens to carry one); else `"sim"` for a
    `--sim` target with neither; else `None`."""
    if override:
        return override
    name = getattr(args, "name", None)
    if name:
        return name
    if getattr(args, "sim", False):
        return _SIM_DEFAULT_NAME
    return None


# ---------------------------------------------------------------------------
# Find -- connect to an already-running daemon, if one exists.
# ---------------------------------------------------------------------------

def find_daemon(
    name: str, *, socket_dir: Path | None = None, timeout: float = DEFAULT_FIND_TIMEOUT_S,
) -> ClientConnection | None:
    """Try to connect to a daemon already listening for `name`
    (`daemon.socket_path_for_name()`'s own resolution rule). Returns
    `None` -- never raises -- for any reason the socket is not there or
    not answering (missing file, refused connection, or a stale
    endpoint that does not speak `daemon_protocol`): "not found" and "a
    dead daemon's leftover socket" are the same outcome to a caller
    deciding whether to fall back to direct-connect or spawn."""
    socket_path = daemon.socket_path_for_name(name, socket_dir=socket_dir)
    try:
        transport = _UnixSocketTransport(socket_path, connect_timeout=timeout)
    except OSError:
        return None

    wire = _DaemonWireClient(transport)
    try:
        # A cheap probe proving the far end genuinely speaks
        # daemon_protocol, not just "something is listening on this
        # path" -- daemon.LIVENESS_PROBE_VERB is answered directly by
        # UnixSocketListener._serve_client(), BEFORE DaemonServer.submit(),
        # so this probe can never queue behind a busy worker thread (see
        # that constant's own docstring). Previously sent
        # "session_highest_acked" -- an ordinary priority-1 RPC that COULD
        # queue behind another client's in-flight dispatch, creating a
        # chicken-and-egg gap where a cold client's own connectivity probe
        # could time out and silently fall back to a direct connection
        # before the daemon's estop-priority path was ever reached (sprint
        # 004 ticket 002's root-cause fix; see daemon.py's module
        # docstring's "Liveness-probe fast path" section).
        wire.call(daemon.LIVENESS_PROBE_VERB, {}, timeout=timeout)
    except (OSError, dp.ProtocolError, DaemonUnavailableError, TransportClosed):
        transport.close()
        return None
    return ClientConnection(transport=transport, session=_RemoteSession(wire))


def _wait_for_daemon(
    name: str, *, socket_dir: Path | None, timeout: float, poll_interval: float = 0.05,
) -> ClientConnection | None:
    """Poll `find_daemon()` until it succeeds or `timeout` seconds
    elapse -- the "wait for it to become reachable" half of the
    auto-spawn policy."""
    deadline = time.monotonic() + timeout
    while True:
        found = find_daemon(name, socket_dir=socket_dir, timeout=min(poll_interval, timeout))
        if found is not None:
            return found
        if time.monotonic() >= deadline:
            return None
        time.sleep(poll_interval)


# ---------------------------------------------------------------------------
# Spawn -- a real OS subprocess, not a Python import of daemon.py. See
# module docstring's "Spawning: default_spawn_argv() boots the real
# serve subcommand" section for why the default argv now invokes the
# CLI router module's own `serve` subcommand (ticket 009 reconciliation)
# rather than this module's own standalone worker.
# ---------------------------------------------------------------------------

def default_spawn_argv(
    args: argparse.Namespace, *, name: str, idle_timeout: float, socket_dir: Path | None = None,
) -> list[str]:
    """Build the subprocess argv `get_connection(..., spawn=True)` uses
    when a caller does not supply its own `spawn_argv` -- boots the
    `serve` subcommand of the CLI router module (`python -m rogo.cli
    serve`, robust whether or not the `rogo` console script is on
    `PATH`, since `-m` resolves the module directly) against the same
    `--sim`/`--connect`/`--port` target `args` names, with `name`
    already resolved (so the daemon it starts never re-resolves it via
    its own HELLO round trip) and `idle_timeout` passed through
    explicitly (`--idle-timeout`, self-terminates the spawned daemon
    once that many seconds pass with no dispatched request -- an
    auto-spawned worker has no interactive user to Ctrl-C it)."""
    argv = [sys.executable, "-m", "rogo.cli", "serve"]
    if getattr(args, "sim", False):
        argv.append("--sim")
    elif getattr(args, "connect", None):
        argv += ["--connect", str(args.connect)]
    elif getattr(args, "port", None):
        argv += ["--port", str(args.port)]
    argv += ["--name", name, "--idle-timeout", str(idle_timeout)]
    if socket_dir is not None:
        argv += ["--socket-dir", str(socket_dir)]
    return argv


def _spawn_daemon(argv: Sequence[str]) -> subprocess.Popen:
    """Launch `argv` as a detached subprocess that outlives this call --
    `start_new_session=True` takes it out of this process's own session
    (POSIX) so it is not signaled/killed alongside a `rogo repl`/`rogo
    mcp` invocation that spawned it; stdio is discarded (a daemon worker
    prints nothing of its own -- see `run_daemon_worker()`) so this
    process never blocks on a pipe the child might fill."""
    return subprocess.Popen(  # noqa: S603 -- argv is this module's own construction
        list(argv),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


# ---------------------------------------------------------------------------
# The public policy entry point -- ticket 009/010's own call site.
# ---------------------------------------------------------------------------

def get_connection(
    args: argparse.Namespace,
    *,
    spawn: bool,
    name: str | None = None,
    socket_dir: Path | None = None,
    spawn_timeout: float = DEFAULT_SPAWN_TIMEOUT_S,
    idle_timeout: float = DEFAULT_IDLE_TIMEOUT_S,
    find_timeout: float = DEFAULT_FIND_TIMEOUT_S,
    spawn_argv: Sequence[str] | None = None,
) -> connection.Connection | ClientConnection:
    """Resolve `args` into a live connection, per whichever policy
    `spawn` selects (module docstring's own "Two policies" section):

    - Always looks for an already-running daemon first (`find_daemon()`),
      for BOTH policies -- reusing one is always preferred over spawning
      or connecting directly, regardless of which policy is asked for.
    - `spawn=False` (auto-detect-only, one-shot CLI commands): falls
      back to `rogo.connection.resolve()` UNCHANGED when none is found.
      Never spawns.
    - `spawn=True` (auto-spawn-if-absent, `rogo repl`/`rogo mcp`): spawns
      one when none is found, waits up to `spawn_timeout` seconds for it
      to become reachable, then connects. Raises
      `RobotNameRequiredError` if no name could be determined at all
      (nothing to spawn a socket under), or `DaemonUnavailableError` if
      the spawned daemon never becomes reachable in time.

    `name`, when given, is used verbatim instead of
    `resolve_client_name(args)` -- lets a caller with its own resolved
    name (e.g. a `--name` flag `cli.py` parses itself) skip this
    module's own narrower resolution tiers entirely.
    """
    resolved_name = resolve_client_name(args, override=name)
    if resolved_name is not None:
        found = find_daemon(resolved_name, socket_dir=socket_dir, timeout=find_timeout)
        if found is not None:
            return found

    if not spawn:
        return connection.resolve(args)

    if resolved_name is None:
        raise RobotNameRequiredError(
            "cannot auto-spawn a daemon: no robot name could be determined "
            "without opening a direct connection first -- pass name= "
            "explicitly, or use --sim"
        )

    argv = (
        list(spawn_argv) if spawn_argv is not None
        else default_spawn_argv(args, name=resolved_name, idle_timeout=idle_timeout,
                                 socket_dir=socket_dir)
    )
    proc = _spawn_daemon(argv)
    connected = _wait_for_daemon(resolved_name, socket_dir=socket_dir, timeout=spawn_timeout)
    if connected is None:
        if proc.poll() is None:
            with contextlib.suppress(OSError):
                proc.terminate()
        raise DaemonUnavailableError(
            f"spawned daemon for {resolved_name!r} did not become reachable "
            f"within {spawn_timeout}s (argv={argv!r})"
        )
    return connected


# ---------------------------------------------------------------------------
# The bootable worker -- `python -m rogo.daemon_client ...`. See module
# docstring's "Spawning: default_spawn_argv() boots the real serve
# subcommand" section: this is now an ALTERNATIVE, dependency-light boot
# path (no argparse/CLI-router module involved) rather than what
# get_connection(..., spawn=True)'s own default targets.
# ---------------------------------------------------------------------------

def _build_worker_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rogo.daemon_client",
        description=(
            "Internal daemon worker: boots a Unix-socket rogo daemon "
            "speaking the generic session-RPC protocol daemon_client.py's "
            "own ClientConnection expects, and self-terminates after "
            "--idle-timeout idle seconds with no dispatched request. Not "
            "a public CLI surface -- ticket 009's rogo serve subcommand "
            "is what get_connection()'s own auto-spawn policy "
            "(default_spawn_argv()) targets by default; this worker "
            "remains callable directly (run_daemon_worker()) or as a "
            "standalone subprocess, in tests or embedded, with no "
            "dependency on the CLI router module at all."
        ),
    )
    connection.add_target_arguments(parser)
    parser.add_argument(
        "--name", default=None,
        help="robot name to bind the Unix socket under (skips this "
             "process's own HELLO-based resolution when given)",
    )
    parser.add_argument(
        "--socket-dir", default=None,
        help="override the well-known socket directory (default: "
             "daemon.default_socket_dir())",
    )
    parser.add_argument(
        "--idle-timeout", type=float,
        default=float(os.environ.get(IDLE_TIMEOUT_ENV_VAR, DEFAULT_IDLE_TIMEOUT_S)),
        help=f"self-terminate after this many idle seconds with no "
             f"dispatched request (env: {IDLE_TIMEOUT_ENV_VAR}, default: "
             f"{DEFAULT_IDLE_TIMEOUT_S:g})",
    )
    return parser


def run_daemon_worker(args: argparse.Namespace) -> None:
    """Resolve `args` into a connection (`rogo.connection.resolve()` --
    the exact same resolution every other `rogo` entry point uses, so a
    `--sim` target here freshly builds/reuses `tools/sim` identically),
    bind a `UnixSocketListener` at the resolved name's socket path, and
    serve the generic session-RPC dispatch table until `idle_timeout`
    idle seconds elapse with no dispatched request -- then tear
    everything down (listener, server, connection) and return. Blocks
    for the worker's whole lifetime; `__main__` below is this function's
    only caller in production, but it is a plain function (not a script)
    so tests can call it directly, or run it in a subprocess via
    `default_spawn_argv()`."""
    conn = connection.resolve(args)
    try:
        name = args.name if getattr(args, "name", None) else daemon.resolve_robot_name(
            conn.session, sim=bool(getattr(args, "sim", False)))
        socket_dir = Path(args.socket_dir) if getattr(args, "socket_dir", None) else None
        socket_path = daemon.socket_path_for_name(name, socket_dir=socket_dir)
        idle_timeout = float(getattr(args, "idle_timeout", DEFAULT_IDLE_TIMEOUT_S))

        last_activity = [time.monotonic()]
        table = _with_activity_tracking(build_session_dispatch_table(), last_activity)

        server = daemon.DaemonServer(conn, table, is_estop=is_estop_request)
        server.start()
        try:
            listener = daemon.UnixSocketListener(server, socket_path)
            listener.start()
            try:
                poll = min(1.0, idle_timeout) if idle_timeout > 0 else 0.0
                while time.monotonic() - last_activity[0] < idle_timeout:
                    time.sleep(poll if poll > 0 else 0.05)
            finally:
                listener.stop()
        finally:
            server.stop()
    finally:
        conn.transport.close()


if __name__ == "__main__":
    run_daemon_worker(_build_worker_parser().parse_args())

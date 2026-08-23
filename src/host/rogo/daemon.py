"""daemon.py -- `rogo serve`'s server core: holds ONE robot connection
open for the whole daemon process's lifetime and routes every client
request to it, escalating any estop/halt request ahead of whatever else
is queued (sprint.md's Architecture Step 3, `rogo.daemon`'s own row;
ticket 005's own Description). Ticket 005 built the CORE
(`DaemonServer`); ticket 006 (this revision) adds the two listener
transports the issue's own Requirement 2 asks for --
`UnixSocketListener` (production: a named Unix domain socket) and
`run_stdio_pipe()` (tests/embedding: the identical framed protocol over
stdin/stdout) -- plus the robot-name/socket-path resolution
(`resolve_robot_name()`, `default_socket_dir()`, `socket_path_for_name()`,
`ensure_socket_dir()`) that decides what a Unix-socket listener's file
is named and where it lives. Both transports are thin: they decode a
request line via `daemon_protocol`, call `DaemonServer.submit()`, and
encode the reply back -- neither owns any dispatch logic of its own.
Ticket 007 (this revision) adds exactly one boot function,
`run_stdio_pipe_from_args()`, tying `rogo.connection.resolve()`'s
already-existing `--sim`/`--connect`/`--port` resolution to a freshly
built `DaemonServer` for the PIPE transport only (issue Requirement 3:
a `--sim` target here freshly builds/reuses `tools/sim` exactly the
way `rogo drive --sim` does today, with no manually started
`tools/sim` process required). The Unix-socket branch and the full
`rogo serve` CLI subcommand (`cmd_serve()`, argument parsing, a
`--stdio-pipe` flag, signal handling) remain ticket 009's job -- this
module still does not decide how a Unix-socket listener's
`DaemonServer` gets built, nor does it import `rogo.cli`.

---- Injection, not import: this module never imports `rogo.cli` ----

This is the one load-bearing design decision this sprint's own
architecture review caught and fixed (sprint.md's Architecture Step 6,
"Decision: daemon receives cli.py's per-verb dispatch by injection, the
same direction repl.py already uses"). `DaemonServer` below calls
whatever per-verb dispatch function it was constructed with (a
`DispatchTable`, keyed by wire verb) -- it does not know or care
whether a given callable ultimately calls into `rogo.cli`'s own
`_run_*()`/`cmd_*()` bodies, `rogo.connection`, or anything else. This
mirrors `rogo.repl`'s own already-established pattern exactly
(`repl.py`'s own module docstring: "this module never imports
`rogo.cli`") and for the identical reason: `rogo.cli`'s new
`cmd_serve()` (ticket 009) must import THIS module to wire the `serve`
subcommand -- an ordinary, unavoidable router-to-implementation edge,
the same shape as `cli.py`'s existing import of `mcp_server.py`/
`repl.py`. If `daemon.py` also imported `cli.py` for dispatch, the two
modules would form a cycle. An earlier draft of this sprint's own
component diagram had the dispatch-reuse edge pointing the wrong way
(`daemon.py` -> `cli.py`) -- caught and fixed in that section's own
self-review (sprint.md's Step 4 "Reading the new edges" note). Do not
reintroduce it: nothing in this module may import `rogo.cli`, nor
anything that itself imports `rogo.cli`.

---- One connection, one wire-owner thread, for the process's whole
lifetime ----

`DaemonServer` is constructed with an already-resolved
`rogo.connection.Connection` -- `DaemonServer.__init__()` itself still
never calls `rogo.connection.resolve()`; that is always a CALLER's job
(ticket 007's own `run_stdio_pipe_from_args()`, for the pipe transport,
or a future `cmd_serve()`, ticket 009, for the rest) -- and holds it
for as long as the server runs: every request, from every client, for
the server's whole lifetime, dispatches against the SAME
`Connection.session`, never a freshly reopened one (this ticket's own
acceptance criterion: "no second `resolve()` call happens per
request"). A single background worker thread (started by `start()`) is
the ONLY thread that ever calls a dispatch function -- and so the only
thread that ever touches the held `Session`/`Transport` -- even though
`submit()` itself may be called concurrently from many client-handling
threads (ticket 006's listener transports will each run their own
accept/read loop and call `submit()` once per decoded request). This
single-wire-owner discipline is what makes it safe for injected
dispatch functions to use `Session` exactly as `rogo.cli`'s own
`cmd_*()` functions already do today, with no locking of their own
required -- see sprint.md's Architecture Step 3, `rogo.daemon`'s own
"Boundary" sentence.

---- Estop-priority queue: the safety-critical part of this ticket ----

Ordinary requests are served FIFO. An estop/halt request (verb
membership in `estop_verbs`, `DEFAULT_ESTOP_VERBS` by default) is
different in two ways, both required by the issue's own safety
carry-over (quoted in sprint.md's Problem section): "an estop/halt
request from ANY client jumps to the front of the work queue and
aborts any in-progress completion wait, so one client's long `drive`
can never delay another client's halt":

  1. It always sorts ahead of every already-QUEUED non-estop request --
     a priority queue, not a FIFO one, specifically for this verb
     class.
  2. If a non-estop dispatch call is CURRENTLY running (already popped
     off the queue, already inside its own dispatch body), that call's
     own `abort` event -- the third argument every `DispatchFn` in a
     `DispatchTable` receives -- is set the instant the estop is
     submitted. A dispatch body that performs a blocking wait (e.g.
     wrapping `Session.wait_for_done()`) is expected to wait on `abort`
     cooperatively (`abort.wait(timeout=...)` rather than an
     unconditional block) so it notices the signal and returns
     promptly instead of running to its own natural completion. This
     is the ONLY way an opaque, injected dispatch function can be
     interrupted from another thread without this module knowing
     anything about what any given verb actually does -- `DaemonServer`
     has no verb-specific knowledge beyond `estop_verbs` membership.

A dispatch body with nothing to wait for (most verbs) may simply ignore
`abort` -- it is always passed so every verb shares one calling
convention, not because every verb needs to consult it.

---- `is_estop`: classification is pluggable, ticket 011's own fix
for a gap this closing ticket's own end-to-end pass surfaced ----

`estop_verbs` classifies by `request.verb` -- the request's own
TOP-LEVEL wire verb -- which is exactly right for a caller whose
dispatch table is keyed by real per-command verb names (a test's own
fake table, `{"estop": ..., "drive": ...}`; ticket 005/006's own
`tests/host/rogo/test_daemon.py` suite). It is NOT right for
`daemon_client.build_session_dispatch_table()`, the table `rogo
serve` actually injects (`cli.cmd_serve()`): every call through that
table travels as one of a handful of GENERIC RPC verbs
(`session_send`, `session_send_unsequenced`, `session_wait_for_done`,
...), with the real wire verb (`STOP`, `WHEELS_V`, `ESTOP`, ...)
carried one level down, as that request's own `wire_verb` PARAMETER --
so a real `ESTOP` call, sent through THIS scheme, arrives with
`request.verb == "session_send_unsequenced"`, never `"estop"`/`"halt"`,
and `estop_verbs` membership can never match it. This is a real
integration gap ticket 011's own end-to-end pass against the REAL
`rogo serve` wiring caught (each of tickets 005-010 tested their own
piece correctly in isolation; nothing exercised the two together
against a genuine `ESTOP` call before this ticket): the safety
carry-over would have silently done nothing in production.

`is_estop`, when given, REPLACES `estop_verbs` matching entirely with
an injected `Callable[[Request], bool]` -- the same injection-not-import
shape `dispatch_table` itself already uses (this module's own
"Injection, not import" section) -- so a caller whose own verb scheme
needs to look inside `request.params` (as `daemon_client.
is_estop_request()` does) can classify correctly without teaching
this module anything about what any given table's params mean.
`cli.cmd_serve()` passes `daemon_client.is_estop_request` for exactly
this reason. Omit it (the default) to keep the plain `estop_verbs`
membership check -- unchanged for every existing caller, including
every test in this module's own suite that constructs a
`DaemonServer` with a directly-named fake dispatch table.
"""

from __future__ import annotations

import argparse
import dataclasses
import heapq
import itertools
import os
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Mapping, TextIO

from robot_v6.reliability import Session

from .connection import Connection, resolve as resolve_connection
from .daemon_protocol import ProtocolError, Reply, Request, decode_request, encode_reply

DispatchFn = Callable[[Session, dict, "threading.Event"], object]
"""One verb's dispatch body: called as `fn(session, params, abort)`.

`session` is the ONE `Session` this `DaemonServer` was constructed with
(see module docstring) -- never a fresh one. `params` is the request's
own `daemon_protocol.Request.params` mapping (a defensive shallow
copy). `abort` is set by `DaemonServer` the instant a higher-priority
(estop-class) request is enqueued while this call is still running; a
dispatch body that blocks waiting for something (a motion's completion,
in particular) should wait on `abort` cooperatively instead of blocking
unconditionally, so an estop is never delayed behind it (module
docstring's "Estop-priority queue" section). A body with nothing to
wait for may ignore `abort` entirely -- it is always provided so every
verb shares one calling convention, not because every verb needs it.

Return value becomes a successful `Reply.result`; a raised exception
becomes a failed `Reply` (`Reply.fail`, `type` set to the exception's
own class name) -- `DaemonServer` never lets a dispatch body's
exception escape past `submit()`, so one bad/buggy verb can never take
down the worker thread or leave another client's `submit()` call
hanging forever.
"""

DispatchTable = Mapping[str, DispatchFn]
"""Verb name -> `DispatchFn`, injected at `DaemonServer` construction
-- see module docstring's "Injection, not import" section. `daemon.py`
places no constraint on what a `DispatchFn` actually does; it is
whatever `rogo.cli` (ticket 009) or a test (this ticket) hands in."""

DEFAULT_ESTOP_VERBS = frozenset({"estop", "halt"})
"""The verb names that jump the priority queue by default (used by the
default `is_estop` classifier, `DaemonServer._verb_in_estop_verbs()`) --
`daemon_protocol`'s own module docstring: "at minimum `estop`/`halt`".
Overridable per-server via `DaemonServer(..., estop_verbs=...)`, since
`daemon_protocol` itself deliberately has no verb table of its own (its
module docstring's own "This module has no verb table of its own"
sentence). These names are placeholders for a caller whose OWN
dispatch table is keyed by real per-command verb names -- a caller
using `daemon_client.build_session_dispatch_table()`'s generic
session-RPC scheme instead should pass `is_estop=daemon_client.
is_estop_request` rather than relying on this default (module
docstring's own `is_estop` section explains why `estop_verbs`
membership alone cannot see a real `ESTOP` call through that table)."""


class DaemonServerError(RuntimeError):
    """Raised by `DaemonServer.submit()`/`start()` for a usage error --
    e.g. `submit()` before `start()`, or `start()` called twice. Never
    raised for anything a client's own request causes (an unknown verb
    or a failing dispatch body both become a failed `Reply` instead --
    see `DispatchFn`'s own docstring)."""


@dataclasses.dataclass(order=True)
class _QueueItem:
    """One `submit()`-ed request, waiting in or being drained from the
    priority queue. Ordered by `(priority, sequence)` ONLY -- every
    other field is `compare=False`, since `heapq` only ever needs `<`
    on these two to decide pop order: `priority` (0 = estop-class, 1 =
    everything else) is the primary key, `sequence` (a monotonic
    counter assigned at `submit()` time, under the same lock that
    pushes onto the heap) is the tie-breaker -- which is exactly what
    makes same-priority items drain in arrival order: FIFO for
    non-estop requests relative to each other, and FIFO among estop
    requests relative to each other too, as a side effect of the
    identical mechanism, not a separate rule."""

    priority: int
    sequence: int
    request: Request = dataclasses.field(compare=False)
    abort_event: threading.Event = dataclasses.field(compare=False)
    done_event: threading.Event = dataclasses.field(compare=False)
    reply: Reply | None = dataclasses.field(default=None, compare=False)


class DaemonServer:
    """Owns one `Connection` for its whole lifetime and dispatches
    every `submit()`-ted request against it, via one background worker
    thread, in estop-priority order. See module docstring for the full
    design rationale (injection boundary, single wire-owner thread,
    the estop-priority/abort mechanism).

    Construct with an already-resolved `Connection` and a
    `DispatchTable`, call `start()` once, then `submit()` requests
    (safe from multiple threads concurrently); `stop()` drains any
    already-`submit()`-ted work and joins the worker thread. Also
    usable as a context manager (`with DaemonServer(...) as server:`),
    which calls `start()`/`stop()` for you.
    """

    def __init__(
        self,
        connection: Connection,
        dispatch_table: DispatchTable,
        *,
        estop_verbs: frozenset[str] = DEFAULT_ESTOP_VERBS,
        is_estop: Callable[[Request], bool] | None = None,
    ) -> None:
        self._connection = connection
        self._session = connection.session
        self._dispatch_table: dict[str, DispatchFn] = dict(dispatch_table)
        self._estop_verbs = frozenset(estop_verbs)
        # See module docstring's own "is_estop: classification is
        # pluggable" section -- defaults to plain estop_verbs membership,
        # unchanged for every existing caller; `cli.cmd_serve()` overrides
        # this with `daemon_client.is_estop_request` instead.
        self._is_estop: Callable[[Request], bool] = (
            is_estop if is_estop is not None else self._verb_in_estop_verbs
        )

        self._heap: list[_QueueItem] = []
        self._sequence_counter = itertools.count()
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)
        self._current_abort_event: threading.Event | None = None
        self._stop_requested = False

        self._worker: threading.Thread | None = None

    def _verb_in_estop_verbs(self, request: Request) -> bool:
        """The default `is_estop` classifier: plain `request.verb`
        membership in `estop_verbs` -- see module docstring's own
        `is_estop` section for why this is not always the right check."""
        return request.verb in self._estop_verbs

    # ---- observable state -----------------------------------------

    @property
    def connection(self) -> Connection:
        """The single `Connection` this server was constructed with --
        resolved exactly once, by the caller, before construction (see
        module docstring)."""
        return self._connection

    @property
    def session(self) -> Session:
        """Shorthand for `connection.session` -- the SAME `Session`
        object handed to every dispatch call, for the server's whole
        lifetime (never re-resolved per request)."""
        return self._session

    @property
    def pending_count(self) -> int:
        """How many `submit()`-ted requests are enqueued but not yet
        popped for dispatch (does NOT include one currently being
        dispatched). A reasonable operational metric for a real
        long-running daemon; also lets tests synchronize
        deterministically on queue state instead of guessing timing
        (see `tests/host/rogo/test_daemon.py`)."""
        with self._lock:
            return len(self._heap)

    # ---- lifecycle ---------------------------------------------------

    def start(self) -> None:
        """Start the single background worker thread that drains the
        priority queue -- see module docstring's "single wire-owner
        thread" section. Raises `DaemonServerError` if already
        started."""
        with self._lock:
            if self._worker is not None:
                raise DaemonServerError("DaemonServer.start() called twice")
            worker = threading.Thread(
                target=self._run_worker, name="rogo-daemon-worker", daemon=True,
            )
            self._worker = worker
            worker.start()

    def stop(self, *, timeout: float | None = 5.0) -> None:  # [s]
        """Ask the worker thread to exit once the queue is fully
        drained -- any request already `submit()`-ted (queued OR
        currently dispatching) still gets a reply; this does not
        abandon in-flight or already-queued work -- then join it. Safe
        to call even if `start()` was never called."""
        with self._lock:
            self._stop_requested = True
            self._not_empty.notify_all()
        if self._worker is not None:
            self._worker.join(timeout)

    def __enter__(self) -> "DaemonServer":
        self.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    # ---- request submission -------------------------------------------

    def submit(self, request: Request) -> Reply:
        """Enqueue `request` and block the CALLING thread until its
        `Reply` is ready, then return it. Safe to call from multiple
        threads concurrently -- ticket 006's transports are expected to
        call this once per decoded request, from whichever thread owns
        that client's connection.

        If this server's `is_estop(request)` classifier says True
        (plain `estop_verbs` membership by default -- module docstring's
        own `is_estop` section), it is placed ahead of every
        already-queued non-estop request (priority, not FIFO, ordering
        -- module docstring), and, if a non-estop dispatch call is
        CURRENTLY running, that call's own `abort` event is set
        immediately so a cooperative dispatch body notices and returns
        early instead of running to its natural completion (module
        docstring's "Estop-priority queue" section).

        An unrecognized verb is not an error here -- it becomes a
        failed `Reply` once dispatched, matching how any other dispatch
        failure is reported (`_execute()`'s own docstring). Raises
        `DaemonServerError` if `start()` was never called.
        """
        if self._worker is None:
            raise DaemonServerError("DaemonServer.submit() called before start()")

        priority = 0 if self._is_estop(request) else 1
        item = _QueueItem(
            priority=priority,
            sequence=next(self._sequence_counter),
            request=request,
            abort_event=threading.Event(),
            done_event=threading.Event(),
        )
        with self._lock:
            heapq.heappush(self._heap, item)
            if priority == 0 and self._current_abort_event is not None:
                self._current_abort_event.set()
            self._not_empty.notify()

        item.done_event.wait()
        assert item.reply is not None  # always set before done_event, in _run_worker
        return item.reply

    # ---- worker thread ------------------------------------------------

    def _run_worker(self) -> None:
        """The single wire-owner loop (module docstring): pop the
        highest-priority queued item, dispatch it (outside the lock, so
        `submit()` calls from other threads are never blocked while a
        dispatch is in progress), then hand back its `Reply`. Exits
        once `stop()` has been called AND the queue is fully drained."""
        while True:
            with self._lock:
                while not self._heap and not self._stop_requested:
                    self._not_empty.wait()
                if not self._heap:
                    return  # stop_requested and nothing left to drain
                item = heapq.heappop(self._heap)
                self._current_abort_event = item.abort_event

            item.reply = self._execute(item.request, item.abort_event)

            with self._lock:
                self._current_abort_event = None
            item.done_event.set()

    def _execute(self, request: Request, abort: threading.Event) -> Reply:
        """Call `request`'s dispatch body and turn its outcome into a
        `Reply` -- never lets an exception escape (see `DispatchFn`'s
        own docstring: a raised exception becomes a failed `Reply`, not
        a crashed worker thread)."""
        handler = self._dispatch_table.get(request.verb)
        if handler is None:
            return Reply.fail(
                request.id,
                f"no dispatch registered for verb {request.verb!r}",
                type="UnknownVerb",
            )
        try:
            result = handler(self._session, dict(request.params), abort)
        except Exception as exc:  # noqa: BLE001 -- any exception -> failed Reply, see DispatchFn
            return Reply.fail(request.id, str(exc), type=type(exc).__name__)
        return Reply.ok(request.id, result)


# ---------------------------------------------------------------------------
# Robot-name resolution -- the socket filename's own source of truth
# (sprint.md's Architecture Design Rationale, "well-known socket
# directory" decision: "Socket filename is the resolved robot name").
# ---------------------------------------------------------------------------

class RobotNameError(RuntimeError):
    """Raised by `resolve_robot_name()` when no name can be determined:
    no `override` was given, the target's `HELLO` produced no usable
    `device` banner within `timeout`, and `sim` is False (so the fixed
    `--sim` default does not apply either). Distinct from
    `DaemonServerError`: this is a pre-`DaemonServer`-construction
    failure -- name resolution happens BEFORE a socket path (and so a
    `UnixSocketListener`) can even be chosen -- not a server usage
    error."""


def resolve_robot_name(
    session: Session,
    *,
    override: str | None = None,
    sim: bool = False,
    default_sim_name: str = "sim",
    timeout: float = 2.0,  # [s]
) -> str:
    """Resolve the name a Unix-socket listener's socket file is named
    after (`<name>.sock`) -- this ticket's own AC: "Robot-name
    resolution follows the hello/identify-response-first,
    flag-override-second, fixed-default-for-`--sim`-third order."

    Precedence, in the order actually consulted:

      1. `override` -- an explicit `--name`-style flag value (parsing
         it from `argparse.Namespace` is the caller's job, e.g. a
         future `cmd_serve()`, ticket 009; this function only consumes
         the already-resolved string). Wins immediately when given.
      2. The robot's own `HELLO`/`device` banner (protocol.md#8.3) --
         sends `HELLO` on `session` (unsequenced, matching
         `rogo.cli._run_hello()`) and waits up to `timeout` seconds for
         a `device` reply, returning its `name` field (the third of
         `role common_name name serial`, `_run_hello()`'s own
         unpacking order) when present and not the banner's own
         "unknown field" placeholder (`"?"`).
      3. `default_sim_name` (`"sim"` by default) -- ONLY when `sim` is
         True (this ticket's own Description: "for a `--sim` target
         with no flag override, fall back to a fixed default name").
         `tools/sim`'s own `device` banner already reports `name=sim`
         in the ordinary case (see `test_end_to_end_sim.py`'s own
         `"name=sim" in out` assertion), so this tier is a robustness
         fallback for a `--sim` target whose `HELLO` round trip does
         not complete within `timeout` (a slow or still-starting
         `tools/sim` subprocess), not the common path.

    Raises `RobotNameError` if none of the three apply -- most notably,
    a non-`--sim` target whose `HELLO` never answers: there is no safe
    made-up name for a real robot's socket file, so this fails closed
    rather than guessing.

    ---- Design note: why `override` is checked BEFORE `HELLO`, despite
    the AC text's own "hello ... first, flag ... second" phrasing ----

    The AC lists the three tiers in the same order this ticket's own
    Description paragraph introduces them ("from the robot's own
    hello/identify response where possible, **overridable by** flag;
    for a --sim target with no flag override, fall back to..."), not as
    a literal trial-order instruction: "overridable BY flag" only reads
    coherently if the flag, when given, wins over the value it is
    overriding -- which can only be `HELLO`'s, since it is the only
    other source that could already have produced a name by the time
    the flag is consulted. A "HELLO always wins, don't even look at the
    flag" reading would make the override flag inert in the ordinary
    case (a reachable target), which is not what an override is for.
    """
    if override:
        return override
    name = _hello_identify_name(session, timeout=timeout)
    if name is not None:
        return name
    if sim:
        return default_sim_name
    raise RobotNameError(
        "could not resolve a robot name -- no override given, HELLO produced "
        f"no usable device banner within {timeout}s, and this is not a --sim "
        "target (no fixed default applies)"
    )


def _hello_identify_name(session: Session, *, timeout: float) -> str | None:
    """Send `HELLO` and wait up to `timeout` seconds for a `device`
    banner, returning its `name` field (`rogo.cli._run_hello()`'s own
    unpacking order: `role common_name name serial`) -- `None` if no
    banner arrived in time, or its `name` field was the banner's own
    "unknown"/missing-field placeholder (`"?"`, `_run_hello()`'s own
    padding value) or empty. A small local helper rather than
    `rogo.cli._pump_until()` reused directly: this module never imports
    `rogo.cli` (module docstring's own "Injection, not import"
    section) -- the two helpers happen to look alike because they solve
    the identical small problem ("poll pump() for a specific reply
    verb"), not because one depends on the other."""
    session.send_unsequenced("HELLO")
    deadline = time.monotonic() + timeout
    collected = []
    while time.monotonic() < deadline:
        collected.extend(session.pump(0.2))
        if any(r.verb == "device" for r in collected):
            break
    banner = next((r for r in collected if r.verb == "device"), None)
    if banner is None or len(banner.fields) < 3:
        return None
    name = banner.fields[2]
    if not name or name == "?":
        return None
    return name


# ---------------------------------------------------------------------------
# Socket directory / path resolution -- sprint.md's Architecture Design
# Rationale, "well-known socket directory" decision.
# ---------------------------------------------------------------------------

def default_socket_dir(*, home: Path | None = None) -> Path:
    """The well-known directory a Unix-socket listener's socket file
    lives under: `$XDG_RUNTIME_DIR/rogo/` when that env var is set (the
    Linux/CI convention), else `<home>/.rogo/run/` (works everywhere,
    including macOS -- this issue's own stated motivating platform,
    where `XDG_RUNTIME_DIR` is typically unset). `home` defaults to
    `Path.home()`; a caller (a test) may pass an explicit value instead
    of monkeypatching `Path.home()` globally."""
    xdg_runtime_dir = os.environ.get("XDG_RUNTIME_DIR")
    if xdg_runtime_dir:
        return Path(xdg_runtime_dir) / "rogo"
    resolved_home = home if home is not None else Path.home()
    return resolved_home / ".rogo" / "run"


def socket_path_for_name(name: str, *, socket_dir: Path | None = None) -> Path:
    """The socket file path for robot `name`: `<socket_dir>/<name>.sock`
    (`socket_dir` defaults to `default_socket_dir()`). Two
    differently-named robots always produce two distinct paths under
    the same directory (this ticket's own AC) since this is a pure
    function of `name` with no other state. Deliberately a pure
    computation -- no filesystem side effect (no directory creation, no
    bind); see `ensure_socket_dir()` for that -- so a future client
    (ticket 008) can compute the SAME path to connect to without
    needing this process's own directory-creation permissions."""
    if not name:
        raise ValueError("robot name must be a non-empty string")
    base = socket_dir if socket_dir is not None else default_socket_dir()
    return base / f"{name}.sock"


def ensure_socket_dir(socket_path: Path) -> None:
    """Create `socket_path`'s containing directory with owner-only
    (0700) permissions if it does not already exist -- this ticket's
    own AC. Leaves an already-existing directory's permissions
    untouched (never tightens something an operator, or a previous
    daemon run, already set up on purpose)."""
    directory = socket_path.parent
    if directory.exists():
        return
    directory.mkdir(parents=True, exist_ok=True)
    # mkdir()'s own `mode` argument is masked by the process umask on
    # most platforms -- an explicit chmod after creation is the only
    # portable way to guarantee 0700 regardless of umask.
    os.chmod(directory, 0o700)


# ---------------------------------------------------------------------------
# Unix-socket listener -- production transport (this ticket's own
# Description: "a named Unix domain socket for production"). Accepts
# multiple concurrent client connections, each served by its own thread;
# every connection's decoded requests go through the SAME
# `DaemonServer.submit()`, so the estop-priority queue applies identically
# no matter which client -- or how many are connected at once -- submits
# it (multiple concurrent clients sharing one robot, e.g. an MCP session
# and a CLI invocation at the same time, is this transport's whole point).
# ---------------------------------------------------------------------------

class UnixSocketListener:
    """Binds a Unix domain socket at `socket_path`, accepts connections
    on a background thread, and serves each one on ITS OWN thread: read
    one framed request line, `server.submit()` it (blocking until the
    reply is ready), write the framed reply line back, repeat until the
    client disconnects. Multiple clients may be connected -- and have
    requests in flight -- at once; each gets its own thread, so one
    client's blocking `submit()` call never blocks another client's read
    loop (only `DaemonServer`'s own single worker thread serializes
    actual dispatch, per its own estop-priority ordering -- see that
    class's module docstring).

    Does NOT start/stop `server` itself -- construct with an
    already-started `DaemonServer`; this class's own `start()`/`stop()`
    govern only the socket, accept loop, and per-client threads
    (mirroring `DaemonServer`'s own start()/stop() shape for a
    consistent lifecycle API across both listener transports in this
    module). Also usable as a context manager.
    """

    def __init__(
        self,
        server: DaemonServer,
        socket_path: Path,
        *,
        accept_poll_interval: float = 0.2,  # [s]
    ) -> None:
        self._server = server
        self._socket_path = Path(socket_path)
        self._accept_poll_interval = accept_poll_interval

        self._sock: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._client_threads: list[threading.Thread] = []
        self._stop_requested = threading.Event()

    @property
    def socket_path(self) -> Path:
        """The Unix-socket path this listener is bound (or about to be
        bound) to."""
        return self._socket_path

    def start(self) -> None:
        """Create `socket_path`'s containing directory (0700, if
        missing -- `ensure_socket_dir()`), remove a stale socket file
        left behind by a crashed previous daemon (AF_UNIX `bind()`
        fails with `OSError` if the path already exists at all), bind,
        and start accepting connections on a background thread. Raises
        `DaemonServerError` if already started."""
        if self._sock is not None:
            raise DaemonServerError("UnixSocketListener.start() called twice")

        ensure_socket_dir(self._socket_path)
        if self._socket_path.exists():
            self._socket_path.unlink()

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.bind(str(self._socket_path))
        except OSError:
            sock.close()
            raise
        sock.listen()
        sock.settimeout(self._accept_poll_interval)
        self._sock = sock

        thread = threading.Thread(
            target=self._accept_loop, name="rogo-daemon-unix-accept", daemon=True,
        )
        self._accept_thread = thread
        thread.start()

    def stop(self, *, timeout: float | None = 5.0) -> None:
        """Stop accepting new connections, close the listening socket
        (and remove its file), and join the accept thread and every
        still-running per-client thread. Safe to call even if `start()`
        was never called. Does NOT stop `server` -- see class
        docstring."""
        self._stop_requested.set()
        if self._accept_thread is not None:
            self._accept_thread.join(timeout)
            self._accept_thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
        for client_thread in self._client_threads:
            client_thread.join(timeout)
        self._client_threads.clear()
        try:
            self._socket_path.unlink()
        except FileNotFoundError:
            pass

    def __enter__(self) -> "UnixSocketListener":
        self.start()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.stop()

    def _accept_loop(self) -> None:
        assert self._sock is not None
        while not self._stop_requested.is_set():
            try:
                client_sock, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return  # listening socket closed underneath us by stop()
            client_thread = threading.Thread(
                target=self._serve_client, args=(client_sock,),
                name="rogo-daemon-unix-client", daemon=True,
            )
            self._client_threads.append(client_thread)
            client_thread.start()

    def _serve_client(self, client_sock: socket.socket) -> None:
        """One client connection's whole lifetime: read a request line,
        `submit()` it, write the reply, repeat -- sequential WITHIN this
        one connection (a client that wants a second request in flight
        before its first reply arrives opens a second connection; this
        keeps replies on any one connection unambiguously ordered with
        no per-connection demultiplexing needed -- concurrency across
        DIFFERENT clients is what this class's own multi-thread accept
        loop already provides). A line that fails to decode
        (`ProtocolError`) is dropped, not replied to -- there is no
        correlation id to answer against for a line that could not even
        be parsed; the connection stays open for the next (well-formed)
        line."""
        with client_sock:
            reader = client_sock.makefile("r", encoding="utf-8", newline="\n")
            try:
                for raw_line in reader:
                    line = raw_line.rstrip("\n")
                    if not line:
                        continue
                    try:
                        request = decode_request(line)
                    except ProtocolError:
                        continue
                    reply = self._server.submit(request)
                    client_sock.sendall((encode_reply(reply) + "\n").encode("utf-8"))
            except OSError:
                pass  # client disconnected mid-read/write
            finally:
                reader.close()


# ---------------------------------------------------------------------------
# Stdio-pipe listener -- tests/embedding transport (this ticket's own
# Description: "a daemon forked as a child, speaking the same framed
# protocol over stdin/stdout"). Deliberately single-threaded/sequential:
# one process's stdio is one duplex pipe pair with exactly one peer, unlike
# the Unix-socket listener's many-concurrent-clients case above -- this
# ticket's own AC only requires ONE request/reply exchange over this
# transport (SUC-003's own Main Flow step 2), and sequential handling keeps
# this the simplest possible transport with no risk of interleaved writes.
# ---------------------------------------------------------------------------

def run_stdio_pipe(
    server: DaemonServer,
    *,
    stdin: TextIO | None = None,
    stdout: TextIO | None = None,
) -> None:
    """Blocking loop: read one framed request line at a time from
    `stdin` (default `sys.stdin`), `server.submit()` it, and write one
    framed reply line to `stdout` (default `sys.stdout`) -- flushing
    after every single reply, so a reader on the far end of the pipe
    sees it immediately rather than sitting in a block buffer (ticket
    003-001's own line-buffering fix; a pipe reader has no tty to make
    `sys.stdout` line-buffer itself automatically, exactly the failure
    mode that fix addresses for `rogo repl`). Returns cleanly on EOF
    (`stdin` closed) -- this function owns only the transport loop, not
    process lifecycle (a future `cmd_serve()`, ticket 009, decides what
    happens after this returns).

    Does NOT start/stop `server` -- construct and `start()` it before
    calling this (matching `UnixSocketListener`'s own "does not own the
    server's lifecycle" contract).

    A line that fails to decode (`ProtocolError`) is dropped -- see
    `UnixSocketListener._serve_client()`'s own docstring for why (no
    correlation id to reply against)."""
    in_stream = stdin if stdin is not None else sys.stdin
    out_stream = stdout if stdout is not None else sys.stdout
    _force_line_buffered(out_stream)

    for raw_line in in_stream:
        line = raw_line.rstrip("\n")
        if not line:
            continue
        try:
            request = decode_request(line)
        except ProtocolError:
            continue
        reply = server.submit(request)
        out_stream.write(encode_reply(reply) + "\n")
        out_stream.flush()


# ---------------------------------------------------------------------------
# --sim/--connect/--port boot wiring for the pipe transport -- ticket 007's
# own deliverable (issue Requirement 3: "rogo serve --sim should reach a
# working daemon the same way rogo drive --sim does today"). Deliberately
# narrow: this ties `rogo.connection.resolve()` to `run_stdio_pipe()` ONLY --
# the Unix-socket branch, the `rogo serve` subcommand itself, and
# `cli.py`'s own per-verb dispatch table remain ticket 009's job (module
# docstring's "Still ... ticket 009" note).
# ---------------------------------------------------------------------------

def run_stdio_pipe_from_args(
    args: argparse.Namespace,
    dispatch_table: DispatchTable,
    *,
    estop_verbs: frozenset[str] = DEFAULT_ESTOP_VERBS,
    stdin: TextIO | None = None,
    stdout: TextIO | None = None,
) -> None:
    """Resolve `args` (as produced by a parser that called
    `rogo.connection.add_target_arguments()`, e.g. `--sim`) into a live
    target `Connection` via `rogo.connection.resolve()` -- the EXACT
    same resolution every one-shot `rogo` command already uses, so a
    `--sim` target here freshly builds/reuses `tools/sim`
    (`connection.ensure_sim_binary()`) and spawns it identically to
    `rogo drive --sim` today (this ticket's own AC #1) -- then build and
    `start()` a `DaemonServer` around it with `dispatch_table`, and run
    it over the pipe transport (`run_stdio_pipe()`) until EOF. Always
    `stop()`s the server and `close()`s the connection's transport
    afterward, success or failure, so a `--sim` subprocess this function
    spawned is never left running once the pipe closes.

    This is the ONE place in this module that ties connection
    resolution to a running server -- ticket 007's own test harness
    (`tests/host/rogo/daemon_test_helpers.py`) forks a subprocess that
    calls this function to prove the whole boot sequence end to end
    against a real `tools/sim` (SUC-003). A future `cmd_serve()`
    (ticket 009) is expected to call this SAME function for its own
    `--stdio-pipe` branch, with `cli.py`'s own per-verb dispatch table,
    rather than reimplementing this sequence -- its Unix-socket branch
    needs its own, separate wiring (name resolution, socket path,
    signal handling), which is out of this function's scope.

    `stdin`/`stdout` pass straight through to `run_stdio_pipe()` (its
    own default-to-`sys.stdin`/`sys.stdout` behavior applies when
    omitted) -- present here purely so this function itself is testable
    in-process, with a fake pair of streams, the same way
    `run_stdio_pipe()` already is (`test_daemon_transports.py`'s own
    `_FakeStdioStream`), with no real subprocess/sim required for that
    coverage.
    """
    conn = resolve_connection(args)
    server = DaemonServer(conn, dispatch_table, estop_verbs=estop_verbs)
    server.start()
    try:
        run_stdio_pipe(server, stdin=stdin, stdout=stdout)
    finally:
        server.stop()
        conn.transport.close()


def _force_line_buffered(stream: TextIO) -> None:
    """Force `stream` into line-buffered mode, guarded exactly like
    `rogo.repl`'s own `_force_line_buffered_stdout()` (that function's
    own docstring explicitly anticipates this: daemon pipe mode "should
    call this SAME helper, or apply
    `sys.stdout.reconfigure(line_buffering=True)` itself"). Duplicated
    locally, in miniature, rather than imported: `_force_line_buffered_
    stdout()` is a private (underscore-prefixed) helper of `rogo.repl`,
    and this module has no other reason to depend on `rogo.repl` at all
    -- a few duplicated lines is cheaper than a cross-module dependency
    on another module's private implementation detail. `run_stdio_pipe()`
    also flushes explicitly after every write (belt-and-suspenders):
    reconfiguring here covers any OTHER write to `stream` (there are
    none today, but nothing stops a future caller from writing to it
    directly), and `flush()` alone still guarantees prompt delivery even
    for a stream with no `reconfigure()` method at all (e.g. some
    embeddings' replacement stdout)."""
    reconfigure = getattr(stream, "reconfigure", None)
    if reconfigure is None:
        return
    try:
        reconfigure(line_buffering=True)
    except (ValueError, OSError):
        pass

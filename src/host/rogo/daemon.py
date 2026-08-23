"""daemon.py -- `rogo serve`'s server core: holds ONE robot connection
open for the whole daemon process's lifetime and routes every client
request to it, escalating any estop/halt request ahead of whatever else
is queued (sprint.md's Architecture Step 3, `rogo.daemon`'s own row;
ticket 005's own Description). This module implements the CORE only --
no listener transports (Unix socket / stdio pipe, ticket 006), no
`--sim`/CLI boot wiring (tickets 007/009).

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
`rogo.connection.Connection` -- this module does not itself call
`rogo.connection.resolve()`; that is the caller's job (a future
`cmd_serve()`/`--sim` boot function, tickets 007/009) -- and holds it
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
"""

from __future__ import annotations

import dataclasses
import heapq
import itertools
import threading
from typing import Callable, Mapping

from robot_v6.reliability import Session

from .connection import Connection
from .daemon_protocol import Reply, Request

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
"""The verb names that jump the priority queue by default --
`daemon_protocol`'s own module docstring: "at minimum `estop`/`halt`".
Overridable per-server via `DaemonServer(..., estop_verbs=...)`, since
`daemon_protocol` itself deliberately has no verb table of its own (its
module docstring's own "This module has no verb table of its own"
sentence)."""


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
    ) -> None:
        self._connection = connection
        self._session = connection.session
        self._dispatch_table: dict[str, DispatchFn] = dict(dispatch_table)
        self._estop_verbs = frozenset(estop_verbs)

        self._heap: list[_QueueItem] = []
        self._sequence_counter = itertools.count()
        self._lock = threading.Lock()
        self._not_empty = threading.Condition(self._lock)
        self._current_abort_event: threading.Event | None = None
        self._stop_requested = False

        self._worker: threading.Thread | None = None

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

        If `request.verb` is a member of this server's `estop_verbs`,
        it is placed ahead of every already-queued non-estop request
        (priority, not FIFO, ordering -- module docstring), and, if a
        non-estop dispatch call is CURRENTLY running, that call's own
        `abort` event is set immediately so a cooperative dispatch body
        notices and returns early instead of running to its natural
        completion (module docstring's "Estop-priority queue" section).

        An unrecognized verb is not an error here -- it becomes a
        failed `Reply` once dispatched, matching how any other dispatch
        failure is reported (`_execute()`'s own docstring). Raises
        `DaemonServerError` if `start()` was never called.
        """
        if self._worker is None:
            raise DaemonServerError("DaemonServer.submit() called before start()")

        priority = 0 if request.verb in self._estop_verbs else 1
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

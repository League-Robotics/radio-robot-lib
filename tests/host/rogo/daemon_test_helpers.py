"""tests/host/rogo/daemon_test_helpers.py -- ticket 007's own reusable
fork-based test harness: forks a real Python subprocess that boots a
`rogo serve --stdio-pipe`-equivalent daemon (via
`rogo.daemon.run_stdio_pipe_from_args()`, this ticket's own boot
function -- see that function's own docstring) against a `--sim` target
by default, and exchanges framed `daemon_protocol` request/reply lines
with it over a real OS pipe (`robot_v6.transport.StdioTransport`) --
exercising the exact wire protocol production uses, over a genuinely
forked child process, not an in-process call (issue Requirement 2's own
testing goal: "A test forks the daemon, writes requests, reads JSON
replies").

This generalizes ticket 006's own one-off `_STDIO_PIPE_SUBPROCESS_SCRIPT`
(`test_daemon_transports.py`) into something importable by tickets
008/009/010/011's own test files, so each of those does not have to
reimplement subprocess spawning, framed-protocol read/write, or
teardown for itself -- this ticket's own AC #2: "usable by tickets
008/009/010/011's own tests without each reimplementing process
management." Only the dispatch table a given caller needs differs per
test (`dispatch_source`, below); the process management around it never
does.

Usage (see `test_daemon_sim_e2e.py`, this ticket's own end-to-end test,
for a full example)::

    from daemon_test_helpers import fork_stdio_daemon

    with fork_stdio_daemon() as forked:               # --sim by default
        reply = forked.request("hello")
        assert reply.result == {"name": "sim"}

A pytest fixture, `forked_sim_daemon` below, wraps this with the
default `ping`/`hello` dispatch table (and this directory's own
session-scoped `built_sim_binary` fixture, so `tools/sim` is already
built BEFORE forking) for tests that just need "a running daemon
talking to a real sim," no custom verbs of their own.

No `__init__.py` lives under `tests/`, so pytest's own "prepend" import
mode (this project's default) puts each test module's own directory on
`sys.path` when it is first collected -- this module is importable from
any sibling test file in this SAME directory as a bare `import
daemon_test_helpers` / `from daemon_test_helpers import ...`, exactly
like a local `conftest.py` fixture would be, with no package prefix
needed.
"""

from __future__ import annotations

import itertools
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator

import pytest

from rogo import daemon_protocol as dp
from robot_v6.transport import StdioTransport

_SRC_HOST = str(Path(__file__).resolve().parents[3] / "src" / "host")

# The default dispatch table `fork_stdio_daemon()` uses unless a caller
# supplies its own `dispatch_source` -- two generic verbs: "ping" proves
# the forked daemon is alive with no session interaction at all, and
# "hello" proves dispatch genuinely reaches the sim-backed connection
# (this ticket's own AC #3) by reusing `daemon.resolve_robot_name()`
# itself -- the SAME function ticket 006 built -- rather than
# re-deriving the HELLO/device-banner exchange here a second time.
# `daemon` is already imported in the subprocess's own namespace (see
# `_WORKER_SCRIPT_TEMPLATE` below), so this source text may reference it
# directly.
DEFAULT_DISPATCH_SOURCE = """
def _hello(session, params, abort):
    del params, abort
    return {"name": daemon.resolve_robot_name(session, sim=True, timeout=2.0)}


DISPATCH_TABLE = {
    "ping": lambda session, params, abort: "pong",
    "hello": _hello,
}
"""

_WORKER_SCRIPT_TEMPLATE = """
import argparse
import sys
sys.path.insert(0, {src_host!r})

from rogo import daemon

{dispatch_source}

_args = argparse.Namespace(sim={sim!r}, connect={connect!r}, port={port!r})
daemon.run_stdio_pipe_from_args(_args, DISPATCH_TABLE)
"""


class ForkedDaemonError(RuntimeError):
    """Raised by `ForkedDaemon.request()` when no reply carrying the
    sent request's id arrived within `timeout` -- distinct from a
    `Reply` whose own `error` field is set (a normal, well-formed
    failure reply FROM the daemon; this exception means the wire never
    answered at all, e.g. a hung dispatch body or a crashed
    subprocess)."""


@dataclass
class ForkedDaemon:
    """A running daemon subprocess (forked by `fork_stdio_daemon()`)
    and its client-side pipe pair. `request()` sends one framed request
    and blocks for ITS OWN correlated reply (matched by id, not arrival
    order -- `daemon_protocol`'s only pairing guarantee); `close()`
    tears the subprocess down cleanly (`StdioTransport.close()`: EOF on
    stdin, then `terminate()`/`kill()` if it does not exit on its own --
    this ticket's own AC #2: "tears the process down cleanly on test
    exit"). Also usable as a context manager.
    """

    transport: StdioTransport
    _id_counter: itertools.count[int] = field(default_factory=lambda: itertools.count(1))
    _pending: list[dp.Reply] = field(default_factory=list)

    def request(
        self, verb: str, params: dict | None = None, *, timeout: float = 10.0
    ) -> dp.Reply:
        """Send one request for `verb` (with `params`, default `{}`)
        and block until ITS OWN reply arrives or `timeout` elapses.
        Raises `ForkedDaemonError` on timeout. A reply whose own
        `error` field is set is still returned normally -- it IS the
        daemon's answer, just a failing one; inspect `.error` rather
        than treating it as this method's own failure."""
        request_id = next(self._id_counter)
        self.transport.send_line(
            dp.encode_request(dp.Request(id=request_id, verb=verb, params=params or {}))
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in self.transport.read_lines(timeout=0.2):
                self._pending.append(dp.decode_reply(line))
            for index, reply in enumerate(self._pending):
                if reply.id == request_id:
                    del self._pending[index]
                    return reply
        raise ForkedDaemonError(
            f"no reply for request id={request_id} verb={verb!r} within {timeout}s"
        )

    def close(self) -> None:
        """Tear the subprocess down cleanly -- safe to call more than
        once (`StdioTransport.close()`'s own idempotence)."""
        self.transport.close()

    def __enter__(self) -> "ForkedDaemon":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


def fork_stdio_daemon(
    *,
    dispatch_source: str = DEFAULT_DISPATCH_SOURCE,
    sim: bool = True,
    connect: str | None = None,
    port: str | None = None,
) -> ForkedDaemon:
    """Fork a Python subprocess that resolves a target connection
    (`--sim` by default -- a freshly built/reused `tools/sim`, exactly
    like every other `--sim` end-to-end test in this directory) and
    runs `rogo.daemon.run_stdio_pipe_from_args()` against it, with a
    dispatch table built from `dispatch_source` (Python source text
    defining a module-level `DISPATCH_TABLE` dict -- see
    `DEFAULT_DISPATCH_SOURCE` above for the shape).

    Returns an already-forked, ready-to-use `ForkedDaemon` -- the
    caller owns `close()`ing it (or uses it as a context manager) once
    done; `forked_sim_daemon` below does this automatically for the
    common case.
    """
    script = _WORKER_SCRIPT_TEMPLATE.format(
        src_host=_SRC_HOST,
        dispatch_source=dispatch_source,
        sim=sim,
        connect=connect,
        port=port,
    )
    transport = StdioTransport([sys.executable, "-c", script])
    return ForkedDaemon(transport=transport)


@pytest.fixture
def forked_sim_daemon(built_sim_binary) -> Iterator[ForkedDaemon]:
    """A ready-to-use `ForkedDaemon` against a `--sim` target, with the
    default `ping`/`hello` dispatch table, torn down automatically at
    test end. Depends on `built_sim_binary` (this directory's own
    conftest.py, session-scoped) so `tools/sim` is already built BEFORE
    forking, rather than each fixture use racing its own fresh compile
    inside the subprocess."""
    del built_sim_binary
    forked = fork_stdio_daemon()
    try:
        yield forked
    finally:
        forked.close()

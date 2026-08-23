"""tests/host/rogo/test_daemon_transports.py -- ticket 006's own two
listener transports (`daemon.UnixSocketListener`, `daemon.run_stdio_pipe`)
and the robot-name/socket-path resolution that feeds the Unix-socket
transport's own filename (`daemon.resolve_robot_name`,
`daemon.default_socket_dir`, `daemon.socket_path_for_name`,
`daemon.ensure_socket_dir`). `test_daemon.py` already covers
`DaemonServer`'s own core behavior (connection ownership, dispatch
injection, estop-priority queue) with no transport involved at all --
this file's own job is proving the two THIN listeners correctly wrap
that already-proven core, not re-proving the core itself.

Unix-domain-socket paths have a short OS-enforced length limit (the
`sun_path` field is ~104 bytes on macOS -- this project's own dev/CI
platform per `darwin` in this repo's environment) that pytest's own
`tmp_path` fixture can exceed once nested under
`/private/var/folders/.../pytest-of-<user>/pytest-<n>/<test-name>0/`.
Every test that actually BINDS a socket below uses `_short_tmp_dir()`
(a `tempfile.mkdtemp(dir="/tmp")`, cleaned up in a `finally`) instead of
`tmp_path`, to stay well under that limit; tests that only exercise pure
path computation (no bind) use ordinary `tmp_path` freely, since no OS
length limit applies to a `pathlib.Path` that is never handed to
`socket.bind()`.
"""

from __future__ import annotations

import argparse
import shutil
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest

from rogo import daemon
from rogo import daemon_protocol as dp
from rogo.connection import Connection
from robot_v6.reliability import Session
from robot_v6.transport import StdioTransport, Transport


# ---------------------------------------------------------------------------
# Shared fakes/helpers.
# ---------------------------------------------------------------------------

class _FakeTransport(Transport):
    """A Transport whose wire is entirely scripted -- used for
    `resolve_robot_name()` tests, which need a `Session` but never a
    real robot/sim. `queued_lines` is drained one `read_lines()` call at
    a time (matching `Transport.read_lines()`'s own "zero, one, or
    several lines per call" contract); once empty, every call returns
    no data (as a real idle connection with nothing new to say would)."""

    def __init__(self, queued_lines: list[str] | None = None) -> None:
        super().__init__()
        self._queued_lines = list(queued_lines or [])
        self.sent: list[bytes] = []

    def _read_chunk(self, timeout: float | None) -> bytes:
        if not self._queued_lines:
            return b""
        line = self._queued_lines.pop(0)
        return (line + "\n").encode("ascii")

    def _write_bytes(self, data: bytes) -> None:
        self.sent.append(data)

    def close(self) -> None:
        pass


def _make_connection(queued_lines: list[str] | None = None) -> Connection:
    transport = _FakeTransport(queued_lines)
    return Connection(transport=transport, session=Session(transport))


def _short_tmp_dir():
    """A short-path temp directory under `/tmp` (see module docstring)
    -- returns a `pathlib.Path`; caller is responsible for
    `shutil.rmtree(..., ignore_errors=True)` in a `finally`."""
    return Path(tempfile.mkdtemp(prefix="rogo-t-", dir="/tmp"))


def _wait_until(predicate, *, timeout: float = 2.0, interval: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def _connect(socket_path: Path) -> socket.socket:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(str(socket_path))
    return sock


def _send_request(sock: socket.socket, request: dp.Request) -> None:
    sock.sendall((dp.encode_request(request) + "\n").encode("utf-8"))


def _read_reply(sock: socket.socket, *, timeout: float = 2.0) -> dp.Reply:
    sock.settimeout(timeout)
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise AssertionError("connection closed before a full reply line arrived")
        buf += chunk
    line, _, _rest = buf.partition(b"\n")
    return dp.decode_reply(line.decode("utf-8"))


# ---------------------------------------------------------------------------
# Robot-name resolution.
# ---------------------------------------------------------------------------

def test_override_wins_immediately_with_no_hello_round_trip_needed():
    # An empty queued-lines transport would hang forever waiting for a
    # HELLO reply if resolve_robot_name() consulted it -- proves the
    # override short-circuits before ever sending HELLO.
    connection = _make_connection(queued_lines=[])
    name = daemon.resolve_robot_name(connection.session, override="tovez", timeout=0.05)
    assert name == "tovez"
    assert connection.transport.sent == []  # HELLO was never sent


def test_hello_device_banner_name_is_used_when_no_override_given():
    connection = _make_connection(queued_lines=["device robot mybot tovez 12345"])
    name = daemon.resolve_robot_name(connection.session, timeout=1.0)
    assert name == "tovez"


def test_sim_default_used_when_hello_never_answers_and_no_override():
    connection = _make_connection(queued_lines=[])  # HELLO never answered
    name = daemon.resolve_robot_name(connection.session, sim=True, timeout=0.1)
    assert name == "sim"


def test_sim_default_is_configurable():
    connection = _make_connection(queued_lines=[])
    name = daemon.resolve_robot_name(
        connection.session, sim=True, default_sim_name="simulator", timeout=0.1)
    assert name == "simulator"


def test_non_sim_target_with_no_hello_answer_and_no_override_raises():
    connection = _make_connection(queued_lines=[])
    with pytest.raises(daemon.RobotNameError):
        daemon.resolve_robot_name(connection.session, sim=False, timeout=0.1)


def test_unknown_placeholder_name_field_is_treated_as_unavailable():
    # A device banner whose "name" field is the wire's own "unknown
    # field" placeholder ("?") must not be handed back as a real name.
    connection = _make_connection(queued_lines=["device robot mybot ? 12345"])
    with pytest.raises(daemon.RobotNameError):
        daemon.resolve_robot_name(connection.session, sim=False, timeout=0.2)


def test_override_takes_precedence_even_when_hello_would_also_answer():
    connection = _make_connection(queued_lines=["device robot mybot tovez 12345"])
    name = daemon.resolve_robot_name(connection.session, override="explicit-name", timeout=1.0)
    assert name == "explicit-name"


# ---------------------------------------------------------------------------
# Socket directory / path resolution -- pure functions, no real bind.
# ---------------------------------------------------------------------------

def test_default_socket_dir_uses_xdg_runtime_dir_when_set(monkeypatch, tmp_path):
    xdg = tmp_path / "xdg-runtime"
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(xdg))
    assert daemon.default_socket_dir() == xdg / "rogo"


def test_default_socket_dir_falls_back_to_home_dot_rogo_run_when_xdg_unset(
    monkeypatch, tmp_path
):
    monkeypatch.delenv("XDG_RUNTIME_DIR", raising=False)
    assert daemon.default_socket_dir(home=tmp_path) == tmp_path / ".rogo" / "run"


def test_socket_path_for_name_is_socket_dir_slash_name_dot_sock(tmp_path):
    path = daemon.socket_path_for_name("tovez", socket_dir=tmp_path)
    assert path == tmp_path / "tovez.sock"


def test_two_differently_named_robots_produce_two_distinct_socket_paths(tmp_path):
    a = daemon.socket_path_for_name("tovez", socket_dir=tmp_path)
    b = daemon.socket_path_for_name("gopiv", socket_dir=tmp_path)
    assert a != b
    assert a.name == "tovez.sock"
    assert b.name == "gopiv.sock"


def test_socket_path_for_name_rejects_empty_name(tmp_path):
    with pytest.raises(ValueError):
        daemon.socket_path_for_name("", socket_dir=tmp_path)


def test_ensure_socket_dir_creates_missing_directory_with_owner_only_permissions(tmp_path):
    socket_path = tmp_path / "nested" / "rogo" / "tovez.sock"
    daemon.ensure_socket_dir(socket_path)
    directory = socket_path.parent
    assert directory.is_dir()
    assert (directory.stat().st_mode & 0o777) == 0o700


def test_ensure_socket_dir_leaves_an_already_existing_directory_untouched(tmp_path):
    directory = tmp_path / "rogo"
    directory.mkdir(mode=0o755)
    socket_path = directory / "tovez.sock"
    daemon.ensure_socket_dir(socket_path)
    # Permissions of an already-existing directory are NOT tightened.
    assert (directory.stat().st_mode & 0o777) == 0o755


# ---------------------------------------------------------------------------
# UnixSocketListener -- production transport, real bind()/accept()/connect().
# ---------------------------------------------------------------------------

def test_unix_socket_listener_completes_one_request_reply_exchange():
    tmp_dir = _short_tmp_dir()
    try:
        connection = _make_connection()
        server = daemon.DaemonServer(connection, {"ping": lambda s, p, a: "pong"})
        server.start()
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        try:
            assert socket_path.exists()
            client = _connect(socket_path)
            try:
                _send_request(client, dp.Request(id=1, verb="ping"))
                reply = _read_reply(client)
                assert reply.id == 1
                assert reply.result == "pong"
                assert reply.error is None
            finally:
                client.close()
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_unix_socket_listener_creates_containing_directory_with_0700():
    tmp_dir = _short_tmp_dir()
    try:
        connection = _make_connection()
        server = daemon.DaemonServer(connection, {})
        server.start()
        socket_dir = tmp_dir / "rundir"
        socket_path = socket_dir / "tovez.sock"
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        try:
            assert (socket_dir.stat().st_mode & 0o777) == 0o700
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_unix_socket_listener_stop_removes_the_socket_file():
    tmp_dir = _short_tmp_dir()
    try:
        connection = _make_connection()
        server = daemon.DaemonServer(connection, {})
        server.start()
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        assert socket_path.exists()
        listener.stop()
        server.stop()
        assert not socket_path.exists()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_two_daemons_for_two_differently_named_robots_run_concurrently_without_colliding():
    tmp_dir = _short_tmp_dir()
    try:
        conn_a = _make_connection()
        conn_b = _make_connection()
        server_a = daemon.DaemonServer(conn_a, {"whoami": lambda s, p, a: "a"})
        server_b = daemon.DaemonServer(conn_b, {"whoami": lambda s, p, a: "b"})
        server_a.start()
        server_b.start()

        path_a = daemon.socket_path_for_name("robot-a", socket_dir=tmp_dir)
        path_b = daemon.socket_path_for_name("robot-b", socket_dir=tmp_dir)
        assert path_a != path_b

        listener_a = daemon.UnixSocketListener(server_a, path_a)
        listener_b = daemon.UnixSocketListener(server_b, path_b)
        listener_a.start()
        listener_b.start()
        try:
            assert path_a.exists() and path_b.exists()

            client_a = _connect(path_a)
            client_b = _connect(path_b)
            try:
                _send_request(client_a, dp.Request(id=1, verb="whoami"))
                _send_request(client_b, dp.Request(id=1, verb="whoami"))
                reply_a = _read_reply(client_a)
                reply_b = _read_reply(client_b)
                # Each daemon answers from its OWN dispatch table -- no
                # cross-talk between the two distinct socket paths.
                assert reply_a.result == "a"
                assert reply_b.result == "b"
            finally:
                client_a.close()
                client_b.close()
        finally:
            listener_a.stop()
            listener_b.stop()
            server_a.stop()
            server_b.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_unix_socket_listener_supports_multiple_concurrent_clients_and_estop_reaches_the_priority_queue():
    # The safety-critical multi-client scenario over the REAL transport
    # (test_daemon.py already proves this against DaemonServer.submit()
    # directly, with no transport at all): two separate client
    # CONNECTIONS to the SAME Unix socket, each on its own thread inside
    # UnixSocketListener -- a long-running "drive" from client A, then
    # an "estop" from client B, must preempt it.
    tmp_dir = _short_tmp_dir()
    try:
        drive_started = threading.Event()

        def fake_drive(session, params, abort):
            drive_started.set()
            aborted = abort.wait(timeout=2.0)
            return {"aborted": aborted}

        def fake_estop(session, params, abort):
            return {"stopped": True}

        connection = _make_connection()
        server = daemon.DaemonServer(
            connection, {"drive": fake_drive, "estop": fake_estop})
        server.start()
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        try:
            client_a = _connect(socket_path)
            client_b = _connect(socket_path)
            try:
                _send_request(client_a, dp.Request(id=1, verb="drive"))
                assert drive_started.wait(timeout=2.0), "drive dispatch never started"

                start = time.monotonic()
                _send_request(client_b, dp.Request(id=2, verb="estop"))
                estop_reply = _read_reply(client_b)
                elapsed = time.monotonic() - start

                drive_reply = _read_reply(client_a)
            finally:
                client_a.close()
                client_b.close()

            assert estop_reply.error is None
            assert estop_reply.result == {"stopped": True}
            assert drive_reply.error is None
            assert drive_reply.result == {"aborted": True}
            # The estop's own reply must not have waited behind drive's
            # (generous) 2s abort timeout -- proves it jumped the queue
            # over the real socket transport, not just in-process.
            assert elapsed < 1.0
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_unix_socket_listener_start_twice_raises():
    tmp_dir = _short_tmp_dir()
    try:
        connection = _make_connection()
        server = daemon.DaemonServer(connection, {})
        server.start()
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        try:
            with pytest.raises(daemon.DaemonServerError):
                listener.start()
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_unix_socket_listener_as_context_manager():
    tmp_dir = _short_tmp_dir()
    try:
        connection = _make_connection()
        server = daemon.DaemonServer(connection, {"ping": lambda s, p, a: "pong"})
        server.start()
        try:
            socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
            with daemon.UnixSocketListener(server, socket_path) as listener:
                assert listener.socket_path == socket_path
                client = _connect(socket_path)
                try:
                    _send_request(client, dp.Request(id=1, verb="ping"))
                    reply = _read_reply(client)
                    assert reply.result == "pong"
                finally:
                    client.close()
            assert not socket_path.exists()  # __exit__ -> stop() removed it
        finally:
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_unix_socket_listener_removes_a_stale_socket_file_left_by_a_crashed_daemon():
    tmp_dir = _short_tmp_dir()
    try:
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        daemon.ensure_socket_dir(socket_path)
        # Simulate a leftover socket file from a crashed previous daemon
        # run (a real AF_UNIX bind() would fail with "address already in
        # use" against this same path otherwise).
        stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        stale.bind(str(socket_path))
        stale.close()
        assert socket_path.exists()

        connection = _make_connection()
        server = daemon.DaemonServer(connection, {"ping": lambda s, p, a: "pong"})
        server.start()
        listener = daemon.UnixSocketListener(server, socket_path)
        try:
            listener.start()  # must not raise "address already in use"
            client = _connect(socket_path)
            try:
                _send_request(client, dp.Request(id=1, verb="ping"))
                reply = _read_reply(client)
                assert reply.result == "pong"
            finally:
                client.close()
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# Stdio-pipe listener -- in-process direct call, then a real subprocess
# round trip (this ticket's own Testing plan: "an end-to-end test that
# starts a daemon in stdio-pipe mode as a subprocess, writes a framed
# request to its stdin, and reads the framed reply from its stdout").
# ---------------------------------------------------------------------------

class _FakeStdioStream:
    """A minimal in-memory stand-in for `sys.stdin`/`sys.stdout`, for
    driving `run_stdio_pipe()` directly (no subprocess) -- an iterable
    of lines for "stdin", and a `write()`/`flush()`-recording buffer for
    "stdout". Has no `reconfigure()` method on purpose, exercising
    `_force_line_buffered()`'s own "guarded, not required" no-op path
    the same way `repl.py`'s docstring notes pytest's `capsys` does."""

    def __init__(self, input_lines: list[str] | None = None) -> None:
        self._input_lines = iter(input_lines or [])
        self.written: list[str] = []
        self.flush_count = 0

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._input_lines) + "\n"

    def write(self, text: str) -> None:
        self.written.append(text)

    def flush(self) -> None:
        self.flush_count += 1


def test_run_stdio_pipe_completes_one_request_reply_exchange_in_process():
    connection = _make_connection()
    server = daemon.DaemonServer(connection, {"ping": lambda s, p, a: "pong"})
    server.start()
    try:
        in_stream = _FakeStdioStream([dp.encode_request(dp.Request(id=1, verb="ping"))])
        out_stream = _FakeStdioStream()
        daemon.run_stdio_pipe(server, stdin=in_stream, stdout=out_stream)
    finally:
        server.stop()

    assert len(out_stream.written) == 1
    reply = dp.decode_reply(out_stream.written[0].rstrip("\n"))
    assert reply.id == 1
    assert reply.result == "pong"
    # Flushed after every reply -- ticket 003-001's own always-flushed
    # guarantee, not just at process/loop exit.
    assert out_stream.flush_count == 1


def test_run_stdio_pipe_drops_a_malformed_line_and_keeps_serving():
    connection = _make_connection()
    server = daemon.DaemonServer(connection, {"ping": lambda s, p, a: "pong"})
    server.start()
    try:
        in_stream = _FakeStdioStream([
            "not even valid json",
            dp.encode_request(dp.Request(id=7, verb="ping")),
        ])
        out_stream = _FakeStdioStream()
        daemon.run_stdio_pipe(server, stdin=in_stream, stdout=out_stream)
    finally:
        server.stop()

    # Only the well-formed second line produced a reply.
    assert len(out_stream.written) == 1
    reply = dp.decode_reply(out_stream.written[0].rstrip("\n"))
    assert reply.id == 7
    assert reply.result == "pong"


def test_run_stdio_pipe_returns_cleanly_on_eof_with_no_lines():
    connection = _make_connection()
    server = daemon.DaemonServer(connection, {})
    server.start()
    try:
        daemon.run_stdio_pipe(server, stdin=_FakeStdioStream([]), stdout=_FakeStdioStream())
    finally:
        server.stop()
    # No assertion beyond "returned instead of hanging" -- an empty
    # iterable's StopIteration is EOF, exactly like a closed real pipe.


_STDIO_PIPE_SUBPROCESS_SCRIPT = """
import sys
sys.path.insert(0, {src_host!r})

from rogo import daemon
from rogo.connection import Connection
from robot_v6.reliability import Session
from robot_v6.transport import Transport


class _FakeTransport(Transport):
    def _read_chunk(self, timeout):
        return b""

    def _write_bytes(self, data):
        pass

    def close(self):
        pass


def ping(session, params, abort):
    return {{"pong": True}}


transport = _FakeTransport()
connection = Connection(transport=transport, session=Session(transport))
server = daemon.DaemonServer(connection, {{"ping": ping}})
server.start()
try:
    daemon.run_stdio_pipe(server)
finally:
    server.stop()
"""


def _read_lines_until_count(transport: StdioTransport, count: int, *, timeout: float = 5.0):
    """Poll `transport.read_lines()` (which may return zero, one, or
    several lines per call -- `Transport.read_lines()`'s own contract)
    until at least `count` lines have been collected or `timeout`
    elapses."""
    deadline = time.monotonic() + timeout
    collected: list[str] = []
    while len(collected) < count and time.monotonic() < deadline:
        collected.extend(transport.read_lines(timeout=0.2))
    return collected


def test_stdio_pipe_mode_completes_one_request_reply_exchange_as_a_real_subprocess():
    # This is SUC-003's own acceptance criterion and this ticket's own
    # Testing plan item: the daemon forked as a CHILD PROCESS, speaking
    # the identical framed protocol over its real stdin/stdout -- the
    # same StdioTransport `rogo.connection`'s own `--sim` resolution
    # uses, and the same subprocess-script pattern
    # test_repl.py's own line-buffering test already established
    # (sys.path.insert(0, src_host) inside the child, since this
    # subprocess has no [build-system]/pytest pythonpath wiring of its
    # own).
    src_host = str(Path(__file__).resolve().parents[3] / "src" / "host")
    script = _STDIO_PIPE_SUBPROCESS_SCRIPT.format(src_host=src_host)
    transport = StdioTransport([sys.executable, "-c", script])
    try:
        transport.send_line(dp.encode_request(dp.Request(id=1, verb="ping")))
        lines = _read_lines_until_count(transport, count=1, timeout=5.0)
        assert len(lines) == 1, f"expected exactly one reply line, got {lines!r}"
        reply = dp.decode_reply(lines[0])
        assert reply.id == 1
        assert reply.result == {"pong": True}
        assert reply.error is None
    finally:
        transport.close()


def test_stdio_pipe_mode_never_constructs_a_socket(monkeypatch):
    # This transport's own AC: "speaking the identical framed protocol
    # over stdin/stdout with no socket created." Asserted structurally,
    # not just by absence of a leftover file: monkeypatch
    # `socket.socket` (as `daemon.py` itself imports and calls it) to
    # raise if invoked at all, then drive a full request/reply exchange
    # through `run_stdio_pipe()` and confirm it still completes --
    # proving that code path never constructs one.
    def _forbidden(*args, **kwargs):
        raise AssertionError("run_stdio_pipe() must never construct a socket.socket()")

    monkeypatch.setattr(daemon.socket, "socket", _forbidden)

    connection = _make_connection()
    server = daemon.DaemonServer(connection, {"ping": lambda s, p, a: "pong"})
    server.start()
    try:
        in_stream = _FakeStdioStream([dp.encode_request(dp.Request(id=1, verb="ping"))])
        out_stream = _FakeStdioStream()
        daemon.run_stdio_pipe(server, stdin=in_stream, stdout=out_stream)
    finally:
        server.stop()

    assert len(out_stream.written) == 1
    reply = dp.decode_reply(out_stream.written[0].rstrip("\n"))
    assert reply.result == "pong"


# ---------------------------------------------------------------------------
# run_stdio_pipe_from_args -- ticket 007's own `--sim`/`--connect`/`--port`
# boot wiring for the pipe transport (module docstring's own "argument
# plumbing" note). These two tests stay in-process, with
# `daemon.resolve_connection` monkeypatched, on purpose -- no real sim
# subprocess involved -- so this file's own fast/no-compiler-dependency
# coverage of the plumbing itself is separate from
# `test_daemon_sim_e2e.py`'s own real, forked, `--sim`-backed end-to-end
# proof (this ticket's own SUC-003 scenario, AC #3).
# ---------------------------------------------------------------------------

def test_run_stdio_pipe_from_args_resolves_connection_via_args_and_serves_requests(monkeypatch):
    fake_conn = _make_connection()
    captured_args = []

    def _fake_resolve(args):
        captured_args.append(args)
        return fake_conn

    monkeypatch.setattr(daemon, "resolve_connection", _fake_resolve)

    in_stream = _FakeStdioStream([dp.encode_request(dp.Request(id=1, verb="ping"))])
    out_stream = _FakeStdioStream()
    ns = argparse.Namespace(sim=True, connect=None, port=None)

    daemon.run_stdio_pipe_from_args(
        ns, {"ping": lambda s, p, a: "pong"}, stdin=in_stream, stdout=out_stream,
    )

    # The SAME args object handed to run_stdio_pipe_from_args() is what
    # reaches connection resolution -- no re-parsing, no second target.
    assert captured_args == [ns]
    assert len(out_stream.written) == 1
    reply = dp.decode_reply(out_stream.written[0].rstrip("\n"))
    assert reply.id == 1
    assert reply.result == "pong"


def test_run_stdio_pipe_from_args_closes_the_resolved_connections_transport_on_eof(monkeypatch):
    fake_conn = _make_connection()
    monkeypatch.setattr(daemon, "resolve_connection", lambda args: fake_conn)
    closed = []
    monkeypatch.setattr(fake_conn.transport, "close", lambda: closed.append(True))

    in_stream = _FakeStdioStream([])  # EOF immediately -- no requests at all
    out_stream = _FakeStdioStream()

    daemon.run_stdio_pipe_from_args(
        argparse.Namespace(sim=True, connect=None, port=None), {},
        stdin=in_stream, stdout=out_stream,
    )

    # The resolved connection's transport is torn down once run_stdio_pipe()
    # returns (EOF), not left open for the caller to remember to close --
    # this is what keeps a --sim subprocess from being orphaned.
    assert closed == [True]

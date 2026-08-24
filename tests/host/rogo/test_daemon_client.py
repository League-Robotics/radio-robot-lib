"""tests/host/rogo/test_daemon_client.py -- ticket 008: `rogo.daemon_client`'s
find/spawn/direct-connect policy.

Every test that needs a genuinely running daemon spawns ONE as a real
subprocess via `daemon_client.default_spawn_argv()` +
`daemon_client._spawn_daemon()` -- the exact same code path
`get_connection(..., spawn=True)` itself uses -- rather than a
lighter-weight in-process fake, so this suite proves the real Unix-socket
find/connect machinery works end to end against `tools/sim`, mirroring
how `test_daemon_transports.py`'s own `UnixSocketListener` tests bind a
real socket rather than mocking one. `running_sim_daemon` below is the
shared fixture every such test uses; it always explicitly terminates its
subprocess in a `finally`, regardless of whether the test under it also
exercises self-termination -- this suite never relies on a timing race to
reap a subprocess.
"""

from __future__ import annotations

import argparse
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest

from robot_v6 import motion
from robot_v6.reliability import Session
from robot_v6.transport import Transport
from rogo import connection, daemon, daemon_client
from rogo import daemon_protocol as dp


def _short_tmp_dir() -> Path:
    """A short-path temp directory under `/tmp` -- AF_UNIX's `sun_path`
    has an OS-enforced length limit (~104 bytes on macOS) that pytest's
    own `tmp_path` fixture can exceed once nested; see
    `test_daemon_transports.py`'s own identical helper/rationale."""
    return Path(tempfile.mkdtemp(prefix="rogo-dc-", dir="/tmp"))


def _terminate(proc: subprocess.Popen, *, timeout: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout)


def _forbid_spawn(monkeypatch: pytest.MonkeyPatch, calls: list) -> None:
    """Monkeypatch `daemon_client._spawn_daemon` to record any call and
    immediately fail the test -- used by every "must not spawn" test
    below so a mistaken spawn shows up as a normal assertion failure
    with the offending argv attached, not a silently-passing test with a
    stray leaked subprocess."""
    def _spawn(argv):
        calls.append(argv)
        raise AssertionError(f"_spawn_daemon should not have been called, got argv={argv!r}")
    monkeypatch.setattr(daemon_client, "_spawn_daemon", _spawn)


@pytest.fixture
def running_sim_daemon(built_sim_binary):
    """A real `--sim`-backed daemon subprocess, already listening and
    proven reachable, torn down unconditionally at fixture teardown.
    Idle timeout is generous (30s) since most tests using this fixture
    do not exercise idle self-termination themselves -- they use
    `_spawn_daemon`/`default_spawn_argv` directly with their own short
    timeout for that (see
    `test_get_connection_auto_spawn_starts_daemon_and_self_terminates_after_idle_timeout`).
    Yields `(name, socket_dir)`."""
    del built_sim_binary
    socket_dir = _short_tmp_dir()
    name = "sim"
    args = argparse.Namespace(sim=True, connect=None, port=None)
    argv = daemon_client.default_spawn_argv(
        args, name=name, idle_timeout=30.0, socket_dir=socket_dir)
    proc = daemon_client._spawn_daemon(argv)
    try:
        connected = daemon_client._wait_for_daemon(name, socket_dir=socket_dir, timeout=10.0)
        assert connected is not None, "fixture's own daemon never became reachable"
        connected.transport.close()
        yield name, socket_dir
    finally:
        _terminate(proc)
        shutil.rmtree(socket_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# find_daemon() -- the "is one already running" probe.
# ---------------------------------------------------------------------------

def test_find_daemon_returns_none_when_nothing_is_listening(tmp_path):
    assert daemon_client.find_daemon("nope", socket_dir=tmp_path, timeout=0.2) is None


def test_find_daemon_connects_to_an_already_running_daemon(running_sim_daemon):
    name, socket_dir = running_sim_daemon
    found = daemon_client.find_daemon(name, socket_dir=socket_dir, timeout=2.0)
    assert found is not None
    try:
        assert isinstance(found, daemon_client.ClientConnection)
        assert found.session.highest_acked == 0
    finally:
        found.transport.close()


# ---------------------------------------------------------------------------
# find_daemon()'s liveness probe vs. a genuinely busy worker thread
# (sprint 004 ticket 002's own regression test -- the mechanism itself,
# isolated from CLI-subprocess/sim timing noise; complements, not
# replaces, tests/host/rogo/test_daemon_e2e_multi_client.py's CLI-level
# end-to-end preemption proof). No real `tools/sim` or CLI subprocess
# here -- an in-process fake `Connection` (the "drive" dispatch body
# never touches `Session` at all) plus a REAL `DaemonServer` +
# `UnixSocketListener` pair over a REAL Unix socket, matching sprint 003
# ticket 011's own lesson that a unit-level classification check alone
# is not sufficient proof for this kind of gap.
# ---------------------------------------------------------------------------

class _InertTransport(Transport):
    """A `Transport` that is never actually read from or written to --
    the busy "drive" dispatch handler below ignores the `Session` it is
    handed entirely, so this only needs to exist and satisfy
    `Session`'s own constructor, not do anything real."""

    def _read_chunk(self, timeout: float | None) -> bytes:
        return b""

    def _write_bytes(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def _make_fake_connection() -> connection.Connection:
    transport = _InertTransport()
    return connection.Connection(transport=transport, session=Session(transport))


def _send_request(sock: socket.socket, request: dp.Request) -> None:
    sock.sendall((dp.encode_request(request) + "\n").encode("utf-8"))


def _read_reply(sock: socket.socket, *, timeout: float = 5.0) -> dp.Reply:
    sock.settimeout(timeout)
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            raise AssertionError("connection closed before a full reply line arrived")
        buf += chunk
    line, _, _rest = buf.partition(b"\n")
    return dp.decode_reply(line.decode("utf-8"))


def test_find_daemon_probe_returns_promptly_while_the_worker_is_busy_with_a_long_dispatch():
    """The regression test for the liveness-probe fast path itself
    (`daemon.LIVENESS_PROBE_VERB`, `UnixSocketListener._serve_client()`):
    with `DaemonServer`'s single worker thread genuinely occupied
    dispatching a long-running call, `daemon_client.find_daemon()`'s own
    connectivity probe -- issued by a completely separate client
    connection -- must still return well within `DEFAULT_FIND_TIMEOUT_S`,
    without waiting for the busy dispatch to free the worker. Before this
    ticket's fix, the probe (`session_highest_acked`) was an ordinary
    priority-1 request submitted through the SAME `DaemonServer.submit()`
    queue as everything else, so it queued FIFO behind the busy dispatch
    and the probe would time out instead."""
    tmp_dir = _short_tmp_dir()
    try:
        drive_started = threading.Event()
        release_drive = threading.Event()

        def fake_drive(session, params, abort):
            del session, params, abort
            drive_started.set()
            # Busy until explicitly released below (or a generous 5s
            # safety timeout) -- nothing here ever sets `abort`; this
            # test is not exercising the estop/abort mechanism, only
            # whether the liveness probe can outrun an ordinary busy
            # dispatch.
            release_drive.wait(timeout=5.0)
            return {"done": True}

        server = daemon.DaemonServer(_make_fake_connection(), {"drive": fake_drive})
        server.start()
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        try:
            drive_client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            drive_client.connect(str(socket_path))
            try:
                _send_request(drive_client, dp.Request(id=1, verb="drive"))
                assert drive_started.wait(timeout=2.0), "drive dispatch never started"

                start = time.monotonic()
                found = daemon_client.find_daemon(
                    "tovez", socket_dir=tmp_dir, timeout=daemon_client.DEFAULT_FIND_TIMEOUT_S,
                )
                elapsed = time.monotonic() - start
            finally:
                release_drive.set()
                drive_reply = _read_reply(drive_client)
                drive_client.close()

            assert drive_reply.result == {"done": True}
            assert found is not None, "probe must succeed even while the worker is busy"
            try:
                assert elapsed < daemon_client.DEFAULT_FIND_TIMEOUT_S, (
                    f"probe took {elapsed:.3f}s -- it queued behind the busy worker "
                    f"instead of using the liveness-probe fast path"
                )
            finally:
                found.transport.close()
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_liveness_probe_does_not_reset_the_idle_activity_clock():
    """`run_daemon_worker()`'s idle-timeout tracking
    (`daemon_client._with_activity_tracking()`) must be unaffected by
    liveness probes -- a probe answered by `UnixSocketListener` outside
    the dispatch table never reaches a wrapped handler at all, so it
    must not reset `last_activity` (which would incorrectly keep an
    otherwise-idle daemon alive just because a client polled it)."""
    last_activity = [time.monotonic() - 100.0]
    table = daemon_client._with_activity_tracking(
        {"ping": lambda session, params, abort: "pong"}, last_activity,
    )
    stamp_before = last_activity[0]

    tmp_dir = _short_tmp_dir()
    try:
        server = daemon.DaemonServer(_make_fake_connection(), table)
        server.start()
        socket_path = daemon.socket_path_for_name("tovez", socket_dir=tmp_dir)
        listener = daemon.UnixSocketListener(server, socket_path)
        listener.start()
        try:
            found = daemon_client.find_daemon(
                "tovez", socket_dir=tmp_dir, timeout=daemon_client.DEFAULT_FIND_TIMEOUT_S,
            )
            assert found is not None
            found.transport.close()

            # The liveness probe alone must not have touched last_activity.
            assert last_activity[0] == stamp_before

            # A real dispatched request through the wrapped table, by
            # contrast, must still update it -- proving the assertion
            # above is because the probe bypasses the table, not because
            # activity tracking itself is broken.
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            client.connect(str(socket_path))
            try:
                _send_request(client, dp.Request(id=1, verb="ping"))
                reply = _read_reply(client)
            finally:
                client.close()
            assert reply.result == "pong"
            assert last_activity[0] > stamp_before
        finally:
            listener.stop()
            server.stop()
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


# ---------------------------------------------------------------------------
# get_connection(spawn=False) -- auto-detect-only policy (one-shot CLI
# commands, ticket 009).
# ---------------------------------------------------------------------------

def test_get_connection_auto_detect_returns_existing_daemon_without_spawning(
    running_sim_daemon, monkeypatch,
):
    name, socket_dir = running_sim_daemon
    calls: list = []
    _forbid_spawn(monkeypatch, calls)

    args = argparse.Namespace(sim=True, connect=None, port=None)
    result = daemon_client.get_connection(args, spawn=False, name=name, socket_dir=socket_dir)
    try:
        assert isinstance(result, daemon_client.ClientConnection)
    finally:
        result.transport.close()
    assert calls == []


def test_get_connection_auto_detect_falls_back_to_direct_connect_when_no_daemon(
    built_sim_binary, monkeypatch, tmp_path,
):
    del built_sim_binary
    calls: list = []
    _forbid_spawn(monkeypatch, calls)

    args = argparse.Namespace(sim=True, connect=None, port=None)
    result = daemon_client.get_connection(args, spawn=False, name="nope", socket_dir=tmp_path)
    try:
        # A direct connection.Connection, NOT a daemon-proxied one --
        # this ticket's own AC: "falls back to
        # rogo.connection.resolve() unchanged when none is found."
        assert isinstance(result, connection.Connection)
        assert not isinstance(result, daemon_client.ClientConnection)
    finally:
        result.transport.close()
    assert calls == []


# ---------------------------------------------------------------------------
# get_connection(spawn=True) -- auto-spawn-if-absent policy (rogo repl /
# rogo mcp, tickets 009/010).
# ---------------------------------------------------------------------------

def test_get_connection_auto_spawn_reuses_running_daemon_without_spawning_again(
    running_sim_daemon, monkeypatch,
):
    name, socket_dir = running_sim_daemon
    calls: list = []
    _forbid_spawn(monkeypatch, calls)

    args = argparse.Namespace(sim=True, connect=None, port=None)
    result = daemon_client.get_connection(args, spawn=True, name=name, socket_dir=socket_dir)
    try:
        assert isinstance(result, daemon_client.ClientConnection)
    finally:
        result.transport.close()
    assert calls == []


def test_get_connection_auto_spawn_starts_daemon_and_self_terminates_after_idle_timeout(
    built_sim_binary, monkeypatch,
):
    del built_sim_binary
    socket_dir = _short_tmp_dir()
    spawned: list[subprocess.Popen] = []
    real_spawn = daemon_client._spawn_daemon

    def _tracking_spawn(argv):
        proc = real_spawn(argv)
        spawned.append(proc)
        return proc

    monkeypatch.setattr(daemon_client, "_spawn_daemon", _tracking_spawn)

    args = argparse.Namespace(sim=True, connect=None, port=None)
    try:
        result = daemon_client.get_connection(
            args, spawn=True, name="sim", socket_dir=socket_dir,
            idle_timeout=1.0, spawn_timeout=10.0,
        )
        assert isinstance(result, daemon_client.ClientConnection)
        assert len(spawned) == 1, "auto-spawn must launch exactly one subprocess"

        # Prove the connection actually works before disconnecting --
        # AC: "spawns one, waits ..., then connects."
        assert result.session.highest_acked == 0
        result.transport.close()

        # No connected clients from here on -- AC: "self-terminates after
        # the configured idle timeout with no connected clients."
        proc = spawned[0]
        proc.wait(timeout=10.0)
        assert proc.returncode == 0, "worker should exit cleanly (its own teardown), not crash"
    finally:
        for proc in spawned:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=5.0)
        shutil.rmtree(socket_dir, ignore_errors=True)


def test_get_connection_auto_spawn_raises_a_clear_error_when_daemon_never_becomes_reachable(
    tmp_path,
):
    # A spawn target that never binds a socket at all -- proves the
    # bounded-wait/clear-error path, not a hang, per this ticket's own
    # AC. The sleeping subprocess exits on its own well within the test;
    # get_connection() also best-effort terminates it once it gives up
    # waiting (see get_connection()'s own docstring/implementation).
    args = argparse.Namespace(sim=True, connect=None, port=None)
    never_listens_argv = [sys.executable, "-c", "import time; time.sleep(2)"]

    start = time.monotonic()
    with pytest.raises(daemon_client.DaemonUnavailableError):
        daemon_client.get_connection(
            args, spawn=True, name="sim", socket_dir=tmp_path,
            spawn_timeout=0.3, spawn_argv=never_listens_argv,
        )
    elapsed = time.monotonic() - start
    assert elapsed < 2.0, "must not wait for the spawned process's own 2s sleep to finish"


def test_get_connection_auto_spawn_without_a_resolvable_name_raises(monkeypatch):
    calls: list = []
    _forbid_spawn(monkeypatch, calls)

    # No --sim, no override, no args.name -- resolve_client_name() has
    # nothing to work with and never opens a connection to find out.
    args = argparse.Namespace(sim=False, connect=None, port=None)
    with pytest.raises(daemon_client.RobotNameRequiredError):
        daemon_client.get_connection(args, spawn=True)
    assert calls == []


# ---------------------------------------------------------------------------
# The remoted session's own call surface -- matches what a direct
# connection's Session produces, for the calls rogo.cli's dispatch bodies
# actually make (send/send_unsequenced/pump/wait_for_ack).
# ---------------------------------------------------------------------------

def test_remote_session_send_and_wait_for_ack_matches_direct_session_outcome(
    running_sim_daemon,
):
    name, socket_dir = running_sim_daemon
    found = daemon_client.find_daemon(name, socket_dir=socket_dir, timeout=2.0)
    assert found is not None
    try:
        # Exactly rogo.cli._run_stop()'s own body: motion.stop() then
        # wait_for_ack() -- unchanged code, run against the remote
        # session instead of a direct one.
        seq_id = motion.stop(found.session)
        acked = found.session.wait_for_ack(seq_id, timeout=5.0)
        assert acked is True
    finally:
        found.transport.close()


def test_remote_session_hello_round_trip_reaches_the_sim_backed_connection(
    running_sim_daemon,
):
    name, socket_dir = running_sim_daemon
    found = daemon_client.find_daemon(name, socket_dir=socket_dir, timeout=2.0)
    assert found is not None
    try:
        # Exactly rogo.cli._run_hello()'s own body: send_unsequenced +
        # pump-until-device-banner.
        found.session.send_unsequenced("HELLO")
        deadline = time.monotonic() + 3.0
        banner = None
        while banner is None and time.monotonic() < deadline:
            for reply in found.session.pump(0.2):
                if reply.verb == "device":
                    banner = reply
                    break
        assert banner is not None, "no device banner received over the remote session"
        fields = list(banner.fields) + ["?"] * max(0, 4 - len(banner.fields))
        assert fields[2] == "sim"  # role common_name name serial -- name is index 2
    finally:
        found.transport.close()


def test_remote_session_unknown_get_field_returns_no_get_lines_like_a_direct_connection(
    running_sim_daemon,
):
    # A negative case too: an unknown config field still acks (no error
    # raised) but produces no `get` reply line -- the SAME shape
    # cli.py._run_config_get()'s own "unknown name -> no get line, but
    # acked" handling relies on, proving errors are not just swallowed
    # or misrepresented by the RPC layer.
    name, socket_dir = running_sim_daemon
    found = daemon_client.find_daemon(name, socket_dir=socket_dir, timeout=2.0)
    assert found is not None
    try:
        seq_id = motion.get(found.session, "no_such_field")
        acked = found.session.wait_for_ack(seq_id, timeout=5.0)
        assert acked is True
        get_replies = [r for r in found.session.pump(0.3) if r.verb == "get"]
        assert get_replies == []
    finally:
        found.transport.close()


# ---------------------------------------------------------------------------
# Client-side name resolution -- no network/connection involved.
# ---------------------------------------------------------------------------

def test_resolve_client_name_override_wins_over_everything():
    args = argparse.Namespace(sim=True, name="ignored")
    assert daemon_client.resolve_client_name(args, override="explicit") == "explicit"


def test_resolve_client_name_uses_args_name_when_no_override():
    args = argparse.Namespace(sim=False, name="tovez")
    assert daemon_client.resolve_client_name(args) == "tovez"


def test_resolve_client_name_falls_back_to_sim_fixed_default():
    args = argparse.Namespace(sim=True, name=None)
    assert daemon_client.resolve_client_name(args) == "sim"


def test_resolve_client_name_returns_none_when_nothing_resolves():
    args = argparse.Namespace(sim=False, name=None)
    assert daemon_client.resolve_client_name(args) is None


# ---------------------------------------------------------------------------
# default_spawn_argv() -- pure construction, no process/network involved.
# ---------------------------------------------------------------------------

def test_default_spawn_argv_for_a_sim_target():
    # Ticket 009 reconciliation: the default spawn target is now the
    # real `rogo serve` subcommand (`python -m rogo.cli serve`, robust
    # whether or not the `rogo` console script is on PATH), not
    # daemon_client's own standalone worker module.
    args = argparse.Namespace(sim=True, connect=None, port=None)
    argv = daemon_client.default_spawn_argv(args, name="sim", idle_timeout=42.0)
    assert argv[:4] == [sys.executable, "-m", "rogo.cli", "serve"]
    assert "--sim" in argv
    assert argv[argv.index("--name") + 1] == "sim"
    assert argv[argv.index("--idle-timeout") + 1] == "42.0"


def test_default_spawn_argv_for_a_connect_target_includes_socket_dir():
    args = argparse.Namespace(sim=False, connect="host:1234", port=None)
    argv = daemon_client.default_spawn_argv(
        args, name="tovez", idle_timeout=5.0, socket_dir=Path("/tmp/rundir"))
    assert "--sim" not in argv
    assert argv[argv.index("--connect") + 1] == "host:1234"
    assert argv[argv.index("--socket-dir") + 1] == "/tmp/rundir"


# ---------------------------------------------------------------------------
# is_estop_request() -- ticket 011's own fix (see this module's own
# header comment above `is_estop_request()`'s definition): a real ESTOP
# call through `build_session_dispatch_table()`'s generic RPC scheme
# always arrives as a `session_send`/`session_send_unsequenced` REQUEST
# with the wire verb nested in `params["wire_verb"]`, never as a
# top-level `"estop"`/`"halt"` request verb -- so `DaemonServer`'s
# default `estop_verbs` membership check could never see it. Pure
# function, no I/O -- exercises every branch directly against
# hand-built `daemon_protocol.Request`s.
# ---------------------------------------------------------------------------

def test_is_estop_request_true_for_a_session_send_unsequenced_estop_call():
    # robot_v6.motion.estop() sends ESTOP unsequenced (protocol.md#8.3) --
    # this is the shape a real `rogo` ESTOP call actually produces on the
    # wire through _RemoteSession.send_unsequenced().
    request = dp.Request(
        id=1, verb="session_send_unsequenced", params={"wire_verb": "ESTOP", "wire_fields": []})
    assert daemon_client.is_estop_request(request) is True


def test_is_estop_request_true_for_a_session_send_estop_call():
    # Not how motion.estop() actually calls it (it uses send_unsequenced),
    # but is_estop_request() checks both RPC verbs -- a future/alternate
    # caller that sent ESTOP sequenced should still be classified as
    # estop-priority.
    request = dp.Request(
        id=1, verb="session_send", params={"wire_verb": "ESTOP", "wire_fields": []})
    assert daemon_client.is_estop_request(request) is True


def test_is_estop_request_false_for_a_non_estop_wire_verb():
    # The exact scenario the gap this ticket fixes allowed through
    # silently: a long-running drive's own WHEELS_V call must NOT be
    # misclassified as estop-priority.
    request = dp.Request(
        id=1, verb="session_send", params={"wire_verb": "WHEELS_V", "wire_fields": [100, 100, 500]})
    assert daemon_client.is_estop_request(request) is False


def test_is_estop_request_false_for_a_different_unsequenced_verb():
    # HELLO is also sent unsequenced (protocol.md#8.3) -- proves this
    # function checks the WRAPPED wire_verb, not merely "is this an
    # unsequenced RPC call at all".
    request = dp.Request(
        id=1, verb="session_send_unsequenced", params={"wire_verb": "HELLO", "wire_fields": []})
    assert daemon_client.is_estop_request(request) is False


def test_is_estop_request_false_for_a_generic_rpc_verb_with_no_wire_verb_param():
    # session_pump/session_highest_acked/session_wait_for_ack/
    # session_wait_for_done carry no "wire_verb" param at all -- must not
    # raise (a plain .get() miss, not a KeyError) and must not be
    # misclassified as estop-priority.
    for verb, params in [
        ("session_pump", {"timeout": 0.2}),
        ("session_highest_acked", {}),
        ("session_wait_for_ack", {"seq_id": 1, "timeout": 5.0}),
        ("session_wait_for_done", {"seq_id": 1, "timeout": 5.0}),
    ]:
        request = dp.Request(id=1, verb=verb, params=params)
        assert daemon_client.is_estop_request(request) is False, verb


# ---------------------------------------------------------------------------
# The generic session-RPC dispatch table's own wait handlers respect
# `abort` -- ticket 011's own fix for the other half of the same gap
# (is_estop_request() alone is not enough: DaemonServer.submit() sets
# `abort` on the CURRENTLY RUNNING call when an estop-class request is
# submitted, but a handler that discards `abort` -- as
# `_dispatch_session_wait_for_ack`/`_dispatch_session_wait_for_done` did
# before this fix -- would never notice). Driven through a REAL
# DaemonServer + build_session_dispatch_table(), against a fake `Session`
# whose wait_for_done()/wait_for_ack() block until explicitly released --
# proving the fix end to end at the dispatch-table level, one layer below
# the full `rogo serve` subprocess scenarios in
# test_daemon_e2e_multi_client.py.
# ---------------------------------------------------------------------------

class _NeverCompletesSession:
    """A minimal stand-in for `robot_v6.reliability.Session`: `send()`
    always assigns seq_id 1; `wait_for_ack()`/`wait_for_done()` block for
    the FULL requested `timeout` and then report "not yet" -- exactly
    what a real Session does while waiting on a robot that has not
    finished a long motion. Used to prove the abort-aware poll loop in
    `_dispatch_session_wait_for_ack`/`_dispatch_session_wait_for_done`
    returns EARLY once `abort` is set, rather than only after this
    session's own full (generous, 30s) per-call timeout.

    `wait_started` is set on every `wait_for_ack()`/`wait_for_done()`
    call, including the first -- a test waits on it before submitting an
    estop, so the estop is only ever submitted once the daemon's single
    worker thread has genuinely popped the wait request and started
    dispatching it (`DaemonServer._run_worker()`'s own `_current_abort_
    event` is only set to a request's own `abort_event` WHILE it is
    executing -- submitting an estop any earlier would queue-jump ahead
    of the wait request instead of aborting it, a different, already
    separately-tested code path, see `test_daemon.py`'s own priority-
    queue tests)."""

    def __init__(self) -> None:
        self.wait_started = threading.Event()

    def send(self, verb, *fields):
        del verb, fields
        return 1

    def send_unsequenced(self, verb, *fields):
        del verb, fields

    def pump(self, timeout=0.0):
        del timeout
        return []

    @property
    def highest_acked(self):
        return 0

    def wait_for_ack(self, seq_id, timeout=5.0):
        del seq_id
        self.wait_started.set()
        time.sleep(timeout)
        return False

    def wait_for_done(self, seq_id, timeout=5.0):
        del seq_id
        self.wait_started.set()
        time.sleep(timeout)
        return None


def _serve_dispatch_table_directly(session) -> daemon.DaemonServer:
    """A `DaemonServer` wired exactly like `cli.cmd_serve()`'s own
    Unix-socket branch (ticket 011's own fix): the real
    `build_session_dispatch_table()`, classified by the real
    `is_estop_request()` -- but around a fake `session` (above) instead
    of a real `robot_v6.reliability.Session`/`tools/sim`, so this test
    stays fast and needs no compiled binary. `transport=None` is fine --
    nothing in this test suite's own dispatch bodies ever touches
    `Connection.transport`, only `Connection.session`."""
    conn = connection.Connection(transport=None, session=session)  # type: ignore[arg-type]
    table = daemon_client.build_session_dispatch_table()
    server = daemon.DaemonServer(conn, table, is_estop=daemon_client.is_estop_request)
    server.start()
    return server


def test_session_wait_for_done_dispatch_returns_early_once_estop_aborts_it():
    session = _NeverCompletesSession()
    server = _serve_dispatch_table_directly(session)
    try:
        result_box = []

        def submit_wait():
            result_box.append(server.submit(dp.Request(
                id=1, verb="session_wait_for_done",
                params={"seq_id": 1, "timeout": 30.0})))

        wait_thread = threading.Thread(target=submit_wait)
        wait_thread.start()
        # Wait for the daemon's OWN worker thread to have actually popped
        # this request and started dispatching it (session.wait_started
        # is only set from INSIDE wait_for_done() itself) -- see
        # _NeverCompletesSession's own docstring for why this, not
        # merely "the submitting thread started", is the correct sync
        # point.
        assert session.wait_started.wait(timeout=2.0), "wait_for_done() never started"

        start = time.monotonic()
        estop_reply = server.submit(dp.Request(
            id=2, verb="session_send_unsequenced",
            params={"wire_verb": "ESTOP", "wire_fields": []}))
        wait_thread.join(timeout=2.0)
        elapsed = time.monotonic() - start
    finally:
        server.stop()

    assert estop_reply.error is None
    assert not wait_thread.is_alive()
    assert result_box[0].result is None  # aborted, not "completed"
    # Well under the 30s the fake session's own wait_for_done() would
    # otherwise have blocked for -- proves abort was actually honored,
    # not merely ignored until a generous test timeout also happened to
    # pass.
    assert elapsed < 2.0


def test_session_wait_for_ack_dispatch_returns_early_once_estop_aborts_it():
    session = _NeverCompletesSession()
    server = _serve_dispatch_table_directly(session)
    try:
        result_box = []

        def submit_wait():
            result_box.append(server.submit(dp.Request(
                id=1, verb="session_wait_for_ack",
                params={"seq_id": 1, "timeout": 30.0})))

        wait_thread = threading.Thread(target=submit_wait)
        wait_thread.start()
        assert session.wait_started.wait(timeout=2.0), "wait_for_ack() never started"

        start = time.monotonic()
        estop_reply = server.submit(dp.Request(
            id=2, verb="session_send_unsequenced",
            params={"wire_verb": "ESTOP", "wire_fields": []}))
        wait_thread.join(timeout=2.0)
        elapsed = time.monotonic() - start
    finally:
        server.stop()

    assert estop_reply.error is None
    assert not wait_thread.is_alive()
    assert result_box[0].result is False  # aborted, not acked
    assert elapsed < 2.0


# ---------------------------------------------------------------------------
# Module boundary discipline -- this ticket's own AC #5.
# ---------------------------------------------------------------------------

def test_module_has_no_dependency_on_cli():
    assert not hasattr(daemon_client, "cli")
    with open(daemon_client.__file__, encoding="utf-8") as f:
        source_text = f.read()
    for forbidden in ("import cli", "from . import cli", "from rogo import cli", "from rogo.cli"):
        assert forbidden not in source_text, f"unexpected cli dependency: {forbidden!r}"

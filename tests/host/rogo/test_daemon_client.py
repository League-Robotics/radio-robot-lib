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
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

from robot_v6 import motion
from rogo import connection, daemon_client


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
    args = argparse.Namespace(sim=True, connect=None, port=None)
    argv = daemon_client.default_spawn_argv(args, name="sim", idle_timeout=42.0)
    assert argv[:3] == [sys.executable, "-m", "rogo.daemon_client"]
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
# Module boundary discipline -- this ticket's own AC #5.
# ---------------------------------------------------------------------------

def test_module_has_no_dependency_on_cli():
    assert not hasattr(daemon_client, "cli")
    with open(daemon_client.__file__, encoding="utf-8") as f:
        source_text = f.read()
    for forbidden in ("import cli", "from . import cli", "from rogo import cli", "from rogo.cli"):
        assert forbidden not in source_text, f"unexpected cli dependency: {forbidden!r}"

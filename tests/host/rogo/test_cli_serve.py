"""tests/host/rogo/test_cli_serve.py -- ticket 009: `rogo serve`
(`rogo.cli.cmd_serve()`), one-shot auto-detect routing, and `rogo repl`
auto-spawn routing -- end to end against the real compiled `tools/sim`
binary, through `cli.py`'s own wiring (`cli.main([...])`), mirroring
test_cli.py's/test_repl.py's own "prove the whole stack together" style
rather than re-exercising `daemon.py`/`daemon_client.py` in isolation
(already covered by test_daemon_transports.py/test_daemon_sim_e2e.py/
test_daemon_client.py).

Every test below isolates `daemon.default_socket_dir()`'s own
`XDG_RUNTIME_DIR` env var to a per-test tmp directory
(`isolated_socket_dir` fixture) -- this ticket's own hard requirement
never to touch this machine's REAL `~/.rogo/run`/`$XDG_RUNTIME_DIR`
socket directory, or leave a background daemon running against it, and
every spawned subprocess is explicitly terminated in a `finally`.

Session-state continuity (AC #5: "two sequential one-shot invocations
... do not reset tools/sim's connection state") is observed through
`robot_v6.reliability.Session`'s own client-side sequence counter,
which starts at 1 for a FRESH `Session` (`_run_stop()`'s own printed
`STOP acked (#N)` line): reusing one already-open daemon connection
across two commands prints `#1` then `#2`; two independent direct
connections each print `#1`.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Iterator

import pytest

from robot_v6.transport import StdioTransport
from rogo import cli, daemon_client, daemon_protocol as dp


def _terminate(proc: subprocess.Popen, *, timeout: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout)


def _short_tmp_dir() -> Path:
    """A short-path temp directory under `/tmp` -- AF_UNIX's `sun_path`
    has an OS-enforced length limit (~104 bytes on macOS) that pytest's
    own `tmp_path` fixture can exceed once nested; see
    test_daemon_transports.py's/test_daemon_client.py's own identical
    helper/rationale."""
    return Path(tempfile.mkdtemp(prefix="rogo-cs-", dir="/tmp"))


@pytest.fixture
def isolated_socket_dir(monkeypatch) -> Iterator[Path]:
    """Redirect `daemon.default_socket_dir()` -- and so every
    `daemon_client`/`cmd_serve()` lookup that does not pass an explicit
    `--socket-dir`/`socket_dir=` -- at a per-test SHORT tmp directory
    (`_short_tmp_dir()`, not pytest's own nested `tmp_path` -- see its
    own docstring), via the SAME `XDG_RUNTIME_DIR` precedence
    `daemon.default_socket_dir()` already documents. A subprocess
    spawned from within a test using this fixture inherits the modified
    env (`subprocess.Popen`'s own default `env=None` behavior), so this
    one env var is enough to keep both a spawned `rogo serve` and the
    parent test process's own daemon lookups pointed at the SAME
    isolated location."""
    xdg_dir = _short_tmp_dir()
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(xdg_dir))
    try:
        yield xdg_dir / "rogo"
    finally:
        shutil.rmtree(xdg_dir, ignore_errors=True)


def _spawn_serve(*extra_args: str) -> subprocess.Popen:
    argv = [sys.executable, "-m", "rogo.cli", "serve", "--sim", *extra_args]
    return subprocess.Popen(  # noqa: S603
        argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, start_new_session=True,
    )


def _wait_for_daemon(name: str, socket_dir: Path, *, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        found = daemon_client.find_daemon(name, socket_dir=socket_dir, timeout=0.2)
        if found is not None:
            return found
        time.sleep(0.05)
    return None


# ---------------------------------------------------------------------------
# rogo serve --stdio-pipe -- AC #1, over the pipe transport, through
# cli.py's own argparse/cmd_serve() wiring (unlike
# daemon_test_helpers.py's own inline-script harness, which builds its
# own dispatch table directly and never touches cli.py at all).
# ---------------------------------------------------------------------------

def test_serve_stdio_pipe_reuses_the_generic_session_rpc_table_and_reaches_sim(
    built_sim_binary,
):
    del built_sim_binary
    transport = StdioTransport([sys.executable, "-m", "rogo.cli", "serve", "--sim", "--stdio-pipe"])
    try:
        transport.send_line(dp.encode_request(dp.Request(
            id=1, verb="session_send_unsequenced",
            params={"wire_verb": "HELLO", "wire_fields": []})))
        transport.send_line(dp.encode_request(dp.Request(
            id=2, verb="session_pump", params={"timeout": 2.0})))

        lines: list[str] = []
        deadline = time.monotonic() + 5.0
        while len(lines) < 2 and time.monotonic() < deadline:
            lines.extend(transport.read_lines(timeout=0.5))
        assert len(lines) == 2, f"expected 2 reply lines, got {lines!r}"

        reply_send, reply_pump = dp.decode_reply(lines[0]), dp.decode_reply(lines[1])
        assert reply_send.error is None
        assert reply_pump.error is None

        banner = next((r for r in reply_pump.result if r["verb"] == "device"), None)
        assert banner is not None, f"no device banner in {reply_pump.result!r}"
        # role common_name name serial -- name is index 2 (_run_hello()'s
        # own unpacking order, protocol.md#8.3).
        assert banner["fields"][2] == "sim"
    finally:
        transport.close()


def test_serve_stdio_pipe_reports_an_unknown_verb_as_a_failed_reply(built_sim_binary):
    # Proves cmd_serve()'s pipe branch really goes through
    # DaemonServer's own dispatch (an unrecognized verb -> UnknownVerb,
    # daemon.py's own _execute() contract), not some ad hoc stub.
    del built_sim_binary
    transport = StdioTransport([sys.executable, "-m", "rogo.cli", "serve", "--sim", "--stdio-pipe"])
    try:
        transport.send_line(dp.encode_request(dp.Request(id=1, verb="no-such-verb")))
        lines = transport.read_lines(timeout=5.0)
        assert len(lines) == 1
        reply = dp.decode_reply(lines[0])
        assert reply.error is not None
        assert reply.error.type == "UnknownVerb"
    finally:
        transport.close()


# ---------------------------------------------------------------------------
# rogo serve -- the Unix-socket branch (production default).
# ---------------------------------------------------------------------------

def test_serve_unix_socket_accepts_a_client_and_self_terminates_after_idle_timeout(
    built_sim_binary, isolated_socket_dir,
):
    del built_sim_binary
    proc = _spawn_serve("--idle-timeout", "1.0")
    try:
        found = _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0)
        assert found is not None, "rogo serve never became reachable"
        try:
            assert found.session.highest_acked == 0
        finally:
            found.transport.close()

        # No further requests dispatched from here -- must self-terminate
        # (--idle-timeout, cmd_serve()'s own _wait_until_stopped()) and
        # exit cleanly (its own listener.stop()/server.stop()/
        # conn.transport.close() teardown), not linger or crash.
        proc.wait(timeout=10.0)
        assert proc.returncode == 0
    finally:
        _terminate(proc)


def test_serve_name_flag_overrides_the_socket_filename(built_sim_binary, isolated_socket_dir):
    del built_sim_binary
    proc = _spawn_serve("--name", "custom", "--idle-timeout", "5.0")
    try:
        found = _wait_for_daemon("custom", isolated_socket_dir, timeout=10.0)
        assert found is not None
        found.transport.close()
        assert (isolated_socket_dir / "custom.sock").exists()
        assert not (isolated_socket_dir / "sim.sock").exists()
    finally:
        _terminate(proc)


def test_serve_with_no_idle_timeout_runs_until_terminated(built_sim_binary, isolated_socket_dir):
    # Default (`--idle-timeout` omitted) must NOT self-terminate on its
    # own -- proven by waiting past a duration comfortably longer than
    # test_serve_unix_socket_accepts_a_client_and_self_terminates_after_
    # idle_timeout()'s own 1.0s window, then confirming the daemon is
    # still reachable, before explicitly terminating it ourselves.
    del built_sim_binary
    proc = _spawn_serve()
    try:
        found = _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0)
        assert found is not None
        found.transport.close()

        time.sleep(2.0)
        assert proc.poll() is None, "rogo serve must not self-terminate with no --idle-timeout"

        still_there = daemon_client.find_daemon("sim", socket_dir=isolated_socket_dir, timeout=2.0)
        assert still_there is not None
        still_there.transport.close()
    finally:
        _terminate(proc)


# ---------------------------------------------------------------------------
# One-shot auto-detect (AC #2) and its regression guard (AC #3).
# ---------------------------------------------------------------------------

def test_one_shot_commands_route_through_a_running_daemon_without_resetting_state(
    built_sim_binary, isolated_socket_dir, capsys,
):
    del built_sim_binary
    proc = _spawn_serve("--idle-timeout", "30.0")
    try:
        assert _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0) is not None

        exit_code_1 = cli.main(["stop", "--sim"])
        out_1 = capsys.readouterr().out
        exit_code_2 = cli.main(["stop", "--sim"])
        out_2 = capsys.readouterr().out

        assert exit_code_1 == 0 and exit_code_2 == 0
        assert "STOP acked (#1)" in out_1
        assert "STOP acked (#2)" in out_2, (
            "the second one-shot invocation must reuse the daemon's "
            f"already-open session (seq_id continuing from the first), "
            f"not reset it -- got {out_2!r}"
        )
    finally:
        _terminate(proc)


def test_one_shot_command_falls_back_to_direct_connect_with_no_daemon_present(
    isolated_socket_dir, built_sim_binary, capsys,
):
    # Regression guard for SUC-001's second AC: with no daemon running
    # for the resolved target, behavior is identical to today (a fresh
    # connection/session per invocation) -- no new process spawned
    # either (spawn=False, auto-detect only).
    del isolated_socket_dir, built_sim_binary

    exit_code_1 = cli.main(["stop", "--sim"])
    out_1 = capsys.readouterr().out
    exit_code_2 = cli.main(["stop", "--sim"])
    out_2 = capsys.readouterr().out

    assert exit_code_1 == 0 and exit_code_2 == 0
    assert "STOP acked (#1)" in out_1
    assert "STOP acked (#1)" in out_2, (
        f"with no daemon running, each one-shot invocation must still "
        f"get its own fresh session (#1 each time) -- got {out_2!r}"
    )


# ---------------------------------------------------------------------------
# rogo repl auto-spawn (AC #4) and cross-command state continuity (AC #5).
# ---------------------------------------------------------------------------

def test_repl_auto_spawns_a_daemon_when_none_is_running(
    built_sim_binary, isolated_socket_dir, monkeypatch, capsys,
):
    del built_sim_binary
    spawned: list[subprocess.Popen] = []
    real_spawn = daemon_client._spawn_daemon

    def _tracking_spawn(argv):
        proc = real_spawn(argv)
        spawned.append(proc)
        return proc

    monkeypatch.setattr(daemon_client, "_spawn_daemon", _tracking_spawn)

    try:
        exit_code = cli.main(["repl", "--sim", "hello", "stop", "quit"])
        out = capsys.readouterr().out

        assert exit_code == 0
        assert "name=sim" in out
        assert "STOP acked (#" in out
        assert len(spawned) == 1, "repl must auto-spawn exactly one daemon when none is running"

        # The auto-spawned daemon outlives the repl session that spawned
        # it (by design -- another client may reuse it) -- prove it is a
        # REAL, still-reachable Unix-socket daemon, not merely a claim.
        found = daemon_client.find_daemon("sim", socket_dir=isolated_socket_dir, timeout=2.0)
        assert found is not None
        found.transport.close()
    finally:
        for proc in spawned:
            _terminate(proc)


def test_one_shot_followed_by_repl_reuses_the_same_daemon_connection(
    built_sim_binary, isolated_socket_dir, monkeypatch, capsys,
):
    # SUC-001's own first AC, verified end to end here for the first
    # time: a one-shot invocation, then a `rogo repl` invocation,
    # against the SAME already-running daemon, must not reset
    # tools/sim's connection state between them.
    del built_sim_binary
    proc = _spawn_serve("--idle-timeout", "30.0")
    try:
        assert _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0) is not None

        def _forbid_spawn(argv):
            raise AssertionError(
                f"must reuse the already-running daemon, not spawn a new one: {argv!r}")
        monkeypatch.setattr(daemon_client, "_spawn_daemon", _forbid_spawn)

        exit_code_1 = cli.main(["stop", "--sim"])
        out_1 = capsys.readouterr().out
        assert exit_code_1 == 0
        assert "STOP acked (#1)" in out_1

        exit_code_2 = cli.main(["repl", "--sim", "stop", "quit"])
        out_2 = capsys.readouterr().out
        assert exit_code_2 == 0
        assert "STOP acked (#2)" in out_2, (
            f"repl must reuse the already-running daemon's open session "
            f"(seq_id continuing from the one-shot's #1) -- got {out_2!r}"
        )
    finally:
        _terminate(proc)

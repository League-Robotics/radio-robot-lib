"""tests/host/rogo/test_daemon_e2e_multi_client.py -- ticket 011's own
closing pass: end-to-end scenarios that exercise the WHOLE daemon
subsystem together, against a REAL `rogo serve` subprocess and a REAL
compiled `tools/sim` binary, rather than each module (tickets 004-010)
in isolation. Each test below maps onto one clause of this ticket's own
AC #1 (sprint.md Success Criteria for the daemon stream, "run together
in one scenario where practical") and the source issue's own
requirements/safety carry-over
(clasi/sprints/003-.../issues/rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md):

  - `test_estop_from_one_client_preempts_another_clients_in_flight_wait_
    through_the_real_daemon_wiring` -- the safety carry-over: "an
    estop/halt request from ANY client jumps to the front of the work
    queue and aborts any in-progress completion wait, so one client's
    long `drive` can never delay another client's halt." THROUGH the
    real `cli.cmd_serve()` wiring (`daemon_client.
    build_session_dispatch_table()`, not a test's own directly-named
    fake dispatch table) -- this is what this ticket's own end-to-end
    pass discovered was silently broken in production despite every
    per-ticket unit suite passing (see daemon.py's own `is_estop`
    module docstring section and daemon_client.py's own
    `is_estop_request()` header comment for the two-part gap and fix).
  - `test_unix_socket_daemon_serves_one_shot_repl_and_mcp_concurrently_
    over_one_shared_session` -- "rogo mcp and rogo CLI/repl all route
    through the same running daemon concurrently," Unix-socket mode,
    session-state continuity observed across all three client shapes in
    ONE scenario (test_cli_serve.py/test_mcp_server.py each already
    prove PAIRS of these; this proves the three-way combination this
    ticket's own AC asks for).
  - `test_stdio_pipe_daemon_holds_state_across_multiple_sequential_
    client_sessions_with_no_reset` -- the same "no reset between
    sessions" guarantee, stdio-pipe transport, through the REAL
    dispatch table (`daemon_client.build_session_dispatch_table()`) --
    every existing stdio-pipe test either uses a fake ping/hello table
    (`daemon_test_helpers.py`) or asserts only ONE request/reply
    exchange (`test_daemon_transports.py`); this is the first test in
    this directory to prove session continuity across MULTIPLE real
    motion commands over one held-open pipe.

Every test spawns its own `rogo serve` subprocess and explicitly
terminates it in a `finally`; every test isolates
`daemon.default_socket_dir()`'s own `XDG_RUNTIME_DIR` to a per-test
SHORT `/tmp` directory (AF_UNIX's `sun_path` length limit -- see
test_daemon_transports.py's/test_cli_serve.py's own identical
helper/rationale) -- this suite never touches this machine's real
`~/.rogo/run`/`$XDG_RUNTIME_DIR` socket directory.
"""

from __future__ import annotations

import asyncio
import json
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Iterator

import pytest

from robot_v6 import motion
from robot_v6.transport import StdioTransport
from rogo import cli, daemon_client, mcp_server


def _short_tmp_dir() -> Path:
    """A short-path temp directory under `/tmp` -- see
    test_daemon_transports.py's own identical helper/rationale (AF_UNIX's
    `sun_path` length limit)."""
    return Path(tempfile.mkdtemp(prefix="rogo-e2e-", dir="/tmp"))


def _terminate(proc: subprocess.Popen, *, timeout: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout)


@pytest.fixture
def isolated_socket_dir(monkeypatch) -> Iterator[Path]:
    """Redirect `daemon.default_socket_dir()` -- and so every
    `daemon_client`/`cmd_serve()` lookup that does not pass an explicit
    `--socket-dir`/`socket_dir=` -- at a per-test SHORT tmp directory.
    Matches test_cli_serve.py's/test_mcp_server.py's own identical
    fixture (duplicated rather than imported across test files, this
    project's own established precedent -- see e.g. daemon.py's own
    `_force_line_buffered()`)."""
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
# The safety-critical scenario -- issue's own "Safety carry-over" section,
# proven through the REAL `rogo serve` wiring this time (cli.cmd_serve()
# -> daemon_client.build_session_dispatch_table() -> daemon.DaemonServer),
# with two independent real socket clients, rather than a directly-named
# fake dispatch table (test_daemon.py) or an in-process fake Session
# (test_daemon_client.py's own `_NeverCompletesSession` tests -- those
# prove the SAME fix at the unit level with precise synchronization; this
# is the full-stack version against a real tools/sim).
# ---------------------------------------------------------------------------

def test_estop_from_one_client_preempts_another_clients_in_flight_wait_through_the_real_daemon_wiring(
    built_sim_binary, isolated_socket_dir,
):
    del built_sim_binary
    proc = _spawn_serve("--idle-timeout", "30.0")
    try:
        assert _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0) is not None

        client_a = daemon_client.find_daemon("sim", socket_dir=isolated_socket_dir, timeout=2.0)
        client_b = daemon_client.find_daemon("sim", socket_dir=isolated_socket_dir, timeout=2.0)
        assert client_a is not None and client_b is not None
        try:
            # Client A: a real WHEELS_V, in flight, then a completion
            # wait for the id RIGHT AFTER it -- one that will never
            # actually be dispatched. `tools/sim`'s own FakeMotionAdapter
            # is a "NO TIMER, NO CLOCK" test double (fake_motion_adapter.h's
            # own header comment): it completes an accepted motion as
            # fast as its own internal tick loop runs, entirely
            # independent of the wire's own `duration_ms` field, so a
            # real WHEELS_V cannot be relied on to still be "in flight"
            # by the time this test observes it -- waiting on the NEXT
            # id instead (never issued, so `lastDone` can never reach
            # it) reproduces the SAME "a long completion wait is
            # currently blocking the daemon's one worker thread"
            # condition the issue's own safety carry-over describes,
            # deterministically, exercising the identical production
            # dispatch path (`daemon_client._dispatch_session_wait_for_
            # done()`) either way.
            seq_id = motion.wheels_v(client_a.session, 100, 100, 500)
            unreachable_seq_id = seq_id + 1

            done_box: list = []

            def wait_for_completion():
                done_box.append(
                    client_a.session.wait_for_done(unreachable_seq_id, timeout=10.0))

            wait_thread = threading.Thread(target=wait_for_completion)
            wait_thread.start()
            # Give the daemon's single worker thread a moment to have
            # actually popped client A's session_wait_for_done request
            # and started blocking inside it -- otherwise the estop
            # below might merely queue-jump ahead of a still-QUEUED (not
            # yet dispatching) wait, which is a different, already
            # covered code path (test_daemon.py's own priority-queue
            # tests) rather than the in-progress-abort path this test is
            # about. Generous relative to a same-host subprocess/socket
            # round trip; the daemon has no other work queued at this
            # point, so dispatch should begin in well under this window.
            time.sleep(0.5)

            # Client B: ESTOP, from an entirely separate connection.
            start = time.monotonic()
            motion.estop(client_b.session)
            # send_unsequenced's own round trip through _RemoteSession
            # is itself a blocking RPC call (session_send_unsequenced) --
            # by the time motion.estop() returns, the daemon has already
            # replied to it, proving THIS request's own reply was not
            # stuck in FIFO order behind client A's long wait.
            estop_elapsed = time.monotonic() - start

            wait_thread.join(timeout=10.0)
            total_elapsed = time.monotonic() - start
        finally:
            client_a.transport.close()
            client_b.transport.close()
    finally:
        _terminate(proc)

    assert not wait_thread.is_alive(), "client A's wait_for_done never returned"
    # The estop's own reply must not have waited behind client A's
    # 10s-timeout completion wait.
    assert estop_elapsed < 2.0, (
        f"ESTOP's own reply took {estop_elapsed:.2f}s -- it must jump the "
        f"queue ahead of client A's in-progress completion wait, not be "
        f"served FIFO behind it"
    )
    # Client A's wait_for_done() must have returned (aborted) well before
    # its own 10s client-side timeout -- proves the abort signal
    # genuinely interrupted it, rather than this test merely having
    # waited long enough for that timeout to also elapse on its own
    # (this is exactly the gap ticket 011's own end-to-end pass found:
    # before the fix, this assertion is what fails -- roughly 10s
    # elapsed, not aborted at all).
    assert total_elapsed < 3.0, (
        f"client A's wait_for_done() took {total_elapsed:.2f}s total -- "
        f"expected it to be aborted by client B's estop well under its "
        f"own 10s client-side timeout"
    )
    # An aborted wait reports "not done" (None), the same shape a plain
    # timeout would report -- see daemon_client.py's own
    # _dispatch_session_wait_for_done() docstring/header comment for why
    # this is the intentional, sufficient signal (the caller experiences
    # "did not complete," which is exactly what an interrupted wait is).
    assert done_box == [None]


# ---------------------------------------------------------------------------
# Unix-socket mode: one-shot CLI, `rogo repl`, and `rogo mcp` all sharing
# ONE already-running daemon concurrently -- AC #1's "rogo mcp and rogo
# CLI/repl all route through the same running daemon concurrently."
# Session-state continuity (robot_v6.reliability.Session's own
# client-side sequence counter, starting at 1 for a fresh Session) is
# the same observable test_cli_serve.py/test_mcp_server.py already rely
# on for PAIRS of these three -- this test chains all three together in
# one scenario, in the order a real classroom session plausibly would:
# a one-shot probe, an interactive repl segment, then a concurrent mcp
# tool call, all against the SAME daemon, all continuing the SAME
# sequence.
# ---------------------------------------------------------------------------

def test_unix_socket_daemon_serves_one_shot_repl_and_mcp_concurrently_over_one_shared_session(
    built_sim_binary, isolated_socket_dir, monkeypatch, capsys,
):
    del built_sim_binary
    proc = _spawn_serve("--idle-timeout", "30.0")
    try:
        assert _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0) is not None

        # 1. One-shot `rogo stop --sim` -- auto-detects and reuses the
        #    already-running daemon (spawn=False).
        exit_code_1 = cli.main(["stop", "--sim"])
        out_1 = capsys.readouterr().out
        assert exit_code_1 == 0
        assert "STOP acked (#1)" in out_1

        # 2. `rogo repl --sim` -- auto-detects the SAME daemon
        #    (spawn=True, but one is already running, so nothing new is
        #    spawned), continuing the same sequence.
        exit_code_2 = cli.main(["repl", "--sim", "stop", "quit"])
        out_2 = capsys.readouterr().out
        assert exit_code_2 == 0
        assert "STOP acked (#2)" in out_2, (
            f"repl must continue the daemon's already-open session "
            f"(seq #2, not reset to #1) -- got {out_2!r}"
        )

        # 3. `rogo mcp --sim` -- also auto-detects the same daemon.
        #    mcp_server.serve() itself is stubbed out (it would otherwise
        #    block forever on the real MCP stdio protocol loop, which is
        #    not what this test is about -- mirrors test_mcp_server.py's
        #    own module docstring/`_stub_serve()`); the stub does its
        #    tool call FROM INSIDE itself, before returning -- `cmd_mcp()`'s
        #    own `finally: conn.transport.close()` closes the mcp
        #    session's socket the INSTANT `serve()` returns (same caveat
        #    test_mcp_server.py's own module docstring documents), so a
        #    tool call issued AFTER `cli.main()` has already returned
        #    would hit an already-closed transport.
        results: list = []

        def _stub_serve(session, *, listen=None, allow_remote=False):
            del listen, allow_remote
            server = mcp_server.build_server(session)
            results.append(asyncio.run(asyncio.wait_for(
                _call_tool(server, "stop", {}), timeout=5.0)))
            return 0
        monkeypatch.setattr(mcp_server, "serve", _stub_serve)

        exit_code_3 = cli.main(["mcp", "--sim"])
        assert exit_code_3 == 0
        assert len(results) == 1

        body = json.loads(results[0].content[0].text)
        assert body == {"acked": True, "seq_id": 3}, (
            f"the mcp session's own stop tool call must continue the SAME "
            f"daemon session (seq #3, following the one-shot's #1 and "
            f"repl's #2) -- got {body!r}"
        )

        # No new daemon was spawned by any of the three steps above --
        # exactly one process is listening for "sim" throughout.
        still_there = daemon_client.find_daemon("sim", socket_dir=isolated_socket_dir, timeout=2.0)
        assert still_there is not None
        still_there.transport.close()
    finally:
        _terminate(proc)


async def _call_tool(server, name: str, arguments: dict):
    return await server.call_tool(name, arguments)


# ---------------------------------------------------------------------------
# stdio-pipe mode: multiple real motion commands over ONE held-open pipe,
# through the REAL `daemon_client.build_session_dispatch_table()` (not a
# fake ping/hello table) -- proves "no reset between sessions" for THIS
# transport too (issue Requirement 2's own two-transports-same-protocol
# framing), the missing half of AC #1's "Unix-socket mode AND stdio-pipe
# mode, both against tools/sim."
# ---------------------------------------------------------------------------

def test_stdio_pipe_daemon_holds_state_across_multiple_sequential_client_sessions_with_no_reset(
    built_sim_binary,
):
    del built_sim_binary
    # `StdioTransport` spawns and owns the subprocess itself -- the SAME
    # helper `daemon_test_helpers.fork_stdio_daemon()` and
    # test_cli_serve.py's/test_daemon_transports.py's own real-subprocess
    # tests already use, here pointed at the real `rogo serve --sim
    # --stdio-pipe` subcommand (not a fake dispatch table).
    transport = StdioTransport(
        [sys.executable, "-m", "rogo.cli", "serve", "--sim", "--stdio-pipe"])
    try:
        session = daemon_client._RemoteSession(
            daemon_client._DaemonWireClient(transport))

        # Three sequential "client sessions" over the ONE held-open
        # pipe -- each a plain STOP, mirroring test_cli_serve.py's own
        # session-continuity observable (the client-side seq_id counter
        # starting at 1 for a fresh Session, continuing thereafter for
        # the SAME one).
        for expected_seq_id in (1, 2, 3):
            seq_id = motion.stop(session)
            assert seq_id == expected_seq_id, (
                f"expected seq_id {expected_seq_id} (no reset between "
                f"sequential commands over one held-open stdio pipe), "
                f"got {seq_id}"
            )
            acked = session.wait_for_ack(seq_id, timeout=5.0)
            assert acked is True
    finally:
        transport.close()

"""tests/host/rogo/test_connection.py -- `rogo.connection`: target
resolution (`--sim`/`--connect`/`--port` -> a live `Transport`/
`Session` pair) and the `ensure_sim_binary()` on-demand build it uses
for `--sim`.

Mirrors tests/host/robot_v6/test_transport.py's own patterns: a
background-thread local TCP server for `SocketTransport`/`--connect`
(no tools/sim needed for that path at all), and the real compiled
`tools/sim` binary (built once per session via the `built_sim_binary`
fixture below, itself a thin wrapper over
`rogo.connection.ensure_sim_binary()` rather than a duplicate of
tests/host/robot_v6/conftest.py's own `sim_binary` fixture -- this
directory's own fixture proves the PRODUCTION build path, not a
second copy of it) for `--sim`.
"""

from __future__ import annotations

import argparse
import socket
import threading

import pytest

from robot_v6.transport import SocketTransport, StdioTransport

from rogo import connection


def _ns(**kwargs) -> argparse.Namespace:
    """A bare Namespace with the three target attributes `resolve()`
    reads, defaulted the way `add_target_arguments()` would leave them
    unset -- lets tests exercise `resolve()`'s own validation directly,
    without going through argparse's mutually-exclusive-group check
    first (that's covered separately, below)."""
    ns = argparse.Namespace(sim=False, connect=None, port=None)
    for key, value in kwargs.items():
        setattr(ns, key, value)
    return ns


# ---------------------------------------------------------------------------
# add_target_arguments() -- argparse wiring and its own mutual exclusion.
# ---------------------------------------------------------------------------

def test_add_target_arguments_are_mutually_exclusive():
    parser = argparse.ArgumentParser()
    connection.add_target_arguments(parser)
    with pytest.raises(SystemExit):
        parser.parse_args(["--sim", "--connect", "127.0.0.1:1"])


def test_add_target_arguments_defaults_to_no_target_selected():
    parser = argparse.ArgumentParser()
    connection.add_target_arguments(parser)
    args = parser.parse_args([])
    assert args.sim is False
    assert args.connect is None
    assert args.port is None


# ---------------------------------------------------------------------------
# resolve() -- validation (no target / multiple targets / bad --connect).
# ---------------------------------------------------------------------------

def test_resolve_raises_when_no_target_given():
    with pytest.raises(connection.TargetError):
        connection.resolve(_ns())


def test_resolve_raises_when_multiple_targets_given():
    # Bypasses argparse's own mutual-exclusion check (test above) to
    # prove _resolve_transport()'s own defense-in-depth check too.
    with pytest.raises(connection.TargetError):
        connection.resolve(_ns(sim=True, connect="127.0.0.1:1"))


@pytest.mark.parametrize("bad_value", ["no-colon-here", "host:", ":1234", "host:notanumber"])
def test_resolve_raises_on_malformed_connect_target(bad_value):
    with pytest.raises(connection.TargetError):
        connection.resolve(_ns(connect=bad_value))


# ---------------------------------------------------------------------------
# resolve() with --connect -- a real local TCP server, no tools/sim needed.
# ---------------------------------------------------------------------------

def _serve_one_hello_reply(server: socket.socket) -> None:
    conn, _ = server.accept()
    with conn:
        buf = b""
        while b"\n" not in buf:
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
        conn.sendall(b"device NEZHA2 robot testpeer SERIAL0001\n")


def test_resolve_connect_returns_a_working_socket_transport_and_session():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    thread = threading.Thread(target=_serve_one_hello_reply, args=(server,))
    thread.start()
    try:
        conn = connection.resolve(_ns(connect=f"127.0.0.1:{port}"))
        try:
            assert isinstance(conn.transport, SocketTransport)
            conn.session.send_unsequenced("HELLO")
            lines = conn.transport.read_lines(timeout=2.0)
            assert lines == ["device NEZHA2 robot testpeer SERIAL0001"]
        finally:
            conn.transport.close()
    finally:
        thread.join(timeout=2.0)
        server.close()


# ---------------------------------------------------------------------------
# resolve() with --sim -- the real compiled tools/sim, end to end.
# `built_sim_binary` is this directory's own conftest.py fixture (shared
# with test_cli.py) -- see that file's own docstring for why it wraps
# `ensure_sim_binary()` itself rather than duplicating its compile
# command the way tests/host/robot_v6/conftest.py's `sim_binary` does.
# ---------------------------------------------------------------------------

def test_ensure_sim_binary_builds_a_runnable_executable(built_sim_binary):
    assert built_sim_binary.exists()
    assert built_sim_binary.is_file()


def test_ensure_sim_binary_is_cached_on_a_second_call(built_sim_binary):
    mtime_before = built_sim_binary.stat().st_mtime
    again = connection.ensure_sim_binary()
    assert again == built_sim_binary
    assert again.stat().st_mtime == mtime_before  # not recompiled


def test_resolve_sim_returns_a_working_stdio_transport_and_session(built_sim_binary):
    del built_sim_binary  # ensures the binary is built before resolve() runs
    conn = connection.resolve(_ns(sim=True))
    try:
        assert isinstance(conn.transport, StdioTransport)
        conn.session.send_unsequenced("HELLO")
        lines = []
        for _ in range(25):
            lines.extend(conn.transport.read_lines(timeout=0.5))
            if lines:
                break
        assert lines, "no reply arrived from the real tools/sim binary"
        assert lines[-1].startswith("device NEZHA2 robot sim ")
    finally:
        conn.transport.close()


def test_ensure_sim_binary_raises_when_sim_source_is_missing(tmp_path):
    with pytest.raises(connection.SimBinaryError):
        connection.ensure_sim_binary(repo_root=tmp_path)


def test_ensure_sim_binary_raises_a_clear_error_on_compile_failure(monkeypatch, tmp_path):
    repo_root = tmp_path
    sim_dir = repo_root / "tools" / "sim"
    sim_dir.mkdir(parents=True)
    (sim_dir / "sim_main.cpp").write_text("int main() { return 0; }\n")
    protocol_dir = repo_root / "src" / "protocol"
    protocol_dir.mkdir(parents=True)
    (protocol_dir / "protocol_handler.cpp").write_text("")

    class _FailedResult:
        returncode = 1
        stderr = "fake compile error"

    monkeypatch.setattr(connection.subprocess, "run", lambda *a, **k: _FailedResult())

    with pytest.raises(connection.SimBinaryError):
        connection.ensure_sim_binary(repo_root=repo_root)

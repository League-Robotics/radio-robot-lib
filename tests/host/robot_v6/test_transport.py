"""tests/host/robot_v6/test_transport.py -- robot_v6.transport: the
shared line-reassembly logic (against a minimal fake), then each real
implementation (PipeTransport over os.pipe(), StdioTransport over a
real subprocess, SocketTransport over a local TCP server), plus the
lazy-import contract SerialTransport is built on.
"""

from __future__ import annotations

import os
import pathlib
import socket
import subprocess
import sys
import threading

import pytest

from robot_v6.transport import (
    PipeTransport,
    SocketTransport,
    StdioTransport,
    Transport,
    TransportClosed,
)

_PYTHON = sys.executable


# ---------------------------------------------------------------------------
# Line reassembly, tested against a scripted fake -- independent of any
# real fd/socket, so it pins the CONTRACT `read_lines()` promises
# (docs/design/protocol.md S3.1's own list, transposed to the host
# side) rather than any one transport's own plumbing.
# ---------------------------------------------------------------------------

class _ScriptedTransport(Transport):
    """Replays a fixed sequence of `_read_chunk()` results, one per
    call; writes are just recorded."""

    def __init__(self, chunks: list[bytes]):
        super().__init__()
        self._chunks = list(chunks)
        self.written: list[bytes] = []

    def _read_chunk(self, timeout):
        del timeout
        if not self._chunks:
            return b""
        return self._chunks.pop(0)

    def _write_bytes(self, data: bytes) -> None:
        self.written.append(data)

    def close(self) -> None:
        pass


def test_send_line_appends_terminator():
    t = _ScriptedTransport([])
    t.send_line("WHEELS_V 100 100 1000 #1")
    assert t.written == [b"WHEELS_V 100 100 1000 #1\n"]


def test_read_lines_several_complete_lines_in_one_chunk():
    t = _ScriptedTransport([b"ack 1 0 none\nack 2 0 none\n"])
    assert t.read_lines() == ["ack 1 0 none", "ack 2 0 none"]


def test_read_lines_a_chunk_ending_mid_line_is_buffered_across_calls():
    t = _ScriptedTransport([b"ack 1 0 ", b"none\n"])
    assert t.read_lines() == []
    assert t.read_lines() == ["ack 1 0 none"]


def test_read_lines_a_chunk_that_is_only_a_fragment():
    t = _ScriptedTransport([b"ack 1"])
    assert t.read_lines() == []


def test_read_lines_strips_a_lone_trailing_cr():
    t = _ScriptedTransport([b"ack 1 0 none\r\n"])
    assert t.read_lines() == ["ack 1 0 none"]


def test_read_lines_mixed_multi_line_and_partial_remainder():
    t = _ScriptedTransport([b"ack 1 0 none\nack 2 0 none\nack 3 0 "])
    assert t.read_lines() == ["ack 1 0 none", "ack 2 0 none"]
    t._chunks.append(b"none\n")
    assert t.read_lines() == ["ack 3 0 none"]


def test_read_lines_blank_lines_pass_through_untouched():
    # Unlike the C++ handler (which drops a blank line silently at the
    # protocol layer, docs/design/protocol.md S2), this class has no
    # protocol opinion at all -- Session.pump() is where blank-line
    # filtering happens on the host side (reliability.py).
    t = _ScriptedTransport([b"\nack 1 0 none\n\n"])
    assert t.read_lines() == ["", "ack 1 0 none", ""]


def test_read_lines_empty_chunk_returns_empty_list_not_raise():
    t = _ScriptedTransport([b""])
    assert t.read_lines(timeout=0.01) == []


# ---------------------------------------------------------------------------
# PipeTransport, over a real os.pipe() pair.
# ---------------------------------------------------------------------------

def test_pipe_transport_write_then_read_round_trip():
    host_read_fd, sim_write_fd = os.pipe()
    sim_read_fd, host_write_fd = os.pipe()
    host = PipeTransport(host_read_fd, host_write_fd)
    sim = PipeTransport(sim_read_fd, sim_write_fd)
    try:
        host.send_line("HELLO")
        assert sim.read_lines(timeout=2.0) == ["HELLO"]
        sim.send_line("device NEZHA2 robot sim SIMHOST0001")
        assert host.read_lines(timeout=2.0) == ["device NEZHA2 robot sim SIMHOST0001"]
    finally:
        for fd in (host_read_fd, sim_write_fd, sim_read_fd, host_write_fd):
            try:
                os.close(fd)
            except OSError:
                pass


def test_pipe_transport_read_timeout_returns_empty_not_raise():
    read_fd, write_fd = os.pipe()
    t = PipeTransport(read_fd, write_fd)
    try:
        assert t.read_lines(timeout=0.05) == []
    finally:
        os.close(read_fd)
        os.close(write_fd)


def test_pipe_transport_eof_raises_transport_closed():
    read_fd, write_fd = os.pipe()
    t = PipeTransport(read_fd, write_fd)
    os.close(write_fd)  # the ONLY writer -- next read is EOF
    with pytest.raises(TransportClosed):
        t.read_lines(timeout=2.0)
    os.close(read_fd)


# ---------------------------------------------------------------------------
# StdioTransport, over a real subprocess (a tiny line-echoing Python
# script -- this is exactly the shape tools/sim's own --stdio mode has,
# without needing the C++ binary compiled just to test the transport).
# ---------------------------------------------------------------------------

_ECHO_SCRIPT = (
    "import sys\n"
    "for line in sys.stdin:\n"
    "    sys.stdout.write('echo:' + line)\n"
    "    sys.stdout.flush()\n"
)


def test_stdio_transport_round_trip_with_a_real_subprocess():
    t = StdioTransport([_PYTHON, "-c", _ECHO_SCRIPT])
    try:
        t.send_line("HELLO")
        lines = []
        for _ in range(20):
            lines.extend(t.read_lines(timeout=1.0))
            if lines:
                break
        assert lines == ["echo:HELLO"]
    finally:
        t.close()


def test_stdio_transport_close_is_idempotent_and_waits_for_exit():
    t = StdioTransport([_PYTHON, "-c", _ECHO_SCRIPT])
    t.close()
    t.close()  # must not raise
    assert t.process.poll() is not None


def test_stdio_transport_partial_line_across_two_subprocess_writes():
    # A subprocess that writes a line in two separate flushes -- proves
    # the reassembly buffer survives a REAL fd's own chunking, not just
    # the scripted fake above.
    script = (
        "import sys, time\n"
        "sys.stdout.write('ack 1 0 ')\n"
        "sys.stdout.flush()\n"
        "time.sleep(0.1)\n"
        "sys.stdout.write('none\\n')\n"
        "sys.stdout.flush()\n"
    )
    t = StdioTransport([_PYTHON, "-c", script])
    try:
        lines: list[str] = []
        for _ in range(30):
            lines.extend(t.read_lines(timeout=1.0))
            if lines:
                break
        assert lines == ["ack 1 0 none"]
    finally:
        t.close()


# ---------------------------------------------------------------------------
# SocketTransport, over a real local TCP server (a background thread
# that echoes one line back with a prefix, mirroring the stdio test's
# own shape).
# ---------------------------------------------------------------------------

def _serve_one_echo_connection(server: socket.socket) -> None:
    conn, _ = server.accept()
    with conn:
        buf = b""
        while b"\n" not in buf:
            chunk = conn.recv(4096)
            if not chunk:
                return
            buf += chunk
        conn.sendall(b"echo:" + buf)


def test_socket_transport_round_trip_against_a_local_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    thread = threading.Thread(target=_serve_one_echo_connection, args=(server,))
    thread.start()
    try:
        t = SocketTransport("127.0.0.1", port, connect_timeout=2.0)
        try:
            t.send_line("PING")
            lines = t.read_lines(timeout=2.0)
            assert lines == ["echo:PING"]
        finally:
            t.close()
    finally:
        thread.join(timeout=2.0)
        server.close()


def test_socket_transport_peer_close_raises_transport_closed():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]

    def _accept_then_close():
        conn, _ = server.accept()
        conn.close()

    thread = threading.Thread(target=_accept_then_close)
    thread.start()
    try:
        t = SocketTransport("127.0.0.1", port, connect_timeout=2.0)
        with pytest.raises(TransportClosed):
            for _ in range(20):
                t.read_lines(timeout=0.2)
        t.close()
    finally:
        thread.join(timeout=2.0)
        server.close()


# ---------------------------------------------------------------------------
# SerialTransport's lazy import: importing robot_v6.transport (and so
# the whole robot_v6 package) must never import `serial` as a side
# effect -- run in a FRESH subprocess so this repo's own already-primed
# sys.modules (or a `serial` package some other test happens to have
# imported) can't hide a regression.
# ---------------------------------------------------------------------------

def test_serial_transport_import_is_lazy():
    repo_root_src_host = str(
        pathlib.Path(__file__).resolve().parents[3] / "src" / "host")
    script = (
        "import sys\n"
        f"sys.path.insert(0, {repo_root_src_host!r})\n"
        "import robot_v6.transport\n"
        "print('serial' in sys.modules)\n"
    )
    result = subprocess.run([_PYTHON, "-c", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "False", (
        "importing robot_v6.transport must not import pyserial as a "
        f"side effect -- stdout={result.stdout!r} stderr={result.stderr!r}")

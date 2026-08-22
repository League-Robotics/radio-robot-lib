"""transport.py -- one Transport abstraction, three implementations,
interchangeable to everything above them (Session, reliability.py).

Stakeholder, verbatim, on why this exists: "The Rogo program should be
able to talk to either your compiled host version of the firmware with
a socket, or it should be able to talk to the Rogo server with a
socket. I'm not really particular about it being a socket. It could be
a pipe or whatever you want. The Rogo program should be able to deal
with either of those in the same way."

`Transport` is the "either of those, the same way" seam: raw protocol-v6
lines flow over it regardless of whether the far end is a TCP socket
(`SocketTransport`), a subprocess's stdio pipes (`StdioTransport`, the
generic `PipeTransport` it is built from), or a real serial port
(`SerialTransport`). Every concrete transport implements only three
primitives -- `_read_chunk()`, `_write_bytes()`, `close()` -- and this
base class supplies the line-oriented `send_line()`/`read_lines()` API
everything else in this package actually calls, including the SAME
partial-line reassembly discipline `ProtocolHandler::feed()` implements
in C++ (docs/design/protocol.md S3.1): a read may hand back half a
line, several lines, or nothing at all, and the buffering has to survive
every one of those shapes across calls.
"""

from __future__ import annotations

import abc
import os
import select
import socket
import subprocess
from typing import Sequence


class TransportClosed(Exception):
    """Raised by `_read_chunk()` (and so surfaces out of `read_lines()`)
    when the far end is genuinely gone -- EOF on a pipe/socket, or a
    closed serial port. Distinct from an ordinary read TIMEOUT (which
    returns `b""`/`[]` with the transport still very much alive): a
    caller's retry loop needs to tell "nothing to read yet" apart from
    "there is nothing to read ever again" to behave correctly on
    either one.
    """


class Transport(abc.ABC):
    """Line-oriented duplex byte stream. Subclasses supply the three
    primitives below; everything else here is shared, verb-agnostic
    plumbing -- exactly protocol_handler.h's own "no allocation, no
    verb knowledge in feed() itself" split, minus the firmware
    allocation constraint, which does not apply on the host.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    @abc.abstractmethod
    def _read_chunk(self, timeout: float | None) -> bytes:  # [s]
        """Return newly available bytes. `b""` means "no data arrived
        within `timeout` seconds" (`timeout=None` may block
        indefinitely) -- NOT end-of-stream; raise `TransportClosed` for
        a real disconnect/EOF instead of returning `b""` for it, so
        `read_lines()` can tell the two apart.
        """

    @abc.abstractmethod
    def _write_bytes(self, data: bytes) -> None:
        """Write `data` in full (or raise) -- no partial-write return
        value for callers to check, matching Sink::write()'s own "one
        call, fully consumed" contract on the C++ side.
        """

    @abc.abstractmethod
    def close(self) -> None:
        """Release whatever this transport owns (sockets, fds, a
        subprocess). Idempotent: calling it twice must not raise.
        """

    # ---- shared line-oriented API -------------------------------------

    def send_line(self, line: str) -> None:
        """Encode one already-formatted line (no '\\n' -- codec.py
        never includes one) as ASCII and write it, terminator
        included."""
        self._write_bytes((line + "\n").encode("ascii"))

    def read_lines(self, timeout: float | None = None) -> list[str]:  # [s]
        """Return every complete line that has become available,
        reassembling partial reads exactly the way
        `ProtocolHandler::feed()` does (docs/design/protocol.md S3.1):
        a call may return zero, one, or several lines, and a line
        split across two calls is buffered, not lost or duplicated. A
        lone trailing '\\r' on a line is stripped as a terminal
        artifact (S2), matching the wire's own rule for the OTHER
        direction. Blank lines ARE returned here (unlike the C++
        handler, which drops them itself) -- Session.pump() is the
        layer that discards them, keeping this class a pure byte-to-
        line reassembler with no protocol opinion at all.
        """
        chunk = self._read_chunk(timeout)
        if not chunk:
            return []
        self._buffer.extend(chunk)
        lines: list[str] = []
        while True:
            index = self._buffer.find(b"\n")
            if index < 0:
                break
            raw = bytes(self._buffer[:index])
            del self._buffer[: index + 1]
            if raw.endswith(b"\r"):
                raw = raw[:-1]
            lines.append(raw.decode("ascii", errors="replace"))
        return lines


class SocketTransport(Transport):
    """A TCP connection to a `--listen HOST:PORT` peer (tools/sim, or a
    future rogo server) -- the first of the stakeholder's "either of
    those" two cases.
    """

    def __init__(self, host: str, port: int, *, connect_timeout: float = 5.0):  # [s]
        super().__init__()
        self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        self._sock.settimeout(None)  # blocking; _read_chunk uses select() for pacing

    def _read_chunk(self, timeout: float | None) -> bytes:
        ready, _, _ = select.select([self._sock], [], [], timeout)
        if not ready:
            return b""
        data = self._sock.recv(4096)
        if data == b"":
            raise TransportClosed("socket closed by peer")
        return data

    def _write_bytes(self, data: bytes) -> None:
        self._sock.sendall(data)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


class PipeTransport(Transport):
    """Talks over an arbitrary pair of raw file descriptors -- the
    general case a subprocess's stdio pipes are one instance of. Not
    typically constructed directly by a caller driving a subprocess
    (use `StdioTransport` for that); this class exists so anything
    ELSE that hands you a read fd and a write fd (a PTY pair, `os.pipe()`
    in a test) gets the same reassembly/pacing behavior for free.
    """

    def __init__(self, read_fd: int, write_fd: int):
        super().__init__()
        self._read_fd = read_fd
        self._write_fd = write_fd

    def _read_chunk(self, timeout: float | None) -> bytes:
        ready, _, _ = select.select([self._read_fd], [], [], timeout)
        if not ready:
            return b""
        data = os.read(self._read_fd, 4096)
        if data == b"":
            raise TransportClosed("pipe closed (EOF)")
        return data

    def _write_bytes(self, data: bytes) -> None:
        os.write(self._write_fd, data)

    def close(self) -> None:
        # Base PipeTransport does not own the fds it was given (a test
        # using os.pipe() directly owns closing them); StdioTransport
        # below overrides this to also tear down its subprocess.
        pass


class StdioTransport(PipeTransport):
    """Spawns `command` as a subprocess and talks to its stdin/stdout --
    this is how tests drive tools/sim's `--stdio` mode, and the
    stakeholder's own second "either of those" case in spirit (a pipe
    rather than a socket) even though the concrete peer here happens to
    be a child process rather than a long-running server.
    """

    def __init__(self, command: Sequence[str], **popen_kwargs: object):
        self._process = subprocess.Popen(  # noqa: S603 -- caller-controlled command
            list(command),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            bufsize=0,
            **popen_kwargs,
        )
        assert self._process.stdin is not None and self._process.stdout is not None
        super().__init__(self._process.stdout.fileno(), self._process.stdin.fileno())

    @property
    def process(self) -> subprocess.Popen:
        return self._process

    def close(self) -> None:
        try:
            self._process.stdin.close()  # EOF -- the sim's own clean-shutdown trigger
        except OSError:
            pass
        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
        try:
            self._process.stdout.close()
        except OSError:
            pass


class SerialTransport(Transport):
    """A real serial port -- the hardware end of "either of those".
    `pyserial` is imported LAZILY, inside `__init__`, specifically so
    importing this module (or anything that imports it, including the
    whole rest of this package) never requires `pyserial` to be
    installed. Nothing in this repo's test suite constructs this class
    (there is no hardware here to test against), so this import must
    not become a hard dependency of running the tests at all.
    """

    def __init__(self, port: str, baud: int = 115200, *, read_timeout: float = 0.05):  # [s]
        super().__init__()
        import serial  # local import -- see class docstring

        self._serial = serial.Serial(port, baudrate=baud, timeout=read_timeout)

    def _read_chunk(self, timeout: float | None) -> bytes:
        # pyserial's own read() blocks up to the port's configured
        # `timeout` and returns whatever arrived (possibly nothing) --
        # there is no portable select()-style wait across every pyserial
        # backend, so this loops in `read_timeout`-sized steps until the
        # CALLER's own `timeout` budget is exhausted, matching the
        # semantics `_read_chunk()` promises everywhere else (return
        # "" on a timeout, block indefinitely on `timeout=None`).
        if timeout is None:
            while True:
                data = self._serial.read(4096)
                if data:
                    return data
        remaining = timeout
        while remaining > 0:
            data = self._serial.read(4096)
            if data:
                return data
            remaining -= self._serial.timeout
        return b""

    def _write_bytes(self, data: bytes) -> None:
        self._serial.write(data)

    def close(self) -> None:
        self._serial.close()

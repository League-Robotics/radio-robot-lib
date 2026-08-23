"""connection.py -- resolve one `rogo` invocation's target into a live
`robot_v6.transport.Transport` + `robot_v6.reliability.Session` pair
(sprint.md's Architecture Step 3, `rogo.connection`'s own row).

Three target kinds, matching the stakeholder's own framing in
`transport.py`'s module docstring ("a socket... or a pipe... I'm not
really particular"), plus the one this repo adds for classroom/CI use
with no hardware at all:

- `--sim`   -- spawn a freshly (re)built `tools/sim --stdio` subprocess
              via `StdioTransport`. No robot, no serial port, no CMake
              (see `ensure_sim_binary()` below and tools/sim/README.md).
- `--connect HOST:PORT` -- a TCP peer (a relay, `tools/sim --listen`, or
              a future `rogo serve`) via `SocketTransport`.
- `--port PORT` -- a real serial port via `SerialTransport`.

This module depends only on `robot_v6.transport`/`robot_v6.reliability`
-- no `robot_v6.motion` dependency (sprint.md's Implementation Plan
note: that arrives with ticket 003's `drive`/`turn` commands).
"""

from __future__ import annotations

import argparse
import dataclasses
import subprocess
from pathlib import Path

from robot_v6.reliability import Session
from robot_v6.transport import SerialTransport, SocketTransport, StdioTransport, Transport


class TargetError(ValueError):
    """Raised by `resolve()`/`_resolve_transport()` when the CLI
    arguments name zero or more than one target, or a malformed
    `--connect` value -- a caller-facing usage error, not a connection
    failure once a target has been chosen."""


class SimBinaryError(RuntimeError):
    """Raised by `ensure_sim_binary()` when `tools/sim` cannot be
    (re)built -- e.g. no C++ compiler at `/usr/bin/c++`, or a real
    compile error in the sources. Distinct from `TargetError`: the
    target WAS unambiguous (`--sim`), building it just failed."""


@dataclasses.dataclass(frozen=True)
class Connection:
    """The live pair a resolved target hands back to a caller: the raw
    `Transport` (so a caller can `close()` it when done) and the
    `Session` built on top of it (so a caller can `send()`/`pump()`
    right away)."""

    transport: Transport
    session: Session


def add_target_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the mutually exclusive `--sim`/`--connect`/`--port` flags
    `resolve()` reads. Shared by every `rogo` subcommand that needs a
    target, so each one gets the same three flags with the same help
    text instead of re-declaring them (`rogo.cli`'s own subcommands all
    call this)."""
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--sim", action="store_true",
        help="talk to a freshly (re)built tools/sim subprocess over stdio "
             "(no robot, no serial port required)",
    )
    group.add_argument(
        "--connect", metavar="HOST:PORT", default=None,
        help="talk to a TCP peer -- a relay, or tools/sim --listen",
    )
    group.add_argument(
        "--port", metavar="PORT", default=None,
        help="talk to a real robot over a serial port",
    )


def resolve(args: argparse.Namespace) -> Connection:
    """Resolve `args` (as produced by a parser that called
    `add_target_arguments()`) into a live `Connection`. Raises
    `TargetError` if no target (or more than one) was named, and
    whatever the chosen `Transport` subclass itself raises on a
    genuine connection failure (`OSError`, `SimBinaryError`, ...)."""
    transport = _resolve_transport(args)
    return Connection(transport=transport, session=Session(transport))


def _resolve_transport(args: argparse.Namespace) -> Transport:
    sim = bool(getattr(args, "sim", False))
    connect = getattr(args, "connect", None)
    port = getattr(args, "port", None)

    chosen = [flag for flag, value in
              (("--sim", sim), ("--connect", connect), ("--port", port)) if value]
    if len(chosen) > 1:
        raise TargetError(f"choose exactly one target, got {', '.join(chosen)}")

    if sim:
        binary = ensure_sim_binary()
        return StdioTransport([str(binary), "--stdio"])
    if connect:
        host, tcp_port = _split_host_port(connect)
        return SocketTransport(host, tcp_port)
    if port:
        return SerialTransport(port)
    raise TargetError("no target specified -- pass --sim, --connect HOST:PORT, or --port PORT")


def _split_host_port(value: str) -> tuple[str, int]:
    host, sep, port_text = value.rpartition(":")
    if not sep or not host or not port_text.isdigit():
        raise TargetError(f"--connect expects HOST:PORT, got {value!r}")
    return host, int(port_text)


# ---------------------------------------------------------------------------
# tools/sim on-demand build -- the same `/usr/bin/c++` + explicit `-I`
# pattern tests/host/robot_v6/conftest.py's own `sim_binary` fixture
# uses, but cached under tools/sim/.build/ (mtime-checked) rather than a
# pytest tmp dir, since a `rogo --sim` invocation has no fixture to own
# that for it -- a classroom user should not have to hand-build
# tools/sim per its README before `rogo --sim` works the first time.
# ---------------------------------------------------------------------------

def _repo_root() -> Path:
    # src/host/rogo/connection.py -> rogo -> host -> src -> repo root.
    return Path(__file__).resolve().parents[3]


def ensure_sim_binary(repo_root: Path | None = None) -> Path:
    """Return the path to a `tools/sim` executable, (re)building it
    first if it is missing or older than its own sources. Raises
    `SimBinaryError` with the compiler's own stderr if the build
    fails -- the message points at tools/sim/README.md for a manual
    build as the fallback."""
    root = repo_root if repo_root is not None else _repo_root()
    sim_dir = root / "tools" / "sim"
    sim_main = sim_dir / "sim_main.cpp"
    protocol_src = root / "src" / "protocol" / "protocol_handler.cpp"
    if not sim_main.exists():
        raise SimBinaryError(
            f"tools/sim source not found at {sim_main} -- see tools/sim/README.md"
        )

    build_dir = sim_dir / ".build"
    exe_path = build_dir / "robot_sim"
    sources = [sim_main, protocol_src]
    if not _needs_rebuild(exe_path, sources):
        return exe_path

    build_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        "/usr/bin/c++", "-std=c++20", "-O2", "-Wall", "-Wextra",
        "-I", str(root / "src" / "protocol"),
        "-I", str(root / "tests" / "protocol"),
        *[str(s) for s in sources],
        "-o", str(exe_path),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
    except OSError as exc:
        raise SimBinaryError(
            f"could not run the C++ compiler ({cmd[0]}): {exc} -- "
            "see tools/sim/README.md to build tools/sim manually"
        ) from exc
    if result.returncode != 0:
        raise SimBinaryError(
            "failed to build tools/sim -- see tools/sim/README.md to build it "
            f"manually.\ncommand: {' '.join(cmd)}\nstderr:\n{result.stderr}"
        )
    return exe_path


def _needs_rebuild(exe_path: Path, sources: list[Path]) -> bool:
    if not exe_path.exists():
        return True
    exe_mtime = exe_path.stat().st_mtime
    return any(source.exists() and source.stat().st_mtime > exe_mtime for source in sources)

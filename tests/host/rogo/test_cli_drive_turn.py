"""tests/host/rogo/test_cli_drive_turn.py -- ticket 003's `drive`/`turn`
subcommands: argument validation and internal helpers as fast unit
tests against scripted fake transports (mirrors tests/host/robot_v6/
test_motion.py's own `_RecordingTransport` pattern, extended here with
scripted REPLIES since `drive --mm`'s soft-warning path needs a same-id
`err` reply `tools/sim`'s `FakeMotionAdapter` never actually produces --
see module docstring below), plus end-to-end runs against the real
compiled `tools/sim` binary for `drive --ms`, `drive --mm`, `drive
stream`, and `turn` (this ticket's own Testing plan).
"""

from __future__ import annotations

import argparse

import pytest

from robot_v6.reliability import Session
from robot_v6.transport import Transport

from rogo import cli, connection


# ---------------------------------------------------------------------------
# Fake transports -- no tools/sim needed for the tests in this section.
# ---------------------------------------------------------------------------

class _ScriptedReplyTransport(Transport):
    """Replays a fixed sequence of `_read_chunk()` results, one per
    call; records every line written. Used to synthesize protocol.md
    #8.9's own "ack THEN a same-id err" merits-rejection shape --
    `tools/sim` links `FakeMotionAdapter`, which accepts and completes
    every motion verb (test_motion.py's own module docstring), so it
    can never actually produce a `kUnknown` outcome for this test to
    observe end to end."""

    def __init__(self, chunks: list[bytes]):
        super().__init__()
        self._chunks = list(chunks)
        self.written: list[str] = []

    def _read_chunk(self, timeout):
        del timeout
        if not self._chunks:
            return b""
        return self._chunks.pop(0)

    def _write_bytes(self, data: bytes) -> None:
        self.written.append(data.decode("ascii").rstrip("\n"))

    def close(self) -> None:
        pass


class _AutoAckTransport(Transport):
    """Immediately acks every sequenced line it is asked to write
    (parsing the trailing '#<id>' straight back off it) -- a fast,
    deterministic stand-in for a real peer, used only to keep
    `_cmd_drive_stream()`'s loop/STOP path from blocking on a reply that
    a scripted-chunks fake has no way to react to as sends happen."""

    def __init__(self):
        super().__init__()
        self.written: list[str] = []
        self._pending_replies: list[bytes] = []

    def _read_chunk(self, timeout):
        del timeout
        if not self._pending_replies:
            return b""
        return self._pending_replies.pop(0)

    def _write_bytes(self, data: bytes) -> None:
        line = data.decode("ascii").rstrip("\n")
        self.written.append(line)
        if "#" in line:
            seq_id = line.rsplit("#", 1)[1]
            self._pending_replies.append(f"ack {seq_id} 0 none\n".encode("ascii"))

    def close(self) -> None:
        pass


def _raise_after(n: int):
    """A fake `sleep()` that raises `KeyboardInterrupt` on its `n`th
    call -- stands in for a real Ctrl-C after a bounded, deterministic
    number of stream-loop iterations."""
    calls = {"count": 0}

    def _sleep(_seconds: float) -> None:
        calls["count"] += 1
        if calls["count"] >= n:
            raise KeyboardInterrupt

    return _sleep


# ---------------------------------------------------------------------------
# _wheels_x_fields() -- the speed-shaped-positionals -> distance-shaped
# WHEELS_X-fields reshape `drive --mm` needs, as a pure function.
# ---------------------------------------------------------------------------

def test_wheels_x_fields_symmetric_forward():
    left_d, right_d, cruise, timeout_ms = cli._wheels_x_fields(100, 100, 200)
    assert (left_d, right_d, cruise) == (200, 200, 100)
    assert timeout_ms == 6000  # 3x the naive ETA (1000*200/100 = 2000ms)


def test_wheels_x_fields_opposite_signs_matches_turn_like_distances():
    left_d, right_d, cruise = cli._wheels_x_fields(-100, 100, 50)[:3]
    assert (left_d, right_d, cruise) == (-50, 50, 100)


def test_wheels_x_fields_timeout_has_a_1000ms_floor():
    _, _, _, timeout_ms = cli._wheels_x_fields(1000, 1000, 1)
    assert timeout_ms == 1000


def test_wheels_x_fields_zero_speed_raises_value_error():
    with pytest.raises(ValueError):
        cli._wheels_x_fields(0, 0, 50)


# ---------------------------------------------------------------------------
# drive --mm's soft-warning path (STAKEHOLDER DECISION, sprint 001
# stakeholder_approval gate): a kUnknown merits rejection is a warning,
# not a hard error -- exit 0, the outcome printed plainly.
# ---------------------------------------------------------------------------

def test_drive_mm_prints_a_soft_warning_and_exits_zero_on_kunknown(capsys):
    transport = _ScriptedReplyTransport([b"ack 1 0 none\nerr 1 #1\n"])
    session = Session(transport)

    exit_code = cli._cmd_drive_mm(session, 100, 100, 50)

    assert exit_code == 0
    assert transport.written == ["WHEELS_X 50 50 100 1500 #1"]
    out, err = capsys.readouterr()
    assert "WHEELS_X sent (#1)" in out
    assert "kUnknown" in out or "ERR_UNKNOWN" in out
    assert "warning" in err.lower()


def test_drive_mm_reports_not_acked_as_a_hard_error(monkeypatch, capsys):
    # Bypasses _await_ack_and_err()'s own real pump/timeout loop (which
    # would otherwise burn this test's own wall-clock _DEFAULT_TIMEOUT
    # waiting for an ack a transport with no scripted replies at all
    # will never deliver) -- this test's own job is `_cmd_drive_mm`'s
    # "not acked" -> exit 1 branch, not `_await_ack_and_err`'s timeout
    # behavior (already exercised for real by `hello`'s own
    # `wait_for_ack`-based ticket-002 tests).
    monkeypatch.setattr(
        cli, "_await_ack_and_err",
        lambda session, seq_id, timeout=cli._DEFAULT_TIMEOUT: (False, None))
    transport = _ScriptedReplyTransport([])
    session = Session(transport)

    exit_code = cli._cmd_drive_mm(session, 100, 100, 50)

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "not acked" in err


# ---------------------------------------------------------------------------
# drive stream / bare mode -- re-issues WHEELS_V at the resend cadence
# until Ctrl-C, then sends STOP (this ticket's own AC, tested against a
# fast deterministic fake rather than a real Ctrl-C).
# ---------------------------------------------------------------------------

def test_stream_resends_wheels_v_then_stops_on_keyboard_interrupt():
    transport = _AutoAckTransport()
    session = Session(transport)

    exit_code = cli._cmd_drive_stream(session, 100, 100, 50, sleep=_raise_after(2))

    assert exit_code == 0
    wheels_v_lines = [line for line in transport.written if line.startswith("WHEELS_V")]
    stop_lines = [line for line in transport.written if line.startswith("STOP")]
    assert len(wheels_v_lines) == 2
    assert len(stop_lines) == 1


def test_stream_wheels_v_lease_is_capped_at_the_wire_ceiling():
    transport = _AutoAckTransport()
    session = Session(transport)
    cli._cmd_drive_stream(session, 100, 100, 5000, sleep=_raise_after(1))
    # resend_ms * _STREAM_LEASE_MULTIPLE (5000*3=15000) would exceed
    # protocol.md#5's own WHEELS_V 5000ms ceiling -- must be capped there.
    wheels_v_line = next(line for line in transport.written if line.startswith("WHEELS_V"))
    assert wheels_v_line == "WHEELS_V 100 100 5000 #1"


# ---------------------------------------------------------------------------
# Argument validation -- fails fast, before ever resolving a target.
# ---------------------------------------------------------------------------

def test_drive_ms_and_mm_are_mutually_exclusive(capsys):
    exit_code = cli.main(["drive", "100", "100", "--ms", "500", "--mm", "50"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "mutually exclusive" in err


def test_drive_stream_and_ms_are_mutually_exclusive(capsys):
    exit_code = cli.main(["drive", "100", "100", "stream", "--ms", "500"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "mutually exclusive" in err


def test_drive_unexpected_positional_reports_a_clear_error(capsys):
    exit_code = cli.main(["drive", "100", "100", "bogus"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "bogus" in err


def test_drive_resend_must_be_positive(capsys):
    exit_code = cli.main(["drive", "100", "100", "stream", "--resend", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--resend" in err


def test_turn_speed_must_be_positive(capsys):
    exit_code = cli.main(["turn", "90", "--speed", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--speed" in err


def test_turn_reports_a_clear_error_when_no_active_robot_config(monkeypatch, capsys):
    monkeypatch.setattr(cli.config, "load_active_robot", lambda: None)
    exit_code = cli.main(["turn", "90"])
    err = capsys.readouterr().err
    assert exit_code == 1
    assert "no active robot config" in err


# ---------------------------------------------------------------------------
# End to end against the real compiled tools/sim binary -- this ticket's
# own Testing plan ("drive --ms, drive --mm, drive stream, turn").
# `tools/sim`'s FakeMotionAdapter accepts and completes every motion
# verb (unlike the real DiffDriveAdapter), so `drive --mm` here proves
# correct wire encoding + outcome reporting on the "adapter DOES
# implement it" side of UC-002 -- the kUnknown soft-warning side is
# covered above with a scripted fake instead.
# ---------------------------------------------------------------------------

def test_drive_ms_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["drive", "100", "100", "--ms", "60", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "WHEELS_V acked" in out
    assert "done reason=" in out


def test_drive_mm_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["drive", "100", "100", "--mm", "50", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "WHEELS_X acked" in out
    assert "done reason=" in out


def test_drive_stream_end_to_end_against_sim(built_sim_binary, capsys):
    # Deliberately calls `_cmd_drive_stream()` directly against a real
    # `--sim` connection, rather than going through `cli.main()` with a
    # monkeypatched `time.sleep()`: `time.sleep` is a genuinely GLOBAL
    # name (this module's own `import time` and every other module's
    # own `import time` share the same module object), and this
    # subprocess-backed connection's own teardown
    # (`StdioTransport.close()` -> `subprocess.Popen.wait()`) uses
    # `time.sleep()` internally to poll for process exit -- patching it
    # process-wide made an EARLIER version of this test's fake
    # `KeyboardInterrupt` fire inside that unrelated polling loop
    # instead of (or in addition to) the stream loop, corrupting
    # teardown. Passing `sleep=` directly to the function under test
    # gets the same deterministic "Ctrl-C after N iterations" behavior
    # with no global side effect at all -- see `_cmd_drive_stream()`'s
    # own docstring for why its `sleep` parameter exists.
    del built_sim_binary
    ns = argparse.Namespace(sim=True, connect=None, port=None)
    conn = connection.resolve(ns)
    try:
        exit_code = cli._cmd_drive_stream(conn.session, 100, 100, 50, sleep=_raise_after(2))
    finally:
        conn.transport.close()
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "STOP acked" in out


class _FakeConnection:
    """A `connection.Connection` stand-in with no real transport at all
    -- used only to prove `cmd_drive()`'s own ROUTING decision (bare
    mode, with none of `stream_kw`/`--ms`/`--mm` given, dispatches to
    the same `_cmd_drive_stream()` path literal `stream` does), not to
    exercise `_cmd_drive_stream()`'s own behavior (already covered
    above, against a real sim connection)."""

    def __init__(self):
        self.session = object()
        self.transport = self

    def close(self) -> None:
        pass


def test_drive_bare_mode_dispatches_to_the_same_stream_path_as_literal_stream(monkeypatch):
    calls = []
    monkeypatch.setattr(
        cli, "_cmd_drive_stream",
        lambda session, left, right, resend_ms: calls.append((left, right, resend_ms)) or 0)
    monkeypatch.setattr(connection, "resolve", lambda args: _FakeConnection())

    exit_code = cli.main(["drive", "100", "100"])

    assert exit_code == 0
    assert calls == [(100, 100, 150)]  # 150 == the default --resend cadence


def test_turn_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["turn", "90", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "WHEELS_V" in out
    assert "done reason=" in out

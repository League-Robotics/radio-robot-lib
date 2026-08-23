"""tests/host/rogo/test_cli_goto_config.py -- ticket 004's `goto` and
`config get`/`config set` subcommands: argument validation and wire
encoding as fast unit tests against scripted fake transports (mirrors
test_cli_drive_turn.py's own `_ScriptedReplyTransport`/`_FakeConnection`
pattern), plus end-to-end runs against the real compiled `tools/sim`
binary for the paths it CAN exercise.

**Why `config get`/`config set`'s round-trip test uses a scripted fake,
not `tools/sim`, even though this ticket's own Testing plan says "all
against tools/sim":** `tools/sim` links `Protocol::FakeMotionAdapter`
(tests/protocol/fake_motion_adapter.h), whose own `onGet`/`onSet`
comment says outright "there is no config table here to be wrong
about" -- `onGet` always returns `false` (no config field is EVER
known) and `onSet` always returns `Result::kUnknown` (every `SET` is a
merits rejection, unconditionally, regardless of name). A
`config set <name> <value>` then `config get <name>` round trip can
therefore never succeed against `tools/sim` for ANY name -- there is no
adapter config table behind it to persist into. This is the exact same
structural situation test_cli_drive_turn.py's own module docstring
already documents for `drive --mm`'s `kUnknown` path (there, `tools/sim`
completes every motion verb instead of rejecting it -- the mismatch just
runs the other direction): the fix already established in this
directory is a scripted fake, not `tools/sim`, for the ONE path
`tools/sim`'s fake adapter structurally cannot reproduce. The
round-trip AC is about `rogo.cli`'s own wire encoding/decoding being
correct, which a scripted fake proves just as well as a live adapter
that happens to store config, and more deterministically.

The "unknown name -> err 1" path, by contrast, IS provable end to end
against the real compiled `tools/sim` binary: since `FakeMotionAdapter`
treats every name as unknown, `config set <anything> <value> --sim`
genuinely gets `err 1` back from a real process -- covered below as a
true end-to-end case, alongside `config get`'s own always-empty bare
listing against the same adapter.
"""

from __future__ import annotations

import pytest

from robot_v6.reliability import Session
from robot_v6.transport import Transport

from rogo import cli, connection


# ---------------------------------------------------------------------------
# Fake transport/connection -- no tools/sim needed for most tests below.
# Mirrors test_cli_drive_turn.py's own `_ScriptedReplyTransport`/
# `_FakeConnection` classes (each test file in this directory keeps its
# own copy rather than sharing one -- the existing convention here).
# ---------------------------------------------------------------------------

class _ScriptedReplyTransport(Transport):
    """Replays a fixed sequence of `_read_chunk()` results, one per
    call; records every line written."""

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


class _FakeConnection:
    """A `connection.Connection` stand-in wrapping an already-built
    `Session` -- lets a test drive `cmd_goto()`/`cmd_config_get()`/
    `cmd_config_set()` through `cli.main()`'s real argparse wiring
    (so every default -- `--speed`, `--arrive`, computed `--timeout`,
    etc. -- is the real one) while bypassing real target resolution."""

    def __init__(self, session: Session):
        self.session = session
        self.transport = self

    def close(self) -> None:
        pass


def _resolve_to(monkeypatch, session: Session) -> None:
    monkeypatch.setattr(connection, "resolve", lambda args: _FakeConnection(session))


# ---------------------------------------------------------------------------
# _goto_default_timeout_ms() -- the distance/speed ETA backstop used
# when `--timeout` is not given, mirroring test_cli_drive_turn.py's own
# tests for `_wheels_x_fields()`'s analogous ETA backstop.
# ---------------------------------------------------------------------------

def test_goto_default_timeout_is_3x_the_straight_line_eta():
    # hypot(300, 400) == 500mm at 200mm/s -> 2500ms ETA * 3 == 7500ms.
    assert cli._goto_default_timeout_ms(300, 400, 200) == 7500


def test_goto_default_timeout_has_a_1000ms_floor():
    assert cli._goto_default_timeout_ms(1, 1, 1000) == 1000


def test_goto_default_timeout_at_the_origin_is_just_the_floor():
    assert cli._goto_default_timeout_ms(0, 0, 200) == 1000


# ---------------------------------------------------------------------------
# goto -- wire encoding, one GO_TO_R call.
# ---------------------------------------------------------------------------

def test_goto_encodes_go_to_r_with_explicit_flags(monkeypatch, capsys):
    # `ack 1 1 stop` acks AND reports lastDone=1/stop in the SAME line
    # (protocol.md#8.8: one ack carries the single latest (lastDone,
    # reason) pair) -- this lets wait_for_done() resolve on its very
    # first check with no pump loop needed, so the test runs instantly
    # instead of spinning through a real multi-second timeout waiting
    # for a `done` that a scripted fake with no more chunks would never
    # produce.
    transport = _ScriptedReplyTransport([b"ack 1 1 stop\n"])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(
        ["goto", "300", "400", "--speed", "200", "--arrive", "10",
         "--timeout", "5000", "--sim"])

    assert transport.written == ["GO_TO_R 300 400 200 10 5000 #1"]
    assert exit_code == 0
    out = capsys.readouterr().out
    assert "done reason=stop" in out


def test_goto_computes_a_default_timeout_when_not_given(monkeypatch):
    transport = _ScriptedReplyTransport([b"ack 1 1 stop\n"])
    _resolve_to(monkeypatch, Session(transport))

    cli.main(["goto", "300", "400", "--sim"])

    assert transport.written == ["GO_TO_R 300 400 200 0 7500 #1"]


def test_goto_all_fields_are_whole_numbers_never_decimal_points(monkeypatch):
    # protocol_handler.cpp parses x/y/speed/arrive with parseInt32 and
    # timeout with parseUint32 -- ANY decimal point is a DECODE FAILURE
    # (protocol.md#8.9), not a merits rejection. Pinning this the same
    # way test_motion.py's own test_go_to_r_encodes_all_five_fields_
    # unconverted() does, but through the CLI's own int-typed argparse
    # wiring rather than calling motion.go_to_r() directly.
    transport = _ScriptedReplyTransport([b"ack 1 1 stop\n"])
    _resolve_to(monkeypatch, Session(transport))

    cli.main(["goto", "-150", "400", "--speed", "200", "--arrive", "10",
              "--timeout", "8000", "--sim"])

    line = transport.written[0]
    assert line == "GO_TO_R -150 400 200 10 8000 #1"
    assert "." not in line


# ---------------------------------------------------------------------------
# goto's kUnknown soft-warning path (STAKEHOLDER DECISION, sprint 001
# stakeholder_approval gate, reusing ticket 003's own
# _await_ack_and_err()/_print_soft_warning()): a kUnknown merits
# rejection is a warning, not a hard error -- exit 0, the outcome
# printed plainly, never a false "arrived" claim.
# ---------------------------------------------------------------------------

def test_goto_prints_a_soft_warning_and_exits_zero_on_kunknown(monkeypatch, capsys):
    transport = _ScriptedReplyTransport([b"ack 1 0 none\nerr 1 #1\n"])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(
        ["goto", "300", "400", "--timeout", "5000", "--sim"])

    assert exit_code == 0
    assert transport.written == ["GO_TO_R 300 400 200 0 5000 #1"]
    out, err = capsys.readouterr()
    assert "GO_TO_R sent (#1)" in out
    assert "kUnknown" in out or "ERR_UNKNOWN" in out
    assert "warning" in err.lower()
    assert "arrived" not in out.lower()  # never a false arrival claim


def test_goto_reports_not_acked_as_a_hard_error(monkeypatch, capsys):
    # Bypasses _await_ack_and_err()'s own real pump/timeout loop, same
    # rationale as test_cli_drive_turn.py's own analogous test: this
    # test's job is cmd_goto()'s "not acked" -> exit 1 branch, not
    # _await_ack_and_err()'s timeout behavior (already exercised
    # elsewhere against a real ack).
    monkeypatch.setattr(
        cli, "_await_ack_and_err",
        lambda session, seq_id, timeout=cli._DEFAULT_TIMEOUT: (False, None))
    transport = _ScriptedReplyTransport([])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(["goto", "100", "0", "--sim"])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "not acked" in err


# ---------------------------------------------------------------------------
# goto -- argument validation, fails fast before resolving a target.
# ---------------------------------------------------------------------------

def test_goto_speed_must_be_positive(capsys):
    exit_code = cli.main(["goto", "100", "0", "--speed", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--speed" in err


def test_goto_timeout_must_be_positive(capsys):
    exit_code = cli.main(["goto", "100", "0", "--timeout", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--timeout" in err


# ---------------------------------------------------------------------------
# goto -- end to end against the real compiled tools/sim binary.
# `tools/sim`'s `FakeMotionAdapter` completes GO_TO_R (test_motion.py's
# own module docstring: it "accepts and completes ALL SIX motion verbs
# by default"), unlike the real DiffDriveAdapter's kUnknown gap (covered
# above with a scripted fake instead, since tools/sim cannot reproduce
# that outcome) -- so this proves correct wire encoding + outcome
# reporting on the "adapter DOES implement it" side of UC-002.
# ---------------------------------------------------------------------------

def test_goto_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["goto", "300", "0", "--speed", "100", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "GO_TO_R 300 0 100" in out
    assert "done reason=" in out


# ---------------------------------------------------------------------------
# config get/set -- wire encoding and the round-trip/unknown-name paths,
# against a scripted fake (see this module's own docstring for why not
# tools/sim for the round trip specifically).
# ---------------------------------------------------------------------------

def test_config_set_encodes_name_and_value_and_reports_acked(monkeypatch, capsys):
    transport = _ScriptedReplyTransport([b"ack 1 0 none\n", b""])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(["config", "set", "wheel_control.pid_kp", "1.5", "--sim"])

    assert exit_code == 0
    assert transport.written == ["SET wheel_control.pid_kp 1.5 #1"]
    out = capsys.readouterr().out
    assert "SET wheel_control.pid_kp=1.5 acked (#1)" in out


def test_config_get_named_field_sends_a_named_get(monkeypatch):
    transport = _ScriptedReplyTransport(
        [b"ack 1 0 none\nget wheel_control.pid_kp 1.500000\n", b""])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(["config", "get", "wheel_control.pid_kp", "--sim"])

    assert exit_code == 0
    assert transport.written == ["GET wheel_control.pid_kp #1"]


def test_config_set_then_get_round_trips(monkeypatch, capsys):
    # Two separate commands, each against its OWN scripted session:
    # sharing one session/transport across two top-level `cli.main()`
    # calls doesn't work here -- `_await_ack_and_err()`'s own grace-pump
    # after SET's ack would greedily consume the NEXT scripted chunk
    # (the one meant for GET) before `config get` even runs, since a
    # canned chunk queue has no notion of "only reply once the matching
    # command was actually sent" the way a real peer would. GET's own
    # transport is scripted to hand back the SAME value SET was asked to
    # write, which is what "round-trips" means at the CLI's own
    # encode/decode boundary (see module docstring for why a live
    # `tools/sim` adapter can't stand in for this at all).
    set_transport = _ScriptedReplyTransport([b"ack 1 0 none\n"])
    _resolve_to(monkeypatch, Session(set_transport))
    set_exit = cli.main(["config", "set", "wheel_control.pid_kp", "1.5", "--sim"])

    get_transport = _ScriptedReplyTransport(
        [b"ack 1 0 none\nget wheel_control.pid_kp 1.500000\n"])
    _resolve_to(monkeypatch, Session(get_transport))
    get_exit = cli.main(["config", "get", "wheel_control.pid_kp", "--sim"])

    assert set_exit == 0
    assert get_exit == 0
    assert set_transport.written == ["SET wheel_control.pid_kp 1.5 #1"]
    assert get_transport.written == ["GET wheel_control.pid_kp #1"]
    out = capsys.readouterr().out
    assert "wheel_control.pid_kp=1.500000" in out


def test_config_get_bare_sends_a_bare_get(monkeypatch):
    transport = _ScriptedReplyTransport([
        b"ack 1 0 none\n"
        b"get wheel_control.pid_kp 1.500000\n"
        b"get wheel_control.pid_ki 0.000000\n"
        b"get wheel_control.v_min 5.000000\n",
        b"",
    ])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(["config", "get", "--sim"])

    assert exit_code == 0
    assert transport.written == ["GET #1"]


def test_config_get_bare_lists_every_field(monkeypatch, capsys):
    transport = _ScriptedReplyTransport([
        b"ack 1 0 none\n"
        b"get wheel_control.pid_kp 1.500000\n"
        b"get wheel_control.pid_ki 0.000000\n"
        b"get wheel_control.v_min 5.000000\n",
        b"",
    ])
    _resolve_to(monkeypatch, Session(transport))

    cli.main(["config", "get", "--sim"])

    out = capsys.readouterr().out
    assert "wheel_control.pid_kp=1.500000" in out
    assert "wheel_control.pid_ki=0.000000" in out
    assert "wheel_control.v_min=5.000000" in out


def test_config_get_unknown_name_is_a_clear_error_not_a_silent_success(monkeypatch, capsys):
    # protocol.md#7: an unknown GET name gets NO `get` reply line at
    # all, though the command is still acked -- this must not read as
    # a silent, field-less "success".
    transport = _ScriptedReplyTransport([b"ack 1 0 none\n", b""])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(["config", "get", "bogus.field", "--sim"])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "no such config field" in err
    assert "bogus.field" in err


def test_config_set_unknown_name_surfaces_err_1_as_a_clear_message(monkeypatch, capsys):
    transport = _ScriptedReplyTransport([b"ack 1 0 none\nerr 1 #1\n"])
    _resolve_to(monkeypatch, Session(transport))

    exit_code = cli.main(["config", "set", "bogus.field", "5.0", "--sim"])

    assert exit_code != 0  # a genuine caller mistake -- a hard error, not a warning
    out, err = capsys.readouterr()
    assert "no such config field" in err
    assert "bogus.field" in err
    assert "Traceback" not in out  # never a stack trace


def test_config_set_reports_not_acked_as_a_hard_error(monkeypatch, capsys):
    # Monkeypatches _await_ack_and_err() directly, same rationale as
    # test_cli_drive_turn.py's own analogous test: a transport with no
    # scripted replies at all would otherwise force this test to burn
    # its own wall-clock _DEFAULT_TIMEOUT (3s) in a real pump/timeout
    # loop before giving up.
    monkeypatch.setattr(
        cli, "_await_ack_and_err",
        lambda session, seq_id, timeout=cli._DEFAULT_TIMEOUT: (False, None))
    _resolve_to(monkeypatch, Session(_ScriptedReplyTransport([])))

    exit_code = cli.main(["config", "set", "wheel_control.pid_kp", "1.0", "--sim"])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "not acked" in err


def test_config_get_reports_not_acked_as_a_hard_error(monkeypatch, capsys):
    # Same rationale as test_config_set_reports_not_acked_as_a_hard_
    # error() above, for cmd_config_get()'s own ack-wait helper.
    monkeypatch.setattr(
        cli, "_await_ack_and_get_lines",
        lambda session, seq_id, timeout=cli._DEFAULT_TIMEOUT: (False, []))
    _resolve_to(monkeypatch, Session(_ScriptedReplyTransport([])))

    exit_code = cli.main(["config", "get", "wheel_control.pid_kp", "--sim"])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "not acked" in err


# ---------------------------------------------------------------------------
# config -- end to end against the real compiled tools/sim binary.
# `FakeMotionAdapter` treats every name as unknown (its own onGet/onSet
# comment: "there is no config table here to be wrong about"), so this
# genuinely exercises the "unknown name -> err 1" AC against a real
# process rather than a scripted double.
# ---------------------------------------------------------------------------

def test_config_set_unknown_name_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["config", "set", "bogus.field", "5.0", "--sim"])
    err = capsys.readouterr().err
    assert exit_code == 1
    assert "no such config field" in err


def test_config_get_bare_end_to_end_against_sim(built_sim_binary, capsys):
    del built_sim_binary
    exit_code = cli.main(["config", "get", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "no config fields" in out


# ---------------------------------------------------------------------------
# Argument parsing -- `rogo config` requires a `get`/`set` sub-subcommand.
# ---------------------------------------------------------------------------

def test_config_with_no_subcommand_is_a_usage_error():
    with pytest.raises(SystemExit) as exc_info:
        cli.main(["config"])
    assert exc_info.value.code != 0


def test_help_lists_goto_and_config(capsys):
    with pytest.raises(SystemExit) as exc_info:
        cli.main(["--help"])
    assert exc_info.value.code == 0
    out = capsys.readouterr().out
    assert "goto" in out
    assert "config" in out

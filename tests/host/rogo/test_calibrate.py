"""tests/host/rogo/test_calibrate.py -- ticket 005's manual calibration
flow (`rogo.calibrate`): the pure residual-computation core
(`compute_calibration`/`compute_distance_trial`) as fast unit tests with
no session/transport at all, the interactive trial loop against a
scripted fake transport (mirrors tests/host/rogo/test_cli_drive_turn.py's
own `_AutoAckTransport` pattern, extended here so `wait_for_done()` also
returns immediately), `rogo.cli`'s `calibrate turns`/`calibrate
distance` argument validation, and full end-to-end runs against the real
compiled `tools/sim` binary writing only to a `tmp_path` copy of
`tests/host/rogo/fixtures/gopiv.json` -- never the real
`config/robots/*.json` files (this ticket's own AC #5).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from robot_v6.reliability import Session
from robot_v6.transport import Transport

from rogo import calibrate, cli, config, connection

_FIXTURE = Path(__file__).parent / "fixtures" / "gopiv.json"


def _fixture_copy(tmp_path: Path) -> Path:
    dest = tmp_path / "gopiv.json"
    dest.write_text(_FIXTURE.read_text())
    return dest


def _scripted_input(responses: list[str]):
    """A fake `input_fn` backed by an explicit, already-known list of
    responses -- "scripted stdin" (this ticket's own Testing plan
    wording): raises `EOFError` once exhausted, exactly like real
    `input()` hitting a closed stream, rather than leaking a raw
    `StopIteration` a caller's `except EOFError` would not catch."""
    it = iter(responses)

    def _input(prompt: str = "") -> str:
        del prompt
        try:
            return next(it)
        except StopIteration:
            raise EOFError

    return _input


def _silent_print(_line: str) -> None:
    pass


# ---------------------------------------------------------------------------
# Fake transports -- no tools/sim needed for most of this module.
# ---------------------------------------------------------------------------

class _AutoCompleteTransport(Transport):
    """Immediately acks AND reports completion (`last_done` == the
    sequence id just acked) for every sequenced line written -- both
    `Session.wait_for_ack()` and `Session.wait_for_done()`
    (`calibrate.py`'s trial loop calls both, unlike
    `test_cli_drive_turn.py`'s own `_AutoAckTransport`, which never
    needed `wait_for_done()`) return immediately, keeping these unit
    tests fast with no real `tools/sim` connection."""

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
            self._pending_replies.append(f"ack {seq_id} {seq_id} done\n".encode("ascii"))

    def close(self) -> None:
        pass


class _NeverRepliesTransport(Transport):
    """Never sends any reply at all -- `wait_for_ack()` always times
    out. Used only to prove the trial loop's own "not acked" warning
    path does not crash and still asks for a measurement."""

    def __init__(self):
        super().__init__()
        self.written: list[str] = []

    def _read_chunk(self, timeout):
        del timeout
        return b""

    def _write_bytes(self, data: bytes) -> None:
        self.written.append(data.decode("ascii").rstrip("\n"))

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# compute_calibration() -- pure residual computation, no I/O, no motion.
# This is the "run N trials, collect a measured value per trial, compute
# a result" core, callable with an explicit values list -- no input(),
# no TTY (a future ticket 007 MCP tool's own need, per this ticket's
# Implementation Plan).
# ---------------------------------------------------------------------------

def test_compute_calibration_computes_mean_ratio_and_updated_value():
    result = calibrate.compute_calibration(0.9, 90.0, [99.0, 99.0, 99.0], (0.5, 1.5))
    assert result.mean_ratio == pytest.approx(1.1)
    assert result.updated_value == pytest.approx(0.99)
    assert result.rejected_reason is None
    assert len(result.samples) == 3


def test_compute_calibration_none_current_value_defaults_to_identity():
    result = calibrate.compute_calibration(None, 90.0, [90.0, 90.0, 90.0], (0.5, 1.5))
    assert result.starting_value == 1.0
    assert result.updated_value == pytest.approx(1.0)


def test_compute_calibration_fewer_than_min_trials_returns_no_update():
    result = calibrate.compute_calibration(1.0, 90.0, [91.0, 89.0], (0.5, 1.5))
    assert result.mean_ratio is None
    assert result.updated_value is None
    assert result.rejected_reason is None  # nothing to reject -- just not enough data
    assert len(result.samples) == 2


def test_compute_calibration_rejects_a_value_outside_the_sane_range():
    # AC #4: a computed value outside a defined sane range is rejected
    # with a clear message, not persisted (mirrors motion-api#2.1's own
    # trackwidth-bending caution).
    result = calibrate.compute_calibration(1.0, 90.0, [900.0, 900.0, 900.0], (0.5, 1.5))
    assert result.mean_ratio == pytest.approx(10.0)
    assert result.updated_value is None
    assert result.rejected_reason is not None
    assert "outside the sane range" in result.rejected_reason


def test_compute_calibration_drops_non_positive_measurements():
    result = calibrate.compute_calibration(1.0, 90.0, [90.0, -5.0, 0.0, 90.0, 90.0], (0.5, 1.5))
    assert len(result.samples) == 3
    assert result.mean_ratio == pytest.approx(1.0)


def test_compute_calibration_respects_a_custom_min_trials():
    result = calibrate.compute_calibration(
        1.0, 90.0, [90.0, 90.0], (0.5, 1.5), min_trials=2)
    assert result.updated_value == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# compute_distance_trial() -- the straight-line analog of
# turn_model.compute_turn(), as a pure function.
# ---------------------------------------------------------------------------

def test_compute_distance_trial_identity_scale():
    left, right, duration_ms = calibrate.compute_distance_trial(400.0, 200.0, 1.0)
    assert (left, right) == (200, 200)
    assert duration_ms == 2000


def test_compute_distance_trial_none_scale_falls_back_to_identity():
    with_none = calibrate.compute_distance_trial(400.0, 200.0, None)
    with_one = calibrate.compute_distance_trial(400.0, 200.0, 1.0)
    assert with_none == with_one


def test_compute_distance_trial_scale_below_one_takes_longer():
    _, _, duration_ms = calibrate.compute_distance_trial(400.0, 200.0, 0.8)
    assert duration_ms == 2500


def test_compute_distance_trial_non_positive_scale_falls_back_to_identity():
    with_zero = calibrate.compute_distance_trial(400.0, 200.0, 0.0)
    with_negative = calibrate.compute_distance_trial(400.0, 200.0, -1.0)
    with_one = calibrate.compute_distance_trial(400.0, 200.0, 1.0)
    assert with_zero == with_one
    assert with_negative == with_one


def test_compute_distance_trial_zero_distance_raises_value_error():
    with pytest.raises(ValueError):
        calibrate.compute_distance_trial(0.0, 200.0, 1.0)


def test_compute_distance_trial_zero_speed_raises_value_error():
    with pytest.raises(ValueError):
        calibrate.compute_distance_trial(400.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# _run_interactive_trials() -- the prompt/drive/prompt loop, against a
# fast fake transport (no tools/sim). Exercises skip/invalid/quit-early
# handling and the drive callback's own invocation count.
# ---------------------------------------------------------------------------

def _make_session() -> tuple[Session, _AutoCompleteTransport]:
    transport = _AutoCompleteTransport()
    return Session(transport), transport


def _counting_drive_one(session: Session, calls: list[int]):
    def _drive() -> tuple[int, int]:
        calls.append(1)
        from robot_v6 import motion
        seq_id = motion.wheels_v(session, 100, 100, 10)
        return seq_id, 10
    return _drive


def test_interactive_trials_collects_one_measurement_per_trial():
    session, _ = _make_session()
    calls: list[int] = []
    input_fn = _scripted_input(["", "91.0", "", "89.5", "", "90.2"])
    measured = calibrate._run_interactive_trials(
        session, 3, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=_silent_print)
    assert measured == [91.0, 89.5, 90.2]
    assert len(calls) == 3


def test_interactive_trials_q_quits_before_any_drive():
    session, _ = _make_session()
    calls: list[int] = []
    input_fn = _scripted_input(["q"])
    measured = calibrate._run_interactive_trials(
        session, 5, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=_silent_print)
    assert measured == []
    assert calls == []


def test_interactive_trials_eof_on_start_prompt_stops_early():
    session, _ = _make_session()
    calls: list[int] = []
    input_fn = _scripted_input([])  # EOF immediately
    measured = calibrate._run_interactive_trials(
        session, 5, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=_silent_print)
    assert measured == []
    assert calls == []


def test_interactive_trials_skip_does_not_record_but_continues():
    session, _ = _make_session()
    calls: list[int] = []
    input_fn = _scripted_input(["", "skip", "", "91.0"])
    measured = calibrate._run_interactive_trials(
        session, 2, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=_silent_print)
    assert measured == [91.0]
    assert len(calls) == 2  # both trials drove -- only the measurement was skipped


def test_interactive_trials_invalid_value_is_skipped_with_a_message():
    session, _ = _make_session()
    calls: list[int] = []
    printed: list[str] = []
    input_fn = _scripted_input(["", "not-a-number", "", "91.0"])
    measured = calibrate._run_interactive_trials(
        session, 2, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=printed.append)
    assert measured == [91.0]
    assert any("Invalid" in line for line in printed)


def test_interactive_trials_non_positive_value_is_skipped():
    session, _ = _make_session()
    calls: list[int] = []
    input_fn = _scripted_input(["", "-5.0", "", "91.0"])
    measured = calibrate._run_interactive_trials(
        session, 2, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=_silent_print)
    assert measured == [91.0]


def test_interactive_trials_not_acked_prints_a_warning_but_still_prompts():
    transport = _NeverRepliesTransport()
    session = Session(transport)
    calls: list[int] = []
    printed: list[str] = []
    input_fn = _scripted_input(["", "91.0"])
    measured = calibrate._run_interactive_trials(
        session, 1, "spin", "measured? ", _counting_drive_one(session, calls),
        input_fn=input_fn, print_fn=printed.append, timeout=0.05)
    assert measured == [91.0]
    assert any("not acked" in line for line in printed)


# ---------------------------------------------------------------------------
# calibrate_turns()/calibrate_distance() -- the full flow (trials ->
# compute -> report -> prompt to save), against the fast fake transport
# and a tmp_path copy of the fixture config.
# ---------------------------------------------------------------------------

def test_calibrate_turns_saves_on_confirmation(tmp_path):
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    assert cfg.rotational_slip == 1.0  # gopiv.json's own starting value

    session, _ = _make_session()
    input_fn = _scripted_input(["", "99", "", "99", "", "99", "y"])
    exit_code = calibrate.calibrate_turns(
        session, cfg, trials=3, speed_mm_s=200.0,
        input_fn=input_fn, print_fn=_silent_print)

    assert exit_code == 0
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == pytest.approx(1.1)
    assert on_disk["geometry"]["trackwidth"] == 128  # untouched


def test_calibrate_turns_declines_leaves_file_untouched(tmp_path):
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    before = path.read_text()

    session, _ = _make_session()
    input_fn = _scripted_input(["", "99", "", "99", "", "99", "n"])
    exit_code = calibrate.calibrate_turns(
        session, cfg, trials=3, speed_mm_s=200.0,
        input_fn=input_fn, print_fn=_silent_print)

    assert exit_code == 0
    assert path.read_text() == before


def test_calibrate_turns_insufficient_trials_does_not_prompt_or_write(tmp_path):
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    before = path.read_text()

    session, _ = _make_session()
    # Only 2 trials worth of scripted input -- fewer than the min-trials
    # floor. No "save?" response supplied: if the flow wrongly tried to
    # prompt for one, EOFError would propagate (input_fn is exhausted),
    # not silently succeed.
    input_fn = _scripted_input(["", "91", "", "89"])
    exit_code = calibrate.calibrate_turns(
        session, cfg, trials=2, speed_mm_s=200.0,
        input_fn=input_fn, print_fn=_silent_print)

    assert exit_code == 1
    assert path.read_text() == before


def test_calibrate_turns_rejects_out_of_range_value_and_does_not_write(tmp_path):
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    before = path.read_text()

    session, _ = _make_session()
    input_fn = _scripted_input(["", "900", "", "900", "", "900"])
    exit_code = calibrate.calibrate_turns(
        session, cfg, trials=3, speed_mm_s=200.0,
        input_fn=input_fn, print_fn=_silent_print)

    assert exit_code == 1
    assert path.read_text() == before


def test_calibrate_turns_reports_a_clear_error_with_no_trackwidth(tmp_path):
    path = tmp_path / "no_trackwidth.json"
    path.write_text(json.dumps({"identity": {"robot_name": "bare"}}))
    cfg = config.load_robot_config(path)

    session, _ = _make_session()
    printed: list[str] = []
    exit_code = calibrate.calibrate_turns(
        session, cfg, trials=3, speed_mm_s=200.0,
        input_fn=_scripted_input([]), print_fn=printed.append)

    assert exit_code == 1
    assert any("trackwidth" in line for line in printed)


def test_calibrate_distance_saves_on_confirmation(tmp_path):
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    assert cfg.distance_scale is None  # gopiv.json carries no distance_scale at all

    session, _ = _make_session()
    input_fn = _scripted_input(["", "440", "", "440", "", "440", "y"])
    exit_code = calibrate.calibrate_distance(
        session, cfg, trials=3, distance_mm=400.0, speed_mm_s=200.0,
        input_fn=input_fn, print_fn=_silent_print)

    assert exit_code == 0
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["distance_scale"] == pytest.approx(1.1)


def test_calibrate_distance_declines_leaves_file_untouched(tmp_path):
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    before = path.read_text()

    session, _ = _make_session()
    input_fn = _scripted_input(["", "440", "", "440", "", "440", "n"])
    exit_code = calibrate.calibrate_distance(
        session, cfg, trials=3, distance_mm=400.0, speed_mm_s=200.0,
        input_fn=input_fn, print_fn=_silent_print)

    assert exit_code == 0
    assert path.read_text() == before


def test_calibrate_distance_and_turns_write_back_independently(tmp_path):
    # A robot that has already been through `calibrate turns` (its
    # rotational_slip on disk is non-default) must keep that value when
    # `calibrate distance` writes distance_scale, and vice-versa --
    # `rogo.config.save_robot_config()`'s own independent-fields
    # contract (see test_config.py's own coverage of this), exercised
    # here through the calibrate flow itself.
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    session, _ = _make_session()

    calibrate.calibrate_turns(
        session, cfg, trials=3, speed_mm_s=200.0,
        input_fn=_scripted_input(["", "99", "", "99", "", "99", "y"]),
        print_fn=_silent_print)

    cfg2 = config.load_robot_config(path)
    session2, _ = _make_session()
    calibrate.calibrate_distance(
        session2, cfg2, trials=3, distance_mm=400.0, speed_mm_s=200.0,
        input_fn=_scripted_input(["", "440", "", "440", "", "440", "y"]),
        print_fn=_silent_print)

    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == pytest.approx(1.1)
    assert on_disk["geometry"]["distance_scale"] == pytest.approx(1.1)


# ---------------------------------------------------------------------------
# rogo.cli's `calibrate turns`/`calibrate distance` -- argument
# validation and dispatch wiring, fails fast before ever resolving a
# target or touching a real config file.
# ---------------------------------------------------------------------------

def test_cli_calibrate_turns_speed_must_be_positive(capsys):
    exit_code = cli.main(["calibrate", "turns", "--speed", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--speed" in err


def test_cli_calibrate_turns_trials_must_be_positive(capsys):
    exit_code = cli.main(["calibrate", "turns", "--trials", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--trials" in err


def test_cli_calibrate_turns_reports_a_clear_error_when_no_active_robot_config(
    monkeypatch, capsys
):
    monkeypatch.setattr(cli.config, "load_active_robot", lambda: None)
    exit_code = cli.main(["calibrate", "turns"])
    err = capsys.readouterr().err
    assert exit_code == 1
    assert "no active robot config" in err


def test_cli_calibrate_distance_distance_must_be_positive(capsys):
    exit_code = cli.main(["calibrate", "distance", "--distance", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--distance" in err


def test_cli_calibrate_distance_speed_must_be_positive(capsys):
    exit_code = cli.main(["calibrate", "distance", "--speed", "0"])
    err = capsys.readouterr().err
    assert exit_code == 2
    assert "--speed" in err


def test_cli_calibrate_distance_reports_a_clear_error_when_no_active_robot_config(
    monkeypatch, capsys
):
    monkeypatch.setattr(cli.config, "load_active_robot", lambda: None)
    exit_code = cli.main(["calibrate", "distance"])
    err = capsys.readouterr().err
    assert exit_code == 1
    assert "no active robot config" in err


# ---------------------------------------------------------------------------
# End to end against the real compiled tools/sim binary -- this ticket's
# own AC #5: a full manual calibration run, scripted (non-interactive)
# operator input, writing ONLY to a tmp_path copy of the fixture config.
# ---------------------------------------------------------------------------

def test_calibrate_turns_end_to_end_against_sim_writes_only_the_fixture_copy(
    built_sim_binary, tmp_path
):
    del built_sim_binary
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)

    ns = argparse.Namespace(sim=True, connect=None, port=None)
    conn = connection.resolve(ns)
    try:
        exit_code = calibrate.calibrate_turns(
            conn.session, cfg, trials=3, speed_mm_s=200.0,
            input_fn=_scripted_input(["", "99", "", "99", "", "99", "y"]),
            print_fn=_silent_print)
    finally:
        conn.transport.close()

    assert exit_code == 0
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == pytest.approx(1.1)

    # The checked-in fixture itself must never be touched.
    original = json.loads(_FIXTURE.read_text())
    assert original["geometry"]["rotational_slip"] == 1.0


def test_calibrate_distance_end_to_end_against_sim_writes_only_the_fixture_copy(
    built_sim_binary, tmp_path
):
    del built_sim_binary
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)

    ns = argparse.Namespace(sim=True, connect=None, port=None)
    conn = connection.resolve(ns)
    try:
        exit_code = calibrate.calibrate_distance(
            conn.session, cfg, trials=3, distance_mm=100.0, speed_mm_s=400.0,
            input_fn=_scripted_input(["", "110", "", "110", "", "110", "y"]),
            print_fn=_silent_print)
    finally:
        conn.transport.close()

    assert exit_code == 0
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["distance_scale"] == pytest.approx(1.1)

    original = json.loads(_FIXTURE.read_text())
    assert "distance_scale" not in original["geometry"]


def test_calibrate_turns_cli_end_to_end_against_sim(
    built_sim_binary, tmp_path, monkeypatch
):
    # Exercises the full rogo.cli plumbing (argparse -> cmd_calibrate_turns
    # -> rogo.calibrate.calibrate_turns) against a real tools/sim
    # connection, with `config.load_active_robot()` monkeypatched to the
    # fixture copy so the real config/robots/active_robot.json is never
    # consulted or touched.
    del built_sim_binary
    path = _fixture_copy(tmp_path)
    cfg = config.load_robot_config(path)
    monkeypatch.setattr(cli.config, "load_active_robot", lambda: cfg)
    monkeypatch.setattr(
        "builtins.input", _scripted_input(["", "99", "", "99", "", "99", "y"]))

    exit_code = cli.main(["calibrate", "turns", "--trials", "3", "--sim"])

    assert exit_code == 0
    on_disk = json.loads(path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == pytest.approx(1.1)

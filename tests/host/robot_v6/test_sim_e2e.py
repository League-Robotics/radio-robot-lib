"""tests/host/robot_v6/test_sim_e2e.py -- end to end against the REAL
compiled tools/sim binary over --stdio: this is the one test in this
directory that exercises the whole stack (codec + StdioTransport +
Session) against an actual separate process talking actual bytes over
an actual pipe, with the sim's own wall-clock `--period` cadence
driving telemetry and motion progress -- nothing here is test-shimmed
or in-process. See inprocess_transport.py's own docstring for why the
OTHER tests in this directory deliberately do NOT do this (determinism
under tight control, not realism, is what they need).
"""

from __future__ import annotations

import subprocess
import time

import pytest

from robot_v6.reliability import Session
from robot_v6.transport import StdioTransport


@pytest.fixture
def sim_transport(sim_binary):
    t = StdioTransport([str(sim_binary), "--stdio", "--period", "10"])
    yield t
    t.close()


def _read_until(transport, predicate, timeout=5.0):
    """Poll `read_lines()` until `predicate` is true of the accumulated
    lines, or `timeout` elapses. Returns the accumulated lines."""
    deadline = time.monotonic() + timeout
    lines: list[str] = []
    while time.monotonic() < deadline:
        lines.extend(transport.read_lines(0.2))
        if predicate(lines):
            return lines
    return lines


def test_connect_gets_the_real_banner(sim_transport):
    lines = _read_until(sim_transport, lambda ls: len(ls) >= 1)
    assert lines, "no banner arrived from the sim within the timeout"
    assert lines[0] == "device NEZHA2 robot sim SIMHOST0001"


def test_drive_a_motion_and_observe_telemetry_and_completion_via_ack(sim_transport):
    _read_until(sim_transport, lambda ls: len(ls) >= 1)  # consume the banner

    seen_verbs: set[str] = set()
    session = Session(sim_transport, on_reply=lambda reply: seen_verbs.add(reply.verb))

    seq_id = session.send("WHEELS_V", 100, 100, 60)
    assert session.wait_for_ack(seq_id, timeout=3.0), "the sim never acked WHEELS_V"

    # The sim's own --period 10 cadence drives FakeMotionAdapter::step()
    # and emitTelemetry() on its own -- no test-side step() call at all,
    # unlike the in-process reliability tests. wait_for_done() must
    # observe the completion arrive via the ack/nack piggyback for real.
    done = session.wait_for_done(seq_id, timeout=5.0)
    assert done is not None, "lastDone never reached the commanded id in time"
    assert done.id == seq_id
    assert done.reason in ("stop", "timeout")

    assert "thdr" in seen_verbs, "must have seen at least one telemetry header"
    assert "t" in seen_verbs, "must have seen at least one telemetry frame"


def test_estop_reaches_the_real_sim_and_is_answered(sim_transport):
    _read_until(sim_transport, lambda ls: len(ls) >= 1)  # consume the banner

    session = Session(sim_transport)
    session.send_unsequenced("ESTOP")
    replies = _read_until(sim_transport, lambda ls: "estop" in ls)
    assert "estop" in replies


def test_sim_shuts_down_cleanly_on_stdin_eof(sim_binary):
    transport = StdioTransport([str(sim_binary), "--stdio", "--period", "10"])
    try:
        _read_until(transport, lambda ls: len(ls) >= 1)  # consume the banner

        session = Session(transport)
        seq_id = session.send("WHEELS_V", 100, 100, 5000)
        assert session.wait_for_ack(seq_id, timeout=3.0)

        # Closing stdin is EOF from the sim's own point of view -- its
        # documented clean-shutdown trigger (tools/sim/README.md): it
        # must feed a synthetic ESTOP through its own handler and exit,
        # not hang or leave the motion "running" forever.
        transport.process.stdin.close()
        try:
            exit_code = transport.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            transport.process.kill()
            pytest.fail("sim did not exit within 3s of stdin EOF")
        assert exit_code == 0
    finally:
        transport.close()  # idempotent even though stdin is already closed

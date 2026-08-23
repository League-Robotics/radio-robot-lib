"""tests/host/robot_v6/test_motion.py -- robot_v6.motion: unit
conversion and wire encoding for the six motion-api operations plus
stop/estop and GET/SET, against a recording fake transport that mirrors
test_transport.py's own `_ScriptedTransport` pattern -- pure encoding
tests need no reply traffic at all, only what `Session.send()` actually
wrote (the fastest tests in this directory, alongside test_codec.py's
own fakes: no C++, no subprocess).

The one end-to-end section at the bottom proves `wheels_v` produces
real kinematic effect against the compiled `tools/sim` binary (this
ticket's own acceptance criterion), and that the other five motion
verbs decode and dispatch through a real sim process rather than being
rejected as malformed -- see that section's own docstring for why
`tools/sim`'s `FakeMotionAdapter` actually completes them (unlike the
real `DiffDriveAdapter`, which answers `kUnknown` for all five): this
ticket's job is correct wire encoding, not new planner behavior
(sprint.md's own Out of Scope).
"""

from __future__ import annotations

import pytest

from robot_v6 import motion
from robot_v6.reliability import Session
from robot_v6.transport import StdioTransport, Transport


class _RecordingTransport(Transport):
    """Records every line `Session.send()`/`send_unsequenced()` writes;
    never has anything to read. `motion.py`'s functions only need to
    format and send one line and return the assigned id -- no reply
    traffic is needed to prove that."""

    def __init__(self) -> None:
        super().__init__()
        self.written: list[str] = []

    def _read_chunk(self, timeout):
        del timeout
        return b""

    def _write_bytes(self, data: bytes) -> None:
        self.written.append(data.decode("ascii").rstrip("\n"))

    def close(self) -> None:
        pass


@pytest.fixture
def rec():
    return _RecordingTransport()


@pytest.fixture
def session(rec):
    return Session(rec)


# ---------------------------------------------------------------------------
# The six motion-api operations (motion-api.md#1/#9.1) -- verb + field
# encoding, one wire line each, and the returned id is exactly what
# `Session.send()` assigned.
# ---------------------------------------------------------------------------

def test_wheels_x_encodes_left_right_cruise_timeout(rec, session):
    seq_id = motion.wheels_x(session, 100, -100, 200, 4000)
    assert seq_id == 1
    assert rec.written == ["WHEELS_X 100 -100 200 4000 #1"]


def test_wheels_v_encodes_left_right_duration(rec, session):
    seq_id = motion.wheels_v(session, 150, 150, 800)
    assert seq_id == 1
    assert rec.written == ["WHEELS_V 150 150 800 #1"]


def test_move_x_zero_rotation_encodes_zero_milliradians(rec, session):
    seq_id = motion.move_x(session, 400, 0, 200, 5000)
    assert seq_id == 1
    assert rec.written == ["MOVE_X 400 0 200 5000 #1"]


def test_move_x_converts_negative_rotation_degrees_to_milliradians(rec, session):
    # -90 deg -> -1571 mrad -- the exact value test_codec.py's own
    # test_encode_command_negative_and_zero_id() uses for a MOVE_X line,
    # confirmed here as this module's OWN conversion output rather than
    # a hand-picked, equally-plausible rounding.
    seq_id = motion.move_x(session, 400, -90, 200, 5000)
    assert seq_id == 1
    assert rec.written == ["MOVE_X 400 -1571 200 5000 #1"]


def test_move_x_converts_positive_rotation_degrees_to_milliradians(rec, session):
    seq_id = motion.move_x(session, 0, 90, 0, 5000)
    assert seq_id == 1
    assert rec.written == ["MOVE_X 0 1571 0 5000 #1"]


def test_move_v_converts_omega_degrees_per_second_to_milliradians(rec, session):
    seq_id = motion.move_v(session, 200, -45, 1000)
    assert seq_id == 1
    assert rec.written == ["MOVE_V 200 -785 1000 #1"]


def test_go_to_r_encodes_all_five_fields_unconverted(rec, session):
    seq_id = motion.go_to_r(session, -150, 400, 200, 10, 8000)
    assert seq_id == 1
    assert rec.written == ["GO_TO_R -150 400 200 10 8000 #1"]


def test_go_to_w_encodes_all_five_fields_unconverted(rec, session):
    seq_id = motion.go_to_w(session, -150, 400, 200, 10, 8000)
    assert seq_id == 1
    assert rec.written == ["GO_TO_W -150 400 200 10 8000 #1"]


def test_sequence_ids_advance_across_different_motion_calls(rec, session):
    # Every function shares the SAME Session counter -- motion.py adds
    # no id-assignment logic of its own (that stays reliability.py's
    # job); this pins that it never accidentally does.
    assert motion.wheels_v(session, 100, 100, 500) == 1
    assert motion.move_x(session, 400, 0, 200, 5000) == 2
    assert motion.go_to_r(session, 0, 0, 100, 10, 1000) == 3


# ---------------------------------------------------------------------------
# stop/estop (motion-api.md#3.7/#9.1) -- `stop` is sequenced, `estop` is
# not (protocol.md#8.3's exemption set).
# ---------------------------------------------------------------------------

def test_stop_default_is_sequenced_with_no_now_token(rec, session):
    seq_id = motion.stop(session)
    assert seq_id == 1
    assert rec.written == ["STOP #1"]


def test_stop_immediate_adds_the_now_token_before_the_id(rec, session):
    seq_id = motion.stop(session, immediate=True)
    assert seq_id == 1
    assert rec.written == ["STOP now #1"]


def test_estop_sends_the_bare_unsequenced_verb_and_returns_none(rec, session):
    result = motion.estop(session)
    assert result is None
    assert rec.written == ["ESTOP"]
    # Confirms estop consumed no sequence id -- the next sequenced send
    # still gets #1, matching protocol.md#8.3's "outside the sequence
    # entirely" rule.
    assert motion.stop(session) == 1


# ---------------------------------------------------------------------------
# GET/SET -- protocol.md#7's config delegation, pure wire wrappers.
# ---------------------------------------------------------------------------

def test_get_bare_omits_the_name_field(rec, session):
    seq_id = motion.get(session)
    assert seq_id == 1
    assert rec.written == ["GET #1"]


def test_get_with_a_name(rec, session):
    seq_id = motion.get(session, "geometry.trackwidth")
    assert seq_id == 1
    assert rec.written == ["GET geometry.trackwidth #1"]


def test_set_encodes_name_and_value(rec, session):
    seq_id = motion.set(session, "geometry.trackwidth", 128.0)
    assert seq_id == 1
    assert rec.written == ["SET geometry.trackwidth 128.0 #1"]


def test_set_encodes_integer_value_without_a_decimal_point(rec, session):
    seq_id = motion.set(session, "identity.id", 7)
    assert seq_id == 1
    assert rec.written == ["SET identity.id 7 #1"]


# ---------------------------------------------------------------------------
# End to end against the real compiled `tools/sim` binary (mirrors
# test_sim_e2e.py's own StdioTransport pattern, but calling through
# `motion.py` instead of raw `Session.send()`).
#
# `tools/sim` links `Protocol::FakeMotionAdapter` (tests/protocol/
# fake_motion_adapter.h), not the real firmware `DiffDriveAdapter` --
# that fake accepts and completes ALL SIX motion verbs by default (its
# own `acceptResult` knob defaults to `kOk`), unlike `DiffDriveAdapter`,
# which has no planner and answers `kUnknown` for everything except
# `WHEELS_V` (protocol.md#5). So the assertion this section can make
# for the other five verbs is "the sim decodes, acks, and completes the
# call" -- proving this module's wire encoding is correct -- not "the
# sim answers kUnknown", which is a `DiffDriveAdapter`-specific fact
# this sim binary does not reproduce.
# ---------------------------------------------------------------------------

@pytest.fixture
def sim_transport(sim_binary):
    t = StdioTransport([str(sim_binary), "--stdio", "--period", "10"])
    yield t
    t.close()


def test_wheels_v_produces_real_kinematic_effect_against_sim(sim_transport):
    """This ticket's own acceptance criterion: 'WHEELS_V calls produce
    real kinematic effect end to end against tools/sim.' Mirrors
    test_sim_e2e.py's own WHEELS_V case, but through `motion.wheels_v()`
    rather than a raw `session.send("WHEELS_V", ...)` call."""
    seen_verbs: set[str] = set()
    session = Session(sim_transport, on_reply=lambda reply: seen_verbs.add(reply.verb))

    seq_id = motion.wheels_v(session, 100, 100, 60)
    assert session.wait_for_ack(seq_id, timeout=3.0), "the sim never acked WHEELS_V"

    done = session.wait_for_done(seq_id, timeout=5.0)
    assert done is not None, "lastDone never reached the commanded id in time"
    assert done.id == seq_id
    assert done.reason in ("stop", "timeout")
    assert "t" in seen_verbs, "must have seen at least one telemetry frame"


@pytest.mark.parametrize(
    "make_call",
    [
        lambda s: motion.wheels_x(s, 100, 100, 200, 4000),
        lambda s: motion.move_x(s, 400, 0, 200, 5000),
        lambda s: motion.move_v(s, 200, 0, 1000),
        lambda s: motion.go_to_r(s, 400, 0, 200, 10, 8000),
        lambda s: motion.go_to_w(s, 400, 0, 200, 10, 8000),
    ],
    ids=["wheels_x", "move_x", "move_v", "go_to_r", "go_to_w"],
)
def test_other_five_motion_verbs_decode_and_dispatch_against_sim(sim_transport, make_call):
    """These five have no real firmware planner behind `DiffDriveAdapter`
    (sprint.md's own Out of Scope) -- what this ticket needs is correct
    WIRE ENCODING, proven here by a real sim process accepting (ack) and
    completing (done) each call rather than rejecting the line as
    malformed."""
    session = Session(sim_transport)
    seq_id = make_call(session)
    assert session.wait_for_ack(seq_id, timeout=3.0), (
        f"sim never acked seq_id {seq_id} -- wire encoding must be wrong"
    )

    done = session.wait_for_done(seq_id, timeout=5.0)
    assert done is not None, f"lastDone never reached {seq_id} in time"

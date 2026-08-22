"""tests/host/robot_v6/test_reliability.py -- robot_v6.reliability.Session:
the HOST half of the reliability layer, exercised against the REAL
ProtocolHandler + FakeMotionAdapter running in-process (see
inprocess_transport.py's own docstring for why in-process rather than a
real sim subprocess: deterministic step() pacing, no wall clock).

The stakeholder's own acceptance scenario is
test_dropped_command_in_a_square_tour_is_nacked_and_resent_and_completes
below. Read reliability.py's own module docstring FIRST -- it documents
a real, non-obvious interaction this test file's own development
surfaced: pipelining more than one MOTION command ahead of its own
completion, against an adapter with no queue (this repo's own
FakeMotionAdapter/DiffDriveAdapter, docs/design/protocol.md S5.1), lets
a later command's dispatch silently clobber an earlier one's still-
running motion before it can finish -- including inside an AUTOMATIC
resend burst, since `_maybe_resend_from()` fires a whole backlog with
no pacing between lines. `test_pipelining_two_motions_...` below pins
that down in isolation; the flagship test avoids it entirely by
probing recovery with STATUS (which cannot clobber a motion) rather
than with a second motion command.
"""

from __future__ import annotations

import pytest

from robot_v6.reliability import PendingBufferFull, Session

from inprocess_transport import InProcessTransport
from lossy_transport import LossyTransport


@pytest.fixture
def inner(fake_motion_lib):
    t = InProcessTransport(fake_motion_lib)
    yield t
    t.close()


def _drive_and_settle(inner: InProcessTransport, session: Session, seq_id: int,
                       *, steps: int = 1) -> None:
    """Confirm `seq_id` is acked, then step its motion to completion.
    `InProcessTransport` has no clock of its own (unlike a real sim
    process's --period cadence), so nothing makes a just-updated
    lastDone/reason wire-visible on its own -- this helper calls
    `emit_telemetry()` after every step, mirroring what a real robot's
    own periodic tick does for free (docs/design/protocol.md S8.5)."""
    assert session.wait_for_ack(seq_id, timeout=2.0), f"#{seq_id} was never acked"
    for _ in range(steps):
        inner.step()
        inner.emit_telemetry()
        session.pump(0.2)
    assert session.last_done == seq_id, (
        f"expected last_done == {seq_id}, got {session.last_done}")


# ---------------------------------------------------------------------------
# Sequencing basics: ids increment from 1, send() never blocks
# (pipelining), a cumulative ack retires every earlier id in one shot,
# and the pending buffer has a defined, bounded failure mode.
# ---------------------------------------------------------------------------

def test_ids_increment_from_one(inner):
    session = Session(inner)
    assert session.send("STATUS") == 1
    assert session.send("STATUS") == 2
    assert session.send("STATUS") == 3


def test_send_never_blocks_pipelining_multiple_status_probes(inner):
    # STATUS has no motion side effect at all, so pipelining several of
    # them ahead of any reply is unambiguously safe on a queue-less
    # adapter -- the cleanest vehicle for a PURE pipelining/cumulative-
    # ack test, independent of the motion-clobber question entirely.
    session = Session(inner)
    ids = [session.send("STATUS") for _ in range(5)]
    assert ids == [1, 2, 3, 4, 5]
    assert session.pending_count == 5, "nothing should have been retired yet"

    session.pump(0.5)
    assert session.highest_acked == 5, "a cumulative ack retires every earlier id"
    assert session.pending_count == 0


def test_pending_buffer_full_raises_and_pumping_frees_room(inner):
    session = Session(inner, max_pending=3)
    session.send("STATUS")
    session.send("STATUS")
    session.send("STATUS")
    with pytest.raises(PendingBufferFull):
        session.send("STATUS")

    session.pump(0.5)  # the three STATUS probes get cumulatively acked
    assert session.pending_count == 0
    assert session.send("STATUS") == 4, "room must have freed up after pumping"


# ---------------------------------------------------------------------------
# A single motion command: send -> ack -> step -> lastDone/reason.
# ---------------------------------------------------------------------------

def test_single_motion_command_acks_then_reports_done(inner):
    session = Session(inner)
    inner.set_steps_to_complete(2)

    seq_id = session.send("WHEELS_V", 100, 100, 500)
    assert session.wait_for_ack(seq_id, timeout=2.0)
    assert session.last_done == 0, "not done after just the ack"

    inner.step()
    inner.emit_telemetry()
    session.pump(0.2)
    assert session.last_done == 0, "1/2 steps -- not done yet"

    inner.step()  # completes on this step
    inner.emit_telemetry()
    session.pump(0.2)
    assert session.last_done == seq_id
    assert session.last_done_reason == "stop"


# ---------------------------------------------------------------------------
# THE FLAGSHIP SCENARIO. Stakeholder, verbatim: "If you're driving a
# square and you've got eight movements you send, and you lose a turn,
# the whole square is wrong. The best thing to do there is to NAK and
# resend from that point on."
#
# Each leg is driven to full completion (ack AND lastDone) before the
# next is SENT. When leg 3 is lost, recovery is triggered by probing
# with STATUS rather than blindly sending leg 4 -- a STATUS probe
# cannot clobber leg 3's motion once the automatic resend re-delivers
# it (only another motion verb could -- see the paired characterization
# test below), so this is the safe way to surface the nack. The
# result satisfies the dispatch's own three-part acceptance criterion
# in the strongest sense: the host detects the nack, resends from the
# missing id, and every leg's motion genuinely completes, in order,
# exactly once.
# ---------------------------------------------------------------------------

def test_dropped_command_in_a_square_tour_is_nacked_and_resent_and_completes(inner):
    transport = LossyTransport(inner, drop_outbound={3})
    session = Session(transport)
    inner.set_steps_to_complete(1)

    legs = [(100, 100), (100, -100), (150, 150), (150, -150)]  # 4 of the "8"

    id1 = session.send("WHEELS_V", *legs[0], 500)
    _drive_and_settle(inner, session, id1)
    assert session.last_done == id1

    id2 = session.send("WHEELS_V", *legs[1], 500)
    _drive_and_settle(inner, session, id2)
    assert session.last_done == id2

    # Leg 3 -- DROPPED before it ever reaches the fake robot.
    id3 = session.send("WHEELS_V", *legs[2], 500)
    assert transport.dropped_outbound, "leg 3's own line must have been dropped"
    assert not session.wait_for_ack(id3, timeout=0.3), (
        "leg 3 was never delivered -- nothing can ack it yet")
    assert inner.active() is False, "leg 3 must not have started running"

    # Probe with STATUS, not the next motion -- this surfaces the nack
    # (id > expectedNext_) without risking the automatic resend's OWN
    # burst clobbering leg 3's motion the instant it restarts.
    probe_id = session.send("STATUS")
    session.pump(0.3)  # observe nack(3) and let Session auto-resend id3+probe_id

    assert inner.active_id() == id3, (
        "the automatic resend must have reached the fake robot with "
        "leg 3's OWN id, and the STATUS probe resent alongside it must "
        "not have disturbed it (STATUS has no motion side effect)")

    _drive_and_settle(inner, session, id3)
    assert session.last_done == id3, "leg 3 must complete before leg 4 is even sent"
    assert session.wait_for_ack(probe_id, timeout=1.0)

    # Leg 4, the tour's last movement, now runs normally.
    id4 = session.send("WHEELS_V", *legs[3], 500)
    _drive_and_settle(inner, session, id4)

    assert session.last_done == id4
    assert session.highest_acked == id4
    assert session.pending_count == 0


def test_pipelining_two_motions_past_an_unpaced_resend_lets_the_later_one_clobber_the_earlier(
        inner):
    """The characterization the module docstring promises: if the
    RECOVERY probe is a second MOTION command instead of STATUS -- i.e.
    both the lost command and its follow-up are still pending when the
    nack fires -- FakeMotionAdapter's own lack of a queue means the
    automatic resend's LAST line to arrive is the one left "active";
    an EARLIER one in the same burst can be dispatched and acked and
    then immediately overwritten before it ever gets a step(). This is
    not a Session bug: the ACK stream is still perfectly correct (every
    id acked in order, `highest_acked` tracks correctly) -- it is an
    emergent property of pairing an automatic multi-item resend with a
    queue-less adapter, which is exactly why the flagship test above
    probes with STATUS instead."""
    transport = LossyTransport(inner, drop_outbound={1})
    session = Session(transport)
    inner.set_steps_to_complete(5)  # long enough that neither leg would
                                     # finish before the next line lands

    id1 = session.send("WHEELS_V", 100, 100, 500)   # DROPPED outbound
    id2 = session.send("WHEELS_V", 200, 200, 500)   # delivered -> triggers nack(1)
    assert transport.dropped_outbound

    session.pump(0.3)  # nack(1) arrives on id2's own reply; Session
                        # resends BOTH id1 and id2, back to back, with
                        # no step() in between -- id1 dispatches and is
                        # immediately overwritten by id2's own dispatch.
    assert inner.active_id() == id2, (
        "id2's own resend clobbers id1's still-just-started motion "
        "before id1 ever advances -- the documented characterization")

    inner.step()
    assert session.wait_for_ack(id2, timeout=1.0)
    assert session.highest_acked == 2, (
        "the ACK stream is still correct even though id1's motion "
        "effect was superseded -- both ids were, individually, "
        "delivered, decoded, and acked in order")


# ---------------------------------------------------------------------------
# "A dropped ack self-heals via a later cumulative ack."
# ---------------------------------------------------------------------------

def test_dropped_ack_self_heals_via_a_later_cumulative_ack(inner):
    transport = LossyTransport(inner, drop_inbound={1})  # leg 1's own ack
    session = Session(transport)
    inner.set_steps_to_complete(1)

    id1 = session.send("WHEELS_V", 100, 100, 500)
    session.pump(0.3)
    assert transport.dropped_inbound, "leg 1's own ack must have been dropped"
    assert session.highest_acked == 0, "the dropped ack must not have been seen"
    assert session.pending_count == 1, "id1 stays buffered -- nothing retired it"

    # Pipeline id2 without ever having seen id1's own ack -- exactly
    # the scenario a cumulative scheme exists for.
    id2 = session.send("WHEELS_V", 100, -100, 500)
    session.pump(0.3)

    assert session.highest_acked == id2, "id2's cumulative ack retires id1 too"
    assert session.pending_count == 0


# ---------------------------------------------------------------------------
# "A dropped nack self-heals because the next command re-triggers one."
# ---------------------------------------------------------------------------

def test_dropped_nack_self_heals_via_a_later_commands_own_nack(inner):
    # Inbound line #1 is leg1's ack, #2 is leg2's ack, #3 is the FIRST
    # nack(3) (triggered by the STATUS probe below) -- THAT is the one
    # dropped; #4 is the SECOND nack(3), from a further probe, which
    # must get through and trigger the resend on its own.
    transport = LossyTransport(inner, drop_outbound={3}, drop_inbound={3})
    session = Session(transport)
    inner.set_steps_to_complete(1)

    id1 = session.send("WHEELS_V", 100, 100, 500)
    assert session.wait_for_ack(id1, timeout=2.0)

    id2 = session.send("WHEELS_V", 100, -100, 500)
    assert session.wait_for_ack(id2, timeout=2.0)

    id3 = session.send("WHEELS_V", 150, 150, 500)          # DROPPED outbound
    assert transport.dropped_outbound

    # Probe with STATUS (not a motion) both times, exactly as the
    # flagship test does, so this test is only about ack/nack recovery.
    session.send("STATUS")
    session.pump(0.3)
    assert transport.dropped_inbound, "the first nack(3) must have been dropped"
    assert session.pending_count == 2, "no resend must have happened yet"
    assert inner.active_id() == id2, (
        "leg 2 (never stepped to completion in this test) must still be "
        "the active motion -- leg 3 must not have run at all")

    # A further, unrelated well-formed command re-triggers a FRESH
    # nack(3) (docs/design/protocol.md S8.1: "every subsequent
    # command... is discarded and nacked") -- this one gets through.
    session.send("STATUS")
    session.pump(0.3)

    assert inner.active_id() == id3, "the resend must now have reached the adapter"
    _drive_and_settle(inner, session, id3)
    assert session.last_done == id3


# ---------------------------------------------------------------------------
# ESTOP gets through, and is answered `estop`, while the stream is
# stalled on a gap.
# ---------------------------------------------------------------------------

def test_estop_gets_through_while_the_stream_is_stalled_on_a_gap(inner):
    transport = LossyTransport(inner, drop_outbound={1})  # leg 1 lost -> gap at #1
    session = Session(transport)

    session.send("WHEELS_V", 100, 100, 500)  # id1 -- DROPPED
    session.send("STATUS")                    # id2 -- delivered, triggers nack(1)
    replies = session.pump(0.3)
    assert any(r.verb == "nack" for r in replies), "the gap must be visible"

    session.send_unsequenced("ESTOP")
    replies = session.pump(0.3)
    assert any(r.verb == "estop" for r in replies), (
        "ESTOP must answer even while #1 is an outstanding gap "
        "(docs/design/protocol.md S8.3)")

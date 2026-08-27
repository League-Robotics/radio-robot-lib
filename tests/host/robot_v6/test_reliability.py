"""tests/host/robot_v6/test_reliability.py -- robot_v6.reliability.Session:
the HOST half of the reliability layer, exercised against the REAL
ProtocolHandler + FakeMotionAdapter running in-process (see
inprocess_transport.py's own docstring for why in-process rather than a
real sim subprocess: deterministic step() pacing, no wall clock).

The stakeholder's own acceptance scenario is
test_dropped_command_in_a_square_tour_is_nacked_and_resent_and_completes
below: eight motion commands (four legs, four turns), one dropped mid-
sequence, nacked, resent, and every one of the eight completing exactly
once, in arrival order.

**Corrected 2026-08-22** -- an earlier version of this file reported the
opposite as a protocol design defect: that pipelining a second MOTION
command ahead of the first one's own completion let the later one
silently clobber the earlier one's still-running motion, because
FakeMotionAdapter had no queue. Stakeholder, verbatim, on that report:
"The system absolutely does give you ordered execution. The Reliability
Layer's job is to get commands to the Motion Layer. The Motion Layer
will execute them in order... You have to explicitly replace things."
That was right, and the bug was in the test double, not the protocol --
FakeMotionAdapter (tests/protocol/fake_motion_adapter.h) now owns a real
FIFO motion queue: a command arriving while one is active QUEUES behind
it and later runs to its own completion, in turn. Every test below,
including the one that used to be named
`test_pipelining_two_motions_past_an_unpaced_resend_lets_the_later_one_clobber_the_earlier`,
now demonstrates queuing/ordering rather than working around its
absence.
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
    Nothing makes a just-updated lastDone/reason wire-visible on its
    own any more (2026-08-26, docs/design/protocol.md S8.5: telemetry
    carries no reliability line, and an ack/nack is only ever a direct
    reply) -- so this helper POLLS after every step: a sequenced STATUS
    whose own ack carries the fresh (lastDone, reason) pair. NOTE this
    consumes a sequence id (and an outbound line) per step -- tests
    using LossyTransport index-based drops must count these."""
    assert session.wait_for_ack(seq_id, timeout=2.0), f"#{seq_id} was never acked"
    for _ in range(steps):
        inner.step()
        session.send("STATUS")
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
    session.send("STATUS")  # poll: completion only rides a reply (S8.5)
    session.pump(0.2)
    assert session.last_done == 0, "1/2 steps -- not done yet"

    inner.step()  # completes on this step
    session.send("STATUS")
    session.pump(0.2)
    assert session.last_done == seq_id
    assert session.last_done_reason == "stop"


# ---------------------------------------------------------------------------
# THE FLAGSHIP SCENARIO. Stakeholder, verbatim: "If you're driving a
# square and you've got eight movements you send, and you lose a turn,
# the whole square is wrong. The best thing to do there is to NAK and
# resend from that point on." Eight movements, four legs and four turns
# -- the stakeholder's own example, spelled out literally rather than
# reduced to four.
#
# This is the REAL scenario now, not a STATUS-probe workaround: recovery
# from the drop is triggered by sending the very NEXT MOTION COMMAND
# (movement 4), not a side probe -- FakeMotionAdapter's real FIFO queue
# means that command QUEUES behind movement 3's own resend instead of
# clobbering it, exactly as the stakeholder insisted the system already
# guarantees. Movements 5-8 are pipelined in pairs (sent before the
# first of the pair has even been stepped) purely to keep exercising the
# queue rather than pacing every single command one at a time. The
# result satisfies the dispatch's own three-part acceptance criterion in
# the strongest sense: the host detects the nack, resends from the
# missing id, and all eight movements genuinely complete, in order,
# exactly once, each producing its own lastDone/lastDoneReason.
# ---------------------------------------------------------------------------

def test_dropped_command_in_a_square_tour_is_nacked_and_resent_and_completes(inner):
    # Outbound line 5 is MOVEMENT 3: lines 1-4 are mov1, its settle
    # poll's STATUS, mov2, and ITS settle poll's STATUS (2026-08-26:
    # _drive_and_settle polls per step now that telemetry carries no
    # reliability line, S8.5 -- each poll is an outbound line too).
    transport = LossyTransport(inner, drop_outbound={5})
    session = Session(transport)
    inner.set_steps_to_complete(1)

    # Four legs and four turns -- all eight of the stakeholder's own
    # "eight movements" example, each with distinct left/right speeds so
    # a reordering or aliasing bug would be visible, not just "some
    # WHEELS_V ran".
    movements = [
        (100, 100), (100, -100), (150, 150), (150, -150),
        (200, 200), (200, -200), (250, 250), (250, -250),
    ]

    id1 = session.send("WHEELS_V", *movements[0], 500)
    _drive_and_settle(inner, session, id1)
    assert session.last_done == id1

    id2 = session.send("WHEELS_V", *movements[1], 500)
    _drive_and_settle(inner, session, id2)
    assert session.last_done == id2

    # Movement 3 -- DROPPED before it ever reaches the fake robot.
    id3 = session.send("WHEELS_V", *movements[2], 500)
    assert transport.dropped_outbound, "movement 3's own line must have been dropped"
    assert not session.wait_for_ack(id3, timeout=0.3), (
        "movement 3 was never delivered -- nothing can ack it yet")
    assert inner.active() is False, "movement 3 must not have started running"

    # Recover with the NEXT REAL MOTION COMMAND, not a STATUS dodge --
    # sending it surfaces the nack (id > expectedNext_) exactly the same
    # way a probe would, and once the automatic resend restarts movement
    # 3, movement 4's own resend QUEUES behind it instead of clobbering
    # it.
    id4 = session.send("WHEELS_V", *movements[3], 500)
    session.pump(0.3)  # observe nack(3); Session auto-resends id3 and id4

    assert inner.active_id() == id3, "the resend must restart movement 3 with its own id"

    _drive_and_settle(inner, session, id3)
    assert session.last_done == id3
    assert inner.active_id() == id4, (
        "movement 4 must now be running -- QUEUED behind movement 3, "
        "never clobbering it")
    _drive_and_settle(inner, session, id4)
    assert session.last_done == id4

    # Movements 5-8: pipelined two at a time (sent before the first of
    # each pair is even stepped) to keep proving the queue works, not
    # just recovering from a drop.
    id5 = session.send("WHEELS_V", *movements[4], 500)
    id6 = session.send("WHEELS_V", *movements[5], 500)
    session.pump(0.3)
    assert inner.active_id() == id5, "movement 5 runs first"
    _drive_and_settle(inner, session, id5)
    assert session.last_done == id5
    assert inner.active_id() == id6, "movement 6 QUEUED behind movement 5"
    _drive_and_settle(inner, session, id6)
    assert session.last_done == id6

    id7 = session.send("WHEELS_V", *movements[6], 500)
    id8 = session.send("WHEELS_V", *movements[7], 500)
    session.pump(0.3)
    assert inner.active_id() == id7, "movement 7 runs first"
    _drive_and_settle(inner, session, id7)
    assert session.last_done == id7
    assert inner.active_id() == id8, "movement 8 QUEUED behind movement 7"
    _drive_and_settle(inner, session, id8)
    assert session.last_done == id8

    # lastDone advanced MONOTONICALLY through all eight movements, each
    # producing its own completion -- exactly what "the Motion Layer
    # will execute them in order" means, proven end to end.
    assert session.last_done == id8
    assert session.highest_acked >= id8  # >=: the settle polls' own
    assert session.pending_count == 0    # STATUS ids ack past id8


def test_pipelining_two_motions_past_an_unpaced_resend_queues_the_later_one_behind_the_earlier(
        inner):
    """FakeMotionAdapter now owns a real FIFO motion queue (see its own
    file header), so this is no longer a characterization of a clobber
    bug: even when the automatic resend's two lines dispatch back to
    back with no step() in between, the SECOND one QUEUES behind the
    first rather than overwriting it. Both motions run to completion, in
    order, each producing its own lastDone/lastDoneReason -- exactly
    what the stakeholder's own correction insisted the system already
    guarantees: "The Motion Layer will execute them in order... You
    have to explicitly replace things. You can't put them out of
    order." The ACK stream was never the problem here (every id was
    always acked in order, `highest_acked` always tracked correctly);
    what changed is that the motion EFFECT now matches it."""
    transport = LossyTransport(inner, drop_outbound={1})
    session = Session(transport)
    inner.set_steps_to_complete(5)  # long enough that neither leg would
                                     # finish before the next line lands

    id1 = session.send("WHEELS_V", 100, 100, 500)   # DROPPED outbound
    id2 = session.send("WHEELS_V", 200, 200, 500)   # delivered -> triggers nack(1)
    assert transport.dropped_outbound

    session.pump(0.3)  # nack(1) arrives on id2's own reply; Session
                        # resends BOTH id1 and id2, back to back, with
                        # no step() in between -- id1 dispatches and
                        # STAYS active; id2 queues behind it.
    assert inner.active_id() == id1, (
        "id1 remains the ACTIVE motion -- id2's resend queues behind "
        "it instead of clobbering it")

    for _ in range(5):  # stepsToComplete
        inner.step()
    session.send("STATUS")  # poll: completion only rides a reply (S8.5)
    session.pump(0.3)
    assert session.last_done == id1, "id1 completes FIRST, on its own"
    assert session.last_done_reason == "stop"
    assert inner.active_id() == id2, "id2 is now the active motion, promoted from the queue"

    for _ in range(5):  # stepsToComplete
        inner.step()
    session.send("STATUS")
    session.pump(0.3)
    assert session.last_done == id2, "id2 completes SECOND, with its own done"
    assert session.highest_acked >= id2, (
        "the ACK stream was always correct -- both ids were, "
        "individually, delivered, decoded, and acked in order (the >= "
        "covers the settle polls' own STATUS ids) -- and now the "
        "motion effect matches it: neither was superseded")


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
    assert inner.active_id() == id1, (
        "leg 1 (never stepped to completion in this test) remains the "
        "active motion -- leg 2 QUEUES behind it now instead of "
        "clobbering it, and leg 3 must not have run (or even been "
        "queued) at all")

    # A further, unrelated well-formed command re-triggers a FRESH
    # nack(3) (docs/design/protocol.md S8.1: "every subsequent
    # command... is discarded and nacked") -- this one gets through.
    session.send("STATUS")
    session.pump(0.3)

    # id3's resend reaches the adapter and QUEUES behind legs 1 and 2
    # (neither has been stepped to completion in this test) instead of
    # clobbering them -- draining all three in arrival order (steps=3)
    # is exactly what proves the resend actually landed.
    _drive_and_settle(inner, session, id3, steps=3)
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

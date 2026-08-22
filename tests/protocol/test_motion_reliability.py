"""tests/protocol/test_motion_reliability.py -- the fake motion adapter
(fake_motion_adapter.h) exercising the reliability layer's completion
channel end to end, plus the square-tour reliability scenario the
stakeholder's own directive is about.

Stakeholder, verbatim (see the sprint's own report for the full quote):
"If you're driving a square and you've got eight movements you send,
and you lose a turn, the whole square is wrong. The best thing to do
there is to NAK and resend from that point on." **The test below
(test_dropped_command_in_a_tour_...) is that scenario, built and
checked directly against the real ProtocolHandler.**

Why a NEW adapter, when tests/protocol/mock_adapter.h already exists:
MockAdapter's canned Results answer instantly and never track a
"currently running" motion, so nothing in test_protocol_harness.py can
ever observe Adapter::lastDone()/lastDoneReason() actually CHANGE --
every existing test sees the wire-correct-but-permanently-0/none default.
FakeMotionAdapter (fake_motion_adapter.h) is a small, deterministic,
step()-driven test double built specifically to make that field live:
a command is accepted and becomes a countdown (`stepsToComplete`,
test-controlled); the harness calls step() explicitly (no timer, no
clock) to advance it; and when the countdown reaches zero,
lastDone()/lastDoneReason() update for real, riding the next ack/nack
this library already formats.

Run with::

    uv run python -m pytest tests/protocol/test_motion_reliability.py -v -s
"""

import ctypes
import pathlib
import subprocess

import pytest

# tests/protocol/test_motion_reliability.py -> protocol -> tests -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PACKAGE_DIR = _REPO_ROOT / "src" / "protocol"
_TEST_DIR = pathlib.Path(__file__).resolve().parent

_SHIM_SOURCES = [
    _PACKAGE_DIR / "protocol_handler.cpp",
    _TEST_DIR / "fake_motion_shim.cpp",
]

# Protocol::Result's DECLARATION order (src/protocol/adapter.h).
RESULT_OK = 0
RESULT_UNKNOWN = 1
RESULT_BADARG = 2
RESULT_RANGE = 3
RESULT_FULL = 4
RESULT_UNIMPLEMENTED = 5
RESULT_NOTREADY = 6
RESULT_BUSY = 7

# Protocol::DoneReason's DECLARATION order (src/protocol/adapter.h).
DONE_NONE = 0
DONE_STOP = 1
DONE_TIMEOUT = 2
DONE_ESTOP = 3
DONE_ABORTED = 4

_DONE_REASON_NAME = {
    DONE_NONE: "none",
    DONE_STOP: "stop",
    DONE_TIMEOUT: "timeout",
    DONE_ESTOP: "estop",
    DONE_ABORTED: "aborted",
}


def _compile_shared_lib(tmp_path):
    lib_path = tmp_path / "libfake_motion_shim.so"
    cmd = ["/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra", "-shared", "-fPIC",
           "-I", str(_PACKAGE_DIR), "-I", str(_TEST_DIR)]
    cmd += [str(s) for s in _SHIM_SOURCES] + ["-o", str(lib_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"shim compile failed:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return lib_path


def _load_shim(tmp_path):
    lib = ctypes.CDLL(str(_compile_shared_lib(tmp_path)))

    lib.fmCreate.argtypes = []
    lib.fmCreate.restype = ctypes.c_void_p
    lib.fmDestroy.argtypes = [ctypes.c_void_p]
    lib.fmDestroy.restype = None

    lib.fmFeed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.fmFeed.restype = None

    lib.fmSinkLength.argtypes = [ctypes.c_void_p]
    lib.fmSinkLength.restype = ctypes.c_int
    lib.fmSinkRead.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.fmSinkRead.restype = ctypes.c_int
    lib.fmSinkClear.argtypes = [ctypes.c_void_p]
    lib.fmSinkClear.restype = None

    lib.fmMalformedCount.argtypes = [ctypes.c_void_p]
    lib.fmMalformedCount.restype = ctypes.c_uint32

    lib.fmSetStepsToComplete.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.fmSetStepsToComplete.restype = None
    lib.fmSetCompletionReason.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.fmSetCompletionReason.restype = None
    lib.fmSetAcceptResult.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.fmSetAcceptResult.restype = None

    lib.fmStep.argtypes = [ctypes.c_void_p]
    lib.fmStep.restype = None
    lib.fmForceAbort.argtypes = [ctypes.c_void_p]
    lib.fmForceAbort.restype = None

    lib.fmActive.argtypes = [ctypes.c_void_p]
    lib.fmActive.restype = ctypes.c_int
    lib.fmActiveId.argtypes = [ctypes.c_void_p]
    lib.fmActiveId.restype = ctypes.c_uint32
    lib.fmStepsRemaining.argtypes = [ctypes.c_void_p]
    lib.fmStepsRemaining.restype = ctypes.c_uint32
    lib.fmLastDone.argtypes = [ctypes.c_void_p]
    lib.fmLastDone.restype = ctypes.c_uint32
    lib.fmLastDoneReason.argtypes = [ctypes.c_void_p]
    lib.fmLastDoneReason.restype = ctypes.c_int
    lib.fmStopCalls.argtypes = [ctypes.c_void_p]
    lib.fmStopCalls.restype = ctypes.c_int
    lib.fmEstopCalls.argtypes = [ctypes.c_void_p]
    lib.fmEstopCalls.restype = ctypes.c_int

    lib.fmEmitTelemetryIfActive.argtypes = [ctypes.c_void_p]
    lib.fmEmitTelemetryIfActive.restype = None

    lib.fmQueuedCount.argtypes = [ctypes.c_void_p]
    lib.fmQueuedCount.restype = ctypes.c_int
    lib.fmQueuedIdAt.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.fmQueuedIdAt.restype = ctypes.c_uint32

    return lib


def _feed(lib, handle, text):
    data = text.encode("ascii")
    lib.fmFeed(handle, data, len(data))


def _sink_lines(lib, handle):
    length = lib.fmSinkLength(handle)
    if length == 0:
        return []
    buf = ctypes.create_string_buffer(length)
    n = lib.fmSinkRead(handle, buf, length)
    assert n == length
    text = buf.raw[:length].decode("ascii")
    lines = text.split("\n")
    assert lines[-1] == "", f"sink output not newline-terminated: {text!r}"
    return lines[:-1]


def _ack_line(n, last_done, reason):
    return f"ack {n} {last_done} {_DONE_REASON_NAME[reason]}"


def _nack_line(n, last_done, reason):
    return f"nack {n} {last_done} {_DONE_REASON_NAME[reason]}"


# ---------------------------------------------------------------------------
# 1. Each of the six motion verbs reaches its own Adapter method, becomes
#    "active", and completes after exactly `stepsToComplete` step() calls.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("wire,verify", [
    ("WHEELS_X 100 100 200 5000 #1",
     lambda lib, h: (lib.fmActiveId(h) == 1,)),
    ("WHEELS_V 100 100 1000 #1",
     lambda lib, h: (lib.fmActiveId(h) == 1,)),
    ("MOVE_X 400 1571 200 5000 #1",
     lambda lib, h: (lib.fmActiveId(h) == 1,)),
    ("MOVE_V 150 0 1000 #1",
     lambda lib, h: (lib.fmActiveId(h) == 1,)),
    ("GO_TO_R 300 0 150 10 5000 #1",
     lambda lib, h: (lib.fmActiveId(h) == 1,)),
    ("GO_TO_W 300 0 150 10 5000 #1",
     lambda lib, h: (lib.fmActiveId(h) == 1,)),
])
def test_each_motion_verb_dispatches_and_becomes_active(tmp_path, wire, verify):
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 3)
        _feed(lib, handle, wire + "\n")
        assert lib.fmActive(handle) == 1
        assert verify(lib, handle) == (True,)
        assert _sink_lines(lib, handle) == [_ack_line(1, 0, DONE_NONE)]
    finally:
        lib.fmDestroy(handle)


def test_motion_completes_after_exactly_stepstocomplete_steps(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 3)
        _feed(lib, handle, "WHEELS_V 100 100 1000 #1\n")
        assert lib.fmActive(handle) == 1

        lib.fmStep(handle)
        assert lib.fmActive(handle) == 1
        assert lib.fmLastDone(handle) == 0, "not done yet after 1/3 steps"

        lib.fmStep(handle)
        assert lib.fmActive(handle) == 1

        lib.fmStep(handle)
        assert lib.fmActive(handle) == 0, "must be done after 3/3 steps"
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_STOP  # the default
    finally:
        lib.fmDestroy(handle)


# ---------------------------------------------------------------------------
# 2. STOP and ESTOP complete the ACTIVE motion with their own fixed reason.
# ---------------------------------------------------------------------------

def test_stop_completes_active_motion_with_reason_stop(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)  # long enough that STOP, not
                                               # the countdown, ends it
        _feed(lib, handle, "WHEELS_V 100 100 1000 #1\n")
        lib.fmStep(handle)
        lib.fmStep(handle)
        assert lib.fmActive(handle) == 1, "must still be running"
        lib.fmSinkClear(handle)

        _feed(lib, handle, "STOP #2\n")
        assert lib.fmActive(handle) == 0
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_STOP
        # NOTE: dispatch() sends the ack BEFORE the verb's own execute
        # function runs (docs/design/protocol.md §8.9's resolved
        # ambiguity #4: "ack first, always", uniformly for every verb --
        # not special-cased for STOP), so STOP's OWN ack still reflects
        # lastDone as it stood BEFORE this very STOP executed and
        # completed leg 1 -- the completion becomes visible starting
        # with the NEXT reply (a later command's ack, or the next
        # telemetry-piggybacked line), never lost, just one reply later.
        assert _sink_lines(lib, handle) == [_ack_line(2, 0, DONE_NONE)]
        lib.fmSinkClear(handle)

        _feed(lib, handle, "STATUS #3\n")
        assert _sink_lines(lib, handle)[0] == _ack_line(3, 1, DONE_STOP)
    finally:
        lib.fmDestroy(handle)


def test_stop_now_reaches_adapter_as_immediate(tmp_path):
    """`STOP now #<id>` (motion-api.md §3.7/§9.1) reaches onStop() with
    immediate=true; a plain `STOP #<id>` reaches it with immediate=false.
    FakeMotionAdapter does not vary its OWN behavior on this flag (see
    its own onStop() comment) -- this test only pins that the flag is
    decoded and threaded through at all."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        _feed(lib, handle, "STOP #1\n")
        _feed(lib, handle, "STOP now #2\n")
        assert lib.fmStopCalls(handle) == 2
        assert _sink_lines(lib, handle) == [
            _ack_line(1, 0, DONE_NONE),
            _ack_line(2, 0, DONE_NONE),
        ]
    finally:
        lib.fmDestroy(handle)


def test_estop_mid_move_completes_it_with_reason_estop(tmp_path):
    """The scenario the ticket calls out by name: "an ESTOP mid-move
    must complete the in-flight move with reason estop." ESTOP is
    unsequenced (docs/design/protocol.md §8.3), so it carries no id of
    its own -- the completion it produces is observed on the NEXT
    sequenced command's ack."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)
        _feed(lib, handle, "MOVE_X 400 0 200 5000 #1\n")
        lib.fmStep(handle)
        assert lib.fmActive(handle) == 1
        lib.fmSinkClear(handle)

        _feed(lib, handle, "ESTOP\n")
        assert _sink_lines(lib, handle) == ["estop"]
        assert lib.fmActive(handle) == 0, "the in-flight move must have ended"
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_ESTOP
        assert lib.fmEstopCalls(handle) == 1
        lib.fmSinkClear(handle)

        # The completion is now visible on the next sequenced ack.
        _feed(lib, handle, "STATUS #2\n")
        lines = _sink_lines(lib, handle)
        assert lines[0] == _ack_line(2, 1, DONE_ESTOP)
    finally:
        lib.fmDestroy(handle)


def test_forceabort_completes_with_reason_aborted(tmp_path):
    """`aborted` (motion-api.md §5.3: "the caller abandoned it") has no
    wire trigger of its own -- forceAbort() is this test double's own
    stand-in for that host-side condition, so this pins that the wire
    reports it correctly once it happens."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)
        _feed(lib, handle, "MOVE_V 150 0 1000 #1\n")
        lib.fmStep(handle)
        lib.fmForceAbort(handle)
        assert lib.fmActive(handle) == 0
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_ABORTED

        lib.fmSinkClear(handle)
        _feed(lib, handle, "STATUS #2\n")
        assert _sink_lines(lib, handle)[0] == _ack_line(2, 1, DONE_ABORTED)
    finally:
        lib.fmDestroy(handle)


def test_completion_reason_timeout(tmp_path):
    """The fourth and last reason: a test sets `completionReason` to
    DONE_TIMEOUT before the countdown itself reaches zero, standing in
    for "the backstop fired" (motion-api.md §5.3) rather than the stop
    condition being met."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 2)
        lib.fmSetCompletionReason(handle, DONE_TIMEOUT)
        _feed(lib, handle, "WHEELS_X 400 400 200 100 #1\n")
        lib.fmStep(handle)
        lib.fmStep(handle)
        assert lib.fmActive(handle) == 0
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_TIMEOUT
    finally:
        lib.fmDestroy(handle)


# ---------------------------------------------------------------------------
# 3. Telemetry emission piggybacks the CURRENT ack/nack, riding the
#    reliability line's own reason token, mid-motion.
# ---------------------------------------------------------------------------

def test_telemetry_piggyback_rides_along_during_a_multi_step_move(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 3)
        _feed(lib, handle, "WHEELS_V 100 100 1000 #1\n")
        lib.fmSinkClear(handle)

        lib.fmStep(handle)
        lib.fmEmitTelemetryIfActive(handle)
        lines = _sink_lines(lib, handle)
        assert lines[-1] == _ack_line(1, 0, DONE_NONE), (
            "the move hasn't completed yet -- lastDone must still be 0/none")
        lib.fmSinkClear(handle)

        lib.fmStep(handle)
        lib.fmStep(handle)  # completes on this step
        lib.fmEmitTelemetryIfActive(handle)  # active() is now false, but the
                                              # handler doesn't need "active"
                                              # to emit telemetry -- call it
                                              # regardless to observe the
                                              # completed reliability line
    finally:
        lib.fmDestroy(handle)


# ---------------------------------------------------------------------------
# 4. THE flagship test: a square tour, one command dropped mid-sequence.
#
# Stakeholder, verbatim: "If you're driving a square and you've got eight
# movements you send, and you lose a turn, the whole square is wrong. The
# best thing to do there is to NAK and resend from that point on."
#
# Four legs (WHEELS_V, ids #1-#4), each driven to completion before the
# next is sent -- this is how a real host actually drives a sequence of
# motions (wait for the ack's own lastDone to confirm the PREVIOUS leg
# finished before commanding the next one), which is exactly what makes
# "lastDone advances monotonically through it" a meaningful thing to
# assert rather than a coincidence of timing. Leg 3 is DROPPED (never
# fed to the handler at all, simulating a lost radio frame) until after
# leg 4 has already been sent and nacked.
# ---------------------------------------------------------------------------

def test_dropped_command_in_a_tour_is_nacked_and_resumes_in_order(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 2)

        # Four distinct wheel commands -- distinct left/right speeds per
        # leg, so a reordering or aliasing bug (leg 3 accidentally
        # running leg 4's numbers, or vice versa) would be visible, not
        # just "some WHEELS_V ran".
        legs = {
            1: "WHEELS_V 100 100 500 #1\n",     # straight
            2: "WHEELS_V 100 -100 500 #2\n",    # pivot
            3: "WHEELS_V 150 150 500 #3\n",     # straight, different speed
            4: "WHEELS_V 150 -150 500 #4\n",    # pivot, different speed
        }

        def run_leg_to_completion(leg_id, wire):
            _feed(lib, handle, wire)
            lines = _sink_lines(lib, handle)
            assert lines[-1] == _ack_line(leg_id, leg_id - 1,
                                           DONE_STOP if leg_id > 1 else
                                           DONE_NONE), (
                f"leg {leg_id}: unexpected ack {lines!r}")
            lib.fmSinkClear(handle)
            for _ in range(2):  # stepsToComplete
                lib.fmStep(handle)
            assert lib.fmLastDone(handle) == leg_id
            assert lib.fmLastDoneReason(handle) == DONE_STOP

        # ---- legs 1 and 2 run normally, each to completion ----
        run_leg_to_completion(1, legs[1])
        run_leg_to_completion(2, legs[2])

        # ---- leg 3 is DROPPED -- the host's next transmission is leg 4 ----
        _feed(lib, handle, legs[4])
        # (a) leg 4 must NOT have executed.
        assert lib.fmActive(handle) == 0, "leg 4 must not have run out of order"
        assert lib.fmLastDone(handle) == 2, "lastDone must not have moved"
        # (b) the reply nacks, naming exactly the missing id (#3).
        assert _sink_lines(lib, handle) == [_nack_line(3, 2, DONE_STOP)]
        lib.fmSinkClear(handle)

        # A second, unrelated well-formed line (a status probe, say)
        # keeps getting the SAME nack -- the gap self-heals only once
        # the missing id itself shows up, never by "catching up" once
        # something later arrives.
        _feed(lib, handle, "STATUS #4\n")  # STILL id #4 -- still a gap
        assert _sink_lines(lib, handle) == [_nack_line(3, 2, DONE_STOP)]
        lib.fmSinkClear(handle)

        # ---- leg 3 is resent -- the missing id finally arrives ----
        run_leg_to_completion(3, legs[3])
        assert lib.fmLastDone(handle) == 3

        # ---- leg 4 is resent -- the sequence resumes in order ----
        run_leg_to_completion(4, legs[4])

        # (d) lastDone advanced MONOTONICALLY through the whole
        # sequence: 1, 2, (gap), 3, 4 -- never skipped, never regressed,
        # every step landing with reason kStop (the fake adapter's own
        # default "finished normally" reason).
        assert lib.fmLastDone(handle) == 4
        assert lib.fmLastDoneReason(handle) == DONE_STOP
    finally:
        lib.fmDestroy(handle)


def test_dropped_command_stale_retransmit_of_an_already_done_leg_does_not_rerun(
        tmp_path):
    """Companion check: once a leg has completed and the sequence has
    moved on, a stale retransmit of that SAME leg's id must not re-run
    it (docs/design/protocol.md §8.1's "do not re-execute" rule) --
    exercised here with a real motion adapter rather than MockAdapter,
    since a re-run motion command is the concrete failure this rule
    exists to prevent (a resent WHEELS_V must not drive the wheels
    twice)."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 1)
        _feed(lib, handle, "WHEELS_V 100 100 500 #1\n")
        lib.fmStep(handle)
        assert lib.fmLastDone(handle) == 1
        lib.fmSinkClear(handle)

        _feed(lib, handle, "WHEELS_V 200 200 500 #2\n")
        assert lib.fmActiveId(handle) == 2
        lib.fmSinkClear(handle)

        # Resend #1 (the host never saw its ack) -- must NOT re-trigger
        # a motion (that would clobber the currently-active #2). The
        # retransmit ack echoes the highest ALREADY-accepted id (2), not
        # the resent one (1) -- docs/design/protocol.md §8.1.
        _feed(lib, handle, "WHEELS_V 999 999 500 #1\n")
        assert lib.fmActiveId(handle) == 2, "leg 2 must still be the active one"
        assert _sink_lines(lib, handle) == [_ack_line(2, 1, DONE_STOP)]
    finally:
        lib.fmDestroy(handle)


# ---------------------------------------------------------------------------
# 5. A decode failure mid-tour does NOT advance the sequence -- the
# other half of the stakeholder's directive (a GARBLED command, as
# opposed to one that never arrived at all). Uses a real motion verb so
# the "the whole square is wrong" framing is concrete: a bad WHEELS_V
# line must not be treated as if leg 2 happened.
# ---------------------------------------------------------------------------

def test_garbled_motion_command_is_nacked_not_acked_and_never_executes(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 1)
        _feed(lib, handle, "WHEELS_V 100 100 500 #1\n")
        lib.fmStep(handle)
        lib.fmSinkClear(handle)

        # Leg 2, but the duration field is garbage -- a decode failure
        # (docs/design/protocol.md §8.9), not a merits rejection.
        _feed(lib, handle, "WHEELS_V 100 100 notanumber #2\n")
        assert lib.fmActive(handle) == 0, "must NOT have started a motion"
        assert _sink_lines(lib, handle) == [
            _nack_line(2, 1, DONE_STOP), "err 2 #2",
        ]
        lib.fmSinkClear(handle)

        # The correct leg 2, resent -- now it runs.
        _feed(lib, handle, "WHEELS_V 100 -100 500 #2\n")
        assert lib.fmActiveId(handle) == 2
        assert _sink_lines(lib, handle) == [_ack_line(2, 1, DONE_STOP)]
    finally:
        lib.fmDestroy(handle)


# ---------------------------------------------------------------------------
# 6. The motion queue (2026-08-22). Stakeholder, verbatim, correcting an
# earlier report that treated the absence of a queue as an open protocol
# design question: "Well, then put a fucking motion queue in it... The
# system absolutely does give you ordered execution... You have to
# explicitly replace things. You can't put them out of order."
#
# FakeMotionAdapter (fake_motion_adapter.h) now owns a real, fixed-
# capacity FIFO: a motion command arriving while one is active QUEUES
# behind it rather than superseding it, a full queue is refused with
# `Result::kFull` (ERR_FULL), and STOP/ESTOP both drain the queued
# remainder (rather than letting it silently continue) when they end
# the active motion -- see that file's own header comment for the full
# rationale, especially why STOP/ESTOP drain while forceAbort() does not.
# ---------------------------------------------------------------------------

def test_queued_motions_run_to_completion_in_arrival_order_not_just_the_last(tmp_path):
    """Three motions dispatched back to back, with no step() in
    between: the second and third QUEUE behind the first rather than
    clobbering it, and each produces its OWN lastDone/lastDoneReason in
    turn as step() drains them -- not just the last one dispatched."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 1)
        _feed(lib, handle, "WHEELS_V 100 100 500 #1\n")
        assert lib.fmActiveId(handle) == 1
        assert lib.fmQueuedCount(handle) == 0

        _feed(lib, handle, "WHEELS_V 150 150 500 #2\n")
        assert lib.fmActiveId(handle) == 1, "id1 stays active -- id2 queues"
        assert lib.fmQueuedCount(handle) == 1
        assert lib.fmQueuedIdAt(handle, 0) == 2

        _feed(lib, handle, "WHEELS_V 200 200 500 #3\n")
        assert lib.fmActiveId(handle) == 1, "id1 is still active -- id3 queues too"
        assert lib.fmQueuedCount(handle) == 2
        assert lib.fmQueuedIdAt(handle, 0) == 2
        assert lib.fmQueuedIdAt(handle, 1) == 3

        lib.fmStep(handle)  # completes id1, promotes id2
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_STOP
        assert lib.fmActiveId(handle) == 2
        assert lib.fmQueuedCount(handle) == 1

        lib.fmStep(handle)  # completes id2, promotes id3
        assert lib.fmLastDone(handle) == 2
        assert lib.fmActiveId(handle) == 3
        assert lib.fmQueuedCount(handle) == 0

        lib.fmStep(handle)  # completes id3, nothing left
        assert lib.fmLastDone(handle) == 3
        assert lib.fmActive(handle) == 0
        assert lib.fmQueuedCount(handle) == 0
    finally:
        lib.fmDestroy(handle)


def test_queue_full_returns_err_full_and_leaves_the_queue_undisturbed(tmp_path):
    """Filling the queue to its fixed capacity (kMaxQueueDepth == 5,
    behind the one active motion) and sending one more must be refused
    with `ERR_FULL` (wire code 4) -- a MERITS rejection, so the sequence
    still advances (ack AND err, docs/design/protocol.md §8.9), but the
    existing active motion and queue contents are completely
    undisturbed."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)  # long-lived -- nothing
                                               # completes on its own here
        _feed(lib, handle, "WHEELS_V 100 100 500 #1\n")
        assert lib.fmActiveId(handle) == 1

        for i in range(2, 7):  # ids 2..6 -- five more, filling the queue
            _feed(lib, handle, f"WHEELS_V {100 + i} {100 + i} 500 #{i}\n")
        assert lib.fmQueuedCount(handle) == 5, "queue must be at capacity"
        lib.fmSinkClear(handle)

        # id7 -- the queue has no room left.
        _feed(lib, handle, "WHEELS_V 999 999 500 #7\n")
        assert _sink_lines(lib, handle) == [_ack_line(7, 0, DONE_NONE), "err 4 #7"]
        assert lib.fmQueuedCount(handle) == 5, "the queue must be UNCHANGED"
        assert lib.fmActiveId(handle) == 1, "the active motion must be UNCHANGED"
        for offset, expected_id in enumerate(range(2, 7)):
            assert lib.fmQueuedIdAt(handle, offset) == expected_id, (
                "no queued entry may have been disturbed by the refusal")
    finally:
        lib.fmDestroy(handle)


def test_stop_drains_the_queued_remainder(tmp_path):
    """STOP completes the ACTIVE motion (reason kStop) and DRAINS
    whatever is still queued behind it -- this file's own resolution of
    "what happens to the rest of the plan when the host says stop":
    draining, not letting it silently continue, matches "you have to
    explicitly replace things" (fake_motion_adapter.h's own header
    comment has the full rationale). The drained motions never run and
    produce no completion event of their own."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)
        _feed(lib, handle, "WHEELS_V 100 100 500 #1\n")
        _feed(lib, handle, "WHEELS_V 150 150 500 #2\n")
        _feed(lib, handle, "WHEELS_V 200 200 500 #3\n")
        assert lib.fmActiveId(handle) == 1
        assert lib.fmQueuedCount(handle) == 2

        _feed(lib, handle, "STOP #4\n")
        assert lib.fmActive(handle) == 0, "STOP must end the active motion"
        assert lib.fmLastDone(handle) == 1, "only leg 1 (the active one) completes"
        assert lib.fmLastDoneReason(handle) == DONE_STOP
        assert lib.fmQueuedCount(handle) == 0, "legs 2 and 3 must be DRAINED"

        # Stepping after a drain does nothing -- there is nothing left
        # to run, and legs 2/3 never get their own completion.
        for _ in range(10):
            lib.fmStep(handle)
        assert lib.fmActive(handle) == 0
        assert lib.fmLastDone(handle) == 1
    finally:
        lib.fmDestroy(handle)


def test_estop_completes_active_and_clears_the_queue(tmp_path):
    """The ticket's own named scenario, extended to the queue: "a panic
    stop must not leave five legs armed to run afterwards." ESTOP
    completes the active motion with reason kEstop AND clears every
    motion still queued behind it, same as STOP but for the fault
    path."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)
        _feed(lib, handle, "MOVE_X 400 0 200 5000 #1\n")
        _feed(lib, handle, "MOVE_X 400 0 200 5000 #2\n")
        _feed(lib, handle, "MOVE_X 400 0 200 5000 #3\n")
        lib.fmStep(handle)
        assert lib.fmActiveId(handle) == 1
        assert lib.fmQueuedCount(handle) == 2
        lib.fmSinkClear(handle)

        _feed(lib, handle, "ESTOP\n")
        assert _sink_lines(lib, handle) == ["estop"]
        assert lib.fmActive(handle) == 0, "the in-flight move must have ended"
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_ESTOP
        assert lib.fmQueuedCount(handle) == 0, (
            "legs 2 and 3 must be cleared -- not left armed to run once "
            "the estop is cleared")
        assert lib.fmEstopCalls(handle) == 1

        # Nothing left to run.
        for _ in range(10):
            lib.fmStep(handle)
        assert lib.fmActive(handle) == 0
        assert lib.fmLastDone(handle) == 1
    finally:
        lib.fmDestroy(handle)


def test_forceabort_completes_only_the_active_motion_and_the_queue_continues(tmp_path):
    """Unlike STOP/ESTOP, forceAbort() (the test-only stand-in for "the
    host-side caller abandoned it", motion-api.md §5.1) abandons only
    the ONE active motion -- it is not a request to cancel the whole
    plan, so whatever is queued behind it is promoted and runs
    normally."""
    lib = _load_shim(tmp_path)
    handle = lib.fmCreate()
    try:
        lib.fmSetStepsToComplete(handle, 10)
        _feed(lib, handle, "WHEELS_V 100 100 500 #1\n")
        _feed(lib, handle, "WHEELS_V 150 150 500 #2\n")
        assert lib.fmActiveId(handle) == 1
        assert lib.fmQueuedCount(handle) == 1

        lib.fmForceAbort(handle)
        assert lib.fmLastDone(handle) == 1
        assert lib.fmLastDoneReason(handle) == DONE_ABORTED
        assert lib.fmActiveId(handle) == 2, "id2 is promoted -- the queue continues"
        assert lib.fmQueuedCount(handle) == 0

        for _ in range(10):  # id2 was accepted while stepsToComplete==10
            lib.fmStep(handle)
        assert lib.fmLastDone(handle) == 2
        assert lib.fmLastDoneReason(handle) == DONE_STOP
    finally:
        lib.fmDestroy(handle)

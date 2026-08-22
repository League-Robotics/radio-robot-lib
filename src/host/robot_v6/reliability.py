"""reliability.py -- the HOST half of protocol-v6's reliability layer
(docs/design/protocol.md S8). The robot side (mandatory sequence ids,
cumulative ack/nack, decode-failure-is-a-NAK) is `src/protocol/
protocol_handler.cpp`, already built and tested. Nothing before this
module implemented the CALLER's own half: assigning ids, pipelining
without waiting for each ack, tracking the highest cumulative ack,
resending from a nack, and surfacing `lastDone`/its reason to a caller
that wants to await a motion's completion.

---- The stakeholder's own acceptance scenario ----

"If you're driving a square and you've got eight movements you send,
and you lose a turn, the whole square is wrong. The best thing to do
there is to NAK and resend from that point on." `Session` below is
built specifically to make that recoverable: `send()` never blocks, a
cumulative `ack` retires every earlier buffered command in one shot,
and a `nack` triggers an automatic resend of everything still buffered
from the named id forward, in order.

---- A real gap this file's own existence exposed (read before pipelining
motion commands against FakeMotionAdapter/DiffDriveAdapter) ----

The reliability layer's pipelining guarantee ("never block waiting for
an ack") and this repo's own motion adapters' complete absence of a
queue (docs/design/protocol.md S5.1: "there is no queue in this
library") are in tension, and nothing on the WIRE says so. Every motion
verb's own adapter method (`onWheelsV`/`onMoveX`/...) unconditionally
overwrites whatever motion was previously "active" -- there is no
`ERR_BUSY` refusal for arriving mid-move (`Result::kBusy` exists in the
wire's own error-code space, S6.1's code 10, but neither
`FakeMotionAdapter` nor `DiffDriveAdapter` ever returns it). So
pipelining TWO motion commands ahead of the first one's own completion
is wire-legal, decodes fine, sequences and acks correctly -- and
STILL silently discards the first command's own motion effect the
instant the second one dispatches, because `beginMotion()` just
clobbers `activeId`/`activeKind` with no queue behind it. A caller
resending a multi-command backlog after a `nack` hits this exact
seam: the resend burst delivers every buffered id back-to-back with no
pacing, so on a queue-less adapter only the LAST one in that burst
actually gets to run. **For a queue-less adapter, wait for
`wait_for_done()` on one motion command before sending the next --
pipelining is safe (and intended) for order-independent commands
(GET/SET/STATUS/PING), not for a sequence of motions on this repo's
own test adapters.** See `tests/host/robot_v6/test_reliability.py`'s
own paired tests (one pipelined-and-clobbered, one paced-and-clean) for
this pinned down as an executable example rather than only prose.
"""

from __future__ import annotations

import dataclasses
import time
from collections import OrderedDict
from typing import Callable

from . import codec
from .transport import Transport

# The three verbs docs/design/protocol.md S8.3 exempts from sequencing
# entirely -- HELLO because it RESETS the sequence and so cannot be
# inside it, ESTOP because it is safety-critical and must execute even
# while the stream is stalled on a gap, PING because it is the liveness
# probe and must answer under the same condition.
UNSEQUENCED_VERBS = frozenset({"HELLO", "ESTOP", "PING"})


class PendingBufferFull(RuntimeError):
    """Raised by `Session.send()` once more than `max_pending`
    sequenced commands are outstanding (sent, not yet retired by a
    cumulative ack). This is a HARD error, not a silent block or a
    silent drop: a caller that pipelines unboundedly with no backoff
    is a caller with no flow control at all, and this class has no
    business guessing how long to wait on its behalf. The prescribed
    recovery is explicit -- call `pump()` (or `wait_for_ack()` on the
    oldest outstanding id) until room frees up, then retry the send.
    """


@dataclasses.dataclass(frozen=True)
class DoneEvent:
    """The result of `Session.wait_for_done()`. See that method's own
    docstring for the one real limitation this carries: `reason` is
    whichever reason rode the LATEST `lastDone`/`lastDoneReason` pair
    the robot has reported, which is only guaranteed to be `seq_id`'s
    own reason if `seq_id == lastDone` at the moment it was observed.
    """

    id: int  # noqa: A003 -- matches the wire's own vocabulary
    reason: str


class Session:
    """Owns one sequence-id counter, one outstanding-command buffer,
    and the cumulative ack/nack bookkeeping for a single v6 connection
    over one `Transport`. Does not own a background read loop: call
    `pump()` to process whatever reply lines are currently available;
    `wait_for_ack()`/`wait_for_done()` call `pump()` internally in a
    loop until their own condition is met or a timeout elapses.
    """

    def __init__(
        self,
        transport: Transport,
        *,
        max_pending: int = 64,
        resend_cooldown: float = 0.25,  # [s]
        on_reply: Callable[[codec.Reply], None] | None = None,
    ):
        self._transport = transport
        self._max_pending = max_pending
        self._resend_cooldown = resend_cooldown
        self._on_reply = on_reply

        self._next_id = 1
        # Sequence id -> the exact line text it was sent as (id
        # included), in SEND order -- an OrderedDict rather than a
        # plain dict purely so `_maybe_resend_from()` can iterate it in
        # ascending order without re-sorting every time (Python dicts
        # already preserve insertion order since 3.7, but the
        # OrderedDict spelling says so on purpose for a reader).
        self._pending: "OrderedDict[int, str]" = OrderedDict()

        self._highest_acked = 0
        self._last_done = 0
        self._last_done_reason = "none"

        # Resend-storm guard (docs/design/protocol.md S8.5: a gap keeps
        # re-nacking at the telemetry rate on its own) -- see
        # `_maybe_resend_from()`.
        self._last_nack_next: int | None = None
        self._last_resend_time = 0.0

    # ---- observable state ----------------------------------------

    @property
    def highest_acked(self) -> int:
        """The highest sequence id retired by a cumulative ack so far."""
        return self._highest_acked

    @property
    def last_done(self) -> int:
        """`Adapter::lastDone()` as of the most recent ack/nack seen."""
        return self._last_done

    @property
    def last_done_reason(self) -> str:
        """`Adapter::lastDoneReason()`'s wire spelling, paired with
        `last_done` (`"none"` when `last_done == 0`)."""
        return self._last_done_reason

    @property
    def pending_count(self) -> int:
        """How many sequenced commands are sent but not yet retired by
        a cumulative ack."""
        return len(self._pending)

    # ---- sending ----------------------------------------------------

    def send(self, verb: str, *fields: object) -> int:
        """Assign the next sequential id, format and send the command,
        and buffer it so a later `nack` can trigger a byte-identical
        resend. Never blocks on a reply -- pipelining freely is the
        whole point (docs/design/protocol.md S8.1: "the host may
        pipeline freely"). Returns the assigned id.

        Raises `PendingBufferFull` if `max_pending` commands are
        already outstanding -- see that exception's own docstring.
        """
        if verb in UNSEQUENCED_VERBS:
            raise ValueError(f"{verb} is unsequenced -- use send_unsequenced()")
        if len(self._pending) >= self._max_pending:
            raise PendingBufferFull(
                f"{len(self._pending)} commands outstanding "
                f"(max_pending={self._max_pending}) -- pump()/wait for "
                "an ack before sending more"
            )
        seq_id = self._next_id
        self._next_id += 1
        line = codec.encode_command(verb, *fields, seq_id=seq_id)
        self._pending[seq_id] = line
        self._transport.send_line(line)
        return seq_id

    def send_unsequenced(self, verb: str, *fields: object) -> None:
        """`HELLO`/`ESTOP`/`PING` only -- no id, never buffered, never
        subject to the pending-buffer limit (S8.3's own exemption)."""
        if verb not in UNSEQUENCED_VERBS:
            raise ValueError(f"{verb} is sequenced -- use send()")
        self._transport.send_line(codec.encode_command(verb, *fields))

    # ---- receiving ----------------------------------------------------

    def pump(self, timeout: float | None = 0.0) -> list[codec.Reply]:  # [s]
        """Read whatever reply lines are currently available (blocking
        up to `timeout` seconds waiting for the FIRST chunk;
        `timeout=0.0`, the default, is non-blocking), apply ack/nack
        bookkeeping, and return every parsed reply in arrival order
        (ack/nack included -- a caller that wants those too can read
        them straight off the return value instead of `last_done`/
        `highest_acked`).
        """
        lines = self._transport.read_lines(timeout)
        replies: list[codec.Reply] = []
        for line in lines:
            if not line.strip():
                continue  # S2's own "blank line, ignored silently" rule
            reply = codec.parse_reply(line)
            replies.append(reply)
            self._handle_reply(reply)
            if self._on_reply is not None:
                self._on_reply(reply)
        return replies

    def _handle_reply(self, reply: codec.Reply) -> None:
        if reply.verb == "ack":
            n, last_done, reason = self._ack_nack_fields(reply)
            self._retire_through(n)
            self._last_done, self._last_done_reason = last_done, reason
        elif reply.verb == "nack":
            n, last_done, reason = self._ack_nack_fields(reply)
            self._last_done, self._last_done_reason = last_done, reason
            self._maybe_resend_from(n)
        # Everything else (device/pong/id/ver/status/help/get/ret/err/
        # estop/debug/thdr/t) is application content this class has no
        # opinion about -- the caller reads it off pump()'s own return
        # value, or `on_reply`.

    @staticmethod
    def _ack_nack_fields(reply: codec.Reply) -> tuple[int, int, str]:
        n = int(reply.fields[0])
        last_done = int(reply.fields[1])
        reason = reply.fields[2]
        return n, last_done, reason

    def _retire_through(self, n: int) -> None:
        """One ack covers every earlier id too (docs/design/protocol.md
        S8.1) -- drop every buffered id <= n in one shot."""
        for pending_id in [i for i in self._pending if i <= n]:
            del self._pending[pending_id]
        self._highest_acked = max(self._highest_acked, n)

    def _maybe_resend_from(self, next_id: int) -> None:
        """Resend every still-buffered id >= `next_id`, in ascending
        order (docs/design/protocol.md S8.1: "resend from next
        forward, in order").

        Guarded by a cooldown, not fired on every single nack line
        seen: `emitTelemetry()` re-emits the SAME nack at the telemetry
        rate for as long as a gap is outstanding (S8.5), specifically
        so a host that missed the first one catches a later one for
        free -- if this method resent on every single one of those, a
        stalled stream would retransmit its entire backlog dozens of
        times a second for as long as the gap lasted, which helps
        nothing and floods the link right when it is already in
        trouble. Resending happens at most once per `resend_cooldown`
        seconds for the SAME `next_id`; a nack naming a DIFFERENT
        `next_id` (the gap moved -- the missing command finally arrived
        and a new one opened further along) always resends immediately
        regardless of the cooldown, since that is new information, not
        a repeat.
        """
        now = time.monotonic()
        if (
            self._last_nack_next == next_id
            and now - self._last_resend_time < self._resend_cooldown
        ):
            return
        self._last_nack_next = next_id
        self._last_resend_time = now
        for pending_id in sorted(i for i in self._pending if i >= next_id):
            self._transport.send_line(self._pending[pending_id])

    # ---- waiting ----------------------------------------------------

    def wait_for_ack(self, seq_id: int, timeout: float | None = 5.0) -> bool:  # [s]
        """Block, pumping internally, until a cumulative ack has
        retired `seq_id` (`highest_acked >= seq_id`) or `timeout`
        seconds elapse. Returns whether it was retired in time."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while self._highest_acked < seq_id:
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return False
            self.pump(0.1 if remaining is None else min(0.1, remaining))
        return True

    def wait_for_done(self, seq_id: int, timeout: float | None = 5.0) -> DoneEvent | None:  # [s]
        """Block, pumping internally, until `Adapter::lastDone()` has
        reached or passed `seq_id`, or `timeout` seconds elapse (`None`
        on timeout).

        LIMITATION, worth reading before relying on the returned
        reason: the wire carries only the SINGLE latest `(lastDone,
        reason)` pair, not a history keyed by id (docs/design/
        protocol.md S8.8's own monotonic contract: "a later value
        implies every earlier one completed", which tells a caller
        THAT `seq_id` is done once `last_done >= seq_id`, but not
        necessarily WHY, if a later id has ALSO completed by the time
        this observes it). When `last_done == seq_id` exactly, the
        returned reason is unambiguously `seq_id`'s own. When
        `last_done > seq_id` (this call was slow to notice, or several
        motions finished between polls), the returned reason belongs to
        whatever the LATEST completed id was, not necessarily `seq_id`
        -- there is no way to recover `seq_id`'s own reason after the
        fact once that has happened. A caller that needs the exact
        reason for every single id in a fast sequence must poll often
        enough to catch each one individually; this is a property of
        the wire's own design (a deliberate one -- see docs/design/
        protocol.md S8.8.1's own rejected alternatives), not a bug in
        this method.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        while self._last_done < seq_id:
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return None
            self.pump(0.1 if remaining is None else min(0.1, remaining))
        return DoneEvent(id=seq_id, reason=self._last_done_reason)

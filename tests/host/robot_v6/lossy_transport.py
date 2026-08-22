"""tests/host/robot_v6/lossy_transport.py -- a Transport DECORATOR that
drops specific lines, by position, in either direction. Built to make
the dispatch's own flagship scenario reproducible: "send a sequence of
motion commands through a lossy transport that drops one line, and
assert the host detects the nack, resends from the missing id, and the
sim executes all of them exactly once, in order."

Deliberately INDEX-based, not randomized, as the primary mechanism:
"drop the Nth line" is already fully deterministic and needs no seed at
all, which is a stronger reproducibility guarantee than a seeded RNG
(no dependence on `random`'s own algorithm/version). `with_seeded_drops`
below is offered for the literal "with a fixed seed" phrasing, but it
is a thin convenience that PRECOMPUTES a fixed index set from
`random.Random(seed)` (never the global `random` module) -- once
computed, the actual drop mechanism is exactly the same index check.

Line counting starts at 1 (the first line sent/received is line 1),
matching the wire's own id numbering: `drop_outbound={3}` reads
naturally as "the third command, i.e. the one carrying id #3, never
arrives."
"""

from __future__ import annotations

import random
from typing import Iterable

from robot_v6.transport import Transport


class LossyTransport(Transport):
    def __init__(
        self,
        inner: Transport,
        *,
        drop_outbound: Iterable[int] = frozenset(),
        drop_inbound: Iterable[int] = frozenset(),
    ):
        super().__init__()
        self._inner = inner
        self._drop_outbound = set(drop_outbound)
        self._drop_inbound = set(drop_inbound)
        self._outbound_count = 0
        self._inbound_count = 0
        self.dropped_outbound: list[str] = []
        self.dropped_inbound: list[str] = []

    @classmethod
    def with_seeded_drops(
        cls, inner: Transport, *, count: int, probability: float, seed: int,
        direction: str = "outbound",
    ) -> "LossyTransport":
        """Precompute which of the first `count` lines (in `direction`)
        to drop using `random.Random(seed)` -- deterministic and
        reproducible across runs/platforms, never the global `random`
        module."""
        rng = random.Random(seed)
        indices = {i for i in range(1, count + 1) if rng.random() < probability}
        kwargs = ({"drop_outbound": indices} if direction == "outbound"
                  else {"drop_inbound": indices})
        return cls(inner, **kwargs)

    # ---- overridden at the LINE level, not the byte level: dropping a
    # whole line is the hazard under test (a lost packet), not a
    # mid-line byte corruption, which is a different, separately-tested
    # hazard (test_protocol_adversarial.py's own scope on the C++ side).

    def send_line(self, line: str) -> None:
        self._outbound_count += 1
        if self._outbound_count in self._drop_outbound:
            self.dropped_outbound.append(line)
            return
        self._inner.send_line(line)

    def read_lines(self, timeout: float | None = None) -> list[str]:
        lines = self._inner.read_lines(timeout)
        kept = []
        for line in lines:
            self._inbound_count += 1
            if self._inbound_count in self._drop_inbound:
                self.dropped_inbound.append(line)
                continue
            kept.append(line)
        return kept

    # ---- Transport's own abstract primitives: unused directly (both
    # concrete methods above are overridden instead), but still
    # implemented so this class remains instantiable and so anything
    # that calls them through the base class still reaches `_inner`.

    def _read_chunk(self, timeout: float | None) -> bytes:
        return self._inner._read_chunk(timeout)  # noqa: SLF001

    def _write_bytes(self, data: bytes) -> None:
        self._inner._write_bytes(data)  # noqa: SLF001

    def close(self) -> None:
        self._inner.close()

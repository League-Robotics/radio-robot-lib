"""tests/host/robot_v6/inprocess_transport.py -- a Transport that talks
to an IN-PROCESS Protocol::FakeMotionAdapter (tests/protocol/
fake_motion_shim.cpp, loaded via the `fake_motion_lib` fixture in
conftest.py) instead of a real byte stream.

Why this exists alongside tools/sim (which IS a real byte stream, over
a real subprocess): tools/sim's motion progresses on a wall-clock
`--period` cadence, which makes it the right tool for an honest end-to-
end test (test_sim_e2e.py) but a noisy, timing-sensitive one for
testing the RELIABILITY LAYER itself, where what matters is the exact
sequencing of sends/replies/step() calls, not real elapsed time. This
class gives a test the same control over step() pacing that
tests/protocol/test_motion_reliability.py already has in C++ (step()
is called explicitly, never on a timer) while still exercising the
REAL ProtocolHandler + FakeMotionAdapter, and the REAL robot_v6.Session
on the host side -- only the transport in between is swapped for one
with no wall clock in it at all.
"""

from __future__ import annotations

import ctypes

from robot_v6.transport import Transport


class InProcessTransport(Transport):
    def __init__(self, lib):
        super().__init__()
        self._lib = lib
        self._handle = lib.fmCreate()

    # ---- Transport primitives -----------------------------------------

    def _write_bytes(self, data: bytes) -> None:
        self._lib.fmFeed(self._handle, data, len(data))

    def _read_chunk(self, timeout: float | None) -> bytes:
        # No blocking to do here at all -- feed() above already ran
        # dispatch() to completion synchronously, so anything the
        # handler wrote is sitting in the sink RIGHT NOW or not at all.
        # `timeout` is accepted (Transport's own contract) and ignored.
        del timeout
        n = self._lib.fmSinkLength(self._handle)
        if n == 0:
            return b""
        buf = ctypes.create_string_buffer(n)
        got = self._lib.fmSinkRead(self._handle, buf, n)
        self._lib.fmSinkClear(self._handle)
        return buf.raw[:got]

    def close(self) -> None:
        self._lib.fmDestroy(self._handle)

    # ---- test-only knobs, beyond the Transport interface ---------------

    def step(self) -> None:
        """Advance FakeMotionAdapter's own countdown by exactly one
        tick -- no clock, the test decides the pace (mirrors
        fake_motion_adapter.h's own step() contract)."""
        self._lib.fmStep(self._handle)

    def set_steps_to_complete(self, steps: int) -> None:
        self._lib.fmSetStepsToComplete(self._handle, steps)

    def active_id(self) -> int:
        return self._lib.fmActiveId(self._handle)

    def active(self) -> bool:
        return bool(self._lib.fmActive(self._handle))

    def emit_telemetry(self) -> None:
        self._lib.fmEmitTelemetryIfActive(self._handle)

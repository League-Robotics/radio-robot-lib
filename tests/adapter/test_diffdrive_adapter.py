"""tests/adapter/test_diffdrive_adapter.py -- the DiffDriveAdapter
acceptance harness (Step 4).

This is the one test that links all three pieces built so far:
ProtocolHandler (src/protocol/), DifferentialDrive (src/diffdrive/), and
the DiffDriveAdapter that bridges them (src/adapter/), driven end to end
from Python via ctypes -- no robot, no serial port, no CMake. The combined
extern "C" shim lives in diffdrive_protocol_shim.cpp and reuses step 2's
fake ports (tests/diffdrive/fake_ports.h) unmodified.

Three things this file exists to prove, each with its own test:

1. **The end-to-end acceptance** (Step 4): WHEELS in as
   bytes, encoder counts climb in t: frames, the lease expiry stops the
   wheels, ESTOP latches with no ack.

2. **Twist sign** -- a test that FAILS if the two wheels are swapped
   inside the adapter's onWheels() (docs/design/protocol.md §4 point 3).
   This project shipped exactly that bug once (a physically-swapped
   "left" wheel that negated every wheel-derived heading while leaving
   forward motion correct, so nothing caught it for four downstream
   patches) -- see this test's own docstring for how it was verified to
   actually go red with the wheels swapped.

3. **Lease expiry, measured at the fake motor** -- not inferred from the
   kernel's own `leaseExpired` flag, matching Step 2's own
   standard for the same claim.

Run with::

    uv run python -m pytest tests/adapter/test_diffdrive_adapter.py -v -s
"""

import ctypes
import pathlib
import subprocess

import pytest

# tests/adapter/test_diffdrive_adapter.py -> adapter -> tests -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_DIFFDRIVE_DIR = _REPO_ROOT / "src" / "diffdrive"
_PROTOCOL_DIR = _REPO_ROOT / "src" / "protocol"
_ADAPTER_DIR = _REPO_ROOT / "src" / "adapter"
_FAKE_PORTS_DIR = _REPO_ROOT / "tests" / "diffdrive"
_TEST_DIR = pathlib.Path(__file__).resolve().parent

_SHIM_SOURCES = [
    _DIFFDRIVE_DIR / "differential_drive.cpp",
    _PROTOCOL_DIR / "protocol_handler.cpp",
    _ADAPTER_DIR / "diffdrive_adapter.cpp",
    _TEST_DIR / "diffdrive_protocol_shim.cpp",
]
_INCLUDE_DIRS = [_DIFFDRIVE_DIR, _PROTOCOL_DIR, _ADAPTER_DIR, _FAKE_PORTS_DIR]

# DifferentialDrive::Status ordinals (differential_drive.h) -- paBegin()'s
# return value.
STATUS_OK = 0


def _compile_shared_lib(tmp_path):
    lib_path = tmp_path / "libdiffdrive_adapter_shim.so"
    cmd = ["/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra", "-shared", "-fPIC"]
    for d in _INCLUDE_DIRS:
        cmd += ["-I", str(d)]
    cmd += [str(s) for s in _SHIM_SOURCES] + ["-o", str(lib_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"shim compile failed:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return lib_path


def _load_shim(tmp_path):
    lib = ctypes.CDLL(str(_compile_shared_lib(tmp_path)))

    lib.paCreate.argtypes = [ctypes.c_float]
    lib.paCreate.restype = ctypes.c_void_p

    lib.paDestroy.argtypes = [ctypes.c_void_p]
    lib.paDestroy.restype = None

    lib.paConfigureBasic.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32,
    ]
    lib.paConfigureBasic.restype = None

    lib.paBegin.argtypes = [ctypes.c_void_p]
    lib.paBegin.restype = ctypes.c_int

    lib.paFeed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.paFeed.restype = None

    lib.paMalformedCount.argtypes = [ctypes.c_void_p]
    lib.paMalformedCount.restype = ctypes.c_uint32

    lib.paSinkLength.argtypes = [ctypes.c_void_p]
    lib.paSinkLength.restype = ctypes.c_int

    lib.paSinkRead.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.paSinkRead.restype = ctypes.c_int

    lib.paSinkClear.argtypes = [ctypes.c_void_p]
    lib.paSinkClear.restype = None

    lib.paStep.argtypes = [ctypes.c_void_p]
    lib.paStep.restype = None

    lib.paEmitTelemetryIfEnabled.argtypes = [ctypes.c_void_p]
    lib.paEmitTelemetryIfEnabled.restype = ctypes.c_int

    for name in ("paMotorAppliedDutyLeft", "paMotorAppliedDutyRight",
                 "paMotorVelocityLeft", "paMotorVelocityRight",
                 "paPositionLeft", "paPositionRight"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = ctypes.c_float

    for name in ("paLeaseExpired", "paEstopped"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = ctypes.c_int

    lib.paCycleCount.argtypes = [ctypes.c_void_p]
    lib.paCycleCount.restype = ctypes.c_uint32

    return lib


# countsPerLength [counts/mm]. 10.0 is an arbitrary, easy-to-check-by-hand
# value -- not a real robot's calibration.
_COUNTS_PER_LENGTH = 10.0


def _new_handle(lib, counts_per_length=_COUNTS_PER_LENGTH, max_duty=100.0,
                 full_duty_velocity=1000.0, kp=0.0, ki=6.0, i_max=200.0,
                 pid_max=300.0, cycle_period=24):
    """Create + configure + begin() a handle bundling the kernel, its
    fakes, the adapter, and the protocol handler. maxDuty/fullDutyVelocity/
    cyclePeriod are armed directly here too (not through the wire) -- see
    diffdrive_adapter.h for why those three are not GET/SET-reachable.
    Since DiffDriveAdapter's own constructor now hard-codes those same
    three values (docs/design/protocol.md §9.3), this call is redundant
    with the adapter's own arming rather than load-bearing for it --
    test_begin_succeeds_with_no_external_configure_step below is what
    actually exercises the adapter-only path."""
    handle = lib.paCreate(ctypes.c_float(counts_per_length))
    lib.paConfigureBasic(handle, max_duty, full_duty_velocity, kp, ki, i_max,
                         pid_max, cycle_period)
    status = lib.paBegin(handle)
    assert status == STATUS_OK, f"begin() refused: status={status}"
    return handle


def _feed(lib, handle, line: bytes):
    lib.paFeed(handle, line, len(line))


def _sink_text(lib, handle) -> str:
    n = lib.paSinkLength(handle)
    buf = ctypes.create_string_buffer(n)
    lib.paSinkRead(handle, buf, n)
    return buf.raw[:n].decode("ascii")


# ---------------------------------------------------------------------------
# 1. End-to-end acceptance (Step 4)
# ---------------------------------------------------------------------------

def test_acceptance_wheels_to_lease_expiry_to_estop(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        # feed("WHEELS 100 100 1000 #5\n")  ->  sink contains "ok #5"
        _feed(lib, handle, b"WHEELS 100 100 1000 #5\n")
        assert _sink_text(lib, handle) == "ok #5\n"
        lib.paSinkClear(handle)

        # Subscribe telemetry so subsequent steps produce t: frames.
        _feed(lib, handle, b"TLM POSE\n")
        lib.paSinkClear(handle)

        # step the kernel  ->  t: frames show counts (posl/posr) climbing
        positions = []
        for _ in range(10):
            lib.paStep(handle)
            emitted = lib.paEmitTelemetryIfEnabled(handle)
            assert emitted == 1
            positions.append(lib.paPositionLeft(handle))
        text = _sink_text(lib, handle)
        assert text.startswith("thdr seq now flags posl posr vell velr\n")
        assert text.count("\nt ") == 10
        assert positions == sorted(positions), "position never climbed"
        assert positions[-1] > positions[0], "position never climbed"
        lib.paSinkClear(handle)

        # step past 1000 ms  ->  wheels at zero, lease expired
        for _ in range(40):
            lib.paStep(handle)
        assert lib.paLeaseExpired(handle) == 1
        assert lib.paMotorAppliedDutyLeft(handle) == 0.0
        assert lib.paMotorAppliedDutyRight(handle) == 0.0

        # feed("ESTOP\n")  ->  latched zero, no ack (by design)
        lib.paSinkClear(handle)
        _feed(lib, handle, b"ESTOP\n")
        assert lib.paSinkLength(handle) == 0, "ESTOP must never ack"
        lib.paStep(handle)
        assert lib.paEstopped(handle) == 1
        assert lib.paMotorAppliedDutyLeft(handle) == 0.0
        assert lib.paMotorAppliedDutyRight(handle) == 0.0
        assert lib.paSinkLength(handle) == 0, "ESTOP must never ack"
    finally:
        lib.paDestroy(handle)


# ---------------------------------------------------------------------------
# 2. Twist sign -- must fail if the two wheels are swapped
# ---------------------------------------------------------------------------
#
# Verified to actually go red: with onWheels()'s
#   const float countsLeft = left * countsPerLength_;
#   const float countsRight = right * countsPerLength_;
# temporarily swapped to
#   const float countsLeft = right * countsPerLength_;
#   const float countsRight = left * countsPerLength_;
# in src/adapter/diffdrive_adapter.cpp, this test fails both assertions
# below (motorLeft ends up NEGATIVE and motorRight ends up POSITIVE,
# i.e. exactly backwards) -- then the swap was reverted and the suite
# re-run clean. See the sprint report for the transcript.

def test_twist_sign_left_and_right_are_not_swapped(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        # Command an asymmetric split: left forward, right backward.
        # With DiffDrive's own decomposition (rawLeft = velocity - twist,
        # rawRight = velocity + twist; controlStep() in
        # differential_drive.cpp) and this adapter's
        # velocity=(cL+cR)/2, twist=(cR-cL)/2, the round trip is exact:
        # rawLeft == commanded left counts, rawRight == commanded right
        # counts. If left/right were swapped anywhere in onWheels(), the
        # PHYSICAL motorLeft port would receive the commanded RIGHT
        # speed (and vice versa) -- backwards, not just off by a sign
        # convention footnote.
        _feed(lib, handle, b"WHEELS 150 -150 1000 #1\n")
        assert _sink_text(lib, handle) == "ok #1\n"

        for _ in range(40):
            lib.paStep(handle)

        left_velocity = lib.paMotorVelocityLeft(handle)
        right_velocity = lib.paMotorVelocityRight(handle)
        print(f"left={left_velocity:.1f} counts/s  right={right_velocity:.1f} counts/s")

        # Commanded LEFT was +150 mm/s (forward) -- the physical left
        # motor must be turning FORWARD.
        assert left_velocity > 100.0, (
            f"left motor at {left_velocity} counts/s -- commanded left "
            "(+150 mm/s) did not reach the left port; wheels may be "
            "swapped")
        # Commanded RIGHT was -150 mm/s (reverse) -- the physical right
        # motor must be turning BACKWARD.
        assert right_velocity < -100.0, (
            f"right motor at {right_velocity} counts/s -- commanded "
            "right (-150 mm/s) did not reach the right port; wheels may "
            "be swapped")
    finally:
        lib.paDestroy(handle)


# ---------------------------------------------------------------------------
# 3. Lease expiry, measured at the fake motor
# ---------------------------------------------------------------------------

def test_lease_expiry_stops_the_wheels_measured_at_the_motor(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        lease_ms = 200
        _feed(lib, handle, f"WHEELS 200 200 {lease_ms} #1\n".encode("ascii"))
        assert _sink_text(lib, handle) == "ok #1\n"

        # Well inside the lease: driving, not expired.
        for _ in range(5):
            lib.paStep(handle)
        assert lib.paLeaseExpired(handle) == 0
        assert lib.paMotorAppliedDutyLeft(handle) != 0.0
        assert lib.paMotorAppliedDutyRight(handle) != 0.0

        # Well past the lease: the kernel SAYS expired...
        for _ in range(20):
            lib.paStep(handle)
        assert lib.paLeaseExpired(handle) == 1

        # ...and the assertion that actually matters: the FAKE MOTOR was
        # handed zero duty. Read directly off the port
        # (paMotorAppliedDutyLeft/Right in diffdrive_protocol_shim.cpp),
        # bypassing both the kernel's own Output snapshot AND the wire
        # reply text entirely.
        assert lib.paMotorAppliedDutyLeft(handle) == 0.0
        assert lib.paMotorAppliedDutyRight(handle) == 0.0

        # Stays stopped -- no runaway re-drive on some later cycle.
        for _ in range(10):
            lib.paStep(handle)
            assert lib.paMotorAppliedDutyLeft(handle) == 0.0
            assert lib.paMotorAppliedDutyRight(handle) == 0.0
    finally:
        lib.paDestroy(handle)


# ---------------------------------------------------------------------------
# Supporting coverage: STOP, WHEELS duration ceiling, GET/SET round trip
# ---------------------------------------------------------------------------

def test_stop_calls_neutral_and_acks(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        _feed(lib, handle, b"WHEELS 200 200 5000 #1\n")
        lib.paSinkClear(handle)
        for _ in range(5):
            lib.paStep(handle)
        assert lib.paMotorAppliedDutyLeft(handle) != 0.0

        _feed(lib, handle, b"STOP #2\n")
        assert _sink_text(lib, handle) == "ok #2\n"
        for _ in range(3):
            lib.paStep(handle)
        assert lib.paMotorAppliedDutyLeft(handle) == 0.0
        assert lib.paMotorAppliedDutyRight(handle) == 0.0
    finally:
        lib.paDestroy(handle)


def test_wheels_duration_over_ceiling_is_rejected(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        # spec S5.2's ceiling is 5000 ms; the handler itself does not
        # enforce it (protocol_handler.h ambiguity note #3) -- this
        # adapter does (diffdrive_adapter.h kWheelsDurationCeiling).
        _feed(lib, handle, b"WHEELS 100 100 5001 #1\n")
        assert _sink_text(lib, handle) == "err #1 3\n"  # ERR_RANGE
    finally:
        lib.paDestroy(handle)


def test_get_set_wheel_control_field_round_trips(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        _feed(lib, handle, b"SET wheel_control.pid_kp 0.030000 #9\n")
        assert _sink_text(lib, handle) == "ok #9\n"
        lib.paSinkClear(handle)

        _feed(lib, handle, b"GET wheel_control.pid_kp\n")
        assert _sink_text(lib, handle) == "get wheel_control.pid_kp 0.030000\n"
        lib.paSinkClear(handle)

        _feed(lib, handle, b"GET wheel_control.nonexistent\n")
        assert lib.paSinkLength(handle) == 0, "unknown GET name must be silent"
    finally:
        lib.paDestroy(handle)


def test_identity_and_status_have_plausible_values(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_handle(lib)
    try:
        _feed(lib, handle, b"ID\n")
        assert _sink_text(lib, handle) == "id differential step4 v6-step4\n"
        lib.paSinkClear(handle)

        _feed(lib, handle, b"VER\n")
        assert _sink_text(lib, handle) == "ver v6-step4\n"
        lib.paSinkClear(handle)

        # Output() defaults everything false/0 until the kernel has
        # actually published a cycle -- step once so `ready`/`connL`/
        # `connR` reflect a real reading, not the pre-first-publish
        # default.
        lib.paStep(handle)
        lib.paSinkClear(handle)

        _feed(lib, handle, b"STATUS\n")
        text = _sink_text(lib, handle)
        assert text.startswith("status ready=1 active=0 connL=1 connR=1 ")
    finally:
        lib.paDestroy(handle)


# ---------------------------------------------------------------------------
# 4. Adapter self-arms maxDuty/fullDutyVelocity/cyclePeriod -- no external
#    configure step required (docs/design/protocol.md §9.3, stakeholder
#    decision 2026-08-20: "you can just hard code them")
# ---------------------------------------------------------------------------
#
# Every other test in this file calls paConfigureBasic() before paBegin(),
# matching _new_handle()'s docstring above -- but that call is now
# redundant with what DiffDriveAdapter's own constructor already does.
# This test proves the redundancy claim by skipping paConfigureBasic()
# entirely: begin() must still succeed off the adapter's hard-coded
# values alone. kp/ki/iMax/pidMax stay at Config's own zero defaults here
# (they are unrelated to this change -- still GET/SET-reachable
# wheel_control fields, not hard-coded), which is fine: begin() only
# gates on maxDuty (differential_drive.cpp's begin()).

def test_begin_succeeds_with_no_external_configure_step(tmp_path):
    lib = _load_shim(tmp_path)
    handle = lib.paCreate(ctypes.c_float(_COUNTS_PER_LENGTH))
    try:
        status = lib.paBegin(handle)
        assert status == STATUS_OK, (
            f"begin() refused with no external configure step: status={status}")
    finally:
        lib.paDestroy(handle)

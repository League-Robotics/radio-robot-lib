"""tests/diffdrive/test_diffdrive_harness.py -- the DiffDrive host test
harness (Step 2).

Deterministic fakes for the kernel's four ports (Motor/Clock/Sleeper --
FiberLauncher is DECLINED, see fake_ports.h) live in fake_ports.h.
diffdrive_shim.cpp is the thin extern "C" translation layer ctypes can
call. Both are test scaffolding: nothing under src/ knows either file
exists.

Three claims, one test each:

1. commanding drive() and stepping makes position accumulate and
   velocity settle toward the commanded value;
2. lease expiry stops the wheels -- measured AT THE FAKE MOTOR (the duty
   it was last handed), not merely inferred from the kernel's own
   leaseExpired flag;
3. estop() latches zero within one cycle and holds until estopClear().

Run with::

    uv run python -m pytest tests/diffdrive/test_diffdrive_harness.py -v -s
"""

import ctypes
import pathlib
import subprocess

import pytest

# tests/diffdrive/test_diffdrive_harness.py -> diffdrive -> tests -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PACKAGE_DIR = _REPO_ROOT / "src" / "diffdrive"
_TEST_DIR = pathlib.Path(__file__).resolve().parent

_SHIM_SOURCES = [
    _PACKAGE_DIR / "differential_drive.cpp",
    _TEST_DIR / "diffdrive_shim.cpp",
]

# Status, mirrored from DifferentialDrive::Status (differential_drive.h).
STATUS_OK = 0
STATUS_REFUSED_UNCONFIGURED = 1
STATUS_REFUSED_NOT_BEGUN = 2
STATUS_REFUSED_ESTOPPED = 3
STATUS_REFUSED_NON_FINITE = 4
STATUS_CADENCE_PRESERVED = 5


def _compile_shared_lib(tmp_path, sources, include_dirs, out_name):
    lib_path = tmp_path / out_name
    cmd = ["/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra", "-shared", "-fPIC"]
    for d in include_dirs:
        cmd += ["-I", str(d)]
    cmd += [str(s) for s in sources] + ["-o", str(lib_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"shim compile failed:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return lib_path


def _load_shim(tmp_path):
    """Compile diffdrive_shim.cpp + the package into a shared library and
    bind ctypes signatures for every exported function."""
    lib_path = _compile_shared_lib(
        tmp_path, _SHIM_SOURCES, [_PACKAGE_DIR, _TEST_DIR],
        "libdiffdrive_shim.so")
    lib = ctypes.CDLL(str(lib_path))

    lib.ddCreate.argtypes = []
    lib.ddCreate.restype = ctypes.c_void_p

    lib.ddDestroy.argtypes = [ctypes.c_void_p]
    lib.ddDestroy.restype = None

    lib.ddConfigureBasic.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32,
    ]
    lib.ddConfigureBasic.restype = None

    lib.ddBegin.argtypes = [ctypes.c_void_p]
    lib.ddBegin.restype = ctypes.c_int

    lib.ddDrive.argtypes = [
        ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint32]
    lib.ddDrive.restype = ctypes.c_int

    lib.ddEstop.argtypes = [ctypes.c_void_p]
    lib.ddEstop.restype = None

    lib.ddEstopClear.argtypes = [ctypes.c_void_p]
    lib.ddEstopClear.restype = None

    lib.ddStep.argtypes = [ctypes.c_void_p]
    lib.ddStep.restype = None

    for name in ("ddPositionLeft", "ddPositionRight", "ddVelocityLeft",
                 "ddVelocityRight", "ddAppliedDutyLeft", "ddAppliedDutyRight",
                 "ddMotorAppliedDutyLeft", "ddMotorAppliedDutyRight"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = ctypes.c_float

    for name in ("ddReady", "ddEstopped", "ddLeaseExpired"):
        fn = getattr(lib, name)
        fn.argtypes = [ctypes.c_void_p]
        fn.restype = ctypes.c_int

    lib.ddCycleCount.argtypes = [ctypes.c_void_p]
    lib.ddCycleCount.restype = ctypes.c_uint32

    return lib


def _new_kernel(lib, max_duty=100.0, full_duty_velocity=1000.0, kp=0.0,
                 ki=6.0, i_max=200.0, pid_max=300.0, cycle_period=24):
    """Create + configure + begin() a kernel over the fakes. Gains mirror
    the fidelity gate's own "pure-I closed loop, tovez's shipped posture"
    scenario (fidelity_harness.cpp), just read directly in counts here --
    this harness has no mm/counts bridge to maintain."""
    handle = lib.ddCreate()
    lib.ddConfigureBasic(handle, max_duty, full_duty_velocity, kp, ki, i_max,
                         pid_max, cycle_period)
    status = lib.ddBegin(handle)
    assert status == STATUS_OK, f"begin() refused: status={status}"
    return handle


def test_drive_settles_and_accumulates_position(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_kernel(lib)
    try:
        status = lib.ddDrive(handle, 500.0, 0.0, 5000)
        assert status == STATUS_OK

        for _ in range(120):
            lib.ddStep(handle)

        velocity_left = lib.ddVelocityLeft(handle)
        velocity_right = lib.ddVelocityRight(handle)
        position_left = lib.ddPositionLeft(handle)
        position_right = lib.ddPositionRight(handle)
        print(f"settled: vL={velocity_left:.1f} vR={velocity_right:.1f} "
              f"posL={position_left:.1f} posR={position_right:.1f}")

        assert position_left > 0.0, "left position never accumulated"
        assert position_right > 0.0, "right position never accumulated"
        assert abs(velocity_left - 500.0) < 50.0, (
            f"left settled at {velocity_left}, not near commanded 500")
        assert abs(velocity_right - 500.0) < 50.0, (
            f"right settled at {velocity_right}, not near commanded 500")
    finally:
        lib.ddDestroy(handle)


def test_lease_expiry_stops_the_wheels_measured_at_the_motor(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_kernel(lib)
    try:
        lease = 200  # [ms]
        status = lib.ddDrive(handle, 600.0, 0.0, lease)
        assert status == STATUS_OK

        # Well inside the lease: driving, not expired.
        for _ in range(5):
            lib.ddStep(handle)
        assert lib.ddLeaseExpired(handle) == 0
        assert lib.ddMotorAppliedDutyLeft(handle) != 0.0
        assert lib.ddMotorAppliedDutyRight(handle) != 0.0

        # Well past the lease: the kernel SAYS expired...
        for _ in range(20):
            lib.ddStep(handle)
        assert lib.ddLeaseExpired(handle) == 1

        # ...and, the assertion that actually matters: the FAKE MOTOR was
        # handed zero duty. Not inferred from leaseExpired above -- read
        # directly off the port, bypassing the kernel's own Output
        # snapshot entirely (ddMotorAppliedDutyLeft/Right in
        # diffdrive_shim.cpp).
        assert lib.ddMotorAppliedDutyLeft(handle) == 0.0
        assert lib.ddMotorAppliedDutyRight(handle) == 0.0

        # Stays stopped -- no runaway re-drive on some later cycle.
        for _ in range(10):
            lib.ddStep(handle)
            assert lib.ddMotorAppliedDutyLeft(handle) == 0.0
            assert lib.ddMotorAppliedDutyRight(handle) == 0.0
    finally:
        lib.ddDestroy(handle)


def test_estop_latches_zero_and_holds_until_cleared(tmp_path):
    lib = _load_shim(tmp_path)
    handle = _new_kernel(lib)
    try:
        status = lib.ddDrive(handle, 500.0, 0.0, 5000)
        assert status == STATUS_OK
        for _ in range(5):
            lib.ddStep(handle)
        assert lib.ddMotorAppliedDutyLeft(handle) != 0.0, "not driving yet"

        lib.ddEstop(handle)
        # ONE cycle is enough: estopLatch_ is checked every step(),
        # independent of the lease (differential_drive.cpp's step()).
        lib.ddStep(handle)
        assert lib.ddEstopped(handle) == 1
        assert lib.ddMotorAppliedDutyLeft(handle) == 0.0
        assert lib.ddMotorAppliedDutyRight(handle) == 0.0

        # A new drive() is refused outright while the latch holds.
        status = lib.ddDrive(handle, 500.0, 0.0, 5000)
        assert status == STATUS_REFUSED_ESTOPPED

        # Holds for many more cycles.
        for _ in range(15):
            lib.ddStep(handle)
            assert lib.ddMotorAppliedDutyLeft(handle) == 0.0
            assert lib.ddMotorAppliedDutyRight(handle) == 0.0

        lib.ddEstopClear(handle)
        status = lib.ddDrive(handle, 500.0, 0.0, 5000)
        assert status == STATUS_OK, "drive() still refused after estopClear()"

        for _ in range(20):
            lib.ddStep(handle)
        assert lib.ddEstopped(handle) == 0
        assert lib.ddMotorAppliedDutyLeft(handle) != 0.0, (
            "driving did not resume after estopClear()")
    finally:
        lib.ddDestroy(handle)

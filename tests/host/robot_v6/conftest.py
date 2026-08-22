"""tests/host/robot_v6/conftest.py -- session-scoped build fixtures
shared by this directory's test files.

Two things get compiled here, once per test session (not once per
test), using the same `/usr/bin/c++` + explicit `-I` pattern the rest
of this repo's test suite already uses (see tests/protocol/
test_protocol_harness.py's own `_compile_shared_lib`) -- no CMake:

  - `fake_motion_lib` -- tests/protocol/fake_motion_shim.cpp +
    src/protocol/protocol_handler.cpp, as a shared library loaded via
    ctypes. This is the SAME shim tests/protocol/test_motion_reliability.py
    already builds -- reused here (not copied) so
    `inprocess_transport.InProcessTransport` drives the real
    ProtocolHandler + FakeMotionAdapter deterministically, with no wall
    clock and no subprocess, for the reliability-layer tests that need
    tight control over step() pacing.

  - `sim_binary` -- tools/sim/sim_main.cpp + src/protocol/
    protocol_handler.cpp, as a standalone executable. This is "the
    compiled host version of the firmware" tools/sim/README.md
    describes -- used by test_sim_e2e.py to prove the whole stack
    (codec + transport + Session) against a REAL process talking real
    bytes over a real pipe, not an in-process shim.
"""

from __future__ import annotations

import ctypes
import pathlib
import subprocess
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_PROTOCOL_DIR = _REPO_ROOT / "src" / "protocol"
_TESTS_PROTOCOL_DIR = _REPO_ROOT / "tests" / "protocol"
_TOOLS_SIM_DIR = _REPO_ROOT / "tools" / "sim"

# So `import robot_v6` works even when a test file is invoked in a way
# that bypasses pyproject.toml's own `pythonpath` ini option (e.g. a
# bare `python -m pytest <this file>` run from an unusual cwd).
_SRC_HOST_DIR = _REPO_ROOT / "src" / "host"
if str(_SRC_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_HOST_DIR))


def _compile(sources: list[pathlib.Path], include_dirs: list[pathlib.Path],
             out_path: pathlib.Path, *, extra_flags: list[str] | None = None) -> None:
    cmd = ["/usr/bin/c++", "-std=c++20", "-O0", "-g", "-Wall", "-Wextra"]
    cmd += extra_flags or []
    for d in include_dirs:
        cmd += ["-I", str(d)]
    cmd += [str(s) for s in sources] + ["-o", str(out_path)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"compile failed:\ncmd: {' '.join(cmd)}\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}")


@pytest.fixture(scope="session")
def fake_motion_lib(tmp_path_factory):
    """A loaded, fully-bound ctypes handle onto
    tests/protocol/fake_motion_shim.cpp -- see that file's own exported
    surface (fmCreate/fmFeed/fmStep/fmSinkRead/...) for what is bound.
    """
    build_dir = tmp_path_factory.mktemp("robot_v6_fake_motion_lib")
    lib_path = build_dir / "libfake_motion_shim.so"
    _compile(
        [_PROTOCOL_DIR / "protocol_handler.cpp",
         _TESTS_PROTOCOL_DIR / "fake_motion_shim.cpp"],
        [_PROTOCOL_DIR, _TESTS_PROTOCOL_DIR],
        lib_path,
        extra_flags=["-shared", "-fPIC"],
    )
    lib = ctypes.CDLL(str(lib_path))

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

    return lib


@pytest.fixture(scope="session")
def sim_binary(tmp_path_factory) -> pathlib.Path:
    """Path to a freshly-built tools/sim executable (see tools/sim/
    README.md), built once for the whole test session."""
    build_dir = tmp_path_factory.mktemp("robot_v6_sim_binary")
    exe_path = build_dir / "robot_sim"
    _compile(
        [_TOOLS_SIM_DIR / "sim_main.cpp", _PROTOCOL_DIR / "protocol_handler.cpp"],
        [_PROTOCOL_DIR, _TESTS_PROTOCOL_DIR],
        exe_path,
    )
    return exe_path

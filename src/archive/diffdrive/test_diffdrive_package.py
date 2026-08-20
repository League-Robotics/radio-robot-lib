"""src/tests/diffdrive/test_diffdrive_package.py -- the DiffDrive package
gate: standalone build + control-law fidelity.

``src/firm/diffdrive/`` is a self-contained differential-drive wheel
kernel: ONE class, TWO files, four small ports (Motor/Clock/Sleeper/
FiberLauncher) declared in its own header, and no include beyond
``<cstdint>``/``<algorithm>``/``<cmath>``. It exists to be lifted into a
MakeCode/PXT package and a MicroPython C module; the firmware reconnects
it through one-line forwarding adapters instead of inheritance.

Two claims, each with its own test:

1. STANDALONE: the package compiles with an include path of EXACTLY its
   own directory -- no firmware headers reachable at all. The compile
   flags below ARE the proof; if anyone adds a firmware include to the
   package, this test fails on the spot, which is the whole point.

2. FIDELITY: the package's control law is the SAME law the firmware
   grew. ``golden_ref_drive.{h,cpp}`` is the pre-kernel pipeline frozen
   from commit ``ab43963c``; the harness drives both against identical
   plants and requires the duty they produce to agree -- feedforward
   EXACTLY (worst delta 0.000000), closed loop at steady state to well
   under the device's own duty quantum. See the harness header for why
   the transient is reported, not asserted.

Run with::

    uv run python -m pytest src/tests/diffdrive/ -v -s
"""

import pathlib
import subprocess

import pytest

# src/tests/diffdrive/test_diffdrive_package.py -> diffdrive -> tests -> src -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_PACKAGE_DIR = _REPO_ROOT / "src" / "firm" / "diffdrive"
_TEST_DIR = pathlib.Path(__file__).resolve().parent

_PACKAGE_SOURCES = [
    _PACKAGE_DIR / "differential_drive.cpp",
]
_HARNESS_SOURCES = [
    _TEST_DIR / "fidelity_harness.cpp",
    _TEST_DIR / "golden_ref_drive.cpp",
]


def _compile(tmp_path, sources, include_dirs, out_name):
    binary = tmp_path / out_name
    cmd = ["/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra"]
    for d in include_dirs:
        cmd += ["-I", str(d)]
    cmd += [str(s) for s in sources] + ["-o", str(binary)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"compile failed:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return binary


def test_package_is_standalone(tmp_path):
    """The package compiles seeing ONLY its own directory.

    This is the structural guarantee the MakeCode/MicroPython targets
    depend on: copy two files, implement four ports, done. A firmware
    include creeping into the package makes this fail immediately.
    """
    obj = tmp_path / "differential_drive.o"
    result = subprocess.run(
        ["/usr/bin/c++", "-std=c++20", "-c",
         str(_PACKAGE_DIR / "differential_drive.cpp"),
         "-I", str(_PACKAGE_DIR), "-o", str(obj)],
        capture_output=True, text=True)
    assert result.returncode == 0, (
        f"the package does NOT build standalone:\n{result.stdout}\n{result.stderr}")
    assert obj.is_file()


def test_feedforward_and_stage_a_match_the_firmware_law_exactly(tmp_path):
    """kp=ki=0: pure feedforward through Stage A. Worst duty delta
    0.000000 against the frozen pre-kernel law -- identical, not merely
    within tolerance."""
    binary = _compile(tmp_path, _HARNESS_SOURCES + _PACKAGE_SOURCES,
                      [_PACKAGE_DIR, _TEST_DIR], "fidelity")
    result = subprocess.run([str(binary), "openloop"], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr


def test_closed_loop_settles_where_the_firmware_law_settles(tmp_path):
    """Integral engaged: both laws settle to the same duty (steady-state
    mean deltas measured at 0.000957 and 0.000038 -- far under the ~0.01
    duty quantum the target device can express). The transient ripple is
    REPORTED, not asserted: the two couple samples to control differently
    by construction (see the harness header)."""
    binary = _compile(tmp_path, _HARNESS_SOURCES + _PACKAGE_SOURCES,
                      [_PACKAGE_DIR, _TEST_DIR], "fidelity")
    result = subprocess.run([str(binary), "integral"], capture_output=True, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr

"""tests/protocol/test_protocol_package.py -- the protocol package's
standalone-build gate, mirroring tests/diffdrive/test_diffdrive_package.py
(Step 2's own pattern, applied to Step 3).

``src/protocol/`` is meant to be as portable as ``src/diffdrive/``: no
kernel, no motors, no transport, no include beyond the C++ standard
library. This test is the proof -- it compiles protocol_handler.cpp
with an include path of EXACTLY its own directory (not
tests/protocol/, where the mock adapter and shim live). If a stray
firmware or test-scaffolding include ever creeps into the library,
this fails on the spot.

Run with::

    uv run python -m pytest tests/protocol/test_protocol_package.py -v -s
"""

import pathlib
import subprocess

# tests/protocol/test_protocol_package.py -> protocol -> tests -> root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
_PACKAGE_DIR = _REPO_ROOT / "src" / "protocol"


def test_package_is_standalone(tmp_path):
    object_file = tmp_path / "protocol_handler.o"
    cmd = [
        "/usr/bin/c++", "-std=c++20", "-Wall", "-Wextra",
        "-I", str(_PACKAGE_DIR),
        "-c", str(_PACKAGE_DIR / "protocol_handler.cpp"),
        "-o", str(object_file),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    assert result.returncode == 0, (
        f"standalone compile failed -- a stray include reached outside "
        f"src/protocol/:\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    assert object_file.exists()

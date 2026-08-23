"""tests/host/rogo/conftest.py -- fixtures shared by this directory's
test files.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rogo import connection


@pytest.fixture(scope="session")
def built_sim_binary() -> Path:
    """Builds tools/sim exactly the way a real `rogo --sim` invocation
    would -- via `rogo.connection.ensure_sim_binary()` itself, not a
    copy of its compile command (unlike tests/host/robot_v6/conftest.py's
    own `sim_binary` fixture, which compiles independently to prove the
    TEST harness's own build pattern works; this one proves the
    PRODUCTION `--sim` code path works). Cached under tools/sim/.build/
    (mtime-checked by `ensure_sim_binary()` itself), so this fixture
    only pays a real compile cost once across this whole session, and
    not at all on a second `pytest` run against a clean/unstaled build.
    """
    return connection.ensure_sim_binary()

"""tests/host/rogo/test_end_to_end_sim.py -- ticket 008's own AC #1: one
scripted end-to-end smoke test exercising the FULL ported `rogo` command
surface (hello, drive, turn, goto, config get/set, a scripted
non-interactive calibrate run, repl, and one mcp tool call) against a
single, freshly built `tools/sim` binary, in the sequence sprint.md's
own Description names them.

Every step below reuses the exact same primitives (`cli.main()`,
`rogo.connection.resolve()`, `rogo.mcp_server.build_server()`) the
per-ticket test files (003-007) already established and already prove
correct in isolation; this file's own job is proving the SEAMS between
them -- that ticket 003's drive/turn, ticket 004's goto/config, ticket
005's calibrate, ticket 006's repl, and ticket 007's mcp server all
still work correctly run back-to-back in one process against one
`tools/sim` build, rather than each only ever having been exercised on
its own. Each `--sim` invocation still resolves its own subprocess
(exactly like every other end-to-end test in this directory) except the
repl and mcp steps, which each hold one connection open for their own
duration -- "one scripted session" here means one test asserting the
whole surface in sequence, not one shared transport spanning all eight
steps.

Turn/calibrate/the mcp `turn` tool all need an active robot config with
a `trackwidth`; `cli.config.load_active_robot()` is monkeypatched for
the whole test to a `tmp_path` COPY of
`tests/host/rogo/fixtures/gopiv.json` (never the real
`config/robots/*.json`) -- `rogo.mcp_server` shares the same
`rogo.config` module object (`from . import config` in both), so this
one monkeypatch covers `turn`, `calibrate`, and the mcp server's own
`turn` tool alike.

`config get`/`config set` against `tools/sim` hit
`Protocol::FakeMotionAdapter`'s documented "no config table here to be
wrong about" behavior (every name unknown, matching
test_cli_goto_config.py's own module docstring) -- this test asserts
that real, honest behavior rather than a round trip `tools/sim`
structurally cannot produce.
"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

import pytest

from rogo import cli, config, connection, mcp_server

_FIXTURE = Path(__file__).parent / "fixtures" / "gopiv.json"


def _fixture_copy(tmp_path: Path) -> Path:
    dest = tmp_path / "gopiv.json"
    dest.write_text(_FIXTURE.read_text())
    return dest


def _scripted_input(responses: list[str]):
    """Mirrors test_calibrate.py's own `_scripted_input()` helper: a
    fake `input_fn` (here installed as `builtins.input`) backed by an
    explicit, already-known list of responses, raising `EOFError` once
    exhausted -- exactly like real `input()` hitting a closed stream."""
    it = iter(responses)

    def _input(prompt: str = "") -> str:
        del prompt
        try:
            return next(it)
        except StopIteration:
            raise EOFError

    return _input


def test_full_command_surface_runs_end_to_end_against_sim(
    built_sim_binary, tmp_path, monkeypatch, capsys
):
    del built_sim_binary
    fixture_path = _fixture_copy(tmp_path)
    fixture_cfg = config.load_robot_config(fixture_path)
    monkeypatch.setattr(cli.config, "load_active_robot", lambda: fixture_cfg)

    # 1. hello -- unsequenced probe, proves the target is alive and
    #    answering (protocol.md#8.3's boot-banner-is-HELLO's-own-reply
    #    guarantee).
    exit_code = cli.main(["hello", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "name=sim" in out

    # 2. drive -- one WHEELS_V call, DiffDriveAdapter's one verb with
    #    real kinematic effect today (protocol.md#5).
    exit_code = cli.main(["drive", "100", "100", "--ms", "60", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "WHEELS_V acked" in out
    assert "done reason=" in out

    # 3. turn -- the ported rotation model (turn_model.compute_turn)
    #    computes one WHEELS_V call from the fixture's own
    #    trackwidth/rotational_slip.
    exit_code = cli.main(["turn", "45", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "WHEELS_V" in out
    assert "done reason=" in out

    # 4. goto -- one GO_TO_R call, robot-frame only (sprint.md's Design
    #    Rationale Decision 3: no camera-based closed loop, no
    #    world-frame go_to_w). tools/sim's FakeMotionAdapter accepts and
    #    completes all six motion verbs (unlike the real
    #    DiffDriveAdapter's kUnknown planner gap), so this reports a
    #    real completion outcome here, not a soft warning.
    exit_code = cli.main(["goto", "300", "0", "--speed", "100", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "GO_TO_R 300 0 100" in out
    assert "done reason=" in out

    # 5. config set/get -- FakeMotionAdapter has no config table at all
    #    (test_cli_goto_config.py's own module docstring): every name is
    #    unknown, honestly reported as an error, never a silent "success".
    exit_code = cli.main(["config", "set", "wheel_control.pid_kp", "1.5", "--sim"])
    err = capsys.readouterr().err
    assert exit_code == 1
    assert "no such config field" in err

    exit_code = cli.main(["config", "get", "--sim"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "no config fields" in out

    # 6. calibrate turns -- scripted, non-interactive operator input
    #    (per this ticket's own Description); writes only to the
    #    tmp_path fixture copy, never the checked-in fixture or a real
    #    config/robots/*.json file.
    monkeypatch.setattr(
        "builtins.input", _scripted_input(["", "99", "", "99", "", "99", "y"]))
    exit_code = cli.main(["calibrate", "turns", "--trials", "3", "--sim"])
    capsys.readouterr()
    assert exit_code == 0
    on_disk = json.loads(fixture_path.read_text())
    assert on_disk["geometry"]["rotational_slip"] == pytest.approx(1.1)
    original = json.loads(_FIXTURE.read_text())
    assert original["geometry"]["rotational_slip"] == 1.0  # checked-in fixture untouched

    # 7. repl -- an argument-list session over one persistent connection,
    #    reusing the same per-verb dispatch drive/turn/goto/config use
    #    (ticket 006's own AC #1).
    exit_code = cli.main(
        ["repl", "--sim", "hello", "drive 100 100 --ms 50", "stop"])
    out = capsys.readouterr().out
    assert exit_code == 0
    assert "name=sim" in out
    assert "WHEELS_V acked" in out
    assert "STOP acked" in out

    # 8. mcp -- one tool call against the server ticket 007 built,
    #    exercising the same underlying primitives outside rogo.cli's
    #    own print()-based reporting (mcp_server's own module docstring).
    ns = argparse.Namespace(sim=True, connect=None, port=None)
    conn = connection.resolve(ns)
    try:
        server = mcp_server.build_server(conn.session)
        result = asyncio.run(server.call_tool("turn", {"degrees": 30.0}))
    finally:
        conn.transport.close()
    body = json.loads(result.content[0].text)
    assert body["acked"] is True
    assert body["done_reason"]
    assert body["degrees"] == 30.0
    assert "cmd_l" in body and "cmd_r" in body and "duration_ms" in body

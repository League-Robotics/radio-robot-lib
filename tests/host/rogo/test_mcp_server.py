"""tests/host/rogo/test_mcp_server.py -- ticket 007's `rogo mcp` MCP
tool server: thin unit tests per tool -- schema shape plus dispatch to
the right underlying `robot_v6.motion`/`rogo.config`/`rogo.calibrate`
call, per this ticket's own Testing plan ("not a full MCP-protocol
integration test") -- plus the unreachable-target error path (AC #3)
and the localhost-default `--listen` binding rule (AC #2).

Tools are called directly via `MCPServer.list_tools()`/`.call_tool()`
against a fake in-process transport, mirroring
tests/host/rogo/test_cli_drive_turn.py's own `_ScriptedReplyTransport`
pattern -- a second, independent copy of that small fixture here for
the same reason `rogo.mcp_server` itself doesn't import `rogo.cli`
(see that module's own docstring): this file exercises `mcp_server` in
isolation. No real stdio/JSON-RPC wire, no `tools/sim` subprocess.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

import pytest

from robot_v6.reliability import Session
from robot_v6.transport import Transport, TransportClosed

from rogo import cli, mcp_server
from rogo import config as rogo_config
from rogo import turn_model

_FIXTURE = Path(__file__).parent / "fixtures" / "gopiv.json"


def _fixture_copy(tmp_path: Path) -> Path:
    dest = tmp_path / "gopiv.json"
    dest.write_text(_FIXTURE.read_text())
    return dest


# ---------------------------------------------------------------------------
# Fake transports.
# ---------------------------------------------------------------------------

class _ScriptedReplyTransport(Transport):
    """Replays a fixed sequence of `_read_chunk()` results, one per
    call; records every line written."""

    def __init__(self, chunks: list[bytes]):
        super().__init__()
        self._chunks = list(chunks)
        self.written: list[str] = []

    def _read_chunk(self, timeout):
        del timeout
        if not self._chunks:
            return b""
        return self._chunks.pop(0)

    def _write_bytes(self, data: bytes) -> None:
        self.written.append(data.decode("ascii").rstrip("\n"))

    def close(self) -> None:
        pass


class _RaisingTransport(Transport):
    """Every read raises `TransportClosed` immediately -- simulates a
    sim subprocess that already exited / a socket the peer already
    closed. Used to prove AC #3's "rather than hanging": a tool call
    against this transport must fail FAST, not just eventually."""

    def _read_chunk(self, timeout):
        del timeout
        raise TransportClosed("peer gone")

    def _write_bytes(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def _call(server, name: str, arguments: dict[str, Any]):
    return asyncio.run(server.call_tool(name, arguments))


def _result_json(result) -> Any:
    return json.loads(result.content[0].text)


# ---------------------------------------------------------------------------
# Schema shape -- AC #1's "corresponding MCP tool" for each of
# drive/turn/goto/config get/config set, plus AC #4's calibrate tool
# and the hello/stop pair this module also exposes.
# ---------------------------------------------------------------------------

def test_list_tools_exposes_every_expected_tool_with_correct_required_args():
    session = Session(_ScriptedReplyTransport([]))
    server = mcp_server.build_server(session)

    tools = {tool.name: tool for tool in asyncio.run(server.list_tools())}

    assert set(tools) == {
        "hello", "stop", "drive", "turn", "goto",
        "config_get", "config_set", "calibrate_turns",
    }
    assert set(tools["hello"].input_schema.get("required", [])) == set()
    assert set(tools["stop"].input_schema.get("required", [])) == set()
    assert set(tools["drive"].input_schema["required"]) == {"left", "right", "ms"}
    assert set(tools["turn"].input_schema["required"]) == {"degrees"}
    assert "speed" in tools["turn"].input_schema["properties"]
    assert set(tools["goto"].input_schema["required"]) == {"x", "y"}
    assert {"speed", "arrive", "timeout_ms"} <= set(tools["goto"].input_schema["properties"])
    assert set(tools["config_get"].input_schema.get("required", [])) == set()
    assert set(tools["config_set"].input_schema["required"]) == {"name", "value"}
    assert set(tools["calibrate_turns"].input_schema["required"]) == {"measured_degrees"}
    assert {"target_deg", "save"} <= set(tools["calibrate_turns"].input_schema["properties"])


# ---------------------------------------------------------------------------
# hello / stop -- unsequenced probe and sequenced STOP.
# ---------------------------------------------------------------------------

def test_hello_dispatches_unsequenced_hello_and_parses_device_banner():
    transport = _ScriptedReplyTransport([b"device relay gopiv robot 407711\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "hello", {})

    assert transport.written == ["HELLO"]
    assert _result_json(result) == {
        "role": "relay", "common_name": "gopiv", "name": "robot", "serial": "407711",
    }


def test_stop_dispatches_motion_stop_and_reports_ack():
    transport = _ScriptedReplyTransport([b"ack 1 0 none\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "stop", {})

    assert transport.written == ["STOP #1"]
    assert _result_json(result) == {"acked": True, "seq_id": 1}


# ---------------------------------------------------------------------------
# drive -- one WHEELS_V call.
# ---------------------------------------------------------------------------

def test_drive_dispatches_wheels_v_and_reports_ack_and_done_reason():
    transport = _ScriptedReplyTransport([b"ack 1 1 done\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "drive", {"left": 100, "right": -100, "ms": 500})

    assert transport.written == ["WHEELS_V 100 -100 500 #1"]
    assert _result_json(result) == {"acked": True, "seq_id": 1, "done_reason": "done"}


# ---------------------------------------------------------------------------
# turn -- turn_model.compute_turn() feeding one WHEELS_V call, using the
# active robot's config (mirrors rogo.cli's own _prepare_turn()).
# ---------------------------------------------------------------------------

def test_turn_computes_wheels_v_from_active_config_and_reports_done(tmp_path, monkeypatch):
    cfg = rogo_config.load_robot_config(_fixture_copy(tmp_path))
    monkeypatch.setattr(mcp_server.config, "load_active_robot", lambda: cfg)
    cmd_l, cmd_r, duration_ms = turn_model.compute_turn(
        90.0, mcp_server._DEFAULT_TURN_SPEED_MM_S, cfg.trackwidth_mm, cfg.rotational_slip)

    transport = _ScriptedReplyTransport([b"ack 1 1 done\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "turn", {"degrees": 90.0})

    assert transport.written == [f"WHEELS_V {cmd_l} {cmd_r} {duration_ms} #1"]
    body = _result_json(result)
    assert body["acked"] is True
    assert body["done_reason"] == "done"
    assert body["cmd_l"] == cmd_l
    assert body["cmd_r"] == cmd_r
    assert body["duration_ms"] == duration_ms


def test_turn_reports_error_dict_when_no_active_robot_config(monkeypatch):
    monkeypatch.setattr(mcp_server.config, "load_active_robot", lambda: None)
    session = Session(_ScriptedReplyTransport([]))
    server = mcp_server.build_server(session)

    result = _call(server, "turn", {"degrees": 45.0})

    assert result.is_error is False  # a config problem is data, not a tool error
    assert "error" in _result_json(result)


# ---------------------------------------------------------------------------
# goto -- one GO_TO_R call, including the kUnknown soft-warning path
# (STAKEHOLDER DECISION: reported in the result, tool call still
# succeeds).
# ---------------------------------------------------------------------------

def test_goto_dispatches_go_to_r_and_reports_soft_warning_on_kunknown():
    transport = _ScriptedReplyTransport([b"ack 1 0 none\nerr 1 #1\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "goto", {"x": 300, "y": 0, "timeout_ms": 4000})

    assert transport.written == ["GO_TO_R 300 0 200 0 4000 #1"]
    assert result.is_error is False
    body = _result_json(result)
    assert body["acked"] is True
    assert "kUnknown" in body["warning"] or "ERR_UNKNOWN" in body["warning"]


def test_goto_default_timeout_is_computed_from_distance_and_speed():
    transport = _ScriptedReplyTransport([b"ack 1 1 done\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "goto", {"x": 400, "y": 0, "speed": 200})

    # ETA = 1000*400/200 = 2000ms; 3x multiple -> 6000ms (mirrors
    # rogo.cli's own _goto_default_timeout_ms()).
    assert transport.written == ["GO_TO_R 400 0 200 0 6000 #1"]
    assert _result_json(result)["timeout_ms"] == 6000


# ---------------------------------------------------------------------------
# config_get / config_set -- protocol.md#7's GET/SET delegation.
# ---------------------------------------------------------------------------

def test_config_get_named_field_dispatches_get_with_name():
    transport = _ScriptedReplyTransport([b"ack 1 0 none\nget geometry.trackwidth 128\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "config_get", {"name": "geometry.trackwidth"})

    assert transport.written == ["GET geometry.trackwidth #1"]
    assert _result_json(result) == {"fields": {"geometry.trackwidth": "128"}}


def test_config_get_unknown_field_returns_error_dict_not_raised():
    transport = _ScriptedReplyTransport([b"ack 1 0 none\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "config_get", {"name": "no.such.field"})

    assert result.is_error is False
    assert "error" in _result_json(result)


def test_config_set_dispatches_set_and_reports_ack():
    transport = _ScriptedReplyTransport([b"ack 1 0 none\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "config_set", {"name": "wheel_control.pid_kp", "value": 0.002})

    assert transport.written == ["SET wheel_control.pid_kp 0.002 #1"]
    assert _result_json(result) == {
        "acked": True, "seq_id": 1, "name": "wheel_control.pid_kp", "value": 0.002,
    }


def test_config_set_unknown_field_returns_error_dict_not_raised():
    transport = _ScriptedReplyTransport([b"ack 1 0 none\nerr 1 #1\n"])
    session = Session(transport)
    server = mcp_server.build_server(session)

    result = _call(server, "config_set", {"name": "bogus", "value": 1.0})

    assert result.is_error is False
    body = _result_json(result)
    assert body["acked"] is True
    assert "no such config field" in body["error"]


# ---------------------------------------------------------------------------
# calibrate_turns -- AC #4's non-interactive core: no session traffic,
# no input() -- an explicit list of already-measured values in, an
# updated value (and optional save) out.
# ---------------------------------------------------------------------------

def test_calibrate_turns_computes_updated_value_and_does_not_save_by_default(tmp_path, monkeypatch):
    path = _fixture_copy(tmp_path)
    cfg = rogo_config.load_robot_config(path)
    monkeypatch.setattr(mcp_server.config, "load_active_robot", lambda: cfg)
    session = Session(_ScriptedReplyTransport([]))
    server = mcp_server.build_server(session)

    # gopiv's starting rotational_slip is 1.0; six trials all measuring
    # exactly the 90-degree target -> mean_ratio 1.0, updated_value 1.0.
    result = _call(server, "calibrate_turns", {
        "measured_degrees": [90.0, 90.0, 90.0, 90.0, 90.0, 90.0],
    })

    body = _result_json(result)
    assert body["trials_used"] == 6
    assert body["mean_ratio"] == 1.0
    assert body["updated_value"] == 1.0
    assert body["saved"] is False
    # Never drives the robot -- no wire traffic at all.
    assert session.pending_count == 0

    # Not saved: the fixture copy on disk is untouched.
    saved = rogo_config.load_robot_config(path)
    assert saved.rotational_slip == 1.0


def test_calibrate_turns_saves_when_requested(tmp_path, monkeypatch):
    path = _fixture_copy(tmp_path)
    cfg = rogo_config.load_robot_config(path)
    monkeypatch.setattr(mcp_server.config, "load_active_robot", lambda: cfg)
    session = Session(_ScriptedReplyTransport([]))
    server = mcp_server.build_server(session)

    result = _call(server, "calibrate_turns", {
        "measured_degrees": [99.0, 99.0, 99.0, 99.0, 99.0, 99.0],
        "save": True,
    })

    body = _result_json(result)
    assert body["updated_value"] == pytest.approx(99.0 / 90.0)
    assert body["saved"] is True

    saved = rogo_config.load_robot_config(path)
    assert saved.rotational_slip == pytest.approx(body["updated_value"])


def test_calibrate_turns_insufficient_trials_reports_no_update_without_saving(tmp_path, monkeypatch):
    cfg = rogo_config.load_robot_config(_fixture_copy(tmp_path))
    monkeypatch.setattr(mcp_server.config, "load_active_robot", lambda: cfg)
    session = Session(_ScriptedReplyTransport([]))
    server = mcp_server.build_server(session)

    result = _call(server, "calibrate_turns", {
        "measured_degrees": [90.0], "save": True,
    })

    body = _result_json(result)
    assert body["updated_value"] is None
    assert body["saved"] is False


def test_calibrate_turns_no_active_config_returns_error_dict(monkeypatch):
    monkeypatch.setattr(mcp_server.config, "load_active_robot", lambda: None)
    session = Session(_ScriptedReplyTransport([]))
    server = mcp_server.build_server(session)

    result = _call(server, "calibrate_turns", {"measured_degrees": [90.0] * 6})

    assert result.is_error is False
    assert "error" in _result_json(result)


# ---------------------------------------------------------------------------
# Unreachable target -- AC #3: surfaces through the MCP error channel
# rather than hanging. `_RaisingTransport` fails on the very first
# read, so a hang would show up as this test itself hanging -- pytest's
# own default per-test behavior (no timeout plugin needed) makes that
# visible rather than silently tolerated.
# ---------------------------------------------------------------------------

def test_unreachable_target_surfaces_as_a_tool_error_not_a_hang():
    session = Session(_RaisingTransport())
    server = mcp_server.build_server(session)

    with pytest.raises(Exception, match="connection closed"):
        _call(server, "drive", {"left": 100, "right": 100, "ms": 100})


def test_unreachable_target_during_hello_also_raises():
    session = Session(_RaisingTransport())
    server = mcp_server.build_server(session)

    with pytest.raises(Exception, match="connection closed"):
        _call(server, "hello", {})


# ---------------------------------------------------------------------------
# --listen's localhost-default binding rule -- AC #2, this ticket's own
# binding security requirement (sprint.md's Migration Concerns).
# ---------------------------------------------------------------------------

def test_resolve_listen_target_defaults_to_none_no_listen_flag():
    # No --listen at all -> serve() uses stdio, no network surface --
    # the strongest possible reading of "binds to 127.0.0.1 by default".
    assert mcp_server.resolve_listen_target(None, False) is None


def test_resolve_listen_target_allows_loopback_without_allow_remote():
    assert mcp_server.resolve_listen_target("127.0.0.1:8765", False) == ("127.0.0.1", 8765)
    assert mcp_server.resolve_listen_target("localhost:8765", False) == ("localhost", 8765)


def test_resolve_listen_target_rejects_non_loopback_without_allow_remote():
    with pytest.raises(mcp_server.ListenTargetError, match="allow-remote"):
        mcp_server.resolve_listen_target("0.0.0.0:8765", False)


def test_resolve_listen_target_allows_non_loopback_with_allow_remote():
    assert mcp_server.resolve_listen_target("0.0.0.0:8765", True) == ("0.0.0.0", 8765)


def test_resolve_listen_target_rejects_malformed_value():
    with pytest.raises(mcp_server.ListenTargetError):
        mcp_server.resolve_listen_target("not-a-host-port", False)


def test_cli_mcp_subcommand_defaults_listen_to_none_and_allow_remote_to_false():
    parser = cli.build_parser()
    args = parser.parse_args(["mcp", "--sim"])

    assert args.listen is None
    assert args.allow_remote is False
    assert args.func is cli.cmd_mcp

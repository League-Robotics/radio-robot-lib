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

Ticket 010 changes `cli.cmd_mcp()` to resolve its connection through
`daemon_client.get_connection(args, spawn=True)` (auto-spawn a daemon
when none is running for the resolved target, exactly like `cmd_repl()`
already does, ticket 009) rather than `connection.resolve()` directly.
Every test above this point is unaffected -- each builds `mcp_server.
build_server(session)` directly around an already-constructed `Session`,
never going through `cli.cmd_mcp()`'s own connection acquisition at
all -- proving the tool-body/wire behavior is UNCHANGED (this ticket's
own ACs #2/#3). The section below this point is new: it exercises
`cli.cmd_mcp()`'s own daemon-client connection acquisition end to end
against the real compiled `tools/sim` binary (auto-spawn, reuse, and a
concurrent one-shot command sharing the same daemon -- this ticket's
own AC #5), mirroring test_cli_serve.py's own `rogo repl` auto-spawn
tests. `mcp_server.serve()` itself is stubbed out in that section (it
blocks forever running the real MCP stdio protocol loop, which is not
what daemon-connection acquisition is about) so each test there
observes exactly the connection `cmd_mcp()` resolved, then returns
before that blocking loop would ever start.
"""

from __future__ import annotations

import asyncio
import json
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Iterator

import pytest

from robot_v6.reliability import Session
from robot_v6.transport import Transport, TransportClosed

from rogo import cli, daemon_client, mcp_server
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


# ---------------------------------------------------------------------------
# cli.cmd_mcp()'s own daemon-client connection acquisition (ticket 010) --
# see module docstring for why `mcp_server.serve()` is stubbed out below
# and why `isolated_socket_dir` is required on every test in this section
# (never touch this machine's real ~/.rogo/run/$XDG_RUNTIME_DIR, or leave
# a background daemon running against it -- test_cli_serve.py's own
# `isolated_socket_dir` fixture, duplicated here rather than imported
# across test files, matches this project's own established
# duplicate-rather-than-couple precedent, e.g. daemon.py's
# `_force_line_buffered()`).
# ---------------------------------------------------------------------------

def _terminate(proc: subprocess.Popen, *, timeout: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout)


def _short_tmp_dir() -> Path:
    """A short-path temp directory under `/tmp` -- AF_UNIX's `sun_path`
    has an OS-enforced length limit (~104 bytes on macOS) that pytest's
    own `tmp_path` fixture can exceed once nested; see test_cli_serve.py's/
    test_daemon_client.py's own identical helper/rationale."""
    return Path(tempfile.mkdtemp(prefix="rogo-mcp-", dir="/tmp"))


@pytest.fixture
def isolated_socket_dir(monkeypatch) -> Iterator[Path]:
    """Redirect `daemon.default_socket_dir()` -- and so every
    `daemon_client`/`cmd_mcp()`/`rogo serve` lookup that does not pass an
    explicit `--socket-dir`/`socket_dir=` -- at a per-test SHORT tmp
    directory, via the same `XDG_RUNTIME_DIR` precedence
    `daemon.default_socket_dir()` documents. A subprocess spawned from
    within a test using this fixture inherits the modified env
    (`subprocess.Popen`'s own default `env=None` behavior)."""
    xdg_dir = _short_tmp_dir()
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(xdg_dir))
    try:
        yield xdg_dir / "rogo"
    finally:
        shutil.rmtree(xdg_dir, ignore_errors=True)


def _spawn_serve(*extra_args: str) -> subprocess.Popen:
    argv = [sys.executable, "-m", "rogo.cli", "serve", "--sim", *extra_args]
    return subprocess.Popen(  # noqa: S603
        argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, start_new_session=True,
    )


def _wait_for_daemon(name: str, socket_dir: Path, *, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        found = daemon_client.find_daemon(name, socket_dir=socket_dir, timeout=0.2)
        if found is not None:
            return found
        time.sleep(0.05)
    return None


def _stub_serve(captured: list):
    """Replaces `mcp_server.serve()` for a test: records the `session`
    `cmd_mcp()` resolved and returns 0 immediately, instead of blocking
    forever on the real MCP stdio protocol loop (module docstring)."""
    def _serve(session, *, listen=None, allow_remote=False):
        del listen, allow_remote
        captured.append(session)
        return 0
    return _serve


def test_cmd_mcp_without_a_resolvable_name_reports_a_clean_error_not_a_hang(capsys):
    # --connect with neither --sim nor --name: resolve_client_name()
    # returns None (daemon_client.py's own module docstring) --
    # get_connection(spawn=True) raises RobotNameRequiredError before
    # ever attempting a connection, exactly like cmd_repl() would for
    # the same target -- reported as a clean error, not a hang or a
    # traceback.
    exit_code = cli.main(["mcp", "--connect", "127.0.0.1:1"])
    err = capsys.readouterr().err

    assert exit_code == 1
    assert "error:" in err


def test_cmd_mcp_validates_listen_target_before_resolving_a_connection(monkeypatch, capsys):
    # AC #4: --listen/--allow-remote's binding rule is validated BEFORE
    # daemon_client.get_connection() is ever called -- a disallowed
    # --listen must fail fast, with no daemon lookup/spawn attempted at
    # all (unaffected by ticket 010's own connection-resolution change).
    def _forbid_get_connection(args, **kwargs):
        del args, kwargs
        raise AssertionError("must not resolve a connection before validating --listen")
    monkeypatch.setattr(daemon_client, "get_connection", _forbid_get_connection)

    exit_code = cli.main(["mcp", "--sim", "--listen", "0.0.0.0:8765"])
    err = capsys.readouterr().err

    assert exit_code == 2
    assert "allow-remote" in err


def test_cmd_mcp_auto_spawns_a_daemon_when_none_is_running(
    built_sim_binary, isolated_socket_dir, monkeypatch,
):
    del built_sim_binary
    spawned: list[subprocess.Popen] = []
    real_spawn = daemon_client._spawn_daemon

    def _tracking_spawn(argv):
        proc = real_spawn(argv)
        spawned.append(proc)
        return proc
    monkeypatch.setattr(daemon_client, "_spawn_daemon", _tracking_spawn)

    captured: list = []
    monkeypatch.setattr(mcp_server, "serve", _stub_serve(captured))

    try:
        exit_code = cli.main(["mcp", "--sim"])

        assert exit_code == 0
        assert len(spawned) == 1, "rogo mcp must auto-spawn exactly one daemon when none is running"
        assert len(captured) == 1

        # The auto-spawned daemon outlives the mcp invocation that
        # spawned it (by design -- another client may reuse it) -- prove
        # it is a REAL, still-reachable Unix-socket daemon.
        found = daemon_client.find_daemon("sim", socket_dir=isolated_socket_dir, timeout=2.0)
        assert found is not None
        found.transport.close()
    finally:
        for proc in spawned:
            _terminate(proc)


def test_cmd_mcp_reuses_an_already_running_daemon_without_spawning_a_new_one(
    built_sim_binary, isolated_socket_dir, monkeypatch,
):
    del built_sim_binary
    proc = _spawn_serve("--idle-timeout", "30.0")
    try:
        assert _wait_for_daemon("sim", isolated_socket_dir, timeout=10.0) is not None

        def _forbid_spawn(argv):
            raise AssertionError(
                f"must reuse the already-running daemon, not spawn a new one: {argv!r}")
        monkeypatch.setattr(daemon_client, "_spawn_daemon", _forbid_spawn)

        captured: list = []
        monkeypatch.setattr(mcp_server, "serve", _stub_serve(captured))

        exit_code = cli.main(["mcp", "--sim"])

        assert exit_code == 0
        assert len(captured) == 1
    finally:
        _terminate(proc)


def test_concurrent_one_shot_command_reaches_the_same_daemon_as_an_active_mcp_session(
    built_sim_binary, isolated_socket_dir, monkeypatch, capsys,
):
    # This ticket's own AC #5 (SUC-002's multi-client acceptance
    # criterion): a one-shot `rogo drive`/`stop` invocation running
    # concurrently with an active `rogo mcp` session reaches the SAME
    # robot without contention -- proven here by session-state
    # continuity (robot_v6.reliability.Session's own client-side
    # sequence counter, starting at 1 for a fresh Session), matching
    # test_cli_serve.py's own `test_one_shot_followed_by_repl_reuses_
    # the_same_daemon_connection`.
    del built_sim_binary
    spawned: list[subprocess.Popen] = []
    real_spawn = daemon_client._spawn_daemon

    def _tracking_spawn(argv):
        proc = real_spawn(argv)
        spawned.append(proc)
        return proc
    monkeypatch.setattr(daemon_client, "_spawn_daemon", _tracking_spawn)

    captured: list = []
    monkeypatch.setattr(mcp_server, "serve", _stub_serve(captured))

    try:
        exit_code = cli.main(["mcp", "--sim"])
        assert exit_code == 0
        assert len(captured) == 1
        assert len(spawned) == 1, "the mcp session itself must have auto-spawned the daemon"

        # cmd_mcp()'s own `finally: conn.transport.close()` already
        # closed the mcp session's OWN client socket by the time
        # cli.main() returned above -- but the DAEMON PROCESS, and the
        # sim connection it holds, are unaffected
        # (daemon_client.ClientConnection's own docstring: "the daemon
        # process ... is unaffected"). A concurrent one-shot command
        # must still reach it.
        exit_code_stop = cli.main(["stop", "--sim"])
        out = capsys.readouterr().out

        assert exit_code_stop == 0
        assert len(spawned) == 1, "the one-shot command must reuse the daemon, not spawn a second one"
        assert "STOP acked (#1)" in out, (
            f"a concurrent one-shot command must reach the SAME daemon "
            f"an active rogo mcp session spawned -- got {out!r}"
        )
    finally:
        for proc in spawned:
            _terminate(proc)

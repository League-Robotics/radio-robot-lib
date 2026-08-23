---
id: '007'
title: Implement rogo mcp server
status: done
use-cases:
- SUC-005
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement rogo mcp server

## Description

Implement `rogo mcp`, an MCP server exposing the ported operations
(drive/turn/goto/config get-set/calibrate) as MCP tools, adapting
`radio-robot-elite/src/host/robot_radio/io/robot_mcp.py`'s tool-definition
shell to call `robot_v6.motion`/`rogo.config`/`rogo.calibrate` instead of
the old `SerialConnection`/`Nezha` classes — no duplicated business logic,
the same calls the CLI commands use. Per sprint.md's Migration Concerns
security note (analogous to `protocol.md#6.3`'s `RUN`-allowlist caution
and `wifi-link#11`'s no-authentication-at-this-layer caution), the server
binds to `127.0.0.1` by default and requires an explicit flag to listen
elsewhere.

## Acceptance Criteria

- [x] Each ported CLI operation (drive, turn, goto, config get/set) has a
      corresponding MCP tool with matching behavior/outcome against
      `tools/sim`.
- [x] `rogo mcp` binds to `127.0.0.1` by default; a `--listen HOST:PORT`
      -style flag is required to bind elsewhere.
- [x] A tool call targeting an unreachable robot/relay/sim surfaces a
      transport-level error through the MCP error channel rather than
      hanging.
- [x] At least one calibrate tool (e.g. `calibrate_turns`) is exposed,
      calling ticket 005's non-interactive trial-loop core (explicit
      trial count / measured values as tool arguments) rather than the
      CLI's TTY prompts, since an MCP client can't answer an `input()`
      call.
- [x] Thin unit test per tool: schema shape + dispatch to the right
      underlying call — not a full MCP-protocol integration test.

## Implementation Plan

**Approach**: `src/host/rogo/mcp_server.py` wires MCP `Server`/tool
definitions to the same `robot_v6.motion`/`rogo.config`/`rogo.calibrate`
calls the CLI commands already use.

**Files to create/modify**:
- `src/host/rogo/mcp_server.py` (new)
- `src/host/rogo/calibrate.py` (confirm/extend the non-interactive core
  entry point ticket 005 structured — do not duplicate its trial logic)
- `src/host/rogo/cli.py` (wire the `mcp` subcommand)
- `tests/host/rogo/test_mcp_server.py` (new)

**Documentation updates**: none required this ticket.

## Testing

- **Existing tests to run**: `tests/host/rogo/` (all prior tickets'
  tests), `tests/host/robot_v6/test_motion.py`.
- **New tests to write**: `test_mcp_server.py` — one test per tool
  (schema + correct underlying-call dispatch), plus the
  unreachable-target error path and the localhost-default bind check.
- **Verification command**: `uv run pytest tests/host/rogo/`

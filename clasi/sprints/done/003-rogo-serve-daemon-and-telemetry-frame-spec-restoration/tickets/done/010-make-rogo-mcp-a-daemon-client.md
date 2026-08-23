---
id: '010'
title: Make rogo mcp a daemon client
status: done
use-cases:
- SUC-002
depends-on:
- 008
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Make rogo mcp a daemon client

## Description

`mcp_server.py` stops owning `rogo.connection` directly and resolves
its connection through `daemon_client`'s auto-spawn-if-absent resolver
instead (sprint.md's Architecture, Step 3 `rogo.mcp_server` entry;
issue Requirement 4). Unlike `cli.py`'s one-shot commands,
`mcp_server.py` has no `cli.py`-style injection point to receive an
already-resolved connection through — it is not itself dispatched via
`cli.py`'s `cmd_*()` machinery for its tool calls — so it calls
`daemon_client` directly, the same way `cli.py`'s `cmd_repl()` does.

This also resolves a standing tension recorded in `mcp_server.py`'s own
module docstring: it has always avoided importing `cli.py` because
`cli.py`'s dispatch is shaped for a terminal (print + exit code), not
the structured data an MCP tool call must return. Once the daemon
returns structured JSON (ticket 004's own wire shape), `mcp_server`
gets structured data regardless of which path it goes through — so this
ticket is a genuine simplification opportunity, not just a relocation
of `mcp_server`'s existing wire-glue helpers (`_pump_until()`,
`_await_ack_and_err()`, etc.) — evaluate during implementation whether
any of those can now be retired in favor of the daemon's own structured
reply, and note in the ticket's own completion notes if they are kept
for a reason (e.g. still needed for the auto-spawn fallback's direct-
connect path).

**Unchanged, by design** (sprint.md's Architecture Impact section):
the tool-call error/result *shapes* `mcp_server.py` returns to its own
external MCP clients do not change — a `kUnknown`/merits-rejection
outcome is still a `warning`/`error` key in the tool's own result
(never a raised MCP error), and a genuine unreachable-target failure
still raises `UnreachableTargetError`. Only the *sourcing* of that data
moves from a direct wire read to the daemon's structured reply.

## Acceptance Criteria

- [x] `rogo mcp` no longer calls `rogo.connection.resolve()` directly —
      it resolves via `daemon_client`'s auto-spawn-if-absent policy.
- [x] A `kUnknown`/merits-rejection outcome is still reported as a
      `warning`/`error` key in the tool's own result, unchanged from
      today's behavior, now sourced from the daemon's reply.
- [x] A genuine unreachable-target failure (`TransportClosed`
      equivalent, or a wait that never resolves) still raises
      `UnreachableTargetError` and surfaces through the MCP tool-call
      error channel, unchanged from today.
- [x] `--listen`/`--allow-remote`'s existing security behavior
      (loopback-only unless explicitly opted out) is unaffected — this
      ticket only changes how `mcp_server` reaches the ROBOT connection,
      not the MCP server's own client-facing transport.
- [x] A second daemon client (e.g. a `rogo drive` one-shot command)
      running concurrently with an active `rogo mcp` session can reach
      the same robot without contention (SUC-002's own multi-client
      acceptance criterion).

## Implementation Plan

**Approach**: Replace `mcp_server.py`'s direct `rogo.connection`
usage with a `daemon_client` call at server startup; keep the module's
own tool-function bodies otherwise unchanged in signature, updating
only how each obtains the connection/session it already operates
against.

**Files to modify**:
- `src/host/rogo/mcp_server.py`.

**Testing plan**: Extends ticket 007's fork-based `--sim` harness:
start `rogo mcp` with no daemon running, confirm it auto-spawns one;
run a tool call and a concurrent one-shot `rogo drive --sim` against
the same target, confirm both reach the robot without contention;
re-run `mcp_server.py`'s existing test suite unmodified to confirm no
change to the external tool-result/error shapes. Scoped run: `uv run
python -m pytest -q tests/host/rogo/ -k mcp_server`.

**Documentation updates**: `mcp_server.py`'s own module docstring
updated to reflect the daemon-client resolution path, replacing the
now-resolved "why this module doesn't import `cli.py`" tension with a
note on why it no longer needs to reason about that at all.

## Completion Notes

- The actual call-site change is `cli.cmd_mcp()`: it now resolves
  through `daemon_client.get_connection(args, spawn=True)` instead of
  `connection.resolve()`, mirroring `cmd_repl()`'s exact shape
  (try/except `RobotNameRequiredError`/`DaemonUnavailableError`, then
  try/finally around `mcp_server.serve()` closing `conn.transport`).
  `mcp_server.serve(session, ...)`'s own signature is unchanged --
  `session` is now whatever `cmd_mcp()` resolved, direct or
  daemon-routed, and `serve()`/`build_server()` never need to know
  which.
- Evaluated whether `mcp_server.py`'s own wire-glue helpers
  (`_pump_until()`, `_await_ack_and_err()`, `_await_ack_and_get_lines()`)
  could be retired now that a daemon-routed session is available: they
  are KEPT, unchanged. `daemon_client._RemoteSession` (ticket 008)
  proxies the exact same `Session` surface (`pump()` returning
  `codec.Reply` objects, `wait_for_done()` returning `DoneEvent | None`)
  a direct `Session` does -- the daemon's own structured JSON
  (`daemon_protocol.Reply`) is a lower wire layer these helpers never
  see either way, fully absorbed by `ClientConnection`/`_RemoteSession`.
  So the helpers are needed for BOTH the direct-connect path (when no
  daemon is running for the resolved target and `get_connection()`
  falls back to `connection.resolve()`) and the daemon-routed path
  equally -- not retired, and not a fallback-only dependency.
- `agent_manual.py` needed no changes: its section 8 ("Auto-detect
  (one-shot subcommands)" / "Auto-spawn (`repl`, `mcp`)") already
  described `rogo mcp` as auto-spawning, written ahead of this ticket's
  own implementation -- the pinning test
  (`tests/host/rogo/test_agent_manual.py`) only checks flag/subcommand
  names, and `rogo mcp`'s own flags are unchanged by this ticket.
- New tests added to `tests/host/rogo/test_mcp_server.py` (not a new
  file, per the ticket's own scoped test command) exercise
  `cli.cmd_mcp()`'s daemon-client connection acquisition end to end
  against real `tools/sim`: no-resolvable-name error, `--listen`
  validated before connection resolution, auto-spawn, reuse of an
  already-running daemon, and this ticket's own AC #5 (a concurrent
  one-shot `rogo stop --sim` reaching the same auto-spawned daemon,
  proven via `Session`'s own client-side seq_id continuity).
  `mcp_server.serve()` is stubbed in that section so each test observes
  connection acquisition without blocking on the real MCP stdio
  protocol loop.

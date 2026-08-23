---
id: 009
title: 'Route rogo CLI through the daemon: serve subcommand, one-shot auto-detect,
  repl auto-spawn'
status: done
use-cases:
- SUC-001
depends-on:
- '005'
- 008
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Route rogo CLI through the daemon: serve subcommand, one-shot auto-detect, repl auto-spawn

## Description

Wire `cli.py` into the daemon subsystem (sprint.md's Architecture, Step
3 `rogo.cli` entry), completing SUC-001's user-facing flow. Three
additive changes:

1. **New `serve` subcommand**: `cmd_serve()` imports `daemon.py`
   (ticket 005) and starts its server loop against a resolved target,
   injecting `cli.py`'s own per-verb dispatch functions — the identical
   injection shape `cmd_repl()` already uses for `repl.py`. This is the
   edge sprint.md's architecture review specifically verified is
   one-directional (`cli.py` → `daemon.py`, never the reverse) — do not
   have `daemon.py` import anything from `cli.py`; only `cli.py` may
   import `daemon.py`.
2. **One-shot auto-detect**: each one-shot `cmd_*()`'s existing
   `connection.resolve()` call is replaced with a call to
   `daemon_client`'s auto-detect-only resolver (ticket 008), which
   falls back to `connection.resolve()` unchanged when no daemon is
   found.
3. **`repl` auto-spawn**: `cmd_repl()` resolves its connection through
   `daemon_client`'s auto-spawn resolver (ticket 008) before injecting
   it into `repl.py` — `repl.py` itself needs no changes for this
   (sprint.md's Architecture explicitly notes `repl.py`'s own module
   boundary stays as narrow as today).

## Acceptance Criteria

- [x] `rogo serve [--sim|--connect|--port] [--stdio-pipe]` starts a
      daemon (delegating to `daemon.py`), reusing `cli.py`'s own
      per-verb dispatch functions by injection.
- [x] A one-shot `rogo drive`/`turn`/`goto`/`config`/`calibrate`
      invocation routes through a running daemon when one exists for
      the resolved robot, with no new process spawned.
- [x] The same one-shot invocation, with no daemon running, behaves
      identically to today (direct connect, no auto-spawn) — regression
      guard for SUC-001's second acceptance criterion.
- [x] `rogo repl` auto-spawns a daemon when none is running for its
      resolved target, then behaves identically to a direct connection
      from the user's perspective.
- [x] Two sequential one-shot invocations (or a one-shot followed by
      `repl`) against the same daemon do not reset `tools/sim`'s
      connection state between them (SUC-001's own first acceptance
      criterion, verified end to end here for the first time).

## Implementation Plan

**Approach**: Modify `cli.py`'s existing `cmd_*()` functions' target
resolution call sites; add `cmd_serve()` and its `build_parser()` entry
following the same subcommand-registration pattern `cmd_repl()`/
`cmd_mcp()` already use.

**Files to modify**:
- `src/host/rogo/cli.py` — new `serve` subcommand; resolution call
  sites updated for one-shot auto-detect and `repl` auto-spawn.

**Testing plan**: Extends ticket 007's fork-based `--sim` harness:
start a daemon, run two sequential one-shot `rogo` invocations against
it via subprocess, assert no reset occurred (SUC-001's core scenario);
separately, assert one-shot behavior is byte-for-byte unchanged with no
daemon present (compare against this ticket's own pre-change baseline
or an existing golden output, if one exists in
`tests/host/rogo/test_cli.py`). Scoped run: `uv run python -m pytest -q
tests/host/rogo/ -k "cli or serve"`.

**Documentation updates**: `src/host/rogo/README.md` and
`src/host/rogo/agent_manual.py` (this project's existing pattern for
documenting `rogo` subcommands to an agent audience) gain a `serve`
section and a note on when one-shot commands auto-route through a
daemon.

---
id: '011'
title: Daemon end-to-end test pass and documentation updates
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004]
depends-on: ["007", "009", "010"]
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Daemon end-to-end test pass and documentation updates

## Description

Closing ticket for the daemon stream: an end-to-end test pass that
exercises the whole subsystem together (rather than each module in
isolation, as tickets 004-010 each already did), plus the documentation
updates the sprint's Success Criteria require but no earlier ticket
owns outright. This is also the natural point to confirm the sprint's
own architecture held up in practice — in particular, that no
accidental `daemon.py` → `cli.py` import crept in during
implementation (the cycle risk this sprint's architecture review caught
and fixed on paper).

## Acceptance Criteria

- [ ] All of this sprint's Success Criteria (sprint.md) for the daemon
      stream are demonstrated by a passing test, run together in one
      scenario where practical: `rogo serve` holds one connection open
      across multiple client sessions with no reset between them
      (Unix-socket mode AND stdio-pipe mode, both against `tools/sim`);
      an estop from one client preempts another's in-flight command;
      `rogo mcp` and `rogo` CLI/`repl` all route through the same
      running daemon concurrently; `rogo repl` and daemon pipe-mode
      output are confirmed line-flushed.
- [ ] None of the three new modules import `cli.py` (confirms the
      cycle-avoidance decision from this sprint's architecture review
      held in the real implementation): `grep -r "import cli"
      src/host/rogo/daemon.py src/host/rogo/daemon_client.py
      src/host/rogo/daemon_protocol.py` finds nothing.
- [ ] `src/host/rogo/README.md` documents `rogo serve`, the two
      transports, and the auto-detect/auto-spawn routing behavior for
      `rogo`'s other subcommands.
- [ ] `src/host/rogo/agent_manual.py` (this project's existing
      agent-facing documentation module) is updated to describe the
      daemon-sharing behavior, so an agent driving `rogo` learns it can
      rely on a shared connection instead of reasoning about port
      resets itself.
- [ ] Full scoped test run across all daemon-stream modules together
      (not just per-ticket subsets) passes.

## Implementation Plan

**Approach**: Primarily a test-writing and documentation ticket — no
new production code expected beyond what tickets 004-010 already
built, unless this pass surfaces an integration gap between them (in
which case, fix it here and note the gap in this ticket's own
completion notes).

**Files to modify**:
- `src/host/rogo/README.md`, `src/host/rogo/agent_manual.py` —
  documentation.
- Any of the daemon-stream modules, only if this ticket's own
  integration testing surfaces a real gap between tickets.

**Testing plan**: New end-to-end test(s) in `tests/host/rogo/`
combining scenarios from tickets 004-010 into fewer, broader
integration tests reflecting how a real user session actually uses
`rogo serve` (start daemon, drive it from `repl`, a one-shot command,
and `mcp` concurrently). Scoped run: `uv run python -m pytest -q
tests/host/rogo/` (the whole package, since this ticket is specifically
about cross-module integration — narrower scoping would defeat the
point). The full project suite still runs exactly once, at
`close_sprint`, per this project's own testing rule — this ticket's
"whole package" scope is `tests/host/rogo/` only, not the entire repo.

**Documentation updates**: Covered above — this ticket IS the
documentation-update ticket for the daemon stream.

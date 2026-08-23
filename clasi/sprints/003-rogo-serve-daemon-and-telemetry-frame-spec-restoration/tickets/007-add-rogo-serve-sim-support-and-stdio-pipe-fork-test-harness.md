---
id: '007'
title: Add rogo serve --sim support and stdio-pipe fork test harness
status: open
use-cases: [SUC-003]
depends-on: ["006"]
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add rogo serve --sim support and stdio-pipe fork test harness

## Description

Wire `rogo serve --sim` so a test run is "start sim → start daemon
against it → talk to the daemon" with no manually started `tools/sim`
process required (issue Requirement 3), and build the reusable
fork-based test harness other tickets (008, 009, 010, 011) will use to
drive a real daemon in their own tests without a physical robot.

`--sim` on `rogo serve` should reuse the exact same `--sim` resolution
`rogo.connection.resolve()` already implements for one-shot commands
(`ensure_sim_binary()`, `StdioTransport` spawning `tools/sim`) — the
daemon's own connection acquisition (ticket 005) already calls
`rogo.connection.resolve()`, so this ticket is primarily about
confirming/testing that path end to end through the daemon rather than
building new sim-launching logic.

## Acceptance Criteria

- [ ] `rogo serve --sim` reaches a working daemon with no manually
      started `tools/sim` process (freshly builds/reuses `tools/sim`
      the same way `rogo drive --sim` already does today).
- [ ] A reusable test helper/fixture forks `rogo serve --sim
      --stdio-pipe` (or equivalent), yields a connected client-side pipe
      pair, and tears the process down cleanly on test exit — usable by
      tickets 008/009/010/011's own tests without each reimplementing
      process management.
- [ ] SUC-003's full flow (fork daemon in pipe mode, exchange a
      request/reply cycle, confirm dispatch reaches the sim-backed
      connection) passes as an end-to-end test.

## Implementation Plan

**Approach**: No new connection-acquisition logic — confirm
`daemon.py`'s existing `rogo.connection.resolve()` call (ticket 005)
already handles `--sim` correctly when invoked from `rogo serve`, and
build the fork/teardown test helper as the ticket's main deliverable.

**Files to modify**:
- `src/host/rogo/daemon.py` — `--sim` argument plumbing if not already
  covered by ticket 005/006's argument parsing.

**Files to create**:
- `tests/host/rogo/daemon_test_helpers.py` (or similar) — the
  fork/teardown fixture other daemon tickets' tests import.

**Testing plan**: The fork-based harness itself IS this ticket's test
deliverable; validate it with the SUC-003 end-to-end scenario above.
Scoped run: `uv run python -m pytest -q tests/host/rogo/ -k "sim or
daemon_test_helpers"`.

**Documentation updates**: `tools/sim/README.md` gains a one-line
cross-reference to `rogo serve --sim` as another way to reach the sim
binary, alongside its existing `--stdio`/`--listen` invocation docs.

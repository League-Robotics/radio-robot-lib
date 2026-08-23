---
id: 008
title: 'Build the daemon client library: find / spawn / direct-connect policy'
status: open
use-cases: [SUC-001, SUC-002]
depends-on: ["004", "006"]
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build the daemon client library: find / spawn / direct-connect policy

## Description

Build `daemon_client.py` (sprint.md's Architecture, Step 3
`rogo.daemon_client` entry): the only module that knows the
find-vs-spawn-vs-direct-connect policy, returning an object presenting
the same call surface `rogo.connection.resolve()` already returns so
`cli.py`'s existing per-verb dispatch code needs no changes beyond how
it obtains its connection.

**Two policies, chosen by caller** (sprint.md's Architecture Design
Rationale — "mcp/repl auto-spawn; one-shot cli auto-detects only"):
- **Auto-detect-only**: look for a running daemon (by robot name →
  socket path, ticket 006's resolution rule); if found, connect to it;
  if not found, fall back to `rogo.connection.resolve()` unchanged
  (today's direct-connect behavior — zero regression for a caller that
  never runs `rogo serve`). Used by `cli.py`'s one-shot subcommands
  (ticket 009).
- **Auto-spawn-if-absent**: same lookup; if not found, spawn `rogo
  serve` as a subprocess (not a Python import of `daemon.py` — a real
  OS process that outlives the spawning call), wait for it to become
  reachable, then connect. Used by `cli.py`'s `cmd_repl()` and by
  `mcp_server.py` (tickets 009/010). The spawned daemon self-terminates
  after an idle timeout with no connected clients (default value: this
  ticket's own decision — sprint.md's Architecture Step 7 flags the
  exact duration as open; document whatever default is chosen and make
  it overridable via flag/env var).

The returned client connection implements enough of the same interface
`Connection`/`Session` already expose (whatever `cli.py`'s per-verb
dispatch bodies actually call) by translating each call into one framed
request to the daemon (via `daemon_protocol`, ticket 004) and mapping
the reply back into the same result shapes.

## Acceptance Criteria

- [ ] Auto-detect-only mode: connects to a running daemon when one
      exists for the resolved robot name; falls back to
      `rogo.connection.resolve()` unchanged when none is found, with no
      process spawned.
- [ ] Auto-spawn-if-absent mode: spawns `rogo serve` as a subprocess
      when no daemon is found, waits for it to become reachable within a
      bounded timeout, then connects; raises a clear error (not a hang)
      if the spawned daemon never becomes reachable.
- [ ] The object returned by either mode presents the same call surface
      `rogo.connection.resolve()`'s `Connection` already does, so a
      caller's existing dispatch code does not need to branch on
      direct-vs-daemon-proxied.
- [ ] An auto-spawned daemon self-terminates after the configured idle
      timeout with no connected clients (verified with a short timeout
      in the test).
- [ ] `daemon_client.py` has no dependency on `cli.py` (it is a library
      module several callers depend on, not the other way around).

## Implementation Plan

**Approach**: One module exposing two entry points (or one function
with a `spawn: bool` policy parameter) implementing the lookup, the
subprocess-spawn-and-wait, the direct-connect fallback, and a thin
client-side object wrapping `daemon_protocol` request/reply calls
behind the existing `Connection`-shaped interface.

**Files to create**:
- `src/host/rogo/daemon_client.py`.

**Testing plan**: New tests in `tests/host/rogo/`, using ticket 007's
fork-based test harness against `--sim`: (1) auto-detect finds an
already-running daemon and does not spawn one; (2) auto-detect with no
daemon present falls back to direct-connect (assert no subprocess was
spawned); (3) auto-spawn with no daemon present spawns one, connects,
and the daemon later self-terminates after the idle timeout; (4) the
returned client object's calls reach the daemon and produce the same
observable outcome (ack/nack/err) a direct connection would. Scoped
run: `uv run python -m pytest -q tests/host/rogo/ -k daemon_client`.

**Documentation updates**: Module docstring documents the two policies
and which callers use which, cross-referencing sprint.md's Architecture
Design Rationale for the "why".

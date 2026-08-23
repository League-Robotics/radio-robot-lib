---
id: '005'
title: 'Build the rogo serve daemon server core: connection ownership, dispatch injection,
  estop-priority queue'
status: done
use-cases:
- SUC-001
- SUC-004
depends-on:
- '004'
github-issue: ''
issue: rebuild-rogo-serve-daemon-on-v6-named-sockets-pipe-mode-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build the rogo serve daemon server core: connection ownership, dispatch injection, estop-priority queue

## Description

Build `daemon.py`'s server core (sprint.md's Architecture, Step 3
`rogo.daemon` entry): the object that holds ONE robot connection open
for the daemon process's whole lifetime and routes client requests to
it, escalating any estop request ahead of whatever else is queued
(issue's own safety carry-over: "an estop/halt request from ANY client
jumps to the front of the work queue and aborts any in-progress
completion wait"). This ticket covers the core logic only — no
listener transports yet (ticket 006 adds Unix socket + stdio pipe;
this ticket can use an in-process/loopback transport for its own
tests) — and no `--sim`/CLI wiring yet (tickets 007/009).

**Critical boundary (do not violate)**: `daemon.py` must NOT import
`rogo.cli`. It receives `cli.py`'s per-verb dispatch functions BY
INJECTION — a parameter this ticket's own server-start entry point
accepts — the same shape `repl.py` already receives dispatch from
`cli.py`'s `cmd_repl()` today (`repl.py`'s own module docstring: "this
module never imports `rogo.cli`"). `cli.py` will need to import
`daemon.py` in ticket 009 to wire the new `serve` subcommand; if
`daemon.py` also imported `cli.py`, the two modules would form a
circular import. This was caught and fixed in this sprint's own
architecture self-review — do not reintroduce it.

The daemon does not reimplement per-verb command semantics itself: it
calls whatever dispatch function it was handed for the incoming
request's verb, against the one `Session`/`Connection` it resolved via
`rogo.connection.resolve()` at startup and holds for its lifetime.

## Acceptance Criteria

- [x] The server accepts a dispatch table/callable by injection at
      construction — it does not import `cli.py`.
- [x] One `rogo.connection.Connection` is resolved once at server
      startup and reused for every subsequent request (verified: no
      second `resolve()` call happens per request).
- [x] An estop/halt request submitted while another request is
      in-progress is executed ahead of that request's completion wait —
      the queue is priority-ordered, not FIFO, for estop specifically.
- [x] Every other request type is served in arrival order (FIFO)
      relative to other non-estop requests.
- [x] Server core has zero dependency on `src/host/rogo/cli.py`
      (verified: no `import` of `rogo.cli`/`from . import cli` anywhere
      in `daemon.py`).

## Implementation Plan

**Approach**: A server object constructed with a resolved
`Connection`/`Session` and an injected dispatch table (verb name →
callable, the same callables `cli.py`'s `cmd_repl()` already injects
into `repl.py` today). Incoming requests (decoded via
`daemon_protocol`, ticket 004) are placed on a priority queue keyed so
an estop request always sorts ahead of any pending non-estop request;
a single worker loop drains the queue and calls the matching injected
dispatch function against the held `Session`.

**Files to create**:
- `src/host/rogo/daemon.py` — the server core.

**Files to modify**: None in this ticket (listener wiring is ticket
006; `cli.py`'s `serve` subcommand and dispatch injection call site are
ticket 009).

**Testing plan**: New tests in `tests/host/rogo/` using an in-process
fake transport (no socket/pipe yet — that is ticket 006's own test
surface) and a fake/mocked dispatch table: verify FIFO ordering for
ordinary requests, verify an estop submitted mid-processing preempts an
in-flight long-running request's completion wait, verify the server
never calls `Connection`/`Session` resolution more than once. Scoped
run: `uv run python -m pytest -q tests/host/rogo/ -k daemon`.

**Documentation updates**: Module docstring explaining the
injection-not-import boundary explicitly (this is the load-bearing
design decision from this sprint's architecture review — it must not
move, matching how `protocol.md` flags its own load-bearing decisions).

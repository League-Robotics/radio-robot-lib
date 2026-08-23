---
id: '004'
title: Add rogo estop command
status: executing
branch: sprint/004-add-rogo-estop-command
use-cases:
- SUC-001
issues:
- add-a-rogo-estop-cli-subcommand.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 004: Add rogo estop command

## Goals

Give `rogo` a `estop` subcommand that sends the unsequenced `ESTOP`
panic-stop, routed through a running daemon by auto-detect (never
auto-spawn) so it reaches the estop-priority path sprint 003 already
built in `daemon.py`/`daemon_client.py`.

## Problem

`rogo` has no CLI surface for the daemon's estop-priority path: an
`ESTOP` from any client is supposed to jump the work queue and abort
another client's in-flight completion wait, but there is no command
that sends one. An operator or agent wanting a panic stop today has to
call `robot_v6.motion.estop(session)` directly from Python. Sprint
003's ticket 011 flagged this gap explicitly and deferred it rather
than expanding that sprint's scope. `rogo stop` is not a substitute —
it sends the sequenced `STOP`, which waits behind whatever else is
already queued, the opposite of what a panic stop needs.

## Solution

Add `cmd_estop()` to `src/host/rogo/cli.py`, modeled on the existing
`cmd_stop()`/`cmd_hello()` pattern: send `robot_v6.motion.estop(session)`
(the unsequenced `ESTOP` — no `#id`, never acked, never made to wait
behind a sequenced command), resolved through
`daemon_client.get_connection(args, spawn=False)` — the same
auto-detect/no-auto-spawn rule every other one-shot subcommand already
uses, so a panic stop never pays daemon-startup latency and falls back
to today's direct connection when no daemon is running. Register an
`estop` subparser alongside `stop`'s, with the same shared target
options (`--sim`/`--connect HOST:PORT`/`--port PORT`). At ticket time,
decide and document whether the command pumps briefly for the robot's
bare `estop` confirmation line (protocol.md §8.3) or returns
immediately — `cmd_hello()`'s handling of the unsequenced `HELLO` is
the closest precedent, though `estop` cannot reuse its return-1-on-no-
reply behavior verbatim: the issue requires exiting non-zero only on a
genuine transport failure, never on the absence of a confirmation line
the protocol doesn't guarantee timing for. Update the `--agent` manual
to document the new command and its distinction from `stop`.

## Success Criteria

- `rogo estop` sends `ESTOP` and is covered by a test against the sim.
- A test proves `rogo estop` preempts another daemon client's
  in-flight command — exercising the estop-priority path end-to-end
  through a running daemon, not just asserting
  `daemon_client.is_estop_request()`'s classification in isolation
  (sprint 003 ticket 011 found that classification silently broken
  once already).
- `rogo estop --help` works, and `--agent` documents the command, its
  distinction from `rogo stop`, and its no-auto-spawn routing.
- `tests/host/rogo/test_agent_manual.py`'s pinning test stays green.

## Scope

### In Scope

- One new subcommand, `rogo estop`, in `src/host/rogo/cli.py`, with
  its subparser registration and the shared target options.
- A test that `rogo estop` reaches the sim and sends `ESTOP`.
- A test that `rogo estop` preempts an in-flight command via a running
  daemon (the estop-priority path).
- An `--agent` manual update covering the new command.

### Out of Scope

- Any change to `rogo stop`'s sequenced-`STOP` behavior.
- Any change to `daemon.py`'s estop-priority queue logic or
  `daemon_client.is_estop_request()`'s classification itself — sprint
  003 built and tested that path; this sprint only adds the CLI
  surface that calls into it.
- Any new daemon protocol verb or wire-format change.
- Any change to `robot_v6.motion.estop()`'s signature or behavior.

## Test Strategy

Two layers, both against the real compiled `tools/sim` binary, no new
test infrastructure: (1) a sim-level send test in `test_cli.py`
mirroring the existing `test_stop_end_to_end_against_sim`/
`test_hello_end_to_end_against_sim` pattern — `cli.main(["estop",
"--sim"])` exits 0 and prints confirmation; (2) an end-to-end
daemon-preemption test extending `test_daemon_e2e_multi_client.py`'s
existing estop-priority scenario so Client B is a real `rogo estop`
subprocess rather than a raw `motion.estop()` call — proving the CLI's
own wiring reaches the estop-priority path sprint 003 built, not just
the already-tested underlying primitive. Both isolate
`XDG_RUNTIME_DIR` to a short per-test `/tmp` directory (AF_UNIX
`sun_path` limit) and never touch the real `~/.rogo`. No test of
`daemon.py`'s queue logic or `is_estop_request()`'s classification
itself is needed — sprint 003 already covers those; this sprint tests
only that the new CLI surface calls into them correctly.

## Architecture

**Trivial** — one new CLI subcommand (`rogo estop`) added to one
existing module (`src/host/rogo/cli.py`), reusing the daemon-routing
and estop-priority machinery sprint 003 already built
(`daemon_client.get_connection()`, `daemon_client.is_estop_request()`,
`daemon.py`'s priority queue); no new module, no new cross-module
dependency, no data-model change.

### Architecture Overview

N/A — trivial. `cmd_estop()` follows the exact same shape as the
existing `cmd_stop()`/`cmd_hello()`: resolve a connection via
`daemon_client.get_connection(args, spawn=False)`, send one call on
the resulting `Session`, report the outcome, close the transport. No
new component is introduced and no existing dependency direction
changes — `cli.py` already depends on `daemon_client` and
`robot_v6.motion` for every other one-shot subcommand.

### Design Rationale

One decision worth recording, since the issue leaves it open
(requirement 5): `cmd_estop()` pumps briefly for the bare `estop`
confirmation line (protocol.md §8.3), mirroring `_run_hello()`'s
bounded-pump pattern — but unlike `_run_hello()`, which returns 1 when
no `device` banner arrives in time, a pump timeout with no
confirmation is *not* treated as a failure; `cmd_estop()` still exits
0. Alternative considered: return immediately after the unsequenced
send, as fire-and-forget as `ESTOP` itself. Rejected because a human
or agent issuing a panic stop benefits from on-screen confirmation the
robot actually received it, and a short bounded pump costs nothing
when a target is present. Consequence: the exit code can never
distinguish "robot didn't reply in time" from "robot replied" — both
exit 0 — which is deliberate per the issue's requirement 5 (exit
non-zero only on a genuine transport failure, never on the absence of
a confirmation the protocol doesn't guarantee timing for).

### Migration Concerns

None — new subcommand only; no existing command's behavior, wire
format, or data model changes.

## Use Cases

Compact — one new use case, briefly stated: the one behavior worth
calling out at sprint level is a CLI-issued panic stop reaching the
daemon's estop-priority path and preempting another client's in-flight
command, distinguishing it from the sequenced `rogo stop`.

### SUC-001: Send a panic stop from the CLI through the daemon's estop-priority path
Parent: UC-005

- **Actor**: CLI / tooling user (operator or AI agent) — the same
  actor as UC-014, extended into panic-stop territory.
- **Preconditions**: `rogo` is installed; a target (real robot, relay,
  or `tools/sim`) is reachable via `--sim`/`--connect`/`--port`. A
  `rogo serve` daemon may or may not already be running for that
  target.
- **Main Flow**:
  1. Another client has a command queued or in progress against a
     running `rogo serve` daemon.
  2. User runs `rogo estop` — no daemon spawn is attempted
     (`daemon_client.get_connection(args, spawn=False)`).
  3. If a daemon is running, the request classifies as estop-priority
     (`daemon_client.is_estop_request()`) and jumps the queue,
     preempting the other client's in-flight completion wait; if no
     daemon is running, `rogo estop` falls back to a direct
     connection.
  4. `robot_v6.motion.estop(session)` sends the unsequenced `ESTOP` —
     no `#id`, exempt from sequencing, never acked.
  5. CLI reports the outcome and exits 0 unless the transport itself
     failed.
- **Postconditions**: The robot's estop latch is set; the other
  client's preempted command is aborted; the CLI has exited 0.
- **Acceptance Criteria**:
  - [ ] `rogo estop` sends `ESTOP` and is verified against the sim.
  - [ ] With a daemon running, `rogo estop` measurably preempts
        another client's in-flight command (not just a unit-level
        classification check of `is_estop_request()`).
  - [ ] `rogo estop --help` and `--agent` both document the command
        and its distinction from `rogo stop`.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [x] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add rogo estop CLI subcommand routed through daemon auto-detect | — |

Tickets execute serially in the order listed.

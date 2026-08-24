---
id: '004'
title: Add rogo estop command
status: done
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
- Any change to `DaemonServer`'s priority-queue ordering or
  abort/preemption mechanism itself, or to
  `daemon_client.is_estop_request()`'s classification — sprint 003
  built and tested that path, and it is confirmed correct: the
  exception analyzed in the Revision section below shows the queue and
  `is_estop_request()` already behave correctly once a request reaches
  `DaemonServer.submit()`. **(Revised 2026-08-24 — see Revision below:
  this bullet originally forbade any change to `daemon.py`/
  `daemon_client.py` at all; narrowed after the exception, because what
  actually needed fixing was the connectivity *probe*'s own routing —
  which never reached the estop-priority queue's logic at all — not the
  queue logic itself.)**
- Any protocol-v6 (robot-facing) wire-format change — `docs/design/protocol.md`
  §8.3 is untouched. **(Revised 2026-08-24: the sprint's own internal
  host-to-host daemon wire protocol — `daemon_protocol.py`'s JSON
  framing between `rogo` and `rogo serve`, unrelated to protocol-v6 —
  gains one new reserved verb for the connection-liveness probe; see
  ticket 002 and the Revision section below. This is the one narrow
  exception to the original "no new daemon protocol verb" bullet.)**
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

**Added 2026-08-24 (see Revision below)**: a third layer, owned by
ticket 002 — a targeted regression test (no CLI subprocess, no real
`tools/sim`) proving that with the daemon's single worker thread
genuinely occupied by a long-running dispatch, a client's
connectivity-liveness probe still returns promptly instead of queuing
behind it. This isolates the fixed mechanism from CLI-subprocess/sim
timing noise; it complements, and does not replace, the CLI-level
preemption test above.

## Revision (2026-08-24) — daemon connection-establishment fix added to scope

**Exception**: ticket 001 (`thrown_by: programmer`) implemented the CLI
surface correctly and completely (`cmd_estop()`, the subparser, the
manual update, and the sim-level smoke test all pass — uncommitted in
the working tree, needs no rework), then wrote the ticket's required
proof — a CLI-level test that `rogo estop`, run as a real subprocess,
preempts another client's in-flight command through a real running
`rogo serve` daemon — and that test failed deterministically. Full
root-cause detail is preserved in ticket 001's own `exception:`
frontmatter and in ticket 002's Description; summarized here:

`daemon_client.find_daemon()`'s own connectivity probe sends
`session_highest_acked` — a request `is_estop_request()` does not (and
should not) classify as estop-priority — through `DaemonServer`'s
single shared priority queue, exactly like any other ordinary request.
When another client's abort-aware wait already occupies the daemon's
one worker thread, this "cheap probe" queues behind it and does not
run until that dispatch call returns. The client-side
`DEFAULT_FIND_TIMEOUT_S` (1.0s) elapses first; `find_daemon()` returns
`None`; `get_connection(spawn=False)` silently falls back to a
brand-new **direct** connection, completely bypassing the daemon and
its already-correct estop-priority path. A genuine chicken-and-egg
gap: a cold, one-shot client cannot submit an estop-priority request
until *after* its own non-privileged connectivity probe clears the
same single-worker queue the estop exists to jump. Production impact:
on a serial robot, `rogo estop` issued during another client's
in-flight `drive` would fail to preempt it and would instead try to
open a serial port the daemon already holds — a real safety-relevant
gap, not a test artifact. The exception also confirmed empirically
that the obvious CLI-only mitigation (a larger client-side
`find_timeout=`) does not fix the guarantee, only trades one bad
outcome for another (see ticket 002's Description for why).

**Stakeholder decision**: escalated as `user-visible` (the gap sits
squarely inside SUC-001's own postcondition — "the other client's
preempted command is aborted"). The stakeholder chose to **fix it
properly within sprint 004** rather than ship the CLI command with the
gap, or defer it to a future sprint. Sprint 004's scope now legitimately
includes `src/host/rogo/daemon.py` and `src/host/rogo/daemon_client.py`
— both shipped by sprint 003 — narrowly, for the one mechanism the
exception identified.

**Scope change resulting from this decision**:
- Out of Scope (above) narrowed — see the two revised bullets there.
- Architecture re-sized from Trivial to **Compact** (below); the
  `architecture_review` gate is re-recorded as `passed` (was
  `skipped`) after a scoped self-review of the daemon-side change.
- New **ticket 002** (daemon-side fix) added, sequenced *before*
  ticket 001 via `depends-on` (ticket 001 now has `depends-on:
  ['002']` — execution follows the dependency graph, so ticket 002 runs
  first despite the higher ticket-001-first numbering).
- Ticket 001 reopened (was `exception`, now `open`); its own
  CLI-surface implementation is unaffected and needs no rework — only
  its already-written, currently-failing preemption test needs ticket
  002's fix to turn green.
- Test Strategy (above) gains a third layer: a targeted, non-CLI
  regression test for the fixed mechanism itself (ticket 002).

## Architecture

**Compact** — the daemon-side fix changes two existing modules already
part of the daemon subsystem sprint 003 shipped
(`src/host/rogo/daemon.py`, `src/host/rogo/daemon_client.py`): no new
module, no new cross-module dependency (`daemon_client.py` already
imports `daemon.py`), no dependency-direction change, and no persisted
data-model change — only a narrow addition to the internal
host-to-host daemon wire vocabulary (one new reserved verb),
comparable in scope to how `_ESTOP_WIRE_VERBS` already adds a
recognized value to that same vocabulary. This revises the sprint's
original "Trivial" sizing (recorded when scope was CLI-surface-only,
per the Definition of Ready gate history) up one tier, per the
stakeholder-approved scope expansion in the Revision section above. No
diagram: this is a routing change on an already-existing path between
two already-coupled modules, not a new composition of components — the
prose below (and Step 3's one-sentence purpose statements) already say
everything a component diagram would show for two modules with an
unchanged dependency edge between them.

### What Changed

- **`src/host/rogo/daemon.py`** — `UnixSocketListener._serve_client()`
  gains a small, queue-bypassing fast path: a new reserved
  connection-liveness verb is decoded and answered directly, on the
  listener's own per-client thread, without ever calling
  `DaemonServer.submit()`. `DaemonServer` itself — its priority queue,
  `_execute()`, and its `abort`-event preemption mechanism — is
  unchanged.
- **`src/host/rogo/daemon_client.py`** — `find_daemon()`'s connectivity
  probe switches from sending `session_highest_acked` to sending the
  new reserved verb. `_wait_for_daemon()` and `get_connection()` are
  unchanged beyond inheriting this through `find_daemon()`.

### Why

The daemon's estop-priority mechanism (sprint 003) was already correct
once a request reached `DaemonServer.submit()`: a priority-0 (estop-
class) submission immediately sets the currently-dispatching item's
`abort` event, and an abort-aware dispatch body (`_dispatch_session_
wait_for_done`, etc.) notices within one poll interval. The bug was
never in that logic — it was that a brand-new client's own
connectivity probe, needed *before* any real request can be sent, is
an ordinary priority-1 request with no way to signal urgency, so it
can be serialized behind an already-dispatching non-estop item with no
bound on how long that takes. Removing the probe from the queue
entirely (rather than reordering it within the queue) fixes this at
its root: the probe carries no session state and needs no
serialization against other clients' dispatches at all, so it has no
reason to touch the queue in the first place.

### Impact on Existing Components

- `DaemonServer`, `is_estop_request()`, and the estop-priority
  queue/abort mechanism: unchanged in behavior.
- `daemon_protocol.py`: unchanged — it has no verb table of its own
  (its own module docstring), so the new verb is just a string two
  already-coupled modules agree on, the same pattern
  `_ESTOP_WIRE_VERBS` already uses.
- `cli.py`'s `cmd_estop()` (ticket 001): unchanged and requires no
  rework — its own wiring into `daemon_client.get_connection()` was
  already correct; it only needed `get_connection()` itself to stop
  stalling underneath it.
- `session_highest_acked`'s behavior for every *other* caller:
  unchanged — it remains an ordinary, non-priority RPC; only the
  probe's own choice of which verb to send changes.
- Test infrastructure that constructs caller-supplied fake dispatch
  tables using `"ping"`/`"hello"` as example verb names
  (`daemon_test_helpers.py`, `test_daemon.py`,
  `test_daemon_transports.py`, `test_daemon_sim_e2e.py`): unaffected,
  *provided* the new reserved verb name is chosen to be distinct from
  those — called out explicitly in ticket 002 as a collision risk to
  avoid, not simply left implicit.
- `run_daemon_worker()`'s idle-timeout tracking: unaffected in the
  sense that matters — a liveness probe answered outside the dispatch
  table never touches `last_activity`, so it correctly does not keep
  an idle daemon artificially alive just because a client polled it.

### Design Rationale

**Decision**: answer the connection-liveness probe directly in
`UnixSocketListener`, bypassing `DaemonServer`'s priority queue
entirely, rather than (a) reclassifying `session_highest_acked` as
estop-priority, or (b) increasing the client-side `find_timeout`.

**Context**: the exception's own repro scripts (preserved in ticket
001's `exception:` frontmatter) empirically tested option (b) already.

**Alternatives considered**:
- *(a) Classify `session_highest_acked` as estop-priority.* Rejected:
  `highest_acked` is a general-purpose RPC used in ordinary (non-probe)
  operation elsewhere in the codebase, not something used only by
  `find_daemon()`. Making it priority-0 globally would let *any*
  client's routine `highest_acked` poll spuriously abort *another*
  client's in-flight wait — a far broader and more dangerous behavior
  change than fixing the probe, and not something the classifier could
  distinguish (it sees only the wire request, not caller intent).
- *(b) Raise the client's `find_timeout`.* Rejected: confirmed
  empirically not to fix the guarantee — it only trades a wrong silent
  fallback for `get_connection()` itself blocking for the same ~9.5s,
  by which point client A's own wait has already resolved via its
  natural timeout rather than an abort.
- *(c, chosen) Bypass the queue entirely for the probe.* The probe
  needs no session state and no ordering guarantee relative to other
  clients' dispatches — it only needs to prove "a real daemon is
  listening here." Removing it from the queue sidesteps both rejected
  alternatives' problems without touching the estop-priority mechanism
  at all.

**Consequences**: `daemon.py` gains one new reserved wire-verb name
that no caller-supplied `DispatchTable` may reuse — a documented, not
runtime-enforced, constraint (ticket 002 flags the collision risk
explicitly, since `"ping"` is already reused pervasively as a fake
dispatch-table verb name across this test suite). The probe no longer
counts as daemon "activity" for idle-timeout purposes, which is
correct behavior, not a side effect to work around.

### Open Questions

None blocking implementation. The exact reserved verb name is ticket
002's own implementation choice, constrained only by "must not collide
with `\"ping\"`/`\"hello\"`/the six `session_*` names" (see ticket 002's
Acceptance Criteria).

### Migration Concerns

None — additive wire-protocol addition to an internal-only,
same-sprint protocol; no persisted state, and both ends of this wire
(`daemon.py`, `daemon_client.py`) ship together in this sprint, so
there is no coordinated-upgrade concern.

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
        classification check of `is_estop_request()`) — including when
        the daemon's worker is genuinely busy at the moment `rogo
        estop` is invoked, i.e. connection establishment itself (the
        liveness probe) must not be able to queue behind that busy
        worker (added 2026-08-24, see Revision — this is precisely the
        gap ticket 001's exception found and ticket 002 fixes).
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
| 002 | Bypass the daemon's priority queue for connection-liveness probes so estop can always preempt a busy worker | — |
| 001 | Add rogo estop CLI subcommand routed through daemon auto-detect | 002 |

Tickets execute in dependency order, not numeric order: 002 (the
daemon-side fix, added 2026-08-24 per the Revision above) runs first,
then 001 (the CLI surface, reopened from `exception`), whose only
remaining work is confirming its already-written preemption test now
passes.

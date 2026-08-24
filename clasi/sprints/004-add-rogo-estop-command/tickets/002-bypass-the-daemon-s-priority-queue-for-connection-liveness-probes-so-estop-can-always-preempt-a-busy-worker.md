---
id: '002'
title: Bypass the daemon's priority queue for connection-liveness probes so estop
  can always preempt a busy worker
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: add-a-rogo-estop-cli-subcommand.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bypass the daemon's priority queue for connection-liveness probes so estop can always preempt a busy worker

## Description

Ticket 001's own attempt to prove `rogo estop` preempts another client's
in-flight command through a real running daemon (issue
`add-a-rogo-estop-cli-subcommand.md` Requirement 4) threw an exception
when that proof test failed deterministically. This ticket implements
the fix the exception's root-cause analysis identified, so that ticket
001's already-written CLI-surface work (uncommitted, all passing) can
finally pass its one remaining, deliberately-failing test.

**Root cause** (full detail preserved in ticket 001's `exception:`
frontmatter — summarized here so it survives independently of that
field):

`daemon_client.find_daemon()` — called by `get_connection(args,
spawn=False)` before any real request is sent — probes an already-open
socket with a `session_highest_acked` request to confirm the far end
genuinely speaks `daemon_protocol`, not just that something is
listening on the path. `daemon_client.is_estop_request()` does not (and
should not — see Design Rationale below) classify
`session_highest_acked` as estop-priority, so this probe is submitted
to `DaemonServer`'s single shared priority queue exactly like any other
ordinary request (`daemon.py`'s `UnixSocketListener._serve_client()`:
`reply = self._server.submit(request)`, uniformly, no special-casing).

When the daemon's one worker thread is already busy dispatching another
client's abort-aware `session_wait_for_done` call (e.g. a long `drive`
in progress), the probe queues FIFO behind it and does not run until
that dispatch call returns — either via its own natural completion, or
via an `abort` signal that only ever gets set when a *priority-0*
(estop-class) item is submitted while it is running. The probe itself
is priority-1, so it never sets that signal. The client-side
`_DaemonWireClient.call()` timeout (`DEFAULT_FIND_TIMEOUT_S = 1.0s`,
used by `find_daemon()`) elapses first, `find_daemon()` returns `None`,
and `get_connection(spawn=False)` silently falls back to a brand-new
**direct** connection — completely bypassing the daemon, and the real
`ESTOP` never reaches the priority queue at all. By the time anything
does, client A's own wait has already resolved via its own natural
timeout, not an abort.

This is a genuine chicken-and-egg gap: a brand-new (cold, one-shot)
client cannot submit an estop-priority request until *after* its own
non-privileged connectivity probe clears the same single-worker queue
the estop exists to jump. Production impact: on a serial robot, `rogo
estop` issued while another client's `drive` is in flight would fail to
preempt it and would instead try to open a serial port the daemon
already holds.

Ticket 001's exception also records that the obvious CLI-only
workaround (passing a larger `find_timeout=` into `get_connection()`,
something `cli.py` can already do with zero `daemon.py`/`daemon_client.py`
changes) does **not** fix the guarantee — it only trades a wrong silent
fallback for `get_connection()` itself blocking for the same ~9.5s,
arriving too late to still abort client A's wait before that wait's own
timeout fires. A real fix requires changing how the probe is routed,
which is what this ticket does.

**The fix**: give the connection-liveness probe its own path that never
touches `DaemonServer`'s queue or worker thread at all — it needs no
session state and nothing it says depends on serialized ordering
against other clients' dispatches, so removing it from the queue is
simpler and safer than reordering it within the queue (see Design
Rationale in sprint.md's Architecture section for the two rejected
alternatives — reclassifying `session_highest_acked` as estop-priority
globally, and raising the client-side find timeout — and why this
approach was chosen over both).

**IMPORTANT — do not touch ticket 001's already-completed, currently
uncommitted work**: `src/host/rogo/cli.py`, `src/host/rogo/agent_manual.py`,
and `tests/host/rogo/test_cli.py` are complete and passing; leave them
alone. `tests/host/rogo/test_daemon_e2e_multi_client.py` already has the
new CLI-level preemption test
(`test_estop_via_real_cli_subprocess_preempts_another_clients_in_flight_wait_through_the_real_daemon_wiring`,
currently failing) — this is the ticket's primary proof; it should turn
green once this ticket's fix lands, ideally with no change to its own
assertions.

## Acceptance Criteria

- [ ] A new, clearly-reserved daemon-protocol wire verb (its exact name
      is this ticket's own implementation choice, but it MUST NOT be
      `"ping"`, `"hello"`, or any of the six `session_*` names
      `build_session_dispatch_table()` already defines) is answered
      directly by `UnixSocketListener._serve_client()`, without ever
      calling `DaemonServer.submit()` — so its latency is bounded only
      by the listener's own per-client accept/read thread, never by
      `DaemonServer`'s current dispatch or queue depth. **Collision
      warning**: `"ping"` is already reused pervasively as a fake
      dispatch-table verb name across this test suite's own test
      infrastructure (`tests/host/rogo/daemon_test_helpers.py`'s default
      `ping`/`hello` table, and `test_daemon.py`,
      `test_daemon_transports.py`, `test_daemon_sim_e2e.py`,
      `test_daemon_e2e_multi_client.py`'s own module docstring) —
      intercepting it at the listener would silently break every one of
      those tests' own caller-supplied `"ping"` handlers. Pick a name
      that cannot plausibly collide with a caller-supplied dispatch
      table (e.g. a reserved/namespaced string), and document the
      reservation at its definition site so no future dispatch table
      accidentally reuses it.
- [ ] `daemon_client.find_daemon()`'s connectivity probe sends this new
      verb instead of `session_highest_acked`. `_wait_for_daemon()` (the
      auto-spawn polling loop) and `get_connection()` need no changes
      beyond this — they already call `find_daemon()` as their probe.
- [ ] No behavior change to `DaemonServer`'s priority-queue ordering,
      `_execute()`, the `abort`-event mechanism, or
      `daemon_client.is_estop_request()`'s classification. No change to
      `session_highest_acked`'s own behavior or meaning for any *other*
      caller (it remains an ordinary, non-priority RPC everywhere except
      that it is no longer what the probe itself sends).
- [ ] The existing (uncommitted, currently red)
      `test_estop_via_real_cli_subprocess_preempts_another_clients_in_flight_wait_through_the_real_daemon_wiring`
      in `tests/host/rogo/test_daemon_e2e_multi_client.py` passes.
      Prefer landing it with no change to its own assertions (it was
      written to the ticket 001 Implementation Plan's spec and already
      encodes the right tight elapsed-time bounds); only touch it if the
      fix genuinely requires a different bound, and say why in the
      commit.
- [ ] A new, more targeted regression test (in `test_daemon.py` and/or
      `test_daemon_client.py`, not requiring a real `tools/sim` or CLI
      subprocess) proves the mechanism directly: with the daemon's
      single worker thread genuinely occupied by a long-running,
      abort-aware dispatch (a fake handler blocking on an `Event`/poll
      loop, same pattern `test_daemon.py`'s own estop-preemption tests
      already use), a client's liveness-probe call still returns well
      within `DEFAULT_FIND_TIMEOUT_S`, without waiting for the busy
      dispatch to free the worker. This isolates the fixed mechanism
      from CLI-subprocess/sim timing noise, complementing (not
      replacing) the end-to-end test above — matching sprint 003 ticket
      011's own lesson (repeated once already by this exception) that a
      unit-level check of classification alone is not suffficient
      proof, so this test must still exercise a real `DaemonServer` +
      `UnixSocketListener` pair over a real socket, not a mock.
- [ ] `run_daemon_worker()`'s idle-timeout tracking (`_with_activity_tracking()`)
      is unaffected by liveness probes — a probe answered outside the
      dispatch table never touches `last_activity`, so it must not reset
      the idle clock. Add or confirm a test for this if one does not
      already cover it.
- [ ] Full scoped suite passes: `uv run python -m pytest -q
      tests/host/rogo/`.

## Implementation Plan

### Approach

1. In `src/host/rogo/daemon.py`, define a reserved liveness-probe verb
   constant near `UnixSocketListener` (mirroring how `DEFAULT_ESTOP_VERBS`
   is defined near `DaemonServer` above it), with a docstring explaining
   it is intercepted by the listener BEFORE `submit()` and must never be
   used as a key in any caller-supplied `DispatchTable`.
2. In `UnixSocketListener._serve_client()`'s read loop (~line 748-758),
   after `decode_request(line)` succeeds, special-case this verb: encode
   and write an immediate success `Reply` (e.g. `Reply.ok(request.id,
   {"alive": True})`) straight back on `client_sock`, and `continue` —
   never calling `self._server.submit(request)` for this one verb. Every
   other verb's handling is unchanged.
3. In `src/host/rogo/daemon_client.py`, change `find_daemon()`'s probe
   call (currently `wire.call("session_highest_acked", {}, timeout=timeout)`)
   to send the new verb instead (import the constant from `daemon.py`,
   which this module already imports). No other change to
   `find_daemon()`, `_wait_for_daemon()`, or `get_connection()`'s control
   flow.
4. `daemon_protocol.py` needs no change — it has no verb table of its
   own (its own module docstring), so a new verb name is just a string
   two already-coupled modules (`daemon.py`, `daemon_client.py`) agree
   on, the same pattern `_ESTOP_WIRE_VERBS` already uses.
5. Do not modify `is_estop_request()`, `DaemonServer._execute()`,
   `DaemonServer.submit()`'s priority/abort logic, or `run_stdio_pipe()`
   (the stdio-pipe transport has no discovery/probing concept — a
   caller of `run_stdio_pipe_from_args()` already holds the pipe, so
   `find_daemon()` is never used against it — only `UnixSocketListener`
   needs the fast path).

### Files to Modify

- `src/host/rogo/daemon.py` — add the reserved liveness-verb constant
  and the `UnixSocketListener._serve_client()` fast path.
- `src/host/rogo/daemon_client.py` — `find_daemon()`'s probe call.
- `tests/host/rogo/test_daemon.py` and/or `tests/host/rogo/test_daemon_client.py`
  — new targeted regression test (see Acceptance Criteria).
- `tests/host/rogo/test_daemon_e2e_multi_client.py` — verify the
  existing preemption test now passes; do not rewrite it unless the
  fix genuinely requires a different bound.
- `tests/host/rogo/test_daemon_transports.py`,
  `tests/host/rogo/daemon_test_helpers.py`, `tests/host/rogo/test_daemon_sim_e2e.py`
  — no changes expected, but their own `"ping"`/`"hello"` fake dispatch
  tables must stay green (see the collision warning above); run them to
  confirm.

### Testing Plan

- Scoped run for this ticket: `uv run python -m pytest -q
  tests/host/rogo/` (per `.claude/rules/source-code.md`).
- New: the targeted worker-busy liveness-probe regression test (see
  Acceptance Criteria).
- Must turn green, unmodified if possible: the existing CLI-level
  preemption test in `test_daemon_e2e_multi_client.py`.
- Must stay green: `test_daemon.py`, `test_daemon_transports.py`,
  `test_daemon_client.py`, `test_daemon_sim_e2e.py`,
  `test_daemon_e2e_multi_client.py`'s existing (non-CLI) preemption
  test, `test_cli_serve.py`, `test_mcp_server.py`.

### Documentation Updates

- `daemon.py`'s own module docstring already documents the
  "Estop-priority queue" and `is_estop` mechanisms in detail; add a
  short section (or extend the `UnixSocketListener` class docstring)
  documenting the new liveness-probe fast path and why it exists —
  future readers hitting the same "probe queues behind busy worker"
  question should find the answer here.
- No `docs/design/` changes — this project's `.clasi/config.yaml` has
  `design_docs: disabled`; sprint.md's Architecture section documents
  this ticket's architectural impact directly.

- **Existing tests to run**: `uv run python -m pytest -q
  tests/host/rogo/` (see Testing Plan above).
- **New tests to write**: the targeted worker-busy liveness-probe
  regression test described above.
- **Verification command**: `uv run python -m pytest -q
  tests/host/rogo/`

---
id: '001'
title: Add rogo estop CLI subcommand routed through daemon auto-detect
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: add-a-rogo-estop-cli-subcommand.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add rogo estop CLI subcommand routed through daemon auto-detect

## Description

`rogo` has no CLI surface for the daemon's estop-priority path sprint
003 built. Add `cmd_estop()` and an `estop` subparser to
`src/host/rogo/cli.py`, modeled directly on the existing
`cmd_stop()`/`cmd_hello()` pattern, so an operator or agent can send
the unsequenced `ESTOP` panic stop from the command line instead of
calling `robot_v6.motion.estop(session)` from Python. Resolves through
`daemon_client.get_connection(args, spawn=False)` — auto-detect only,
never auto-spawn, so a panic stop never pays daemon-startup latency.
This is the sprint's only ticket (trivial sizing, sprint.md
Architecture section): one subcommand in one existing module, no new
module, no new cross-module dependency, no data-model change.
Implements SUC-001 and closes issue
`add-a-rogo-estop-cli-subcommand.md`.

## Acceptance Criteria

- [ ] `rogo estop` sends `robot_v6.motion.estop(session)` — the
      unsequenced `ESTOP` (no `#id`, never acked, never made to wait
      behind a sequenced command).
- [ ] Connection resolves via `daemon_client.get_connection(args,
      spawn=False)` — auto-detect only, never auto-spawn — matching
      `cmd_stop()`/`cmd_hello()`.
- [ ] An `estop` subparser is registered with the same shared target
      options as every other one-shot subcommand (`--sim`/`--connect
      HOST:PORT`/`--port PORT`).
- [ ] `rogo estop --help` text and the top-level `--help` listing both
      make clear `estop` is the unsequenced panic stop, distinct from
      the sequenced `stop`.
- [ ] `cmd_estop()` pumps briefly (bounded, reusing
      `_pump_until`/`_DEFAULT_TIMEOUT`) for the robot's bare `estop`
      confirmation line (protocol.md §8.3) and prints it when
      received.
- [ ] A pump timeout with no confirmation line is NOT a failure —
      `cmd_estop()` still exits 0 in that case (sprint.md's Design
      Rationale). The command exits non-zero ONLY on a genuine
      transport failure (`TransportClosed` or connection-resolution
      failure), never on the absence of a confirmation the protocol
      doesn't guarantee timing for.
- [ ] `rogo estop` against `tools/sim` is verified end to end
      (`cli.main(["estop", "--sim"])` pattern, exit 0).
- [ ] A test proves `rogo estop`, invoked as the real CLI subcommand
      (not a raw `motion.estop()`/`daemon_client` call), preempts
      another daemon client's in-flight command through a real running
      `rogo serve` daemon — not merely a unit-level check of
      `daemon_client.is_estop_request()`'s classification. (Sprint 003
      ticket 011 found that classification silently broken once
      already despite green unit suites — this is the point of the
      ticket.)
- [ ] `src/host/rogo/agent_manual.py`'s `MANUAL` documents `estop`:
      what it sends, its distinction from `stop`, its no-auto-spawn
      routing.
- [ ] `tests/host/rogo/test_agent_manual.py`'s pinning test
      (`test_every_subcommand_and_option_appears_in_the_manual`) stays
      green with `estop` added.
- [ ] Scoped test command passes: `uv run python -m pytest -q
      tests/host/rogo/`.

## Implementation Plan

### Approach

1. In `src/host/rogo/cli.py`, add `cmd_estop(args)` immediately after
   `cmd_stop()` (~line 305), following `cmd_hello()`/`cmd_stop()`'s
   resolve/run/report/close shape: `conn =
   daemon_client.get_connection(args, spawn=False)`; run inside a
   try/finally that closes `conn.transport`; catch `TransportClosed`
   and report exit 1, same as the other two. Add a `_run_estop(session)`
   helper (mirroring `_run_hello()`/`_run_stop()`'s existing split,
   available for `repl` to reuse later even though `repl` reuse is out
   of scope here) that:
   - calls `robot_v6.motion.estop(session)` (unsequenced, no id),
   - pumps briefly via `_pump_until(session, lambda rs: any(r.verb ==
     "estop" for r in rs), timeout=_DEFAULT_TIMEOUT)` for the bare
     `estop` confirmation line,
   - prints a confirmation message if a matching reply arrived, else
     prints a note that no confirmation arrived within the timeout —
     but returns 0 either way (a pump timeout is not a failure, per
     sprint.md's Design Rationale).
2. Register the `estop` subparser in the ~1202-1376 block alongside
   `p_hello`/`p_stop`, with the same shared target-option wiring,
   `set_defaults(func=cmd_estop)`, and help text explicit that this is
   the unsequenced panic stop, distinct from the sequenced `stop`.
3. Update `src/host/rogo/agent_manual.py`'s `MANUAL` string: document
   `estop` alongside `stop`, contrasting sequenced-vs-unsequenced
   behavior, its no-auto-spawn daemon routing, and that it is never
   acked/never queued.
4. No changes to `daemon.py`, `daemon_client.py`, `daemon_protocol.py`,
   or `robot_v6/motion.py` — this ticket is CLI-surface only, per
   sprint.md's Out of Scope. `daemon_client.is_estop_request()`
   already classifies a `session_send_unsequenced` call whose
   `wire_verb` is `ESTOP` as estop-priority; routing `cmd_estop()`
   through the normal `daemon_client.get_connection()`/`Session` path
   should reach it without further wiring — the acceptance criteria
   above require proving that with a test rather than assuming it.

### Files to Modify

- `src/host/rogo/cli.py` — add `_run_estop()`, `cmd_estop()`, and the
  `estop` subparser registration.
- `src/host/rogo/agent_manual.py` — add `estop` documentation to
  `MANUAL`.
- `tests/host/rogo/test_cli.py` — add
  `test_estop_end_to_end_against_sim`, mirroring
  `test_stop_end_to_end_against_sim`/`test_hello_end_to_end_against_sim`
  (~line 40): `cli.main(["estop", "--sim"])`, assert exit code 0 and
  the confirmation text in `capsys`' captured stdout.
- `tests/host/rogo/test_daemon_e2e_multi_client.py` — add a CLI-level
  preemption test alongside the existing
  `test_estop_from_one_client_preempts_another_clients_in_flight_wait_through_the_real_daemon_wiring`,
  reusing that file's `isolated_socket_dir` fixture, `_spawn_serve()`/
  `_wait_for_daemon()`/`_terminate()` helpers, and its short-`/tmp`-
  under-`XDG_RUNTIME_DIR` isolation pattern (AF_UNIX `sun_path` length
  limit) — never touching the real `~/.rogo`. Client A: a real
  in-flight `WHEELS_V` plus a completion wait on the next (unreachable)
  sequence id, exactly as the existing test sets up, giving the daemon
  a long in-progress wait to preempt. Client B: instead of calling
  `motion.estop(client_b.session)` directly on an already-open session,
  spawn `rogo estop` as a real subprocess against the same running
  daemon (`python -m rogo.cli estop --sim`, relying on the same
  `XDG_RUNTIME_DIR`-scoped auto-detect `daemon_client.get_connection()`
  already uses) and assert it still preempts client A's in-flight wait
  within the same tight elapsed-time bounds the existing test asserts.
  Reap the spawned subprocess(es) in a `finally`, same as the existing
  test's `_terminate(proc)` pattern. This is the ticket's actual point:
  sprint 003 ticket 011 found `is_estop_request()`'s classification
  silently broken once already despite every per-ticket unit suite
  passing, so the CLI's own wire-up to that path must be proven with an
  end-to-end test, not assumed from the underlying primitive already
  being covered.
- `tests/host/rogo/test_agent_manual.py` — no new test expected;
  `test_every_subcommand_and_option_appears_in_the_manual` introspects
  `build_parser()` generically (sprint 002's convention) and should
  pick up `estop` automatically once the subparser and manual text
  exist — verify it stays green rather than adding a redundant check.

### Testing Plan

- Scoped run for this ticket: `uv run python -m pytest -q
  tests/host/rogo/` (per `.claude/rules/source-code.md`: a per-ticket
  run scoped to the modules touched; the full suite runs once at
  `close_sprint`, not per ticket).
- New tests: `test_estop_end_to_end_against_sim` (test_cli.py) and the
  CLI-level daemon-preemption test (test_daemon_e2e_multi_client.py)
  described above.
- Existing tests that must stay green and are not otherwise modified:
  `test_agent_manual.py`'s pinning test,
  `test_daemon_e2e_multi_client.py`'s existing estop-preemption test
  (unchanged — the new test is additive alongside it, not a
  replacement), and the full `test_cli.py`/`test_cli_serve.py`/
  `test_daemon_client.py` suites (no behavior of `stop`/`hello`/the
  daemon's existing dispatch table changes).

### Documentation Updates

- `src/host/rogo/agent_manual.py`'s `MANUAL` (see Approach step 3 and
  Files to Modify above).
- No `docs/design/` changes — this project's `.clasi/config.yaml` has
  `design_docs: disabled`, and sprint.md's Architecture section
  documents this ticket's (lack of) architectural impact directly.

- **Existing tests to run**: `uv run python -m pytest -q
  tests/host/rogo/` (see Testing Plan above for which existing suites
  must stay green).
- **New tests to write**: `test_estop_end_to_end_against_sim`
  (`tests/host/rogo/test_cli.py`) and a CLI-level estop-preemption test
  in `tests/host/rogo/test_daemon_e2e_multi_client.py` (see Files to
  Modify above for both).
- **Verification command**: `uv run python -m pytest -q
  tests/host/rogo/`

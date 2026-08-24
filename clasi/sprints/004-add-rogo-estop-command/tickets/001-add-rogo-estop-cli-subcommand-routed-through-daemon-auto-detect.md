---
id: '001'
title: Add rogo estop CLI subcommand routed through daemon auto-detect
status: open
use-cases:
- SUC-001
depends-on:
- '002'
github-issue: ''
issue: add-a-rogo-estop-cli-subcommand.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-08-24T00:01:50.084890+00:00'
  attempted: 'Implemented cmd_estop()/_run_estop() and the estop subparser in src/host/rogo/cli.py
    exactly per the plan (resolves via daemon_client.get_connection(args, spawn=False),
    sends robot_v6.motion.estop(session), pumps briefly for the bare ''estop'' confirmation
    line, returns 0 on a pump timeout per sprint.md''s Design Rationale), updated
    agent_manual.py''s MANUAL, and added the sim-level smoke test (test_cli.py::test_estop_end_to_end_against_sim,
    passing) plus help-text tests. All of that works. Then wrote the ticket''s required
    AC #4/#8 proof -- a CLI-level preemption test in test_daemon_e2e_multi_client.py
    extending the existing estop-priority scenario exactly as the Implementation Plan
    specifies: Client A opens a session, sends WHEELS_V, starts a completion wait
    on the next (unreachable) sequence id so the daemon''s single worker thread is
    genuinely blocked dispatching it; only then Client B spawns `python -m rogo.cli
    estop --sim` as a real subprocess against the same daemon. The test failed deterministically
    (reran twice, both times client A''s wait resolved at ~9.50s, not aborted -- confirmed
    not a flake) instead of preempting within the sibling test''s tight bounds. Root-caused
    with two standalone repro scripts (isolated from pytest) run in the foreground:
    daemon_client.find_daemon()''s own connectivity probe (`session_highest_acked`,
    verb not in ("session_send","session_send_unsequenced") so is_estop_request()
    classifies it priority=1, not estop) is submitted to DaemonServer''s SAME single
    global priority queue as every other request (daemon.py''s UnixSocketListener._serve_client():
    `reply = self._server.submit(request)`, uniformly, no special-casing). When another
    client''s abort-aware-but-not-yet-aborted wait_for_done dispatch already occupies
    the one worker thread, this ''cheap probe'' queues behind it and does not run
    until the worker frees up. _DaemonWireClient.call()''s client-side timeout (DEFAULT_FIND_TIMEOUT_S=1.0s,
    used by find_daemon()) elapses first, raising DaemonUnavailableError; find_daemon()
    catches it and returns None; get_connection(spawn=False) then falls back to connection.resolve(args),
    a brand-new DIRECT tools/sim connection completely disconnected from the daemon
    and from Client A''s session -- so the real ESTOP never reaches the daemon''s
    priority queue at all, and Client A''s wait only resolves ~10s later via its own
    natural client-side timeout, not an abort. Tested the obvious CLI-only mitigation
    (passing a larger find_timeout= to get_connection(), a call kwarg cli.py can already
    supply with zero daemon_client.py/daemon.py code changes): confirmed empirically
    via a third repro script that this does NOT fix the latency guarantee -- it only
    avoids the wrong-fallback; get_connection() itself then blocks for the full ~9.5s
    (the probe, still priority=1, still queues behind Client A''s item and only unblocks
    once that item''s OWN natural deadline elapses, since nothing priority=0 can be
    submitted until the probe -- which is what obtains the Session -- itself returns).
    By the time the real ESTOP is finally sent, Client A''s wait has already resolved
    via its own timeout, not via abort. This is a genuine chicken-and-egg gap: a brand-new
    (cold, one-shot) client cannot submit an estop-priority request until AFTER its
    own non-privileged connectivity probe clears the same single-worker queue the
    estop is supposed to jump.'
  conflict: 'sprint.md''s Out of Scope section: "Any change to daemon.py''s estop-priority
    queue logic or daemon_client.is_estop_request()''s classification itself -- sprint
    003 built and tested that path; this sprint only adds the CLI surface that calls
    into it." Ticket 001''s own Implementation Plan step 4 restates this explicitly:
    "No changes to daemon.py, daemon_client.py, daemon_protocol.py, or robot_v6/motion.py
    -- this ticket is CLI-surface only." Fixing the discovered gap requires one of:
    (a) daemon.py''s DaemonServer giving connectivity-probe requests (or find_daemon()''s
    own RPC) a queue-bypassing or priority-aware path so a probe from a brand-new
    client isn''t serialized behind an already-dispatching non-estop item, or (b)
    daemon_client.py reworking find_daemon()/get_connection() so a probe timeout under
    contention does not silently fall back to a disconnected direct connection when
    the daemon IS in fact running (just busy) -- both squarely inside the file changes
    this ticket/sprint explicitly forbids. The ticket''s own Approach step 4 assumed
    "routing cmd_estop() through the normal daemon_client.get_connection()/Session
    path should reach it without further wiring" and asked me to "verify this with
    a test rather than assuming it" (echoing sprint 003 ticket 011); the test the
    Implementation Plan itself specifies (Client A''s in-progress wait set up BEFORE
    Client B''s rogo-estop subprocess is spawned, asserting preemption "within the
    same tight elapsed-time bounds the existing test asserts") is what surfaces that
    the assumption is false for a cold CLI invocation -- I cannot make that assertion
    true without touching the files sprint.md declares out of scope.'
  surface: user-visible
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
This ticket's own scope is one subcommand in one existing module
(`cli.py`), no new module, no new cross-module dependency, no
data-model change — CLI-surface only. (Originally the sprint's only
ticket, sized Trivial; see the Revision note immediately below and
sprint.md's own Revision/Architecture sections — the sprint as a whole
is now sized Compact and has a second ticket, 002, which this ticket
depends on.) Implements SUC-001 and closes issue
`add-a-rogo-estop-cli-subcommand.md`.

## Revision note (reopened after exception)

This ticket's CLI-surface implementation (`cli.py`'s `cmd_estop()`/
`_run_estop()`/subparser, `agent_manual.py`'s `MANUAL` update, and
`test_cli.py`'s `test_estop_end_to_end_against_sim`) is **complete and
passing** — currently uncommitted in the working tree, and requires no
further changes or rework. This ticket threw an exception when its
required proof (a CLI-level preemption test through a real daemon,
already added to `test_daemon_e2e_multi_client.py`, also uncommitted)
failed deterministically: `daemon_client.find_daemon()`'s own
connectivity probe queues behind another client's in-flight dispatch on
the daemon's single worker thread, so a cold `rogo estop` invocation
falls back to a disconnected direct connection instead of reaching the
daemon at all. Full root-cause analysis is preserved in this ticket's
own `exception:` frontmatter above, and summarized in sprint.md's
`## Revision` section and in ticket 002's Description.

The stakeholder decided to fix this within sprint 004 rather than ship
with the gap. **Ticket 002 (new, `depends-on` this ticket) implements
that fix** in `daemon.py`/`daemon_client.py`, and now runs BEFORE this
ticket (see this ticket's own `depends-on: ['002']`). Once ticket 002
lands, this ticket's remaining work is verification only: re-run the
scoped suite and confirm the already-written CLI-level preemption test
(`test_estop_via_real_cli_subprocess_preempts_another_clients_in_flight_wait_through_the_real_daemon_wiring`
in `test_daemon_e2e_multi_client.py`) now passes. No further changes to
`cli.py`/`agent_manual.py`/`test_cli.py` are expected; Approach step 4
below ("no changes to daemon.py/daemon_client.py — CLI-surface only")
remains true **for this ticket** — those files are ticket 002's scope,
not this one's.

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
   (Post-exception: proving it is exactly what surfaced that
   `get_connection()`'s own connectivity probe — not `cmd_estop()`'s
   own wiring, which was and remains correct — could queue behind a
   busy worker. That gap is now ticket 002's scope, sequenced before
   this one; this step's own "CLI-surface only" boundary for THIS
   ticket is otherwise unchanged.)

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
- `tests/host/rogo/test_daemon_e2e_multi_client.py` — **already done,
  uncommitted, currently red** (`test_estop_via_real_cli_subprocess_preempts_another_clients_in_flight_wait_through_the_real_daemon_wiring`);
  do not rewrite it, just confirm it passes once ticket 002 lands. Kept
  below for reference to what it does and why: add a CLI-level
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

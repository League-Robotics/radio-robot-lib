---
status: in-progress
sprint: '004'
tickets:
- 004-001
- 004-002
---

# Add a `rogo estop` CLI subcommand

## Description

`rogo` has no `estop` subcommand. Sprint 003 built the daemon's
estop-priority path — an `ESTOP` from any client jumps the work queue
and aborts another client's in-flight completion wait — but there is no
CLI surface that reaches it. An operator or agent wanting a panic stop
today has to call `robot_v6.motion.estop(session)` directly from Python.
Sprint 003's ticket 011 flagged this explicitly and documented it rather
than expanding scope; this issue closes it.

`rogo stop` is not the same thing and must not be conflated with it:
`STOP` is a sequenced, ordinary halt that waits behind whatever else is
queued. `ESTOP` is the unsequenced panic path (protocol.md §8.3's
exemption set) that must execute even while the stream is stalled on a
sequence gap.

## Requirements

1. **`rogo estop` sends the unsequenced `ESTOP`** via
   `robot_v6.motion.estop(session)`. It carries no `#id`, is never
   acked or nacked, and must not be made to wait behind a sequenced
   command.

2. **Same shared target options** as every other one-shot subcommand
   (`--sim` / `--connect HOST:PORT` / `--port PORT`).

3. **Daemon routing: auto-detect, never auto-spawn.** Resolve through
   `daemon_client.get_connection(args, spawn=False)`, matching the other
   one-shot commands. Spawning a daemon during a panic stop would add
   startup latency to the one command that must never wait; if no daemon
   is running, fall back to today's direct connection.

4. **The estop-priority path must actually be hit.** When a daemon is
   running, `rogo estop` must preempt another client's in-flight
   command. `daemon_client.is_estop_request()` already classifies a
   `session_send_unsequenced` call whose `wire_verb` is `ESTOP` as
   estop-priority, so routing through the normal client path should
   reach it — verify this with a test rather than assuming it, since
   sprint 003's ticket 011 found this exact classification silently
   broken once already.

5. **Reporting and exit code.** `ESTOP` is never acked, so the command
   cannot wait for an ack the way `rogo stop` does. The robot replies
   with a bare `estop` line confirming the stop executed
   (protocol.md §8.3). Decide — and document — whether the command
   pumps briefly for that confirmation or returns immediately;
   `cmd_hello()`'s handling of the unsequenced `HELLO` is the closest
   existing precedent. Exit non-zero only on a genuine transport
   failure, never on the absence of a confirmation line the protocol
   does not guarantee timing for.

## Verification

- `rogo estop` sends `ESTOP` and is covered by a test against the sim.
- A test proves `rogo estop` preempts another daemon client's
  in-flight command (the requirement 4 check).
- `rogo estop --help` works, and the `--agent` manual documents the
  command, its distinction from `rogo stop`, and its no-auto-spawn
  routing. The manual's pinning test
  (`tests/host/rogo/test_agent_manual.py`) must stay green — it fails if
  a subcommand or option is added without a matching manual update.

## Related

- `src/host/rogo/cli.py` — `cmd_stop()` is the closest structural
  analogue; subparsers are registered around lines 1202-1376.
- `src/host/robot_v6/motion.py:140` — `estop(session)`.
- `src/host/rogo/daemon_client.py:510` — `is_estop_request()`.
- `docs/design/protocol.md` §8.3 (the unsequenced exemption set),
  `docs/design/motion-api.md` §3.7.
- Sprint 003 ticket 011, which flagged this gap.

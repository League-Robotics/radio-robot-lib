---
status: done
sprint: '003'
tickets:
- 003-001
- 003-004
- 003-005
- 003-006
- 003-007
- 003-008
- 003-009
- 003-010
- 003-011
---

# Rebuild the rogo serve daemon on the v6 stack: named per-robot Unix sockets, stdio pipe mode, sim support, MCP/CLI routed through it

## Background

The original `rogo serve` daemon lives in radio-robot-elite
(`src/host/robot_radio/io/server.py` + `client.py`, documented in
`agent_manual.py`) and was deliberately left out of the sprint 001 import
because it targeted protocol v5's binary plane (sprint 001's sprint.md,
"The rogo-revival worktree" section). Its purpose stands: on macOS,
closing the serial port drops DTR (HUPCL) and resets the MCU, wiping
live-pushed config — a daemon that holds the port open means one-shot
tools and multiple programs can share a robot without rebooting it
between commands. There is no v6 version anywhere; this is a rebuild on
this repo's `robot_v6` stack, using the v5 daemon's design (framed JSON
replies with echoed request ids, single wire-owner executor thread,
estop-priority queue) as the spec, not its code.

## Requirements

1. **`rogo serve` daemon.** Holds ONE connection (serial `--port`, sim
   `--sim`, or TCP `--connect`) open for its whole lifetime and exposes
   the command grammar to clients over a framed wire protocol: one
   request line in, one JSON reply line out, with echoed correlation
   ids (per the elite daemon's protocol shape). Human repl text is not
   the wire format.

2. **Two transports, same protocol:**
   - **Unix socket (production):** the daemon listens on a Unix domain
     socket at a well-known path, named after the robot it is connected
     to (e.g. `tovez` → `.../rogo/tovez.sock`). Multiple robots on one
     host = multiple daemons, each discoverable by robot name. Exact
     well-known directory is an architecture decision (e.g.
     `$XDG_RUNTIME_DIR/rogo/` or `~/.rogo/run/`).
   - **stdio pipes (testing/embedding):** the daemon can instead run as
     a forked child speaking the SAME framed protocol over
     stdin/stdout. A test forks the daemon, writes requests, reads JSON
     replies — exercising the exact wire protocol production uses.
   - Robot name resolution for the socket name: from the robot's own
     hello/identify response where possible, overridable by flag.

3. **Sim support.** The daemon must be able to open the simulator as
   its target (same `--sim` resolution `rogo.connection` already does),
   so a test run is: start sim → start daemon against it → talk to the
   daemon. Ideally `rogo serve --sim` boots the sim itself the way the
   CLI does today.

4. **MCP server becomes a daemon client.** `rogo mcp` should connect to
   a running daemon (by robot name / socket path) instead of owning the
   serial connection itself, so an MCP session and other tools can
   share one robot without fighting over the port. (Fallback behavior
   when no daemon is running — auto-spawn vs. error — is an
   architecture decision.)

5. **CLI routed through the daemon.** One-shot `rogo <cmd>` invocations
   and `rogo repl` should be able to run through a daemon (elite had
   `--connect`-style routing; here it would be "by robot name" or
   socket path), fixing the one-shot-reset failure mode. Whether
   daemon-routing is opt-in or auto-detected is an architecture
   decision.

6. **Unbuffered output, always.** Independent of the daemon:
   `rogo repl` (and the daemon's pipe mode) MUST always flush output
   line-by-line — never block-buffered when stdout is a pipe. No
   `PYTHONUNBUFFERED=1` countermeasure should be required of the
   caller. This is a small standalone fix (flush on every emitted line
   or force line buffering at startup) that could land ahead of the
   daemon work.

## Safety carry-over from the v5 design

The elite daemon's halt path is a hard requirement in any rebuild: an
`estop`/`halt` request from ANY client jumps to the front of the work
queue and aborts any in-progress completion wait, so one client's long
`drive` can never delay another client's halt.

## References

- Elite daemon: `radio-robot-elite/src/host/robot_radio/io/server.py`,
  `client.py`, `agent_manual.py` (v5-era, spec-only)
- Why it exists: elite issue
  `clasi/issues/later/A-port-close-resets-the-robot-live-config-still-wiped.md`
- Sprint 001 exclusion rationale: `clasi/sprints/done/001-import-rogo-cli-onto-the-v6-host/sprint.md`
- This repo's current single-consumer holders: `rogo repl`
  (`src/host/rogo/repl.py`), `rogo mcp` (`src/host/rogo/mcp_server.py`)

# rogo — command-line control for a protocol-v6 robot

`rogo` is the command-line surface for driving/turning/calibrating a
robot, and for exposing it over MCP, all through this repo's own
`robot_v6` host client (`src/host/robot_v6/`) and protocol v6 — a
robot, a relay, or `tools/sim`, all the same way. It was adapted (not
vendored) from `radio-robot-elite/src/host/robot_radio`'s CLI onto this
stack; the full rationale for what ported and what didn't lives in
[`clasi/sprints/001-import-rogo-cli-onto-the-v6-host/sprint.md`](../../../clasi/sprints/001-import-rogo-cli-onto-the-v6-host/sprint.md)'s
Architecture and Design Rationale sections and is not repeated here.

Install: `[project.scripts]` in this repo's `pyproject.toml` registers
the `rogo` console script (`rogo.cli:main`).

Run `rogo --agent` for the complete agent-oriented reference — every
subcommand, every option, units, exit-code semantics, and the MCP tool
list — in one page; `rogo --help`/`rogo <subcommand> --help` stay the
short usage below.

## Commands

Every subcommand accepts a target: `--sim` (spawn a freshly built
`tools/sim` subprocess, no hardware needed), `--connect HOST:PORT` (a
relay or `tools/sim --listen`), or `--port PORT` (a real serial port) —
see `rogo.connection`.

- `rogo hello` — send `HELLO`, print the device banner.
- `rogo stop` — send the sequenced `STOP`.
- `rogo drive <L> <R> [--ms N | --mm N | stream] [--resend MS]` — one
  `WHEELS_V`/`WHEELS_X` call, or a `WHEELS_V` keepalive stream until
  Ctrl-C.
- `rogo turn <degrees> [--speed]` — one `WHEELS_V` call, using the
  active robot's `trackwidth`/`rotational_slip` (`rogo.turn_model`).
- `rogo goto <x> <y> [--speed] [--arrive] [--timeout]` — one `GO_TO_R`
  call, **robot-frame only** (see "Not ported" below).
- `rogo config get [name]` / `rogo config set <name> <value>` — raw
  `GET`/`SET` wire delegation (`protocol.md#7`); this library keeps no
  field table of its own.
- `rogo calibrate turns|distance [--speed] [--trials N] [...]` — a
  manual, tape-measure/protractor-verified multi-trial run against the
  *active* robot (`config/robots/active_robot.json`), writing an
  updated `rotational_slip`/`distance_scale` back to that robot's JSON
  config on confirmation. `turns` defaults to a 90° target rotation
  (`rogo.calibrate.DEFAULT_TURN_TARGET_DEG`) — a quarter turn is the
  natural human-measurable analog of elite's own camera-read full spin.
  `distance_scale` is a field new to this library (`rogo.config`'s own
  module docstring); no staged `config/robots/*.json` file carries it
  under either shape until a robot has been through `calibrate
  distance` at least once.
- `rogo repl [COMMAND ...]` — run one or more commands (an argument
  list, piped stdin, or an interactive prompt) over a single persistent
  connection, reusing `rogo.cli`'s own `hello`/`stop`/`drive`/`turn`/
  `goto`/`config` dispatch. `calibrate` is deliberately **not**
  available inside a repl line — its own multi-trial wizard is not a
  single self-contained command. Resolves its connection through
  `rogo.daemon_client`'s **auto-spawn** policy — see "`rogo serve`" below.
- `rogo mcp [--listen HOST:PORT [--allow-remote]]` — start an MCP
  server exposing `hello`/`stop`/`drive`/`turn`/`goto`/`config_get`/
  `config_set`/`calibrate_turns` (8 tools) as MCP tools. Defaults to
  `stdio` transport (no network surface at all); `--listen` opts into
  TCP, restricted to loopback unless paired with `--allow-remote`.
- `rogo serve [--sim|--connect|--port] [--name NAME] [--socket-dir DIR]
  [--idle-timeout SECONDS] [--stdio-pipe]` — start a daemon holding one
  connection open for the whole process's lifetime, serving any number
  of clients (see below).

## `rogo serve` — the daemon (`rogo.daemon`/`rogo.daemon_client`)

`rogo.daemon` (sprint 003) holds one robot/relay/sim connection open for
a process's whole lifetime and serves it to any number of clients over
a framed JSON request/reply wire (`rogo.daemon_protocol`), with an
estop-priority queue so any client's `ESTOP` jumps ahead of another
client's in-flight command, aborting that command's own completion wait
in progress if one is currently running (`DaemonServer`, ticket 005;
`DaemonServer(..., is_estop=...)`, ticket 011 — the classifier
`cli.cmd_serve()` injects, `daemon_client.is_estop_request()`, is what
makes this hold for a REAL `ESTOP` sent through the generic session-RPC
dispatch table `rogo serve` actually runs, not just for a directly-named
`"estop"` verb in a test's own fake dispatch table; see `daemon.py`'s
own `is_estop` docstring section for the gap this closed). The planned,
sequenced `stop` every other subcommand sends is NOT estop-priority —
only `ESTOP` is (`robot_v6.motion.estop()`; `rogo` itself has no
dedicated `estop` subcommand today). It exposes that server core over
two interchangeable listener transports — same protocol, different I/O
— plus the robot-name resolution that decides what the Unix-socket
transport's file is named.

`rogo.cli`'s `serve` subcommand (`cmd_serve()`, ticket 009) wires this
up end to end: it injects `rogo.daemon_client.
build_session_dispatch_table()` — a generic Session-RPC dispatch table,
not a per-CLI-verb one, so every `cli.py` dispatch body (`_run_hello`,
`_dispatch_drive_mode`, …) runs completely unchanged whether its
connection is direct or daemon-proxied. Every one-shot subcommand
(`hello`/`stop`/`drive`/`turn`/`goto`/`config`/`calibrate`)
**auto-detects** an already-running daemon for its resolved target and
routes through it when found, falling back to a direct connection
unchanged when none is found; `rogo repl`/`rogo mcp` **auto-spawn** one
when none is running (`rogo.daemon_client.get_connection()`, ticket 008)
— an auto-spawned daemon outlives the session that spawned it and
self-terminates after an idle timeout (5 minutes by default, overridable
via `ROGO_DAEMON_IDLE_TIMEOUT`).

- **Unix domain socket (production)** — `UnixSocketListener` binds a
  socket at `$XDG_RUNTIME_DIR/rogo/<name>.sock` when that env var is
  set, else `~/.rogo/run/<name>.sock` (`daemon.default_socket_dir()`/
  `daemon.socket_path_for_name()`); the containing directory is created
  with owner-only (`0700`) permissions if missing
  (`daemon.ensure_socket_dir()`). `<name>` is the target's resolved
  robot name, so two differently-named robots on one host run two
  independent, independently-discoverable daemons with no collision.
  Multiple clients may connect at once, each on its own thread — an MCP
  session and a CLI invocation can share one robot through the same
  socket.
- **stdio pipe (tests/embedding)** — `daemon.run_stdio_pipe(server)`
  speaks the identical framed protocol over the process's own
  stdin/stdout, with no socket file created at all. This is how a test
  forks the daemon as a subprocess and exchanges real wire-protocol
  request/reply lines against it with no `tools/sim` process or Unix
  socket involved — see `tests/host/rogo/test_daemon_transports.py`.
  Output is always flushed line-by-line (ticket 003-001's fix), so a
  reader on the far end of the pipe never waits on a block buffer.

**Robot-name resolution** (`daemon.resolve_robot_name()`) — an explicit
override, if given, wins immediately; otherwise the target's own
`HELLO`/`device` banner supplies the name (protocol.md#8.3); a `--sim`
target with neither falls back to a fixed default (`"sim"`).

## What was deliberately not ported, and why

See sprint.md's Design Rationale (Decisions 3-5) and Scope's "Out of
Scope" section for the full reasoning; only the pointers, not the
argument, are repeated here:

- **No camera-based `goto`/`turnto`, no `--auto` calibration.** Elite's
  closed-loop pure-pursuit `goto`/`turnto` and `--auto` calibration mode
  both depend on an `aprilcam` camera daemon this repo has no concept
  of. `rogo goto` maps onto the wire-level `GO_TO_R` verb instead
  (robot-frame only — world-frame `go_to_w` stays unavailable until a
  pose source exists, `specification.md#13`); `rogo calibrate` ports
  only elite's fully self-contained manual/tape-measure mode.
- **`rogo serve` was deferred, then rebuilt (sprint 001 decision,
  superseded by sprint 003).** Sprint 001 left it out because
  `robot_v6.transport.SocketTransport` already lets any client connect
  directly to a robot, relay, or `tools/sim` with no relay in between —
  but a direct connection still means every client owns and closes its
  own connection, which resets the robot on macOS (DTR/HUPCL) between
  invocations. Sprint 003 rebuilt `rogo serve` on this repo's v6 stack
  for exactly that reason — see "`rogo serve` — the daemon" above.
- **No digital/analog port, gripper, color/line-sensor, or OTOS/pose
  commands.** `DiffDriveAdapter` doesn't expose any of this hardware
  surface, and this sprint is host-side only (no firmware changes) —
  see sprint.md's Scope.

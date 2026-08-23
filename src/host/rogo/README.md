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
  single self-contained command.
- `rogo mcp [--listen HOST:PORT [--allow-remote]]` — start an MCP
  server exposing `hello`/`stop`/`drive`/`turn`/`goto`/`config_get`/
  `config_set`/`calibrate_turns` (8 tools) as MCP tools. Defaults to
  `stdio` transport (no network surface at all); `--listen` opts into
  TCP, restricted to loopback unless paired with `--allow-remote`.

## The daemon's two transports (`rogo.daemon`)

`rogo.daemon` (sprint 003) holds one robot/relay/sim connection open for
a process's whole lifetime and serves it to any number of clients over
a framed JSON request/reply wire (`rogo.daemon_protocol`), with an
estop-priority queue so any client's halt jumps ahead of another
client's in-flight command (`DaemonServer`, ticket 005). It exposes
that server core over two interchangeable listener transports — same
protocol, different I/O — plus the robot-name resolution that decides
what the Unix-socket transport's file is named. (No `rogo serve` CLI
subcommand exists yet — see "No `rogo serve` daemon" below; these are
today library-level building blocks a later sprint 003 ticket wires up.)

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
- **No `rogo serve` daemon (sprint 001 decision, superseded by sprint
  003).** Sprint 001 left this out because `robot_v6.transport.
  SocketTransport` already lets any client connect directly to a
  robot, relay, or `tools/sim` with no relay in between — but a direct
  connection still means every client owns and closes its own
  connection, which resets the robot on macOS (DTR/HUPCL) between
  invocations. Sprint 003 rebuilds `rogo serve` on this repo's v6 stack
  for exactly that reason; see "The daemon's two transports" above for
  what exists so far.
- **No digital/analog port, gripper, color/line-sensor, or OTOS/pose
  commands.** `DiffDriveAdapter` doesn't expose any of this hardware
  surface, and this sprint is host-side only (no firmware changes) —
  see sprint.md's Scope.

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
- **No `rogo serve` daemon.** Elite's multi-client TCP relay exists to
  work around one platform hazard (macOS resets the robot when its
  serial port closes); `robot_v6.transport.SocketTransport` already
  lets any client connect directly to a robot, relay, or `tools/sim`
  with no Rogo-specific relay in between.
- **No digital/analog port, gripper, color/line-sensor, or OTOS/pose
  commands.** `DiffDriveAdapter` doesn't expose any of this hardware
  surface, and this sprint is host-side only (no firmware changes) —
  see sprint.md's Scope.

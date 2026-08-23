"""The rogo agent manual -- printed verbatim by ``rogo --agent``.

Same convention as elite's own ``rogo --agent``
(``radio-robot-elite/src/host/robot_radio/io/agent_manual.py``): a
self-contained Markdown manual written for AI coding agents (and power
users) driving the tool non-interactively. ``rogo --help`` /
``rogo <subcommand> --help`` stay the short per-subcommand usage; THIS
document is the complete reference for the actual command surface this
repo's `rogo` ships against a protocol-v6 robot, relay, or `tools/sim`
(sprint.md's Architecture and Design Rationale, clasi/sprints/002-add-
rogo-agent-manual/sprint.md) -- deliberately NOT a port of elite's own
manual content: no daemon (`rogo serve`), no socket-relay protocol, no
binary command plane, no macOS-HUPCL serial-port-reset advice. None of
that exists in this repo's `rogo`.

Keep this text in lockstep with ``rogo.cli.build_parser()``: a pinning
test (``tests/host/rogo/test_agent_manual.py``) introspects the parser's
own subcommand names, sub-subcommand names, and every registered option
string, then asserts each one appears somewhere in ``MANUAL`` below --
so a future rename or addition fails that test loudly instead of
silently staling this manual (sprint.md's Design Rationale: the pinning
test derives its own expectations from `build_parser()`, not a
hand-maintained checklist).
"""
from __future__ import annotations

MANUAL = r"""# rogo -- Agent Manual

Written for AI coding agents (and power users) driving `rogo`
non-interactively. `rogo -h` / `rogo --help` (and `rogo <subcommand>
-h`/`--help`) print argparse's own short usage; this document is the
complete reference: every subcommand, every option, units, exit-code
semantics, and the operational knowledge `--help` text does not carry.

`rogo` talks to a protocol-v6 robot, relay, or `tools/sim` -- a single
plain-ASCII line grammar (no COBS/CRC/binary command plane). There is no
daemon in this repo (unlike elite's own `rogo serve`): every subcommand
below either resolves one connection and exits (`hello`, `stop`,
`drive`, `turn`, `goto`, `config get`/`set`, `calibrate turns`/
`distance`), or holds one connection open for a whole session (`repl`,
`mcp`).

---

## 1. Command surface

```
rogo [--agent] <subcommand> ...
```

| Subcommand | What it does |
|---|---|
| `hello` | One-shot probe: send `HELLO`, print the device banner. |
| `stop` | Send the sequenced `STOP` (a PLANNED stop, not a halt). |
| `drive` | `rogo drive <L> <R> [--ms N \| --mm N \| stream] [--resend MS]` -- one `WHEELS_V`/`WHEELS_X` call, or a `WHEELS_V` keepalive stream until Ctrl-C. |
| `turn` | `rogo turn <degrees> [--speed]` -- one `WHEELS_V` call using the active robot's rotation model. |
| `goto` | `rogo goto <x> <y> [--speed] [--arrive] [--timeout]` -- one `GO_TO_R` call, robot-frame only. |
| `config get`/`config set` | Raw `GET`/`SET` wire delegation -- this library keeps no field table of its own. |
| `calibrate turns`/`calibrate distance` | A manual, tape-measure/protractor-verified multi-trial run against the *active* robot config. |
| `repl` | Run one or more commands over ONE persistent connection: an argument list, piped stdin, or an interactive prompt. |
| `mcp` | Start an MCP server exposing 8 tools (`hello`/`stop`/`drive`/`turn`/`goto`/`config_get`/`config_set`/`calibrate_turns`), `stdio` by default. |

`rogo --agent` prints this manual and exits 0, checked BEFORE any
subcommand is required and BEFORE any target (`--sim`/`--connect`/
`--port`) is ever resolved -- it never opens a connection, never spawns
`tools/sim`, and works with no other arguments at all.

---

## 2. Shared target options

Every subcommand except `--agent` itself needs a target, added by
`rogo.connection.add_target_arguments()` and mutually exclusive with
each other:

| Option | Meaning |
|---|---|
| `--sim` | Spawn a freshly (re)built `tools/sim` subprocess over stdio. No robot, no serial port, no manual build step -- `rogo.connection.ensure_sim_binary()` (re)compiles it on demand if missing or stale. |
| `--connect HOST:PORT` | Talk to a TCP peer -- a relay, or `tools/sim --listen`. |
| `--port PORT` | Talk to a real robot over a serial port. |

Naming zero targets, or more than one, is a usage error: `error: no
target specified -- pass --sim, --connect HOST:PORT, or --port PORT`
(exit 2) or `error: choose exactly one target, got ...` (exit 2). A
`--sim` build failure (no compiler, a real compile error) raises its own
`error: ...` message and exits 3, distinct from every other error path
below.

---

## 3. Subcommand reference

### `rogo hello [--sim|--connect|--port]`

Sends unsequenced `HELLO`, waits for the `device` banner, and prints it:

```
$ rogo hello --sim
role=<role> common_name=<common_name> name=sim serial=<serial>
```

Exit 0 on a banner received; exit 1 (`no device banner received`,
stderr) if none arrives within the default 3s timeout.

### `rogo stop [--sim|--connect|--port]`

Sends the sequenced `STOP` (a PLANNED stop -- it queues behind any
in-flight move; it is NOT a halt/estop).

```
$ rogo stop --sim
STOP acked (#1)
```

Exit 0 if acked; exit 1 (`STOP sent (#N) but not acked within 3.0s`,
stderr) otherwise.

### `rogo drive <left> <right> [--ms N | --mm N | stream] [--resend MS] [--sim|--connect|--port]`

`left`/`right` are per-wheel speeds in mm/s (int). Exactly one shape:

- `--ms N` (int, ms): one `WHEELS_V` call held for `N` ms, waits for the
  completion ack.

  ```
  $ rogo drive 150 150 --ms 800 --sim
  WHEELS_V acked (#2), done reason=timeout
  ```

- `--mm N` (int, mm): one `WHEELS_X` call for `N` mm of per-wheel
  distance at a cruise speed derived from `left`/`right`'s magnitude.
  `WHEELS_X` is `kUnknown` on `DiffDriveAdapter` today -- see section 4's
  soft-warning rule; the call is still genuinely sent and acked.
- `stream` (literal keyword, or bare `drive <L> <R>` with neither flag):
  re-issues `WHEELS_V` at `--resend MS` cadence (int, default 150) until
  Ctrl-C, then sends `STOP`:

  ```
  $ rogo drive 150 -150 stream --resend 100 --sim
  streaming WHEELS_V 150 -150 (resend every 100ms, lease 300ms) -- Ctrl-C to stop
  ^C
  STOP acked (#3)
  ```

`--ms` and `--mm` are mutually exclusive with each other and with
`stream`; `--resend` must be > 0. Any violation is a usage error, exit 2.

### `rogo turn <degrees> [--speed MM_S] [--sim|--connect|--port]`

`degrees` (float, signed, positive = CCW) and `--speed` (float, mm/s,
default 200) drive `rogo.turn_model.compute_turn()`, which needs the
*active* robot's `geometry.trackwidth` (falls back to no-slip if
`rotational_slip` is absent) -- see section 6. One `WHEELS_V` call:

```
$ rogo turn 90 --speed 200 --sim
turn +90.0deg -> WHEELS_V -200 200 707 (#4), done reason=timeout
```

Exit 1 if no active robot config with `geometry.trackwidth` exists; exit
2 if `--speed <= 0`; exit 0/1 on the ack/completion outcome exactly like
`drive --ms` above (same soft-warning rule too, since this also goes
over `WHEELS_V`).

### `rogo goto <x> <y> [--speed MM_S] [--arrive MM] [--timeout MS] [--sim|--connect|--port]`

Robot-frame only (`x` forward, `y` left, both int, mm) via one
`GO_TO_R` call. `--speed` (int, mm/s, default 200), `--arrive` (int, mm,
default 0 -- 0 takes the adapter's own configured default arrival
tolerance), `--timeout` (int, ms, default: a generous ETA-based backstop
computed from distance/speed when omitted).

```
$ rogo goto 300 0 --speed 200 --sim
goto (300, 0) -> GO_TO_R 300 0 200 0 1500 (#5), done reason=timeout
```

`GO_TO_R` is `kUnknown` on `DiffDriveAdapter` today -- the same soft
warning rule as `drive --mm` applies (section 4); an ack only means the
call was ACCEPTED, `done reason=...` is the only arrival signal ever
printed.

### `rogo config get [name] [--sim|--connect|--port]`

Bare `GET` lists every field the adapter reports, one `name=value` line
each; a `name` argument asks for just that one.

```
$ rogo config get --sim
wheel_control.pid_kp=1.5
...
$ rogo config get wheel_control.pid_kp --sim
wheel_control.pid_kp=1.5
```

An unknown `name` gets acked but produces NO `get` line at all
(protocol-level behavior) -- reported here as `error: no such config
field: 'name'` (stderr, exit 1), not a silent empty success. A bare
`GET` with zero fields prints `(adapter reports no config fields)` and
exits 0.

### `rogo config set <name> <value> [--sim|--connect|--port]`

`value` is a **float** -- the one float-typed wire field in this whole
CLI (see section 5).

```
$ rogo config set wheel_control.pid_kp 2.0 --sim
SET wheel_control.pid_kp=2.0 acked (#6)
```

An unrecognized `name` is a genuine caller mistake, reported as a HARD
error and a NONZERO exit code (`error: SET <name> rejected -- no such
config field: '<name>'`, exit 1) -- this is deliberately NOT the same
soft-warning treatment `drive --mm`/`goto` get for the identical
`ERR_UNKNOWN` wire code; see section 4 for why the two cases differ.

### `rogo calibrate turns [--speed MM_S] [--trials N] [--sim|--connect|--port]`

Manual, tape-measure/protractor-verified multi-trial run: prompts to
spin a fixed 90-degree target (not a CLI-configurable target), reads the
operator's measured degrees each trial, computes an updated
`rotational_slip`, and prompts to save it to the active robot's config
file. `--speed` (float, mm/s, default 200), `--trials` (int, default 6,
minimum 3 usable samples needed to compute a result). This is an
INTERACTIVE wizard (`input()`-based prompts) -- an agent driving `rogo`
non-interactively should prefer the `mcp` server's `calibrate_turns`
tool instead (section 7), which takes already-measured values as a
single non-interactive call.

### `rogo calibrate distance [--distance MM] [--speed MM_S] [--trials N] [--sim|--connect|--port]`

The straight-line analog of `calibrate turns`: drives `--distance` mm
(float, default 400) at `--speed` mm/s (float, default 200) for
`--trials` (int, default 3, minimum 3), computes an updated
`distance_scale`, and prompts to save it. Same interactive-wizard caveat
as `calibrate turns`.

Both calibrate commands exit 1 if no active robot config exists, or if
fewer than 3 usable trials were recorded, or if the computed value falls
outside the sane range `[0.5, 1.5]` (a value outside that range is
refused, not silently saved -- see `rogo.calibrate`'s own module
docstring). Exit 2 on a bad `--speed`/`--trials`/`--distance` (must all
be > 0).

---

## 4. Exit-code semantics -- the kUnknown soft-warning rule

Most one-shot subcommands share the same three broad outcomes: exit 0
(the operation completed, or was accepted with only a soft warning),
exit 1 (a timeout, a transport failure, or a hard content error), or
exit 2 (a usage error caught before any wire call was even sent).

**The kUnknown soft warning (STAKEHOLDER DECISION, binding):** `drive
--mm` (`WHEELS_X`) and `goto` (`GO_TO_R`) both hit `DiffDriveAdapter`'s
documented planner gap on this adapter today -- the wire verb genuinely
was sent and genuinely was ACKED (the sequence still advances on a
merits rejection), so the adapter's rejection is reported as a plain
`warning: ... was acked and sent, but the adapter rejected it on merit:
no planner for this verb (kUnknown/ERR_UNKNOWN)` (stderr) plus a `...
sent (#N); adapter reports ...` line (stdout) -- and the process still
**exits 0**. This is NOT a silent success and NOT a crash: the call
really happened on the wire, it just wasn't honored kinematically.

**`config set`'s hard-error contrast:** an unrecognized field name hits
the exact same wire error code (`ERR_UNKNOWN`, err code 1) but is
reported as a genuine caller mistake instead -- `error: SET <name>
rejected -- no such config field: ...` and **exit 1**, not a warning.
There is no adapter anywhere that would ever accept a mistyped field
name, unlike `drive --mm`/`goto`, which DO work against an adapter that
implements the verb (e.g. `tools/sim`'s own motion adapter can differ
from a real robot's `DiffDriveAdapter`).

**Everything else**: a not-acked-within-timeout, or acked-but-never-
completed, condition is always exit 1 (printed to stderr). A malformed
argument (mutually exclusive drive flags, `--speed <= 0`, no target
named, more than one target named) is always exit 2, caught before any
connection is ever opened. `--sim`'s own build failure is exit 3.

---

## 5. Units and wire-field typing

All linear units are **mm** and **mm/s**; durations are **ms**; angles
are **degrees**. Watch the int/float split -- it is NOT uniform:

| Field(s) | CLI type | Why |
|---|---|---|
| `drive`'s `left`/`right`, `--ms`, `--mm`, `--resend` | `int` | `WHEELS_V`/`WHEELS_X` decode these with `parseInt32`/`parseUint32` firmware-side; a value with a decimal point (`"100.0"`) is a DECODE FAILURE, not a rejected-but-acked call. |
| `goto`'s `x`, `y`, `--speed`, `--arrive`, `--timeout` | `int` | All five of `GO_TO_R`'s wire fields are `parseInt32`/`parseUint32`'d, same reasoning. |
| `turn`'s `degrees`, `--speed` | `float` | CLI-side physics only -- `turn_model.compute_turn()` does its own math in float and rounds to `int` (`cmd_l`/`cmd_r`/`duration_ms`) only at the very end, before the `WHEELS_V` call. |
| `calibrate turns`'s `--speed`, `calibrate distance`'s `--distance`/`--speed` | `float` | Same CLI-side-physics reasoning as `turn`; the final `WHEELS_V` call still gets rounded ints. |
| `config set`'s `value` | `float` | The ONE genuinely float-typed WIRE field in this whole CLI -- `SET`'s value is `parseFloatField`'d firmware-side, unlike every motion verb above. |

Passing a fractional value to any of the `int`-typed fields above (e.g.
`rogo goto 100.5 0`) is a CLI-side `argparse` type error before any
connection is even attempted -- not a wire-level soft warning.

---

## 6. `repl` -- one persistent session

`rogo repl [COMMAND ...] [--sim|--connect|--port]` resolves **ONE**
connection for its entire lifetime and reuses that same connection for
every line -- unlike every other subcommand above, which opens a fresh
connection per invocation. Three ways in:

```
rogo repl "hello" "drive 100 100 --ms 200" "stop" --sim   # argument list
cat script.rogo | rogo repl --sim                          # piped stdin, one command per line
rogo repl --sim                                             # interactive prompt (tty)
```

Each line is parsed by the SAME `argparse` parser `rogo.cli.
build_parser()` builds for direct CLI use -- the same flags, same
defaults, same per-verb reporting. `quit`/`exit` end the loop; a blank
line or a line starting with `#` is ignored. A bad line (unknown verb,
bad flag, `--help`) prints its own error and does NOT end the session --
only EOF, Ctrl-C, or an explicit `quit`/`exit` does.

**`calibrate` and `mcp` are NOT available as repl lines** -- typing
either inside a repl session prints `error: 'calibrate' is not
supported inside 'rogo repl' -- run it as its own separate rogo command
instead` (or the same for `'mcp'`) and exits that one line with 2; the
session itself keeps running. `calibrate`'s own multi-trial wizard and
`mcp`'s own long-running server both need to own the whole process, not
one line of a shared session.

The repl's own overall process exit code is always 0 for a clean
session end (EOF/quit/Ctrl-C) -- it does NOT track whether any
individual dispatched line itself failed; each line's own outcome is
reported inline (via the same printing every direct subcommand uses),
not accumulated into the session's own exit code.

---

## 7. `mcp` -- the MCP server

`rogo mcp [--sim|--connect|--port] [--listen HOST:PORT [--allow-remote]]`
resolves ONE connection (same resolution as every other subcommand) and
serves it for the whole server lifetime via 8 tools:

| Tool | Arguments | Notes |
|---|---|---|
| `hello` | (none) | Returns `{role, common_name, name, serial}`. |
| `stop` | (none) | Returns `{acked, seq_id}`. |
| `drive` | `left: int, right: int, ms: int` | One `WHEELS_V` call only -- `--mm`/`stream` are NOT exposed as MCP tools (no one-shot MCP shape fits a Ctrl-C-driven stream). |
| `turn` | `degrees: float, speed: float = 200.0` | Same rotation model as `rogo turn`. |
| `goto` | `x: int, y: int, speed: int = 200, arrive: int = 0, timeout_ms: int \| None = None` | Same `GO_TO_R` call as `rogo goto`. |
| `config_get` | `name: str \| None = None` | Returns `{fields: {...}}` or `{error: ...}`. |
| `config_set` | `name: str, value: float` | Returns `{acked, seq_id, name, value}` or an `error` key -- never raised as an MCP tool error for a kUnknown/unknown-field outcome (see below). |
| `calibrate_turns` | `measured_degrees: list[float], target_deg: float = 90.0, save: bool = False` | Non-interactive: takes ALREADY-MEASURED trial values (drive each trial separately first, e.g. via the `turn` tool, then measure it externally) -- it does not drive the robot itself, and does not prompt. |

**Transport**: `stdio` by default -- a separate pipe from the resolved
robot/relay/sim connection above, with NO network surface at all.
Passing `--listen HOST:PORT` opts into a TCP transport instead, subject
to a binding rule: a loopback host (`127.0.0.1`/`localhost`/`::1`) is
allowed outright; any other host additionally requires `--allow-remote`
(`error: --listen '...' names a non-loopback host ... pass
--allow-remote ...`, exit 2) -- `rogo mcp` is an external control
surface with no authentication of its own, so a non-loopback bind must
be an explicit, auditable opt-in.

**Error reporting inside tool calls**: exactly the same kUnknown
soft-warning rule as section 4 applies, but reported as a `warning`
key in the tool's own JSON result (never raised as an MCP error) --
only a genuine unreachable-target condition (`TransportClosed`, or an
ack/done that never arrives within the bounded wait) raises and
surfaces as an MCP tool error.

---

## 8. Robot config files

Robot configs live at `config/robots/*.json`; `config/robots/
active_robot.json` is a pointer file (a `{"path": "..."}` shape,
resolved by BASENAME ONLY against `config/robots/`, or a full config
inline). `rogo turn`, `rogo calibrate turns`, and `rogo calibrate
distance` all read `geometry.trackwidth`/`rotational_slip`/
`distance_scale` from the active robot's file (falling back to
`calibration.rotational_slip`/`calibration.distance_scale` if present,
though every file staged today carries these fields under `geometry`
instead); `calibrate turns`/`calibrate distance` write an updated
`rotational_slip`/`distance_scale` back into that same file's `geometry`
group on confirmation, leaving every other field untouched. No active
robot config, or a missing `geometry.trackwidth`, is reported as a clear
`error: ...` (exit 1) rather than a crash or a guessed default.
"""

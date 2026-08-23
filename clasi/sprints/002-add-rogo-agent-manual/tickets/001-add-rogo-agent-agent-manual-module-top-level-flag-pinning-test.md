---
id: '001'
title: 'Add rogo --agent: agent manual module, top-level flag, pinning test'
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: add-rogo-agent-flag-one-page-agent-manual.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add rogo --agent: agent manual module, top-level flag, pinning test

## Description

Port elite's `--agent` convention onto this repo's `rogo`: a top-level
`rogo --agent` flag that prints a single, self-contained Markdown manual
written for AI coding agents (and power users) driving `rogo`
non-interactively, then exits — before any target (`--sim`/`--connect`/
`--port`) is ever resolved. `rogo --help` stays the short per-subcommand
usage; `--agent` is the complete reference: every subcommand shipped in
sprint 001, every option/argument for each, the shared target flags, and
the operational knowledge an agent needs that `--help` text doesn't carry
(units, exit-code semantics, `repl`'s session property, `mcp`'s tool
list). Reference: `radio-robot-elite/src/host/robot_radio/io/agent_manual.py`
and its `io/cli.py` wiring for format/tone and the pinning-test discipline
only — not its v5 content (daemon/`serve`, binary command plane,
macOS-HUPCL serial-port-reset advice all describe things this repo's
`rogo` does not have, per sprint.md's Architecture and the issue's own
Reference section).

Read the actual shipped surface before writing manual prose —
`src/host/rogo/cli.py`'s `build_parser()` (every subparser/sub-subparser
and option, including `drive`'s bare/`--ms`/`--mm`/`stream` modes and
`--resend`, `turn`'s `--speed`, `goto`'s `x`/`y`/`--speed`/`--arrive`/
`--timeout`, `config get`/`set`, `calibrate turns`/`distance`'s
`--speed`/`--trials`/`--distance`, `repl`'s `commands`, `mcp`'s
`--listen`/`--allow-remote`, and the shared `--sim`/`--connect`/`--port`
target flags from `rogo.connection.add_target_arguments()`),
`rogo/mcp_server.py` (the 8 registered tools, `stdio` default, the
loopback-unless-`--allow-remote` binding rule enforced by
`resolve_listen_target()`), `rogo/calibrate.py` (manual/tape-measure
trial flow, `SLIP_SANE_RANGE`/`DISTANCE_SCALE_SANE_RANGE`, the "not
available inside repl" carve-out), `rogo/connection.py` (the three target
kinds), and `rogo/config.py` (`config/robots/*.json` +
`active_robot.json` layout) — so the manual describes what actually
ships, not an idealized or stale version of it.

## Acceptance Criteria

- [ ] `rogo --agent` (alone, no other flags/subcommand) exits 0 and
      prints a non-empty Markdown manual to stdout, resolving no target
      (`--sim`/`--connect`/`--port`) and requiring no built `tools/sim`.
- [ ] The manual documents every subcommand `rogo` currently ships
      (`hello`, `stop`, `drive`, `turn`, `goto`, `config get`/`config
      set`, `calibrate turns`/`calibrate distance`, `repl`, `mcp`) and
      every option/argument for each, including the shared target
      options (`--sim`/`--connect HOST:PORT`/`--port PORT`).
- [ ] The manual gives concrete example invocations for the main
      subcommands, not just a restatement of `--help` text.
- [ ] The manual states expected output shape per command (e.g. what
      `hello` prints, what a `WHEELS_V acked (#N), done reason=...` line
      means).
- [ ] The manual documents exit-code semantics, explicitly including the
      kUnknown soft-warning rule (an acked-but-merits-rejected call —
      e.g. `drive --mm`/`goto` hitting `DiffDriveAdapter`'s planner gap —
      prints a warning and still exits 0) and how `config set`'s
      unknown-field-name error differs (hard error, exit 1).
- [ ] The manual documents units for every numeric argument (mm, mm/s,
      ms, degrees) and calls out which wire fields are integer-typed
      (`goto`'s five numeric fields, `drive`'s wheel speeds) versus
      float-typed (`config set`'s value — the one float wire field).
- [ ] The manual documents `repl`'s one-persistent-session property and
      that `calibrate`/`mcp` are not available as repl lines.
- [ ] The manual documents `mcp`'s `stdio`-by-default transport, lists
      all 8 tools it exposes, and states the `--listen`
      loopback-unless-`--allow-remote` binding rule.
- [ ] The manual documents where robot configs live
      (`config/robots/*.json`, `active_robot.json`) and that
      `calibrate`/`turn` read `trackwidth`/`rotational_slip`/
      `distance_scale` from there.
- [ ] A pinning test introspects `rogo.cli.build_parser()`'s argparse
      tree (every subparser name, every sub-subparser name, every
      registered option string) and asserts each one appears in
      `agent_manual.MANUAL` — not a hand-written checklist of expected
      strings (sprint.md's Design Rationale).
- [ ] `src/host/rogo/README.md` mentions `rogo --agent`.
- [ ] No existing `rogo` subcommand's behavior, options, or exit codes
      change.

## Implementation Plan

**Approach**: Add `src/host/rogo/agent_manual.py` holding a verbatim
`MANUAL` Markdown string (module docstring following the same convention
`rogo`'s other modules use — cross-reference sprint.md rather than
re-deriving rationale inline). Wire a top-level `--agent`
`action="store_true"` flag into `cli.py`'s `build_parser()`, and check it
in `main()` before `parser.parse_args()` dispatches to a subcommand's
`func` — actually before requiring a subcommand at all, since
`build_parser()`'s subparsers are currently `required=True`; `--agent`
must work with no subcommand given, so check `args.agent` (or parse with
subparsers temporarily not required, whichever keeps `build_parser()`
itself the single source of truth other tests rely on) before that
requirement would otherwise reject a bare `rogo --agent` invocation.
Write the pinning test by walking `build_parser()`'s own
`_subparsers`/`_actions` (or `argparse`'s public iteration where
available) to collect: every top-level subcommand name, every
sub-subcommand name (`config get`/`set`, `calibrate turns`/`distance`),
and every registered `option_strings` entry, then assert each is a
substring of `agent_manual.MANUAL`.

**Files to create/modify**:
- `src/host/rogo/agent_manual.py` (new)
- `src/host/rogo/cli.py` (add `--agent` flag + `build_parser()`/`main()`
  wiring)
- `src/host/rogo/README.md` (one line mentioning `--agent`)
- `tests/host/rogo/test_agent_manual.py` (new)

**Documentation updates**: `src/host/rogo/README.md` only (see above) —
the manual itself is the documentation this ticket produces.

## Testing

- **Existing tests to run**: `tests/host/rogo/` (full directory — confirm
  no existing subcommand's parsing/behavior regresses from the
  `build_parser()` change).
- **New tests to write**: `test_agent_manual.py` — (1) the
  `build_parser()`-introspection pinning test asserting every
  subcommand/sub-subcommand/option string appears in `MANUAL`; (2) a
  smoke test that `rogo --agent` (via `cli.main(["--agent"])` or
  equivalent) exits 0 and produces non-empty output with no target
  flag and no `tools/sim` dependency.
- **Verification command**: `uv run pytest tests/host/rogo/`

---
id: '002'
title: Add rogo --agent manual
status: executing
branch: sprint/002-add-rogo-agent-manual
use-cases:
- SUC-001
issues:
- add-rogo-agent-flag-one-page-agent-manual.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 002: Add rogo --agent manual

## Goals

Give `rogo` a single top-level `--agent` flag that prints a self-contained,
one-page Markdown manual — written for AI coding agents and power users
driving the CLI non-interactively — covering the entire command surface
`rogo` shipped in sprint 001 (`hello`, `stop`, `drive`, `turn`, `goto`,
`config get`/`config set`, `calibrate turns`/`calibrate distance`, `repl`,
`mcp`), every option and argument for each, and the shared target options
(`--sim`/`--connect HOST:PORT`/`--port PORT`). `rogo --help` stays the
short usage; `--agent` is the complete reference. Port the convention from
`radio-robot-elite/src/host/robot_radio/io/agent_manual.py` and its
`--agent` wiring in `io/cli.py`, plus its pinning-test discipline
(`test_rogo_agent_manual.py`) so a command/option rename breaks a test
instead of silently staling the manual — but adapt only the *format and
agent-oriented tone*, not the v5 content: this repo's `rogo` has no daemon
(`rogo serve`), no binary command plane, and none of elite's macOS-HUPCL
serial-port-reset advice.

## Problem

`rogo`'s only current documentation is `--help`'s short per-subcommand
usage strings. An agent (or a person) trying to drive the tool
non-interactively — what exit code means what, which units each argument
takes, how `repl`'s one persistent session differs from the one-shot
subcommands, what the MCP server exposes — has to read source. Elite
solved exactly this with a single `--agent` flag printing one page anyone
can paste into a prompt or read top to bottom; this repo's `rogo` has no
equivalent yet.

## Solution

Add a `rogo/agent_manual.py` module holding a verbatim `MANUAL` Markdown
string (mirroring elite's convention), wire a top-level `--agent` flag in
`rogo.cli` that prints it and exits before subcommand dispatch, and add a
pinning unit test asserting every registered subparser command and every
option string appears somewhere in the manual text (so a future rename
fails loudly instead of leaving the manual stale). Content is written
fresh for this repo's actual command surface and exit-code/unit
conventions — it is not a copy of elite's manual text, only of the
`--agent`/pinning-test *pattern*.

## Success Criteria

- `rogo --agent` prints one Markdown page and exits 0, without requiring
  a connected target (no `--sim`/`--connect`/`--port` needed just to read
  the manual).
- The manual documents every subcommand shipped in sprint 001 (`hello`,
  `stop`, `drive`, `turn`, `goto`, `config get`/`set`, `calibrate turns`/
  `distance`, `repl`, `mcp`) with every option/argument, the shared target
  options, units (mm, mm/s, ms, degrees), the kUnknown soft-warning
  exit-code rule, `repl`'s one-persistent-session behavior, the MCP
  server's stdio default and its tool count, and where robot configs live
  (`config/robots/*.json`, `active_robot.json`).
- A pinning test fails if a subcommand or option is added/renamed without
  a matching manual update.

## Scope

### In Scope

- `rogo/agent_manual.py` (new module, verbatim `MANUAL` constant).
- Top-level `--agent` flag wired into `rogo.cli`'s argument parser.
- A pinning unit test (mirroring elite's `test_rogo_agent_manual.py`)
  checking every registered subcommand/option string appears in the
  manual.
- A brief mention of `--agent` in `src/host/rogo/README.md` (added in
  sprint 001's closing ticket), pointing readers at it.

### Out of Scope

- Anything describing elite's `rogo serve` daemon, its socket protocol,
  server-local verbs, `RogoClient`, or macOS HUPCL serial-port-reset
  advice — none of that exists in this repo's `rogo`.
- Any new `rogo` subcommand or option — this sprint only documents the
  surface sprint 001 already shipped.
- Elite's binary command-plane (`raw <arm>`, envelope encoding) content —
  this repo's protocol v6 is a single plain-ASCII grammar with no binary
  plane to document.

## Test Strategy

Unit tests only, no new integration surface: (1) a pinning test for
`rogo/agent_manual.py` that introspects `rogo.cli.build_parser()`'s
argparse tree (subparser names, sub-subparser names, every registered
option string) and asserts each appears in `MANUAL`, so a future
command/option addition or rename fails this test instead of silently
staling the manual; (2) a smoke test that `rogo --agent` exits 0 and
prints non-empty output with no target flag and no built `tools/sim`
required. No `tools/sim` end-to-end run is needed — `--agent` deliberately
never resolves a connection.

## Architecture

**Compact** — adds one new module (`rogo/agent_manual.py`) and a single
top-level `--agent` flag wired into the existing `cli.py` router. No new
cross-subsystem dependency: the new module is a leaf (it imports nothing
from `robot_v6`/`rogo`'s other modules, and is imported only by `cli.py`);
the pinning test introspects `cli.py`'s own `build_parser()` rather than
adding a runtime dependency edge. No dependency-direction change, no
data-model change.

**What Changed**: A new module `src/host/rogo/agent_manual.py` holds a
verbatim Markdown `MANUAL` string documenting the full shipped `rogo`
command surface — `hello`, `stop`, `drive` (bare/`--ms`/`--mm`/`stream`),
`turn`, `goto`, `config get`/`config set`, `calibrate turns`/`calibrate
distance`, `repl`, `mcp` — every option/argument for each, and the shared
target flags (`--sim`/`--connect HOST:PORT`/`--port PORT`). Content is
oriented at an agent driving `rogo` non-interactively: concrete
invocations, units (mm, mm/s, ms, degrees — including the int-vs-float
wire-field split: `goto`'s five numeric fields and `drive`'s speeds are
int-wire per `protocol_handler.cpp`'s `parseInt32`, while `config set`'s
value is the one float-wire field), the kUnknown soft-warning rule
(sprint 001's stakeholder-approved decision: an acked-but-merits-rejected
call like `drive --mm`/`goto` on `DiffDriveAdapter`'s planner gap prints a
`warning:`/`err N` line and still exits 0 — `_print_soft_warning()`),
`config set`'s differing hard-error treatment of a genuinely unknown field
name (exit 1, not a soft warning — `_print_config_set_error()`), `repl`'s
one-persistent-`Session` property and its "`calibrate`/`mcp` not available
inside a repl line" carve-out, and `mcp`'s `stdio`-by-default transport,
its 8 tools (`hello`, `stop`, `drive`, `turn`, `goto`, `config_get`,
`config_set`, `calibrate_turns`), and its `--listen`
loopback-unless-`--allow-remote` binding rule. `cli.py`'s `build_parser()`
gains a top-level `--agent` flag, checked and handled before subcommand
dispatch (`main()`) so it runs and exits 0 with no target flag, no
subcommand, and no connection ever resolved. A pinning unit test derives
its expected command/option strings by introspecting `build_parser()`'s
own argparse tree — not a hand-written checklist — and asserts each
appears in `MANUAL`. `src/host/rogo/README.md` gets one line pointing
readers at `rogo --agent`.

**Why**: `rogo`'s only documentation today is `--help`'s short
per-subcommand usage; an agent driving it non-interactively has to read
source to learn units, exit-code semantics, or the MCP tool surface.
Elite already solved this with its own `--agent` convention
(`agent_manual.py` + `cli.py` wiring + a pinning test); this issue ports
that convention's format/tone and pinning-test discipline, not elite's
v5-specific content (daemon/`serve`, binary command planes, macOS-HUPCL
advice — none of which exists in this repo's `rogo`).

**Impact on Existing Components**: `cli.py` gains one new top-level flag
and one new import (`agent_manual`) — no change to any existing
subcommand's behavior, options, or exit codes. No other module is
touched. Purely additive.

**Design Rationale**: One decision worth recording — deriving the pinning
test's expected strings from `build_parser()`'s own argparse tree at test
time, rather than a hand-maintained checklist (elite's own
`test_rogo_agent_manual.py` pins specific sections by hand). A
hand-maintained list can drift exactly as easily as the manual itself: a
new option added to `cli.py` wouldn't automatically appear in a
hand-written expected-list either, defeating the "a rename breaks a test
instead of silently staling the manual" purpose the issue asks for.
Introspecting `build_parser()` ties the test's expectations to the single
source of truth. Consequence: the test can only assert presence (does
this string appear somewhere in `MANUAL`), not that the surrounding prose
is accurate or complete — that residual gap is inherent to any automated
pinning test and is exactly why the manual stays hand-authored prose
rather than a generated `--help` dump.

**Migration Concerns**: None — no data model, no wire format, no existing
command's behavior changes.

## Use Cases

Compact — one new use case, briefly stated.

### SUC-001: Learn rogo's full command surface non-interactively via `rogo --agent`
Parent: UC-014

- **Actor**: CLI / tooling user, especially an AI coding agent driving
  `rogo` non-interactively.
- **Preconditions**: `rogo` is installed.
- **Main Flow**: User runs `rogo --agent`; the CLI prints the one-page
  Markdown manual (every subcommand/option, units, the kUnknown
  soft-warning exit-code rule, `repl`'s one-session property, `mcp`'s
  stdio default and 8-tool list, and `config/robots/*.json` layout) to
  stdout and exits 0, without resolving `--sim`/`--connect`/`--port` or
  any target at all.
- **Postconditions**: The manual's text is available to read or paste
  into a prompt with no robot, relay, or `tools/sim` required.
- **Acceptance Criteria**:
  - [ ] `rogo --agent` (no other flags) exits 0 and prints non-empty
        output with no target resolved and no built `tools/sim` required.
  - [ ] Every subcommand and every option registered in
        `build_parser()` appears somewhere in the manual text, verified
        by a pinning test that introspects the parser rather than a
        hand-written string list.
  - [ ] `src/host/rogo/README.md` mentions `rogo --agent`.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [x] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add rogo --agent: agent manual module, top-level flag, pinning test | — |

Tickets execute serially in the order listed.

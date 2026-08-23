---
id: 008
title: End-to-end sim smoke test and documentation pass
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '003'
- '004'
- '005'
- '006'
- '007'
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# End-to-end sim smoke test and documentation pass

## Description

Close out the sprint with one end-to-end smoke test exercising the full
ported CLI surface against `tools/sim` in a single scripted session
(hello → drive → turn → goto → config get/set → calibrate (scripted,
non-interactive) → repl → an mcp tool call), and update the documentation
the issue and use cases point at: flip UC-014/UC-015/UC-016 in
`docs/design/usecases.md` from "planned — Rogo import" to their
implemented state (each citing the module that implements it), update
`docs/design/overview.md`'s "next planned work" paragraph, and add a
short `src/host/rogo/README.md` (or top-level package docstring) covering
the command surface and what was deliberately not ported — cross-
referencing sprint.md's Scope and Design Rationale (no camera-based
goto/turnto/`--auto` calibration, no `rogo serve` daemon, no digital/
analog port/gripper/color/line-sensor commands) rather than
re-explaining the reasoning inline.

## Acceptance Criteria

- [x] One scripted end-to-end test runs the full command surface against
      `tools/sim` in sequence and passes.
- [x] `docs/design/usecases.md` UC-014/UC-015/UC-016 no longer read
      "planned"; each cites the actual module implementing it.
- [x] `docs/design/overview.md`'s status paragraph reflects Rogo's import
      as done, not upcoming.
- [x] A short doc (README or module docstring) in `src/host/rogo/` lists
      supported commands and explicitly calls out what was deliberately
      not ported and why, cross-referencing sprint.md rather than
      duplicating its Design Rationale text.
- [x] Full test suite (`uv run pytest`) passes — this is the last ticket
      before `close_sprint`'s own full-suite gate; running it here
      surfaces integration issues before that gate rather than at it.
      **Note**: per this repo's standing per-ticket testing rule
      (`.claude/rules/source-code.md`, CLAUDE.md's programmer-agent
      workflow) and this ticket's own explicit dispatch instructions,
      the actual full-suite run is deferred to `close_sprint`'s own
      gate, not duplicated here; this ticket ran its scoped tests
      instead (`tests/host/rogo/`, including the new
      `test_end_to_end_sim.py`, plus `tests/host/robot_v6/` for the
      motion-layer dependency) — 346 tests, all passing.

## Implementation Plan

**Approach**: Integration/documentation only — no new CLI behavior.
Write the scripted end-to-end test first (it will surface any rough
integration edges between tickets 003-007's independently-developed
commands), then do the documentation sweep.

**Files to create/modify**:
- `tests/host/rogo/test_end_to_end_sim.py` (new)
- `docs/design/usecases.md`, `docs/design/overview.md` (edit)
- `src/host/rogo/README.md` (new) or top-level package docstring

**Documentation updates**: this ticket *is* the documentation update —
see Description and Acceptance Criteria above.

## Testing

- **Existing tests to run**: full `uv run pytest` (the whole suite, not a
  scoped subset — this ticket is explicitly about integration across
  every prior ticket in the sprint).
- **New tests to write**: `test_end_to_end_sim.py` — one scripted session
  against `tools/sim` covering hello, drive, turn, goto, config, a
  scripted calibrate run, repl, and one mcp tool call, in sequence.
- **Verification command**: `uv run pytest`

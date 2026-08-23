---
id: '005'
title: Implement rogo calibrate (manual distance/turns)
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement rogo calibrate (manual distance/turns)

## Description

Port the manual/interactive multi-trial calibration flow from
`radio-robot-elite/src/host/robot_radio/io/calibrate.py`'s
`cmd_calibrate_distance`/`cmd_calibrate_turns` — the non-`--auto` branches
only, per sprint.md's Design Rationale Decision 4 — replacing the
camera-daemon ground truth with the same tape-measure/operator-input
prompts elite already uses in manual mode, and replacing the old
binary-plane motion calls with `robot_v6.motion.wheels_v` and the ported
turn model from ticket 003. Results (an updated `rotational_slip`, and a
distance-calibration equivalent) get written back to the active robot's
`config/robots/<robot>.json` via `rogo.config`, preserving elite's
"reject a value outside a sane range, don't silently persist it"
behavior (`_prompt_save`).

## Acceptance Criteria

- [x] `rogo calibrate turns [--speed] [--trials N]` runs N manual trials
      (prompt → spin via `WHEELS_V` → prompt for the operator's measured
      degrees), computes an updated `rotational_slip`, and prompts to
      save.
- [x] `rogo calibrate distance [--distance] [--speed] [--trials N]` runs
      the equivalent straight-line manual trial sequence.
- [x] On confirmation, the result is written to the active robot's file
      in `config/robots/`; declining leaves the file untouched.
- [x] A computed value outside a defined sane range is rejected with a
      clear message and not persisted (mirrors `motion-api#2.1`'s
      trackwidth-bending caution).
- [x] Tests drive a full calibration run against `tools/sim` with
      scripted (non-interactive) operator input, writing only to a
      fixture config file copy — never the real `config/robots/` files.

## Implementation Plan

**Approach**: New `src/host/rogo/calibrate.py` module owning trial
sequencing, prompts, and residual computation; delegates motion to
`robot_v6.motion` (ticket 001) and the turn model (ticket 003), and
persistence to `rogo.config` (ticket 002). Keep elite's residual/slip
adjustment arithmetic but drop the bivariate-polynomial motor-model and
OTOS-linear-scale pieces, which depend on calibration data this repo has
no equivalent source for. Structure the trial loop so the "run N trials,
collect a measured value per trial, compute a result" core is callable
without a TTY (an explicit values list in, not `input()`) — ticket 007's
MCP tool needs this same core without a terminal prompt.

**Files to create/modify**:
- `src/host/rogo/calibrate.py` (new)
- `src/host/rogo/cli.py` (extend: `calibrate distance`/`calibrate turns`
  subcommands, interactive wrapper around the core loop)
- `tests/host/rogo/test_calibrate.py` (new), plus a fixture copy of one
  `config/robots/*.json` file under `tests/host/rogo/fixtures/`

**Documentation updates**: none required this ticket.

## Testing

- **Existing tests to run**: `tests/host/rogo/`, `tests/host/robot_v6/test_motion.py`.
- **New tests to write**: `test_calibrate.py` — trial-loop core with
  scripted measured values against `tools/sim`, sane-range rejection,
  and config-file writeback against a fixture copy.
- **Verification command**: `uv run pytest tests/host/rogo/`

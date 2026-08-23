---
id: '003'
title: Implement rogo drive/turn/stop/hello commands
status: open
use-cases: [SUC-001]
depends-on: ["001", "002"]
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement rogo drive/turn/stop/hello commands

## Description

Implement `rogo drive` — bare mode, `--ms` (timed), and `stream` mode all
over `WHEELS_V`; `--mm` (distance-bounded) over `WHEELS_X`, surfacing
whatever outcome the connected adapter actually reports rather than
assuming kinematic effect. Implement `rogo turn <degrees>`: port the
rotation/turn model (wheelbase/rotational_slip/linear-correction math)
from `radio-robot-elite/src/host/robot_radio/io/cli.py`'s
`_turn_command`, reading `trackwidth`/`rotational_slip` via
`rogo.config`, computing `(cmd_l, cmd_r, duration_ms)`, then issuing one
`WHEELS_V` call via `robot_v6.motion` — this is the one command with real
kinematic effect on `DiffDriveAdapter` today, per sprint.md's SUC-001.
`rogo stop`/`rogo hello` graduate from ticket 002's smoke-test stubs into
the real subcommand table alongside these.

## Acceptance Criteria

- [ ] `rogo drive <L> <R> --ms <N>` against `tools/sim` issues one
      `WHEELS_V <L> <R> <N>` and reports the ack/completion outcome.
- [ ] `rogo drive <L> <R> stream [--resend MS]` re-issues `WHEELS_V` at
      the configured resend cadence until Ctrl-C, then sends `STOP`
      (matches `reliability.py`'s documented "current reading always
      overrides the previous one" semantics for `WHEELS_V` on
      `DiffDriveAdapter`).
- [ ] `rogo drive <L> <R> --mm <N>` issues `WHEELS_X` and reports whatever
      the connected adapter answers (ack + `kUnknown` on `DiffDriveAdapter`
      today, per UC-002) — not a crash, not a false "success."
- [ ] `rogo turn <degrees> [--speed]` computes `(cmd_l, cmd_r, duration)`
      from the ported rotation model using the active robot's
      `trackwidth`/`rotational_slip`, falling back to a no-slip linear
      estimate when calibration data is absent (matching elite's own
      fallback), and issues one `WHEELS_V`.
- [ ] `rogo stop` and `rogo hello` work via the real `rogo.cli` command
      table (not the ticket-002 stub path).
- [ ] Tests cover the rotation-model math as a pure function (no
      transport needed) and an end-to-end `tools/sim` run for `drive
      --ms`, `drive stream`, and `turn`.

## Implementation Plan

**Approach**: Port `_turn_command`'s math (not its I/O — no camera, no
polynomial motor model this repo has no calibration data for) into
`src/host/rogo/turn_model.py` as a pure function `compute_turn(angle_deg,
speed_mm_s, trackwidth_mm, rotational_slip, gain=1.0, offset_deg=0.0) ->
(cmd_l, cmd_r, duration_ms)`. Wire `drive`/`turn`/`stop`/`hello`
subcommands into `rogo.cli`, each resolving a target via
`rogo.connection.resolve()` then calling `robot_v6.motion`.

**Files to create/modify**:
- `src/host/rogo/turn_model.py` (new)
- `src/host/rogo/cli.py` (extend: real `drive`/`turn`/`stop`/`hello`
  subcommands)
- `tests/host/rogo/test_turn_model.py`, `tests/host/rogo/test_cli_drive_turn.py`
  (new)

**Documentation updates**: none required this ticket.

## Testing

- **Existing tests to run**: `tests/host/rogo/` (ticket 002's connection/
  config tests), `tests/host/robot_v6/test_motion.py` (ticket 001).
- **New tests to write**: `test_turn_model.py` (pure-function unit tests,
  including the no-calibration-data fallback path); `test_cli_drive_turn.py`
  (end-to-end against `tools/sim` for `drive --ms`, `drive --mm`, `drive
  stream`, `turn`).
- **Verification command**: `uv run pytest tests/host/rogo/ tests/host/robot_v6/`

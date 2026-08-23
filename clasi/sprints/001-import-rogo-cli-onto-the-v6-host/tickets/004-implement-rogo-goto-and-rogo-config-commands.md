---
id: '004'
title: Implement rogo goto and rogo config commands
status: open
use-cases: [SUC-002]
depends-on: ["001", "002"]
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement rogo goto and rogo config commands

## Description

Implement `rogo goto <x> <y> [--speed] [--arrive] [--timeout]`, mapping
onto `robot_v6.motion.go_to_r` (robot-frame). Per sprint.md's Design
Rationale Decision 3, the aprilcam-camera closed loop from elite's
`goto`/`turnto` does not port — this repo has no camera integration —
and `go_to_w`'s world-frame variant stays unavailable until a pose source
exists (`specification.md#13`). Implement `rogo config get [name]` /
`rogo config set <name> <value>`, delegating to the `get`/`set` wire
wrappers `motion.py` added in ticket 001.

## Acceptance Criteria

- [ ] `rogo goto <x> <y>` sends a well-formed `GO_TO_R` line and prints
      the adapter's actual reply, including today's documented
      `kUnknown` outcome on `DiffDriveAdapter` (per UC-003) — never a
      false "arrived" message.
- [ ] `rogo config set <name> <value>` then `rogo config get <name>`
      round-trips against `tools/sim`/`DiffDriveAdapter`.
- [ ] `rogo config get` with no name lists every field the adapter
      reports (bare `GET #<id>` per `protocol#6`).
- [ ] An unknown config name surfaces the wire's `err 1` (`ERR_UNKNOWN`)
      as a clear CLI message, not a stack trace or silent no-op.
- [ ] Tests cover `goto`'s wire encoding and `config get`/`set`
      round-trip against `tools/sim`.

## Implementation Plan

**Approach**: Extend `rogo.cli`'s subcommand table with `goto` and
`config` (with `get`/`set` sub-subcommands). Both delegate entirely to
`robot_v6.motion` + `rogo.connection`; `rogo.cli` itself stays a thin
dispatcher, consistent with the architecture's cohesion goal for that
module (it routes, it does not implement).

**Files to create/modify**:
- `src/host/rogo/cli.py` (extend: `goto`, `config get`/`config set`
  subcommands)
- `tests/host/rogo/test_cli_goto_config.py` (new)

**Documentation updates**: none required this ticket.

## Testing

- **Existing tests to run**: `tests/host/rogo/`, `tests/host/robot_v6/test_motion.py`.
- **New tests to write**: `test_cli_goto_config.py` — `goto` wire
  encoding and reply surfacing; `config get`/`set` round-trip and
  unknown-name error path, all against `tools/sim`.
- **Verification command**: `uv run pytest tests/host/rogo/ tests/host/robot_v6/`

---
id: '001'
title: Add motion-API convenience layer to robot_v6
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add motion-API convenience layer to robot_v6

## Description

Add `src/host/robot_v6/motion.py`, a thin convenience layer over
`reliability.Session` implementing the six motion-api operations
(`wheels_x`, `wheels_v`, `move_x`, `move_v`, `go_to_r`, `go_to_w`) plus
`stop`/`estop` and `get`/`set` (protocol#7 config delegation), translating
units per `motion-api.md` (degrees → integer milliradians on the wire per
`motion-api#9.1`; mm/mm-s pass straight through). This is the foundational
module every other ticket in this sprint calls into — see sprint.md's
Architecture Step 3 and Design Rationale Decision 1 for why it lives in
`robot_v6` (a generic host-side primitive any future caller can reuse),
not inside the new `rogo` package.

## Acceptance Criteria

- [x] `robot_v6.motion` exposes `wheels_v(session, left, right,
      duration_ms)`, `wheels_x(...)`, `move_x(session, distance_mm,
      rotation_deg, cruise_mm_s, timeout_ms)`, `move_v(...)`,
      `go_to_r(session, x_mm, y_mm, speed_mm_s, arrive_mm, timeout_ms)`,
      `go_to_w(...)`.
- [x] `stop(session, immediate=False)` and `estop(session)` are exposed;
      `estop` uses `Session.send_unsequenced` per `protocol#8.3`'s
      unsequenced-verb rule (estop has no id).
- [x] `get(session, name=None)` (bare `GET` when `name` is omitted) and
      `set(session, name, value)` wire wrappers are exposed for
      `protocol#7`'s `GET`/`SET` delegation.
- [x] Degree-valued arguments are converted to integer milliradians on the
      wire per `motion-api#9.1`; mm/mm-s arguments pass through
      unconverted.
- [x] Each sequenced function returns the assigned sequence id (from
      `Session.send`) so a caller can `wait_for_ack`/`wait_for_done`.
- [x] Unit tests cover unit conversion and verb/field encoding for every
      function against the existing fake-transport pattern in
      `tests/host/robot_v6/`.
- [x] `WHEELS_V` calls produce real kinematic effect end to end against
      `tools/sim`; the other five motion verbs are confirmed to
      decode/dispatch (ack) even where `tools/sim`'s
      `FakeMotionAdapter`/`DiffDriveAdapter` answers `kUnknown` — this
      ticket needs correct wire encoding, not new planner behavior.

## Implementation Plan

**Approach**: Add `motion.py` as a fourth module in `src/host/robot_v6/`,
built purely on `reliability.Session`'s existing `send`/
`send_unsequenced`/`wait_for_ack`/`wait_for_done` — no changes to
`codec.py`, `transport.py`, or `reliability.py`. Mirror `motion-api.md`'s
six-operation table for argument names/units field-for-field.

**Files to create/modify**:
- `src/host/robot_v6/motion.py` (new)
- `tests/host/robot_v6/test_motion.py` (new)

**Documentation updates**: none required this ticket — `docs/design/`
updates land in ticket 008 once the full CLI surface exists.

## Testing

- **Existing tests to run**: `tests/host/robot_v6/` (full directory —
  confirm no regression to `codec`/`transport`/`reliability`).
- **New tests to write**: `tests/host/robot_v6/test_motion.py` — unit
  conversion and encoding for all eight `motion.py` functions using the
  existing fake-transport fixture; one end-to-end case against
  `tools/sim` for `wheels_v`.
- **Verification command**: `uv run pytest tests/host/robot_v6/`

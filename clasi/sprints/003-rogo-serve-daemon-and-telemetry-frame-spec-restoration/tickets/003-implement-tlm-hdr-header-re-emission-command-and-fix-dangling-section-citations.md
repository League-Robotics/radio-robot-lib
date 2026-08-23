---
id: '003'
title: Implement TLM HDR header re-emission command and fix dangling section citations
status: open
use-cases: [SUC-005]
depends-on: ["002"]
github-issue: ''
issue: restore-the-telemetry-frame-specification-and-add-a-host-requested-header-command.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement TLM HDR header re-emission command and fix dangling section citations

## Description

The protocol has never had a way to recover a lost `thdr` header:
`ProtocolHandler` only re-emits `thdr` when the column set changes
(`headerChanged()` in `protocol_handler.cpp:1071`), so a host that
misses the header (dropped frame, mid-stream reconnect) has no recovery
path today. Add `TLM HDR #<id>` — sequenced like every other `TLM`
form, reusing the existing mode-token slot (sprint.md's Architecture
Design Rationale) — which forces `thdr` to re-emit before the next `t`
frame WITHOUT changing the current subscription mode.

**Implementation is contained entirely within `src/protocol/`**, per
the issue's own "no other behavior change to `src/protocol/`"
constraint:
- `src/protocol/adapter.h` — add `TlmMode::kHdr` to the enum (alongside
  `kOff`/`kPose`/`kFull`/`kNow`/`kAuto`/`kBuffer`).
- `src/protocol/protocol_handler.cpp` — `parseTlmMode()`'s mode table
  gains a `{"HDR", TlmMode::kHdr}` entry; `execTlm()` special-cases
  `kHdr` by clearing the handler's own remembered-header state
  (`everEmittedHeader_ = false`, the same field `headerChanged()`
  already checks) directly, and does **not** call
  `adapter_.onTlm(mode)` for this case — `kHdr` is a header-recovery
  request, not a subscription change, and the handler already owns the
  state that needs clearing (`headerCount_`/`headerNames_`/
  `headerHex_`/`everEmittedHeader_`, `protocol_handler.h:434-437`), so
  there is no reason for the Adapter to see it at all. This is what
  keeps `src/adapter/diffdrive_adapter.cpp` untouched: `DiffDriveAdapter
  ::onTlm()` never receives `kHdr`, so its existing `if (mode !=
  TlmMode::kNow) mode_ = mode;` logic (which would otherwise wrongly
  persist `kHdr` as the current mode, since it only excludes `kNow`)
  never runs against it.

Also fix the three dangling `§6.x` code citations, now that ticket 002
has landed §10:
- `src/protocol/adapter.h:134` — `spec §6.2` → `docs/design/protocol.md
  §10`
- `src/protocol/protocol_handler.cpp:1057` — `spec §6.5` → `§10`
- `src/archive/protocol-v6/wire_v6_telemetry.h:11` — `spec §6.3 POSE /
  §6.4 FULL` → the corresponding `§10` subsection(s)

## Acceptance Criteria

- [ ] `TLM HDR #<id>` is accepted as a well-formed `TLM` command
      (decodes like any other mode token).
- [ ] Sending `TLM HDR` after the header has been "lost" (simulate by
      resetting/not tracking the remembered header on the test's host
      side) causes the next `emitTelemetry()` call to emit `thdr` before
      the next `t` frame.
- [ ] The current subscription mode (`OFF`/`POSE`/`FULL`/`AUTO`/
      `BUFFER`) is unchanged after `TLM HDR` — verified by checking mode
      state before and after.
- [ ] `src/adapter/diffdrive_adapter.cpp` has zero diff — confirms the
      "no other behavior change to `src/protocol/`" constraint (checked
      literally: `onTlm()`'s existing logic is untouched).
- [ ] All three dangling `§6.x` citations listed above resolve to the
      real §10 section (or subsection) landed by ticket 002.
- [ ] Existing golden-vector and adversarial `tests/protocol/` suites
      pass unmodified except for the one new `TLM HDR` vector this
      ticket adds.

## Implementation Plan

**Approach**: Additive-only change to `TlmMode`'s enum, the mode-token
parse table, and `execTlm()`'s dispatch — no changes to
`ProtocolHandler`'s public interface, `Adapter`'s interface, or any
concrete adapter.

**Files to modify**:
- `src/protocol/adapter.h` — new `TlmMode::kHdr` enumerator.
- `src/protocol/protocol_handler.cpp` — `parseTlmMode()` table entry;
  `execTlm()` branch; citation fix at line 1057.
- `src/archive/protocol-v6/wire_v6_telemetry.h` — citation fix at
  line 11 (comment-only; this file is auto-generated from
  `scripts/wire_v6_tables.py`, so if the generator itself embeds the
  citation, update the generator's source comment too, not just the
  generated header).

**Testing plan**: New test in `tests/protocol/` (golden-vector style,
matching the existing convention): subscribe, receive a `thdr`+`t`
pair, simulate losing the header, send `TLM HDR #<id>`, assert the very
next `emitTelemetry()` call emits `thdr` before `t` and that
`STATUS`'s `tlm=` field is unchanged from before the `TLM HDR` call.
Scoped run: `uv run python -m pytest -q tests/protocol/` (or the
project's C++ test runner if `tests/protocol/` is compiled rather than
pytest-driven — confirm against the existing suite's own invocation
convention before writing this ticket's test).

**Documentation updates**: None beyond the two citation fixes above —
ticket 002 already wrote the §10 content this implementation matches.

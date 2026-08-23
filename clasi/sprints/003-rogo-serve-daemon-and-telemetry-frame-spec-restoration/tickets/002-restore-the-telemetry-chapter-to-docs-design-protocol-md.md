---
id: '002'
title: Restore the Telemetry chapter to docs/design/protocol.md
status: open
use-cases: [SUC-005]
depends-on: []
github-issue: ''
issue: restore-the-telemetry-frame-specification-and-add-a-host-requested-header-command.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Restore the Telemetry chapter to docs/design/protocol.md

## Description

Commit `34d12c2` folded `docs/protocol-v6-spec.md` into
`docs/design/protocol.md` but dropped the old spec's entire §6
Telemetry chapter. `protocol.md` currently references `thdr`/`t`
frames, six `TLM` modes, and a "telemetry cadence" throughout (line 33,
§5.2, the verb table, §8.5) with no section anywhere defining the frame
grammar, mode semantics, or column layout. This ticket restores that
content as a new §10 (sprint.md's Architecture Design Rationale explains
the numbering choice: appending avoids renumbering every existing
cross-reference in an already heavily cross-referenced document — do
NOT renumber existing §6-§9). This ticket is documentation-only; the
`TLM HDR` command itself is ticket 003.

Recovery source for the dropped chapter:
`git show 34d12c2^:docs/protocol-v6-spec.md` (§6, roughly lines
359-480). `src/archive/protocol-v6/wire_v6_telemetry.h` carries the
archived full-robot column tables in code form, for the "how the
full-robot tables relate" pointer.

**Not a verbatim copy**: the old §6.3/§6.4 tables describe a full robot
(OTOS pose, line sensors, colour); this library's own projection is the
reduced set §5.2 already summarizes. Write the new §10 against what
`DiffDriveAdapter` actually publishes, not the archived full-robot
tables.

**Do not repeat the old, incorrect claim** that `TLM NOW` recovers a
lost header (§5.2 of this ticket's own sprint.md Problem section
explains why: the implementation only ever re-emits `thdr` on a
column-set change, never on `NOW`). The restored text should describe
`TLM HDR` (ticket 003) as the recovery mechanism instead, even though
this ticket lands before that command exists in code — write it as the
specified behavior ticket 003 will implement, and reference "ticket 003"
or "the next section" rather than asserting it already works.

## Acceptance Criteria

- [ ] New §10 "Telemetry" section covers: `TLM <mode>` subscription
      semantics (the six-mode table, per-connection mode, re-emit
      `thdr` on mode change, rate floor); the `thdr`/`t` line grammar
      and emission rules (header on first frame and on column-set
      change, values in header order, decimal integers, `flags` as
      lowercase hex with no `0x`, matching `emitHeader`/`emitFrame` in
      `protocol_handler.cpp:1029-1069`); this library's actual column
      sets (`posl`/`posr` `[mm]`, `vell`/`velr` `[mm/s ×10]`, `FULL`
      adding `lambda`/`biasl`/`biasr`/`cyc`, `flags`' local bit layout
      from `computeFlags()`); a pointer to the recoverable full-robot
      POSE/FULL column tables for ports that grow beyond DiffDrive; and
      the `TLM HDR` command's wire form and semantics (forward-written
      against ticket 003's planned implementation).
- [ ] §5.2 gains a forward-pointer to §10 ("see §10 for the full
      telemetry-frame specification").
- [ ] No restored text repeats the old "`TLM NOW` recovers the header"
      claim.
- [ ] Section renders correctly (valid Markdown, table formatting
      consistent with the rest of `protocol.md`).

## Implementation Plan

**Approach**: Recover the old chapter's content via `git show
34d12c2^:docs/protocol-v6-spec.md`, rewrite it against this library's
actual projection and implementation (per §5.2's existing summary and
`protocol_handler.cpp`'s real `emitHeader`/`emitFrame`/`headerChanged`
behavior), and append as new §10.

**Files to modify**:
- `docs/design/protocol.md` — new §10 Telemetry section; forward-pointer
  added to existing §5.2.

**Testing plan**: Documentation-only ticket, no automated test. Verify
by review: check off each Acceptance Criteria item above; grep
`docs/design/protocol.md` for `§6.2`, `§6.3`, `§6.4`, `§6.5` to confirm
no stray old-numbering references remain inside the new section itself
(the three dangling *code* citations are ticket 003's own AC, since
fixing them depends on this ticket's final section number existing
first).

**Documentation updates**: This ticket IS the documentation update.

---
id: '012'
title: Fix four remaining dangling spec section citations in src/protocol
status: done
use-cases:
- SUC-005
depends-on:
- '002'
- '003'
github-issue: ''
issue: fix-four-remaining-dangling-spec-section-citations-in-src-protocol.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix four remaining dangling spec section citations in src/protocol

## Description

Ticket 003 fixed three explicitly-named dangling `spec §6.x` code
comments left behind when the old telemetry spec chapter was folded into
[docs/design/protocol.md](../../docs/design/protocol.md) and then restored
as **§10** (rather than reclaiming the old §6 numbers) in ticket 002.

A `grep -n "§6\." src/protocol/` run during that ticket surfaced four MORE
citations of exactly the same kind. They were out of ticket 003's named
scope, so they were left untouched. They are still dangling:

- `src/protocol/adapter.h:55` — "TLM subscription modes (spec §6.1)" on the
  `TlmMode` enum doc comment. §6.1 in the CURRENT `protocol.md` is
  **"Outcomes"** (`Result`/`err` codes), not TLM modes — that content is now
  **§10.1 `TLM <mode>` — telemetry is a subscription**. This citation is
  actively *wrong* now, not merely imprecise: a reader who follows it lands
  on the error-code section.
- `src/protocol/adapter.h:121` — "already fully scaled per spec §6.3/§6.4"
  on the `Column` struct doc comment. Points at **§10.3 This library's
  column sets**, which is where the `×10`/`×1000` scaling convention this
  sentence is about is actually stated (verified: `protocol.md:1845`).
- `src/protocol/adapter.h:123` — "`hex` selects spec §6.5's one exception"
  on the same `Column` comment. Points at **§10.2's Value encoding
  paragraph** (`protocol.md:1810`) — matching the wording commit `27f24f1`
  already used for the equivalent citation at `protocol_handler.cpp`'s
  `emitFrame()`: `// flags: lowercase hex, no "0x" prefix
  (docs/design/protocol.md §10.2's Value encoding paragraph).`
- `src/protocol/protocol_handler.h:213` — "changes (spec §6.2); t: every
  call" on the `emitTelemetry()` doc comment. Points at **§10.2 `thdr` /
  `t` — the frame is self-describing**.

## Explicitly out of scope — verified correct, leave alone

- `src/protocol/adapter.h:40` — "wire codes per spec §6.1" on the `Result`
  enum. Current §6.1 *is* "Outcomes" (`Result`/`err` codes). Correct —
  byte-identical, do not touch.
- `src/protocol/README.md:55` — "protocol.md §6.3 for …". Current §6.3 *is*
  `RUN` — invocation by name, unrelated to telemetry. Correct —
  byte-identical, do not touch.

Also out of scope: the `§6.x` citations under `src/adapter/` and
`src/archive/protocol-v6/`. The archived-spec ones are deliberate references
to the *archived* numbering (`protocol.md` §10.3 itself writes "the
archived spec's §6.5 bit numbering"), and `src/adapter/` was not audited by
this issue. Do not touch either.

## Acceptance Criteria

- [x] `src/protocol/adapter.h:55` now cites `docs/design/protocol.md §10.1`
      (`TLM <mode>` — telemetry is a subscription) instead of `spec §6.1`.
- [x] `src/protocol/adapter.h:121` now cites `docs/design/protocol.md
      §10.3` (This library's column sets) instead of `spec §6.3/§6.4`.
- [x] `src/protocol/adapter.h:123` now cites `docs/design/protocol.md
      §10.2`'s Value encoding paragraph instead of `spec §6.5`, worded to
      match commit `27f24f1`'s equivalent fix in
      `protocol_handler.cpp`'s `emitFrame()`.
- [x] `src/protocol/protocol_handler.h:213` now cites `docs/design/
      protocol.md §10.2` (`thdr` / `t` — the frame is self-describing)
      instead of `spec §6.2`.
- [x] `src/protocol/adapter.h:40` (Result enum, `spec §6.1`) is
      byte-identical to before this ticket.
- [x] `src/protocol/README.md:55` (`§6.3`) is byte-identical to before
      this ticket.
- [x] No file under `src/adapter/` or `src/archive/protocol-v6/` is
      touched.
- [x] Comment-only change. Zero behavior change — no source line other
      than a comment differs; no new tests; no golden-vector changes.
- [x] Existing `tests/protocol/` suite passes unmodified (scoped run).

## Implementation Plan

**Approach**: Four single-line comment edits, no code changes. Update
each of the four citations named above to name `docs/design/protocol.md`
and its real current section number, matching the wording style commit
`27f24f1` already established for the first three fixes (ticket 003).
Leave the two verified-correct citations and everything under
`src/adapter/`/`src/archive/protocol-v6/` untouched.

**Files to modify**:
- `src/protocol/adapter.h` — lines 55, 121, 123 (three comment edits;
  line 40 stays untouched).
- `src/protocol/protocol_handler.h` — line 213 (one comment edit).

**Testing plan**: No new test — this is a comment-only change with no
behavioral surface. Run the existing `tests/protocol/` suite (scoped,
per this project's testing rule: full suite runs once at `close_sprint`)
to confirm zero regressions: `uv run python -m pytest -q tests/protocol/`
(or the project's C++ test runner if `tests/protocol/` is compiled rather
than pytest-driven — confirm against the existing suite's own invocation
convention before running).

**Documentation updates**: None — `docs/design/protocol.md` itself is
unchanged; only code comments that cite it are corrected.

---
status: in-progress
sprint: '003'
tickets:
- 003-012
---

# Fix four remaining dangling "spec §6.x" citations in src/protocol/

## Description

Ticket 003-003 fixed three explicitly-named dangling `spec §6.x` code
comments left behind when the old telemetry spec chapter was folded into
[docs/design/protocol.md](../../docs/design/protocol.md) and then restored
as **§10** (rather than reclaiming the old §6 numbers) in ticket 003-002.

A `grep -n "§6\." src/protocol/` run during that ticket surfaced four MORE
citations of exactly the same kind. They were out of 003-003's named scope,
so they were left untouched. They are still dangling:

- `src/protocol/adapter.h:55` — "TLM subscription modes (spec §6.1)" on the
  `TlmMode` enum doc comment. §6.1 in the CURRENT protocol.md is
  **"Outcomes"** (`Result`/`err` codes), not TLM modes — that content is now
  **§10.1 `TLM <mode>` — telemetry is a subscription**. This citation is
  actively *wrong* now, not merely imprecise: a reader who follows it lands
  on the error-code section.
- `src/protocol/adapter.h:121` — "already fully scaled per spec §6.3/§6.4"
  on the `Column` struct doc comment. Should point at **§10.3 This library's
  column sets**, which is where the `×10`/`×1000` scaling convention this
  sentence is about is actually stated.
- `src/protocol/adapter.h:123` — "`hex` selects spec §6.5's one exception"
  on the same `Column` comment. Should point at **§10.2's Value encoding
  paragraph** — matching the citation fix 003-003 already applied to
  `emitFrame()` in `protocol_handler.cpp`.
- `src/protocol/protocol_handler.h:213` — "changes (spec §6.2); t: every
  call" on the `emitTelemetry()` doc comment. Should point at **§10.2
  `thdr` / `t` — the frame is self-describing**.

## Explicitly out of scope — verified correct, leave alone

- `src/protocol/adapter.h:40` — "wire codes per spec §6.1" on the `Result`
  enum. Current §6.1 *is* "Outcomes" (`Result`/`err` codes). Correct.
- `src/protocol/README.md:55` — "protocol.md §6.3 for …". Current §6.3 *is*
  `RUN` — invocation by name, unrelated to telemetry. Correct.

Also out of scope: the `§6.x` citations under `src/adapter/` and
`src/archive/protocol-v6/`. The archived-spec ones are deliberate references
to the *archived* numbering (§10.3 itself says "the archived spec's §6.5 bit
numbering"), and `src/adapter/` was not audited by this issue.

## Acceptance

- The four citations above name `docs/design/protocol.md` and its actual
  current section numbers, in the wording style commit `27f24f1` established
  for the first three fixes.
- The two verified-correct citations are untouched.
- Comment-only change. **No behavior change, no new tests.** Existing
  protocol tests still pass.

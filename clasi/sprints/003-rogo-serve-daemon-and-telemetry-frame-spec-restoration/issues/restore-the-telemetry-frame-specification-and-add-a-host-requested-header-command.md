---
status: in-progress
sprint: '003'
tickets:
- 003-002
- 003-003
---

# Restore the telemetry frame specification, and add a host-requested header command

## Description

The v6 protocol spec references telemetry frames everywhere but never defines them. In [docs/design/protocol.md](../../docs/design/protocol.md):

- Line 33 lists `thdr` / `t` among the robot→host reply verbs
- §5.2 describes the column *projection* (what data goes in) and asserts "`thdr` is emitted once on subscribe and names the columns; `t` carries the values in that order"
- The verb table's `TLM` row names six modes (`OFF`/`POSE`/`FULL`/`NOW`/`AUTO`/`BUFFER`) with no definition of what any of them do
- §8.5 piggybacks the entire reliability layer's periodic emission on "the telemetry cadence"

…but no section anywhere specifies the frame itself: no `thdr`/`t` line grammar, no emission rules, no mode semantics, no column tables, no value encoding.

Several code comments still cite the missing sections — the references are dangling:

- `src/protocol/adapter.h:134` — "spec §6.2: whenever the subscription changes…"
- `src/protocol/protocol_handler.cpp:1057` — "flags: lowercase hex, no 0x prefix (spec §6.5)"
- `src/archive/protocol-v6/wire_v6_telemetry.h:11` — "spec §6.3 POSE / §6.4 FULL"

`src/protocol/` is designated an archetype to be ported to MicroPython and JavaScript by reading it against the docs, so a missing frame spec directly breaks the port workflow.

There is also a real protocol hole the restored spec must address, not just transcribe: the frame is self-describing only if the host actually has the header. If the `thdr` line is lost (dropped frame on the radio, host reconnects mid-stream), every `t` frame after it is gibberish to the host — values with no column names. The host *knows* it's gibberish (it has no remembered header, or the field count doesn't match the header it has), but the protocol gives it no way to ask for a fresh one.

## Cause

Commit `34d12c2` ("docs: consolidate protocol specs into design/protocol.md, delete dead docs", 2026-08-21) folded `docs/protocol-v6-spec.md` into `docs/design/protocol.md` but dropped the deleted spec's entire **§6 Telemetry** chapter:

- §6.1 `TLM <mode>` — telemetry is a subscription (mode table, per-connection mode, re-emit `thdr` on mode change, rate floor)
- §6.2 `thdr` / `t` — the self-describing frame grammar
- §6.3 POSE columns (9), §6.4 FULL columns (35), §6.5 `flags` bit layout

The lost-header hole predates the deletion: old §6.2 claimed a host that missed the header "sends `TLM NOW`", but the implementation only emits `thdr` when the column set *changes* (`headerChanged()` in `src/protocol/protocol_handler.cpp:1071`), so `TLM NOW` never actually recovered a lost header.

## Proposed fix

**1. Restore a telemetry-frame section to `docs/design/protocol.md`.** Not a verbatim paste — the old §6.3/§6.4 tables describe the full robot (OTOS pose, line sensors, colour), while this library's projection is the reduced set current §5.2 already documents. The restored section should specify:

- **`TLM <mode>` subscription semantics** — the mode table including `NOW`/`AUTO`/`BUFFER` behavior; mode change re-emits `thdr` before the next `t`
- **`thdr`/`t` line grammar and emission rules** — header on first frame and on column-set change; values in header order; decimal integers; `flags` as lowercase hex without `0x` (must match the implementation in `emitHeader`/`emitFrame`, `src/protocol/protocol_handler.cpp:1029-1069`)
- **This library's actual column sets** — `posl`/`posr` `[mm]`, `vell`/`velr` `[mm/s ×10]`; `FULL` adds `lambda`/`biasl`/`biasr`/`cyc`; `flags` is the local bit layout from `computeFlags()` in `diffdrive_adapter.cpp`
- **How the full-robot POSE/FULL tables relate** — a pointer to the recoverable full-robot column tables for ports that grow beyond DiffDrive
- The restored text must **not** repeat the old "send `TLM NOW` to recover the header" claim (see Cause)

**2. Add a host-requested header re-emission command.**

- Something like `TLM HDR #id` — sequenced like the other `TLM` forms; forces `thdr` before the next `t`; does **not** change the current mode
- Implementation is likely small: a new mode token that sets a "header due" flag (e.g. clear the remembered-header state so `headerChanged()` fires on the next `emitTelemetry()`)
- Fix the dangling `§6.x` code citations to resolve to real sections (either the restored numbering or updated comment references)

Recovery source for the dropped chapter: `git show 34d12c2^:docs/protocol-v6-spec.md` (§6, roughly lines 359–480). `src/archive/protocol-v6/wire_v6_telemetry.h` carries the archived column tables in code form.

## Verification

- `docs/design/protocol.md` has a telemetry-frame section covering the four spec points above, plus the header-request command
- The dangling `§6.x` code citations resolve to real sections
- The header-request command is implemented in `src/protocol/` (handler + adapter surface as needed) with a test: lose/forget the header, request it, verify `thdr` re-emits before the next `t` and the mode is unchanged
- No other behavior change to `src/protocol/`

## Related

- Commit `34d12c2` — the consolidation that dropped the chapter
- [docs/design/protocol.md](../../docs/design/protocol.md) §5.2, §8.5, verb table
- `src/protocol/protocol_handler.cpp` (`emitHeader`/`emitFrame`/`headerChanged`), `src/protocol/adapter.h`, `src/archive/protocol-v6/wire_v6_telemetry.h`

---
id: '006'
title: Implement rogo repl
status: done
use-cases:
- SUC-004
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement rogo repl

## Description

Implement `rogo repl`, running commands over one persistent `Session`:
from an argument list (`rogo repl "drive 100 100 --ms 200" stop`), piped
stdin, or an interactive prompt. Since protocol v6 is a single plain-ASCII
grammar (no COBS/CRC/protobuf translation needed, unlike elite's
binary-plane `RogoSession`/repl machinery), this module is a much smaller
command loop reusing the same `rogo.cli` per-subcommand argument parsers
already built by tickets 003/004, not a reimplementation of elite's
binary-envelope translator.

## Acceptance Criteria

- [x] `rogo repl "drive 100 100 --ms 200" stop` against `tools/sim` runs
      both commands over one connection and exits 0.
- [x] Piped stdin (`cat script.rogo | rogo repl`) and the bare
      interactive prompt (`rogo repl`) both dispatch through the same
      command parser as the argument-list mode — no separate grammar to
      maintain.
- [x] The connection closes cleanly on EOF, Ctrl-C, or an explicit
      `quit`/`exit` command.
- [x] Tests cover all three input modes against `tools/sim`.

## Implementation Plan

**Approach**: `src/host/rogo/repl.py` reuses `rogo.cli`'s existing
per-subcommand argument parsers (built once, dispatched per line) rather
than re-implementing argument parsing or inventing a second grammar;
holds one `Session` open for the whole repl lifetime and drains
replies/telemetry between commands.

**Files to create/modify**:
- `src/host/rogo/repl.py` (new)
- `src/host/rogo/cli.py` (wire the `repl` subcommand)
- `tests/host/rogo/test_repl.py` (new)

**Documentation updates**: none required this ticket.

## Testing

- **Existing tests to run**: `tests/host/rogo/` (tickets 002/003/004's
  connection/config/command tests).
- **New tests to write**: `test_repl.py` — argument-list, piped-stdin,
  and interactive-prompt modes, each against `tools/sim`; clean-shutdown
  path on EOF/`quit`.
- **Verification command**: `uv run pytest tests/host/rogo/`

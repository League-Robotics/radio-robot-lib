---
id: '002'
title: 'Scaffold rogo package: connection resolution, config loader, packaging entry
  point'
status: done
use-cases: []
depends-on:
- '001'
github-issue: ''
issue: import-rogo-cli-adapt-robot-radio-to-v6-host.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Scaffold rogo package: connection resolution, config loader, packaging entry point

## Description

Create the new `src/host/rogo/` package skeleton: `__init__.py`,
`connection.py` (resolve a CLI invocation's target — a real serial port, a
relay socket, or a `tools/sim` subprocess — into a live
`robot_v6.transport.Transport` + `robot_v6.reliability.Session` pair),
`config.py` (load/persist the minimal robot config subset —
`geometry.trackwidth`, `calibration.rotational_slip`, identity — from
`config/robots/*.json` and `active_robot.json`, tolerant of whichever
fields are actually present per sprint.md's Migration Concerns and Design
Rationale Decision 2), and a `cli.py` stub wiring `argparse` with
`hello`/`stop` as the first two working subcommands — the simplest
possible smoke test of the whole stack end to end. This is infrastructure
with no use case of its own; it enables SUC-001 through SUC-005.

Also add the packaging this repo currently lacks: a minimal
`[build-system]` table and a `[project.scripts] rogo = "rogo.cli:main"`
entry in `pyproject.toml`, verified not to break the existing
`pythonpath = ["src/host"]` pytest-only import path other tests rely on
(sprint.md's Migration Concerns).

## Acceptance Criteria

- [x] `rogo.connection.resolve(args)` returns a working `(Transport,
      Session)` pair for at least: a `--sim` flag that spawns `tools/sim
      --stdio` via `StdioTransport`, and a `--connect HOST:PORT` flag via
      `SocketTransport`.
- [x] `rogo.config.load_active_robot()` reads `active_robot.json` and the
      pointed-to file from `config/robots/`, returning `None`/sane
      defaults for any field absent from a given robot's JSON — never a
      crash.
- [x] `rogo hello` and `rogo stop` work end to end against `tools/sim`
      (build it per `tools/sim/README.md` if not already built).
- [x] Installing the package (`pip install -e .` or equivalent) produces a
      working `rogo` console script.
- [x] `uv run pytest` still passes unmodified after the packaging change —
      the `pythonpath` import path for existing `robot_v6`/protocol/etc.
      tests is unaffected.
- [x] `rogo --help` lists at least `hello` and `stop`.

## Implementation Plan

**Approach**: New package under `src/host/rogo/`, following
`robot_v6`'s existing module-per-responsibility pattern.
`connection.py` depends only on `robot_v6.transport`/`reliability` — no
`robot_v6.motion` dependency needed yet at this stage (that arrives with
ticket 003's `drive`/`turn` commands). For packaging, add a minimal
`hatchling`-or-equivalent `[build-system]` and a `[project.scripts]`
entry; confirm `tool.pytest.ini_options.pythonpath` keeps working
unchanged so this doesn't disturb the rest of the test suite's import
story.

**Files to create/modify**:
- `src/host/rogo/__init__.py`, `connection.py`, `config.py`, `cli.py`
  (new)
- `pyproject.toml` (add `[build-system]`, `[project.scripts]`)
- `tests/host/rogo/test_connection.py`, `tests/host/rogo/test_config.py`
  (new)

**Documentation updates**: none required this ticket.

## Testing

- **Existing tests to run**: full `uv run pytest` — this ticket changes
  `pyproject.toml`, which every test's collection depends on; confirm
  nothing regresses.
- **New tests to write**: `tests/host/rogo/test_connection.py` (target
  resolution for `--sim`/`--connect`), `tests/host/rogo/test_config.py`
  (load/persist against fixture JSON, missing-field tolerance).
- **Verification command**: `uv run pytest`

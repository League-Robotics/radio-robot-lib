---
status: in-progress
sprint: '001'
tickets:
- 001-001
- 001-002
- 001-003
- 001-004
- 001-005
- 001-006
- 001-007
- 001-008
---

# Import the Rogo CLI and adapt robot_radio to the v6 host

## Description

Bring the Rogo command-line program over from radio-robot-elite and adapt it
to this repo's v6 host, rather than vendoring it wholesale.

Rogo is the CLI entry point defined in `radio-robot-elite/pyproject.toml`:

```toml
rogo = "robot_radio.io.cli:main"
```

It provides relay-aware robot control: drive, turn, goto, config, REPL,
calibrate, sim, and an MCP server.

## Source

- Package: `radio-robot-elite/src/host/robot_radio` — 108 Python files,
  ~3.8 MB.
- Subpackages: `calibration`, `config`, `controllers`, `field`, `io`,
  `kinematics`, `media`, `nav`, `path`, `pathplan`, `planner`, `robot`,
  `sensors`, `testgui`.
- The CLI itself lives in `robot_radio/io/` alongside `client.py`,
  `serial_conn.py`, `repl.py`, `robot_mcp.py`, `server.py`, `wire_codec.py`,
  `wire_commands.py`, and sim modules.

## Approach

Adapt Rogo onto radio-robot-lib's existing v6 host (`src/host/robot_v6`) and
protocol v6, keeping the CLI surface (command names and behavior) rather than
copying the elite wire/protocol layers wholesale. The elite package predates
the v6 protocol work in this repo; its transport and wire layers likely need
replacement with the v6 equivalents while the higher layers (kinematics, nav,
path planning, calibration) may port more directly.

## Notes

- A `rogo-revival` worktree exists at
  `radio-robot-elite/.claude/worktrees/rogo-revival` and may hold relevant
  prior work worth reviewing before starting.
- The robot configuration files Rogo consumes (per-robot JSON configs,
  `robot_config.schema.json`, `active_robot.json`, `devices.json`) were
  already collected into `config/robots/` in this repo — see
  `config/MANIFEST.md`.

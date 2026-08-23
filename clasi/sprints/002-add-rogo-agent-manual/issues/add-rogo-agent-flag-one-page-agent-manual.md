---
status: in-progress
sprint: '002'
tickets:
- 002-001
---

# Add rogo --agent: one-page agent-oriented manual for the full CLI

## Description

Port the `--agent` convention from radio-robot-elite's rogo: a top-level
`rogo --agent` flag that prints a single, self-contained Markdown manual
written for AI coding agents (and power users) driving the tool
non-interactively. `rogo --help` stays the short usage; `--agent` is the
complete reference — every command, every option, all on one page.

## Stakeholder requirements

- Top-level `--agent` flag (elite precedent: prints a verbatim `MANUAL`
  constant from a dedicated module; same convention as `mbdeploy --agent`).
- One page documents the ENTIRE command surface: `hello`, `stop`, `drive`
  (bare/`--ms`/`--mm`/`stream`), `turn`, `goto`, `config get`/`config set`,
  `calibrate turns`/`calibrate distance`, `repl`, `mcp` — with every option
  and argument for each, including the shared target options
  (`--sim`/`--connect HOST:PORT`/`--port PORT`).
- Oriented around helping agents figure out how to use the program:
  concrete invocations, what output to expect, exit-code semantics
  (including the kUnknown soft-warning rule: warning text, exit 0),
  int-typed wire fields, units (mm, mm/s, ms, degrees), the one-persistent-
  session property of `repl`, the MCP server's stdio default and 8 tools,
  and where robot configs live (`config/robots/*.json`, `active_robot.json`).

## Reference

- Elite source: `radio-robot-elite/src/host/robot_radio/io/agent_manual.py`
  (214 lines) and its `--agent` wiring in `io/cli.py` — adapt the format and
  agent-oriented tone, NOT the v5 content (daemon/serve, binary planes, and
  HUPCL-driven advice do not apply to this repo's rogo).
- Elite pins load-bearing manual sections with a unit test
  (`test_rogo_agent_manual.py`) so renames break a test instead of silently
  staling the manual — do the same here (e.g. assert every registered
  subparser command and every option string appears in the manual).

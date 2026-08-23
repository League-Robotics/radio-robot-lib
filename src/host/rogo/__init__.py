"""rogo -- the command-line control surface for a protocol-v6 robot,
relay, or `tools/sim`, adapted from `radio-robot-elite`'s
`robot_radio.io.cli` onto this repo's own `robot_v6` host stack (see
clasi/sprints/001-import-rogo-cli-onto-the-v6-host/sprint.md).

This package owns CLI concerns only (argument parsing, target
resolution, config load/persist, command dispatch); wire-level
concerns (codec, transport, reliability, motion-api unit conversion)
live in `robot_v6`, one layer down, and have no knowledge `rogo` exists
(sprint.md's Design Rationale Decision 1).
"""

from __future__ import annotations

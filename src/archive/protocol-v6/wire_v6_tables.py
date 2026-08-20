"""wire_v6_tables.py -- hand-written declaration source for two of protocol
v6's three flat generator tables (sprint 137 ticket 001): the verb table
(spec §3) and the telemetry column tables (spec §6.3/§6.4, POSE/FULL).
Read by scripts/gen_messages.py, which emits ONE generated C++ table and
ONE generated Python table per table declared here -- sprint 137's
sprint.md Design Rationale Decision 2 ("one declaration, two generated
outputs"), the same property that makes the v5-era `pid.kff -> kaff`
drift class structurally impossible: there is no second hand-maintained
copy of a verb or column name to drift from.

The THIRD table -- the 80-row config field table (spec §7.3) -- is
deliberately NOT declared here. `src/protos/robot_config.proto`'s 9
wire-addressable groups already ARE that declaration (the same one
`Config::Robot`'s own C++ members and the host pydantic model are
generated from, sprint 132 "configuration discipline") and its field
names/bounds are carried into the v6 spec verbatim -- see that file's own
header comment. Declaring the config fields a second time here would
itself be the two-copies-can-drift shape this whole design exists to
avoid; gen_messages.py's `_v6_config_field_rows()` walks robot_config.proto
directly instead. See that function's own doc comment.

This is NOT a `.proto` file: a verb or telemetry-column table is a flat
name list with no message/field-number/nested-oneof shape a protobuf
schema exists to describe, so routing it through protoc would add
generator machinery for no benefit (sprint.md Design Rationale Decision 2:
"a much smaller [generator]... a lint-sized flat-table emitter, not a
nine-FieldKind protobuf-wire-format walker"). Plain Python literals are
the "lighter declarative format" sprint.md's own Step 7 open question left
as an option, and gen_messages.py is already a Python program, so no new
parser is needed to read this file.

Edit this file to add/remove/reorder a v6 verb or telemetry column, then
regenerate (`python3 src/scripts/gen_messages.py`). Never hand-edit a
generated output (wire_v6_verbs.h/wire_v6_telemetry.h/
wire_v6_tables_generated.py) directly -- see each file's own
AUTO-GENERATED banner.
"""

from __future__ import annotations

from typing import NamedTuple


class V6VerbRow(NamedTuple):
    verb: str        # exact wire text, case-sensitive (spec §2.1) -- UPPERCASE
                      # commands, lowercase replies
    is_command: bool  # True: host -> robot; False: robot -> host reply
    fields: str       # spec's own Fields column, verbatim; "-" if none
    section: str      # spec section reference this row's semantics live in,
                       # e.g. "5.1" -- traceability only, not consumed by the
                       # wire itself


# Commands (host -> robot), spec table §3.1, one row per verb, in the
# spec's own declared order.
V6_COMMANDS: tuple[V6VerbRow, ...] = (
    V6VerbRow("HELLO", True, "-", "4"),
    V6VerbRow("PING", True, "-", "4"),
    V6VerbRow("ID", True, "-", "4"),
    V6VerbRow("VER", True, "-", "4"),
    V6VerbRow("STATUS", True, "-", "4"),
    V6VerbRow("HELP", True, "-", "4"),
    V6VerbRow("GET", True, "[name]", "7"),
    V6VerbRow("SET", True, "name:value[:id]", "7"),
    V6VerbRow("TLM", True, "mode", "6"),
    V6VerbRow("MOVE", True, "kind:a:b:c:stop:limit:timeout:id", "5.1"),
    V6VerbRow("WHEELS", True, "left:right:duration[:id]", "5.2"),
    V6VerbRow("GOTO", True, "x:y:frame:speed:arrive:timeout:id", "5.3"),
    V6VerbRow("STOP", True, "id", "5.4"),
    V6VerbRow("ESTOP", True, "-", "5.4"),
    V6VerbRow("SEED", True, "x:y:h[:id]", "5.5"),
    V6VerbRow("CAL", True, "[samples][:id]", "5.6"),
)

# Replies (robot -> host), spec table §3.2, same order as the spec's table.
V6_REPLIES: tuple[V6VerbRow, ...] = (
    V6VerbRow("device", False, "NEZHA2:robot:<name>:<serial>", "4"),
    V6VerbRow("ready", False, "-", "4"),
    V6VerbRow("pong", False, "<now>", "4"),
    V6VerbRow("id", False, "<drivetrain>:<profile>:<version>", "4"),
    V6VerbRow("ver", False, "<version>", "4"),
    V6VerbRow("status", False, "k=v:k=v:...", "4"),
    V6VerbRow("help", False, "<verb> <verb> ...", "4"),
    V6VerbRow("get", False, "name:value", "7"),
    V6VerbRow("ok", False, "id", "8"),
    V6VerbRow("err", False, "id:code", "8"),
    V6VerbRow("done", False, "id:reason", "8"),
    V6VerbRow("thdr", False, "col:col:...", "6.2"),
    V6VerbRow("t", False, "val:val:...", "6.2"),
    V6VerbRow("dbg", False, "<free text>", "4"),
)

# 30 verbs total (16 commands + 14 replies), spec §3 -- commands first, then
# replies, matching the spec's own §3.1-then-§3.2 table order.
V6_VERBS: tuple[V6VerbRow, ...] = V6_COMMANDS + V6_REPLIES


# Telemetry columns, spec §6.3 (POSE, 9 columns), in the spec's own order.
POSE_COLUMNS: tuple[str, ...] = (
    "seq", "now", "flags", "x", "y", "h", "ox", "oy", "oh",
)

# FULL columns, spec §6.4 -- POSE's 9, then the columns below, in the
# order the spec's own table rows list them.
#
# NOTE -- spec discrepancy, flagged not silently resolved (sprint 137
# ticket 001 completion notes): the §6.4 section header reads "FULL
# columns (30)", but the section's own itemized column table lists 35
# distinct names (POSE's 9 + 26 more, counted below) once every
# multi-column table row (e.g. "`elp` `elv` `ela` `ele`") is expanded to
# its individual column names. This declaration follows the itemized
# list -- ticket 001's own acceptance criterion is matching "the spec's
# own name lists", and the per-row enumeration is the actual, precise name
# list; the section header's parenthetical is a summary count that does
# not match its own body. Not corrected in the spec document itself
# (out of this ticket's scope -- clasi/sprints/137-.../issues/
# protocol-v6-spec.md is a proposal document, not yet promoted to
# docs/protocol-v6.md); raised to the stakeholder/team-lead instead.
_FULL_ADDITIONAL_COLUMNS: tuple[str, ...] = (
    "mode",
    "elp", "elv", "ela", "ele",
    "erp", "erv", "era", "ere",
    "ovx", "ovy",
    "ow",
    "oa",
    "tvx", "tvy",
    "tw",
    "l1", "l2", "l3", "l4",
    "cr", "cg", "cb", "cc",
    "cyb", "cyp",
)

FULL_COLUMNS: tuple[str, ...] = POSE_COLUMNS + _FULL_ADDITIONAL_COLUMNS

---
status: pending
---

# `HELLO` banner: emit the specified colon announcement format

## Description

The `HELLO` banner and the relay's device announcement are **the same
five fields in the same order**, spelled two different ways. The robot
side is the one that diverged, and the divergence silently breaks device
discovery.

    specified   DEVICE:<role>:<common_name>:<device_name>:<serial>
                DEVICE:RADIOBRIDGE:relay:getez:1779042496   (relay, conforming)

    emitted     device NEZHA2 robot <name> <serial>
                device NEZHA2 robot vevov 1198504156        (robot, diverged)

Stakeholder direction (2026-08-27): **put the colon back — it is part of
a separately specified protocol.** The announcement format is owned by
`microbit-radio-relay/docs/announce.md`, not by this library's v6 line
grammar, so `protocol.md` §9.6's colon-to-space migration never had
jurisdiction over it.

## Evidence

All three boards answer `HELLO` immediately over USB (nothing arrives
passively first — `probe_type` holds DTR/RTS low so the board never
resets and the boot banner is never emitted):

    /dev/cu.usbmodem2121202  <  device NEZHA2 robot gopiv 2175407711
    /dev/cu.usbmodem2121102  <  device NEZHA2 robot tovez 2314287040
    /dev/cu.usbmodem2121402  <  device NEZHA2 robot vevov 1198504156

`probe_type` (`devices.py:150-158`) accepts only the colon form:

    if text.startswith("DEVICE:"):
        parts = text.split(":")
        if len(parts) >= 5:

The emitted line fails on both counts — lowercase sentinel,
space-delimited — so `probe_type` returns `None`, `probe_all` takes the
"preserve existing fields" branch (`devices.py:322`), and with nothing to
preserve, `role`/`common_name`/`device_name`/`serial` are never written.
`config/devices.json` confirms it: every robot entry carries only `uid`,
`enum`, `port`, `board_name`, `device_id`.

The DEVICE NAME column still populates only because `board_name` comes
from the separate SWD path (`read_device_id` → `friendly_name`), which
has nothing to do with the announcement.

## Decision: fix the emitter, not the parser

The alternative on the table was widening `probe_type` to accept both
forms. Rejected: two spellings of one format is the defect, and teaching
the parser both makes it permanent. One canonical announcement, specified
first and elsewhere, is the fix.

Bonus once conforming: the announced `<serial>` is the decimal
`FICR.DEVICEID[1]` already in the registry (`gopiv: 2175407711`), so it
cross-checks against `device_id` for free.

## Scope

1. **`docs/design/protocol.md`** — DONE (2026-08-27, docs are not gated).
   New §2.4 records the format, its foreign ownership, the §9.6
   exemption, and the §2.1 flag below. §6's `HELLO` row and
   `sendBanner()`'s declaration comment updated.
2. **`docs/design/usecases.md`** — DONE. UC-009 step 2.
3. **`src/protocol/protocol_handler.cpp:992`** — `sendBanner()`'s
   `snprintf` format string. Needs a ticket; team-lead is blocked from
   `src/`.
4. **`src/protocol/protocol_handler.h:194`** — declaration comment.
5. **`src/host/robot_v6/codec.py:45`** — the comment listing unsequenced
   replies cites `device ...`; check whether anything downstream actually
   parses the banner and would break on the new shape.
6. **Tests** — any fixture asserting the space form.

## FLAGGED — the uppercase sentinel interacts with §2.1

`protocol.md` §2.1 makes case load-bearing: replies are lowercase so that
**a robot's own output can never parse as a command**, which is what keeps
a shared radio channel safe from self-sustaining floods. A lowercase first
token is dropped silently and explicitly does **not** count malformed.

`DEVICE:...` is uppercase. On a shared channel every other robot tokenises
it as one unknown token with no trailing `#id`, fails id resolution, and
takes §8.4's item 1/2 path. **It does not flood** — that path emits no
reply — but it increments `malformedCount()` on every listening robot, so
a diagnostic counter starts counting ordinary neighbour traffic.

**A lowercase sentinel (`device:NEZHA2:robot:vevov:1198504156`) would keep
the colon delimiter, all five fields, and the single shared parse, while
preserving §2.1 exactly.** Its cost is one extra accepted spelling in
`probe_type` — which that code needs regardless, since relays will keep
announcing `DEVICE:`.

The uppercase form is what is specified and what the docs now say.
Recorded here so the choice is visible rather than inherited; the
stakeholder can overrule in one line before the code change lands.

## Cross-repo

`microbit-radio-relay/docs/announce.md` owns this format. If the
lowercase-sentinel variant is chosen instead, that document — not this
one — is where the change belongs, and the relay's own emitter would
follow.

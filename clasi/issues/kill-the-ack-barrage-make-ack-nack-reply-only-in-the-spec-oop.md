---
status: pending
---

# Kill the ack barrage: make `ack`/`nack` reply-only

## Description

Connecting to Tovez through the radio bridge (`nc torture 8760`) shows
`< ack 0 0 none` arriving several times per second with no command ever
sent. The stakeholder's requirement (2026-08-26, verbatim intent): **an
`ack` or `nack` is only a response to an inbound message — never a beacon
or periodic emission.** An idle connection must be silent apart from
telemetry the app explicitly drives.

## Cause

Not an implementation drift — the spec mandates the barrage.
`docs/design/protocol.md` §8.5 ("Periodic emission — piggybacked on
telemetry") requires `emitTelemetry()` to append an `ack`/`nack`
reliability line to every telemetry frame; at RAW250 cadence that is ~4
unsolicited acks per second, forever. Earlier sprint work deleted the
3×-reply-repeat (§8.0), a different mechanism — the periodic piggyback
was never removed.

## Proposed fix

Spec first (this issue's immediate scope, done OOP), code as follow-up.

1. **`docs/design/protocol.md`** — rewrite §8.5 as a dated reversal:
   `ack`/`nack` is emitted only in direct response to an inbound
   sequenced line (§8.1's three-row table and §8.9's decode-failure
   path); `emitTelemetry()` emits `thdr`/`t` frames only. Loss recovery
   is host-driven: a lost `ack` heals via the host's resend (§8.1 middle
   row), a lost `nack` heals because every subsequent inbound line
   re-triggers `nack <expectedNext_>`; a quiet host that wants
   confirmation polls with any sequenced verb (e.g. `STATUS`, which also
   carries `next=`, §8.7). Sweep every cross-reference that promises the
   periodic line: header changelog, §4.1's `(§8.1/§8.5)`, §8.1's state
   list (delete `gapOutstanding_` — its only reader was `emitTelemetry()`),
   §8.9's "re-nacking at the telemetry rate", §9.8 item 5 (mark
   superseded), the "telemetry-piggybacked line" mention in the §9 STOP
   ordering item, and §10.1's `(§8.0/§8.5)` ref.
2. **`docs/design/specification.md`** — replace the "Piggybacked on the
   existing telemetry cadence (`protocol#8.5`)" bullet with the
   reply-only rule. The adjacent bullet's use of "piggyback" for
   `lastDone`/`reason` riding on ack/nack lines remains true; keep it.
3. **Follow-up (code, separate change):** remove the piggyback from
   `src/protocol/protocol_handler.{h,cpp}` (comment at
   `protocol_handler.h:214` documents it, state at `:425`) and adjust the
   host mirror `src/host/robot_v6/reliability.py`, with tests.

## Verification

- `grep -rniE 'piggyback|telemetry rate|8\.5|gapOutstanding' docs/design/`
  — no surviving text promises an unsolicited or periodic `ack`/`nack`.
- Re-read rewritten §8.5, §8.1, §8.9 end-to-end: nothing may claim the
  robot pushes reliability state on a cadence.
- After the code follow-up: an idle `nc torture 8760` session shows no
  `ack`/`nack` lines at all until a command is sent.

## Status (2026-08-26)

Done: spec rewritten (`a943a3a`), this repo's handler + host client
fixed (`513c185`, 716 tests pass), and the SHIPPING firmware repo
`pxt-nezha-diffdrive` fixed the same way (`4a628a0`, 752 tests pass —
its `WireHandler::emitReliability()` was the actual barrage source on
the wire). New hex built and flashed to the board UID-registered as
**tovez** (radio channel 3); verified live over the `torture:8760`
relay: idle link silent, `PING` → `pong`, `STATUS #1` → exactly one
ack.

Remaining: the robot answering on **radio channel 4** self-identifies
as **vevov** (old firmware 1.0.10) and still barrages ~15 acks/sec —
this is the robot the original capture was listening to (`!C 4`). It
is not USB-reachable from this machine. When it is plugged in:
`uv run python tools/make_deploy.py --robot <name> --flash` in
pxt-nezha-diffdrive. The nezha-upy MicroPython port carries the same
piggyback (its own `core/protocol.py` `_emit_reliability_line()`) and
needs the same deletion — tracked separately.

## Related

- `docs/design/protocol.md` §8.0/§8.1/§8.5/§8.9, `docs/design/specification.md` §4
- `src/protocol/protocol_handler.h`, `src/protocol/protocol_handler.cpp`,
  `src/host/robot_v6/reliability.py` (code follow-up)

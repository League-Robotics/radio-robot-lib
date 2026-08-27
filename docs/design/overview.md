# radio-robot-lib — Overview

## What it is

`radio-robot-lib` is a small, hardware-agnostic C++/Python library pair
that lets a classroom differential-drive robot be commanded over an
ASCII radio/serial link, and lets every layer of that link be tested on
a laptop with no robot attached. It is the shared foundation for
robots built on micro:bit/NEZHA-class controllers (the `tovez`/`gopiv`
class of fleet hardware referenced throughout the design docs), and it
is deliberately not an application: nothing in it talks to a specific
board, and nothing in it is shipped by itself — a MicroPython firmware
image and a JavaScript/MakeCode package each embed it separately.

Two things live here, kept deliberately independent so each can be
tested in isolation:

- **DiffDrive** — a self-contained differential-drive wheel control
  kernel: the control law, four small hardware ports (`Motor`, `Clock`,
  `Sleeper`, `FiberLauncher`), a lease watchdog on every motion command,
  and an estop latch. It speaks encoder counts, never millimetres — no
  chassis geometry lives inside it.
- **Protocol** — a protocol-v6 line-grammar handler: one ASCII wire
  format, parsed and dispatched by a `ProtocolHandler` class to an
  `Adapter` class a caller implements. The handler owns every wire
  byte; the adapter never parses or writes one.

They meet in exactly one place: `DiffDriveAdapter`, which turns a
decoded `WHEELS_V` command into `DifferentialDrive::drive(...)`. That
seam is the whole point of the repository, and it is what the test
suite is built to exercise end to end.

A third piece, one layer up, makes the host side of the same protocol
testable without hardware: **`robot_v6`** is a Python protocol-v6 HOST
client — a wire codec, a `Transport` abstraction (socket, pipe, serial),
and the caller-side half of a reliability layer (sequencing,
pipelining, cumulative ack/nack, resend-on-nack). **`tools/sim`** is a
compiled host binary that links the real `ProtocolHandler` against a
fake motion adapter and speaks the identical wire grammar with no
serial port at all.

## Goals

- **One control law, one place.** This repository is the authoritative
  source for the differential-drive control law; prior copies elsewhere
  are deprecated.
- **Testable in isolation, testable end to end.** DiffDrive, Protocol,
  the Adapter seam, and the Python host client each have independent
  test harnesses, plus a no-hardware acceptance path through
  `tools/sim`.
- **Survive a lossy radio link by design.** The wire protocol assumes
  real, substantial packet loss and answers it with a mandatory,
  strictly-incrementing sequence id and cumulative ack/nack — not an
  afterthought bolted onto an already-shipped grammar. (The "~5%" figure
  this bullet carried through 2026-08-26 is withdrawn as unsupported;
  measured per-line delivery on ch4 ranges 66.5-83.3% against a 99.5%
  wired control, and is unstable — `protocol#8.0`.)
- **A minimal, single-sourced wire format.** One ASCII line grammar, no
  binary framing, no CRC, case used structurally to separate commands
  from replies so a robot's own output can never be mistaken for a
  command on a shared radio channel.
- **Narrow scope, explicit boundaries.** Neither library stores
  configuration; geometry and config storage are the adapter's problem,
  not the kernel's or the handler's.

## Main components

| component | role |
|---|---|
| `src/diffdrive/` | the wheel control kernel (ported byte-identical from its origin, held to a fidelity gate) |
| `src/protocol/` | the protocol-v6 `ProtocolHandler` + `Adapter` interface |
| `src/adapter/` | `DiffDriveAdapter`, the concrete link between Protocol and DiffDrive |
| `src/host/robot_v6/` | the Python v6 host client — codec, `Transport`, reliability `Session` |
| `tools/sim/` | a compiled, no-hardware host binary speaking real protocol-v6 |
| `docs/design/` | the canonical specs: `protocol.md`, `motion-api.md`, `diffdrive.md`, `wifi-link.md` |
| `src/archive/` | read-only provenance copies from `radio-robot-elite`; nothing here compiles into the library |

## Status and near-term direction

The core is built and tested: DiffDrive against a golden-reference
fidelity harness, Protocol against golden wire vectors plus adversarial
fuzzing, the Adapter seam end to end through bytes, and `robot_v6`
against both a lossy-transport unit scenario and the real `tools/sim`
binary. Protocol v6 currently implements one motion verb with real
kinematic effect (`WHEELS_V`); the other five verbs of the six-verb
motion surface (`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`) decode
and dispatch correctly but have no planner behind them yet. The
wifi-link design (dual-plane TCP-REPL + UDP-protocol over one AT
module) is specified and bench-proven in a sibling firmware repo but
not yet implemented against this library's own transport. The Rogo CLI
(the `rogo` console command, `src/host/rogo/`) has been imported from
`radio-robot-elite` and adapted onto this repo's own v6 host: `drive`,
`turn`, `goto`, `config`, `calibrate`, `repl`, and `mcp` all run against
a robot, relay, or `tools/sim` through `robot_v6`'s `Transport`/
`Session`, realizing UC-014/UC-015/UC-016 (`docs/design/usecases.md`)
and closing out
`clasi/issues/import-rogo-cli-adapt-robot-radio-to-v6-host.md`. No
further body of work is currently tracked beyond that sprint's own Open
Questions — a `rogo serve` relay daemon, a camera-based `--auto`
calibration mode, and `go_to_w`'s world-frame pose source all remain
deliberately deferred, not scheduled.

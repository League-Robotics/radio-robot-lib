# DiffDrive — a self-contained differential-drive wheel kernel

One class, two files, no dependencies beyond the C++ standard library:

    differential_drive.h    the four ports + the kernel class
    differential_drive.cpp  the control law

`DiffDrive::DifferentialDrive` owns two wheels end to end: the staged
duty write schedule, split-phase encoder sampling, the (velocity, twist)
control law (feedforward + Stage A wheel correction + fast PID +
Stage C bias adaptation + stall/deficit latches), a lease watchdog on
every motion command, and an estop latch. Commands are body velocity +
twist in native encoder counts; `output()` returns a seq-consistent
snapshot. No millimetres, no chassis geometry, no sensors other than the
wheel encoders — those belong to the application above it.

## Ports

Implement four small interfaces (declared at the top of the header):

| port | methods | job |
|---|---|---|
| `Motor` | 13 | one wheel: stage duty, tick (execute + collect), counts out, emergency stop |
| `Clock` | 1 | monotonic microseconds |
| `Sleeper` | 2 | settle/pace sleep + cooperative yield |
| `FiberLauncher` | 1 | start the kernel loop on its own thread — **optional**: call `step()` from your own loop instead and implement this to fail loudly |

These are the package's OWN types. The parent firmware connects them to
its HAL with one-line forwarding adapters; a MakeCode/PXT package or a
MicroPython C module implements them directly against CODAL / the MP
port. No inheritance relationship with any firmware header exists, on
purpose.

## Testing

`src/tests/diffdrive/` holds the package gate:

- **standalone build** — the package compiles with an include path of
  exactly its own directory. A firmware include creeping in fails the
  suite immediately.
- **control-law fidelity** — the package is held, duty-for-duty, to the
  frozen pre-kernel control law it descends from (feedforward exact;
  closed loop equal at steady state).

## Provenance

Extracted 2026-08-18 from `src/firm/control/differential_drive.{h,cpp}`
on branch `explore/differential-drive-kernel` (namespace and includes
changed; the law itself byte-identical). Until the firmware consumes
this package directly, fixes must land in BOTH copies — the fidelity
suite is what keeps them honest.

Design history and the 21-lesson hard-won-behaviour ledger:
`clasi/issues/differentialdrive-one-class-one-fiber-exploratory-worktree.md`.

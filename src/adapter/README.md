# Adapter — the DiffDrive/Protocol seam

One class, two files:

    diffdrive_adapter.{h,cpp}   Protocol::DiffDriveAdapter

`Protocol::DiffDriveAdapter` is the concrete `Protocol::Adapter`
(`src/protocol/adapter.h`) that drives a `DiffDrive::DifferentialDrive`
(`src/diffdrive/differential_drive.h`) — the seam this whole repo exists
to build. Unlike `src/diffdrive/` and `src/protocol/`, this package is
NOT meant to be portable/standalone: its whole job is to depend on both
of the others and hold the one thing neither of them is allowed to know
— a millimetre.

## What it does

- **`WHEELS_V <left> <right> <duration> #<id>`** (renamed from `WHEELS`,
  2026-08-22) → scales `[mm/s]` by `countsPerLength` `[counts/mm]` to
  `[counts/s]`, splits into `velocity`/`twist` (half-sum / half-difference,
  `twist` CCW-positive), and calls `DifferentialDrive::drive()`. Enforces
  the wire's 5000 ms ceiling itself (`ERR_RANGE` above it) — the handler
  holds no bounds table, so this is the adapter's job.
- **`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`** → `kUnknown`,
  always. This adapter has no planner — the wire decodes and dispatches
  these five verbs correctly, but `DiffDriveAdapter` itself cannot give
  them real effect. `kUnknown`, not `kUnimplemented`, matching the same
  posture `RUN`'s own empty registration table already takes (see below).
- **`STOP [now]`** → `neutral()`, immediate regardless of the `now`
  flag (there is no ramp to choose between), no queue (there is none).
  **`ESTOP`** → `estop()`, latched, never acked. Neither ramps — see
  `docs/design/protocol.md` §5.1 for why they are not "smooth vs instant"
  and diffdrive.md §3.2 for the kernel-level detail.
- **`lastDone()`/`lastDoneReason()`** → always `0`/`kNone`. This adapter
  has no queue and no completion event of its own (`WHEELS_V` has no stop
  condition), so the reliability layer's completion channel
  (`docs/design/protocol.md` §8.8) is wire-correct but permanently inert
  here — see `tests/protocol/fake_motion_adapter.h` for an adapter that
  makes it genuinely live.
- **`GET`/`SET`** → 15 `wheel_control.*` fields, mapped 1:1 onto
  `DifferentialDrive::Config`'s plain-float members. Pure delegation: no
  field table lives here beyond a name→member-offset map, and no value
  is cached — every read/write goes straight through
  `DifferentialDrive::config()`/`setConfig()`.
- **`TLM`** → projects `DifferentialDrive::output()` into the wire's
  `Column`/`Snapshot` shape. This is a REDUCED projection, not a literal
  world-frame pose — see `diffdrive_adapter.h`'s file header for exactly
  what differs and why (this library has no odometry, no OTOS, no
  line/colour sensors, so a literal `x`/`y`/`h` world pose is not
  producible here).
- **`RUN`** → `ERR_UNKNOWN`, always. This adapter registers no callable
  functions at all — see `docs/design/protocol.md` §6.3: the
  registration table IS the security allowlist, and this one is
  deliberately empty, not an unfinished stub.

## What is NOT wire-reachable

`maxDuty`, `fullDutyVelocity`, and `cyclePeriod` are `Config` fields but
are not exposed through `GET`/`SET` here — they are the kernel's
authority/calibration/cadence, hard-coded on this class per stakeholder
decision (2026-08-20, see `docs/design/protocol.md` §7) and armed once
at construction, the same way
`tests/diffdrive/diffdrive_shim.cpp`'s `ddConfigureBasic()` already arms
them out-of-band for the diffdrive-only harness.

`countsPerLength` itself is a constructor argument (with a
`setCountsPerLength()` escape hatch), never a `GET`/`SET` field — it is
robot geometry, not a control-law gain, and per docs/design/protocol.md
§7 this repo stores no configuration anywhere.

## Testing

`tests/adapter/` holds the combined acceptance harness — the only place
all three packages (`diffdrive`, `protocol`, `adapter`) meet, driven
from Python via `ctypes`, no CMake, no robot. See that directory's own
test module docstring for the full list; the two tests that must exist
by name are the twist-sign test (fails if the two wheels are swapped)
and the lease-expiry test (measured at the fake motor, not the kernel's
own flag).

## Provenance

New code (2026-08-20). Nothing here is extracted from an existing
implementation — `src/diffdrive/` and `src/protocol/` are both prior
work this file links together for the first time.

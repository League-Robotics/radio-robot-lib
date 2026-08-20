# Adapter — the DiffDrive/Protocol seam

One class, two files:

    diffdrive_adapter.{h,cpp}   Protocol::DiffDriveAdapter

`Protocol::DiffDriveAdapter` is the concrete `Protocol::Adapter`
(`src/protocol/adapter.h`) that drives a `DiffDrive::DifferentialDrive`
(`src/diffdrive/differential_drive.h`) — the class docs/plan.md Step 4
exists to build. Unlike `src/diffdrive/` and `src/protocol/`, this
package is NOT meant to be portable/standalone: its whole job is to
depend on both of the others and hold the one thing neither of them is
allowed to know — a millimetre.

## What it does

- **`WHEELS:<left>:<right>:<duration>[:<id>]`** → scales `[mm/s]` by
  `countsPerLength` `[counts/mm]` to `[counts/s]`, splits into
  `velocity`/`twist` (half-sum / half-difference, `twist` CCW-positive),
  and calls `DifferentialDrive::drive()`. Enforces the wire's 5000 ms
  ceiling itself (`ERR_RANGE` above it) — the handler holds no bounds
  table, so this is the adapter's job (see `protocol_handler.h`'s own
  ambiguity note #3).
- **`STOP`** → `neutral()`. **`ESTOP`** → `estop()`, never acked.
- **`GET`/`SET`** → the 15 `wheel_control.*` fields spec §7.3 names,
  mapped 1:1 onto `DifferentialDrive::Config`'s plain-float members.
  Pure delegation: no field table lives here beyond a name→member-offset
  map, and no value is cached — every read/write goes straight through
  `DifferentialDrive::config()`/`setConfig()`.
- **`TLM`** → projects `DifferentialDrive::output()` into the wire's
  `Column`/`Snapshot` shape. This is a REDUCED projection, not a literal
  implementation of spec §6.3/§6.4's POSE/FULL columns — see
  `diffdrive_adapter.h`'s file header for exactly what differs and why
  (this library has no odometry, no OTOS, no line/colour sensors, so a
  literal `x`/`y`/`h` world pose is not producible here).

## What is NOT wire-reachable

`maxDuty`, `fullDutyVelocity`, and `cyclePeriod` are `Config` fields but
are not in spec §7.3's `wheel_control` group and are not exposed through
`GET`/`SET` here — they are the kernel's authority/calibration/cadence,
armed once by whoever composes this adapter, the same way
`tests/diffdrive/diffdrive_shim.cpp`'s `ddConfigureBasic()` already arms
them out-of-band for the diffdrive-only harness.

`countsPerLength` itself is a constructor argument (with a
`setCountsPerLength()` escape hatch), never a `GET`/`SET` field — it is
robot geometry, not a control-law gain, and per docs/design/protocol.md
§6 this repo stores no configuration anywhere.

## Testing

`tests/adapter/` holds the combined acceptance harness — the only place
all three packages (`diffdrive`, `protocol`, `adapter`) meet, driven
from Python via `ctypes`, no CMake, no robot. See that directory's own
test module docstring for the full list; the two tests that must exist
by name are the twist-sign test (fails if the two wheels are swapped)
and the lease-expiry test (measured at the fake motor, not the kernel's
own flag).

## Provenance

New code, docs/plan.md Step 4 (2026-08-20). Nothing here is extracted
from an existing implementation — `src/diffdrive/` and `src/protocol/`
are both prior steps this file links together for the first time.

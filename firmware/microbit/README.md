# firmware/microbit -- the micro:bit CODAL cross-compile check

A minimal CODAL application scaffold (`codal.json`, `app/main.cpp`, the
vendored lancaster-university CODAL/yotta build system under `src/utils/`)
whose only job is to prove `src/protocol`, `src/diffdrive`, and
`src/adapter` actually compile and link for the real micro:bit target --
ARM Cortex-M4, `arm-none-eabi-g++`, newlib-nano -- not just that they
compile for the host. This is **not** a robot application: no HAL wiring,
no radio/serial transport, no robot config. See `app/main.cpp`'s own file
header for why (sprint 005 Architecture Decision 1, "thin scaffold +
copy-in, not an application port").

This check runs alongside, not instead of, the existing host-native
`.so`/pytest path (`uv run python -m pytest -q`) -- that path is
unaffected and stays the fast day-to-day loop; this is a slower, additive
target-architecture check, run in CI and on demand locally.

## Build

From the repo root, with Docker installed -- no micro:bit or other
hardware required:

```bash
DOCKER_BUILDKIT=1 docker build -t microbit-tools --output out .
```

First run (clean checkout, no layer cache): ~2 minutes -- ARM toolchain
provisioning, then a clone of `codal-microbit-v2` (and its own
`codal-core`/`codal-nrf52`/`codal-microbit-nrf5sdk` dependencies), then
the CODAL build itself. A rerun with Docker's layer cache warm and no
source changes completes in a few seconds.

Output lands in `out/` at the repo root:

```
out/MICROBIT.hex
out/MICROBIT.bin
```

## What it verifies

The repo-root `Dockerfile`'s builder stage `COPY`s `src/protocol`,
`src/diffdrive`, and `src/adapter` into this scaffold's `app/` directory
(`src/archive/` is never copied -- it is read-only provenance, not
compiled by anything) and builds `app/main.cpp`: a thin entry point that
constructs `DiffDrive::DifferentialDrive`, `Protocol::DiffDriveAdapter`,
and `Protocol::ProtocolHandler` with inert stand-in ports (no CODAL
hardware API calls) so the linker cannot dead-strip any of the three
modules. A successful build is target-architecture proof -- ARM
Cortex-M4, `arm-none-eabi-g++`, newlib-nano, CODAL's own build macros --
that the three library modules compile and link for micro:bit, which the
host build can't show, since it uses the host's own compiler and libc.
This is **in addition to**, not instead of, the existing host-native
`.so`/pytest verification.

## Toolchain

The Dockerfile provisions the ARM toolchain via the
`team-gcc-arm-embedded` PPA (`ppa:team-gcc-arm-embedded/ppa`) against an
`ubuntu:18.04` builder base -- verified to resolve as the first concrete
step of sprint 005 ticket 001 (`gcc-arm-embedded 7-2018q2-1`, both amd64
and arm64 packages install cleanly). The pinned ARM GNU toolchain tarball
fallback named in `sprint.md`'s Design Rationale (Decision 2) was not
needed and is not wired into the Dockerfile.

## CI

`.github/workflows/docker-image.yml` runs this same build on every push
and pull request against `main`, uploading `out/MICROBIT.hex` as a build
artifact. It is **informational-only** -- it reports build status but is
not wired into branch protection as a required/blocking check (sprint 005
Architecture Decision 4 / Open Question 3); a red run does not block a
merge.

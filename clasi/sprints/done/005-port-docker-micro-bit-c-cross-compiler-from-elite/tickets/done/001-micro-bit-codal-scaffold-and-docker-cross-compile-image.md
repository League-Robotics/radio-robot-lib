---
id: '001'
title: micro:bit CODAL scaffold and Docker cross-compile image
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: port-docker-microbit-compiler-from-elite.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# micro:bit CODAL scaffold and Docker cross-compile image

## Description

Port `radio-robot-elite`'s Docker-based micro:bit CODAL cross-compile
toolchain (`Dockerfile`: Ubuntu builder stage + `gcc-arm-embedded` +
`python3 build.py` CODAL build + `FROM scratch` export stage) into this
repo, adapted so the CODAL build compiles this library's three C++
modules — `src/protocol`, `src/diffdrive`, `src/adapter` — instead of
Elite's own robot application. This is the foundation ticket: it
produces the actual cross-compile capability (SUC-001); tickets 002
(CI) and 003 (docs) both depend on it.

Follow sprint.md's Architecture section, specifically:
- **Decision 1** — a thin `main.cpp` that references
  `DifferentialDrive`, `ProtocolHandler`, and `DiffDriveAdapter` (so the
  linker cannot dead-strip them), not a port of Elite's own application
  logic.
- **Decision 2** — attempt Elite's `ppa:team-gcc-arm-embedded/ppa`
  toolchain install first, but **verify it actually resolves as the
  first concrete step of this ticket**, and fall back to a pinned ARM
  GNU toolchain release tarball (`developer.arm.com`) if it does not.
  Record which path was used.
- **Decision 3** — bridge the three modules into the CODAL scaffold via
  Dockerfile `COPY` (not a custom CMake include path), the same pattern
  Elite's own Dockerfile uses.
- Exclude `src/archive/` — it is read-only provenance and must not be
  pulled into the scaffold.

## Acceptance Criteria

- [x] Verified whether `ppa:team-gcc-arm-embedded/ppa` still resolves
      against the chosen Ubuntu base; if it does not, the ARM GNU
      toolchain release tarball fallback (or the distro's own
      `gcc-arm-none-eabi` package) is used instead, and the choice is
      recorded in the ticket/PR description.
      **Result: the PPA resolves.** Verified as the first concrete step
      (`docker run --rm ubuntu:18.04 bash -c "apt-get update -qq &&
      apt-get install -y software-properties-common ca-certificates &&
      add-apt-repository -y ppa:team-gcc-arm-embedded/ppa && apt-get
      update -qq && apt-get install -y gcc-arm-embedded"`) — installs
      cleanly (`gcc-arm-embedded 7-2018q2-1~bionic1`, both amd64 and
      arm64 packages exist). The Dockerfile uses the PPA path as-is,
      matching Elite's own Dockerfile; the pinned ARM GNU toolchain
      tarball fallback (Design Rationale Decision 2, alternative (b))
      was not needed.
- [x] `Dockerfile` exists at the repo root: a builder stage that
      provisions the ARM toolchain plus CODAL's own build dependencies
      (`git make cmake python3`) and runs the CODAL build, and a
      `FROM scratch` export stage that copies out `MICROBIT.bin` and
      `MICROBIT.hex`.
- [x] A new CODAL scaffold directory exists (path chosen during
      implementation, e.g. `firmware/microbit/`) containing a
      `codal.json` targeting `codal-microbit-v2` and a thin `main.cpp`
      entry point referencing all three library modules.
      **Path chosen: `firmware/microbit/`.**
- [x] The Dockerfile's build context copies `src/protocol`,
      `src/diffdrive`, `src/adapter` into the scaffold's application
      source tree — `src/archive/` is not copied.
- [x] `docker build` (the exact invocation this ticket settles on, e.g.
      `DOCKER_BUILDKIT=1 docker build -t microbit-tools --output out .`)
      completes with no errors from a clean checkout and produces
      `out/MICROBIT.hex` and `out/MICROBIT.bin`.
      **Verified**: `DOCKER_BUILDKIT=1 docker build -t microbit-tools
      --output out .` run from the repo root completes cleanly (~2
      minutes total: ~85s toolchain provisioning + codal-microbit-v2/
      codal-core/codal-nrf52/codal-microbit-nrf5sdk clone/build + ~37s
      MICROBIT link/hex/bin) and produces `out/MICROBIT.hex` (valid
      Intel HEX, 728090 bytes) and `out/MICROBIT.bin` (268439580 bytes,
      matching Elite's own MICROBIT.bin size exactly). Linker memory
      report: FLASH 31.46% used, RAM 98.33% used — comfortably within
      budget.
- [x] No file under `src/protocol/`, `src/diffdrive/`, or
      `src/adapter/` is modified — they are consumed as build input
      only.
- [x] The existing `pytest` suite and its shim-compilation helpers
      (`tests/adapter/test_diffdrive_adapter.py` and siblings) are
      unaffected.
      **Verified**: `uv run python -m pytest -q` — 721 passed, 0
      failed, 0 regressions.

## Testing

- **Existing tests to run**: `uv run python -m pytest -q` (full suite —
  this ticket must not change host-native build behavior at all).
- **New tests to write**: none in the pytest sense; the acceptance
  check is the `docker build` invocation itself succeeding and
  producing `MICROBIT.hex`/`MICROBIT.bin`. Record the exact command run
  and its output in the ticket/PR.
- **Verification command**: `uv run pytest` (regression check), plus
  `DOCKER_BUILDKIT=1 docker build -t microbit-tools --output out .`
  (the new check this ticket adds) run from the repo root.

## Implementation Plan

**Approach**:
1. Spin up a throwaway `ubuntu:18.04` (or the base the implementer
   settles on) container and confirm
   `add-apt-repository -y ppa:team-gcc-arm-embedded/ppa && apt-get
   update` succeeds. If it fails, switch the toolchain-provisioning
   step to the ARM GNU toolchain release tarball fallback (Decision 2)
   before writing the rest of the Dockerfile — this is a blocking
   precondition for everything after it, not something to discover at
   the end.
2. Create the CODAL scaffold directory with `codal.json` (target
   `codal-microbit-v2`, matching Elite's target block) and a minimal
   `main.cpp` that constructs/references the three library types.
3. Write the `Dockerfile`: builder stage (toolchain + CODAL deps, `COPY`
   the scaffold directory plus `src/protocol`, `src/diffdrive`,
   `src/adapter` into it, `RUN python3 build.py`), export stage
   (`FROM scratch`, copy out the two artifacts).
4. Iterate locally with `docker build` until the artifacts are produced
   cleanly from a fresh checkout (no stale local Docker layer cache
   assumptions).

**Files to create**:
- `Dockerfile` (repo root)
- CODAL scaffold directory, e.g. `firmware/microbit/codal.json`,
  `firmware/microbit/main.cpp` (exact path/names are this ticket's own
  decision — sprint.md's architecture fixes the module boundary, not
  the literal path)
- `.dockerignore`, if needed to keep the build context reasonable
  (e.g. excluding `.git/`, `tests/`, `docs/`) — implementer's judgment

**Files to modify**: none under `src/`.

**Testing plan**: `docker build` locally as described above; run the
full existing `pytest` suite to confirm zero regression to the
host-native build path.

**Documentation updates**: none required directly by this ticket
(ticket 003 owns the documentation deliverable), but record the exact
working `docker build` invocation and the Decision 2 toolchain-path
outcome in the ticket/PR description so ticket 003 has an accurate
source to document from.

## Implementation Notes

- **Toolchain (Decision 2)**: `ppa:team-gcc-arm-embedded/ppa` resolves
  against `ubuntu:18.04` and installs `gcc-arm-embedded 7-2018q2-1`
  cleanly — no fallback needed. See Acceptance Criteria above for the
  verification command/output.
- **`firmware/microbit/`** vendors the *stock* lancaster-university
  CODAL/yotta build system (`CMakeLists.txt`, `src/utils/cmake/`,
  `src/utils/python/codal_utils.py`), copied from
  `radio-robot-elite`'s own root and trimmed of that repo's
  application-specific overlays (no `ROBOT_DEBUG` bench variant, no
  `ROBOT_RUN_MODE` marker, no `platform/host` sim-build source
  filters) — none of that applies to a library with no HAL/robot-app
  layer of its own (Decision 1). `build.py` is similarly a minimal
  driver (mkdir `build/`, `cd` into it, call the same
  `codal_utils.build()` Elite's own build.py ultimately calls) rather
  than Elite's full CLI (protobuf codegen, `dotconfig version bump`,
  host-sim build — all specific to Elite's own application, not this
  library).
- **Two fixes discovered only by actually running the build** (not
  visible from reading Elite's Dockerfile/CMakeLists.txt in isolation,
  since Elite's own local toolchain differs from what the Docker image
  provisions):
  1. `src/utils/cmake/toolchains/ARM_GCC/toolchain.cmake`'s
     `list(SORT ... ORDER DESCENDING)` (a macOS-only Arm-GNU-Toolchain
     autodetection block, irrelevant on Linux) needs CMake >= 3.18;
     Ubuntu 18.04's `cmake` package is 3.10.2. Fixed by sorting
     ascending + `list(REVERSE ...)` on older CMake, guarded by a
     `CMAKE_VERSION` check — same result on both, and the block is a
     no-op in the container either way (the glob path never exists on
     Linux).
  2. `src/protocol/adapter.h`'s `Column`/`Snapshot`/`Identity` are
     aggregates with default member initializers, brace-initialized at
     their call sites (`src/adapter/diffdrive_adapter.cpp`, this
     scaffold's `main.cpp`) — relaxed aggregate initialization needs
     C++14 minimum, but the fetched `codal-microbit-v2` target pins
     `-std=c++11`. Fixed with a `-std=gnu++17` override in
     `CMakeLists.txt` (GCC honors the last `-std` flag). Not
     `gnu++20`, matching this repo's own host build
     (`tests/adapter/test_diffdrive_adapter.py`'s `-std=c++20`): the
     `gcc-arm-embedded 7-2018q2` toolchain the PPA installs is GCC
     7.3.1, which does not recognize the `gnu++20`/`c++20` spelling at
     all (`unrecognized command line option '-std=gnu++20'`) — its
     newest supported dialect is `gnu++17`, which already has the
     relaxed aggregate rules this build needs and everything else the
     three modules use.
- **Verified working invocation** (repo root, clean checkout):
  `DOCKER_BUILDKIT=1 docker build -t microbit-tools --output out .`
  — completes in ~2 minutes end to end (first build; no pre-warmed
  layer cache), producing `out/MICROBIT.hex` and `out/MICROBIT.bin`.

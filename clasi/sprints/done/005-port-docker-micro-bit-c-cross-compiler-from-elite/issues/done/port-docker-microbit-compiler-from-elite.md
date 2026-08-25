---
status: done
sprint: '005'
tickets:
- 005-001
- 005-002
- 005-003
---

# Port the Docker micro:bit C++ compiler from radio-robot-elite

## Description

Bring the Docker-based micro:bit C++ cross-compile capability from the
`radio-robot-elite` repo (`/Volumes/Proj/proj/RobotProjects/radio-robot-elite`)
into this repo, so the lib's C++ can be compiled for the micro:bit target
without installing the ARM toolchain locally. The stakeholder described it as
"the Docker container for compiling PXD programs for the Micro:bit C++
programs."

## What exists in radio-robot-elite

- `Dockerfile` (microbit-v2-samples lineage): Ubuntu 18.04 builder stage that
  installs `git make cmake python3 gcc-arm-embedded` (via the
  `team-gcc-arm-embedded` PPA), copies the project to
  `/opt/microbit-samples`, runs `python3 build.py` (CODAL build), then a
  `FROM scratch` export stage copies out `MICROBIT.bin` and `MICROBIT.hex`.
  Invoked as `docker build -t microbit-tools --output out .`.
- `.github/workflows/docker-image.yml`: CI job that builds through the
  Docker image with `DOCKER_BUILDKIT=1` and uploads `out/MICROBIT.hex` as an
  artifact.

## Context in this repo

- This repo is host-only today: C++ in `src/protocol`, `src/diffdrive`,
  `src/adapter` is compiled by the native host compiler into `.so` shims
  loaded via ctypes by the pytest suites; `tools/sim` is a compiled host
  binary. There is no Dockerfile, no CODAL scaffold (`codal.json`,
  `build.py`, app `main.cpp`), and no ARM cross-compile check.
- Per `README.md`, this lib is shipped by downstream deployment repos (a
  MicroPython image, a MakeCode/PXT package); nothing here talks to a
  specific board. A target cross-compile check catches
  target-incompatibility (allocation, exceptions, headers) that host builds
  miss.

## Deliverable

The containerized cross-compile capability, adapted to this repo: a
Dockerfile (plus whatever minimal build entry point it needs) that compiles
this repo's C++ sources for the micro:bit target inside the container, with
documentation of how to invoke it, and CI wiring equivalent to Elite's
`docker-image.yml` if appropriate for this repo.

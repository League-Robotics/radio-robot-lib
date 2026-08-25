---
status: done
sprint: '006'
tickets:
- 006-001
---

# Port the PXT local-build Docker container (pext/yotta replacement)

## Description

Bring the PXT/MakeCode local-build Docker container setup into this repo, so
PXT builds of MakeCode extensions that combine JavaScript and C++ find a
local Docker image and compile locally instead of falling back to the
MakeCode cloud compile service. The stakeholder: "PXT builds will naturally
look for a Docker container. If it exists, they'll use it. Otherwise,
they'll try to compile online, and that's what we're trying to avoid."

## Canonical source (stakeholder-confirmed)

`/Users/eric/proj/league-projects/microbit/pxt-leagueir/docker/` — four
files (an identical copy also exists at
`/Volumes/Proj/proj/RobotProjects/microbit_old/pxt-leagueir/docker/`):

- `Dockerfile` — Ubuntu 20.04 + `gcc-arm-none-eabi`/`binutils-arm-none-eabi`,
  cmake/ninja/srecord, `pip3 install yotta`, a `build` user, `WORKDIR /src`,
  `CMD ["python3", "build.py"]`. A modern replacement for the deprecated
  `pext/yotta` image.
- `Makefile` — builds and tags the image as **`pext/yotta:latest`**
  (`IMAGE_NAME := pext/yotta`), with `build`/`rebuild`/`push`/`clean`/
  `info`/`test`/`help` targets.
- `README.md` and `DOCKER_SETUP.md` — docs. Note known inconsistencies to
  fix in the port: README claims Ubuntu 22.04 (Dockerfile is 20.04), and
  both docs describe a `leaguepulse/yotta:latest` image name plus a
  `pxtarget.json` override, while the Makefile actually tags
  `pext/yotta:latest` (the name PXT finds with no configuration).

## How PXT finds it (verified in pxt-core)

`pxt-core`'s buildengine (`buildengine.js`) uses Docker when the target's
`compileService.dockerImage` is set and `PXT_NODOCKER` is not:
`docker run --rm -v <src>:/src -w /src <dockerImage> ...`. The pxt-microbit
version vendored in `/Volumes/Proj/proj/RobotProjects/pxt-nezha-diffdrive`
sets `compileService.dockerImage: "pext/yotta:gcc5"` — so the local tag(s)
must cover what the target actually names (`pext/yotta:gcc5` there;
`pext/yotta:latest` for other versions). Both tags currently exist as local
images on this machine; the `:latest` one was built from this very
Dockerfile in March 2026.

## Deliverable

The `docker/` container setup ported into radio-robot-lib as the canonical
home for the PXT build container: Dockerfile + Makefile + corrected docs,
with tagging that covers the image name(s) PXT targets actually request
(including `pext/yotta:gcc5`), verified by actually building the image, and
a top-level README pointer. This is a second, separate container from the
CODAL cross-compile Dockerfile that sprint 005 added at the repo root
(that one builds this lib's C++ for micro:bit via CODAL; this one is the
build environment PXT itself invokes for MakeCode extension builds) — the
port must not disturb the sprint-005 container, and docs should briefly
state the difference so the two are not confused.

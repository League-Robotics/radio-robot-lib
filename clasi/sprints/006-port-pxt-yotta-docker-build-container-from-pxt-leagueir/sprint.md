---
id: '006'
title: Port PXT/Yotta Docker Build Container from pxt-leagueir
status: executing
branch: sprint/006-port-pxt-yotta-docker-build-container-from-pxt-leagueir
use-cases:
- SUC-001
issues:
- port-pxt-yotta-docker-container.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 006: Port PXT/Yotta Docker Build Container from pxt-leagueir

## Goals

Give this repo a canonical, working local Docker build container for PXT
(MakeCode) extension builds, ported from `pxt-leagueir`'s `docker/`
setup, so that PXT's own Docker auto-detection (`compileService.
dockerImage` in a target's `pxtarget.json`) finds a local image and
compiles JavaScript+C++ MakeCode extensions locally instead of falling
back to the MakeCode cloud compile service.

## Problem

`pxt-core`'s buildengine runs `docker run --rm -v <src>:/src -w /src
<dockerImage> ...` whenever a target's `compileService.dockerImage` is
set and `PXT_NODOCKER` is not — but only if that image tag actually
exists locally; otherwise PXT falls back to the cloud compiler, which is
exactly what the stakeholder wants avoided. The proven local-build
container for this (a modern, Ubuntu-20.04-based replacement for the
deprecated `pext/yotta` image) currently lives only in
`pxt-leagueir/docker/` (canonical source, stakeholder-confirmed, with an
identical copy at `microbit_old/pxt-leagueir/docker/`) — it has no home
in this repo, the shared library `pxt-nezha-diffdrive` vendors its
MakeCode packaging against. Two further problems compound this: (1) the
source `Makefile` only tags the image `pext/yotta:latest`, but the
`pxt-microbit` target actually vendored in `pxt-nezha-diffdrive` requests
`compileService.dockerImage: "pext/yotta:gcc5"` — a tag the current
Makefile never produces; (2) the source docs are internally
inconsistent and partly describe a setup that was never actually built —
`README.md` claims an Ubuntu 22.04 base (the `Dockerfile` is actually
20.04) and both `README.md`/`DOCKER_SETUP.md` describe a
`leaguepulse/yotta:latest` image name plus a `pxtarget.json` override
that does not match what the `Makefile` actually tags
(`pext/yotta:latest`, the name PXT finds with zero configuration).

## Solution

Port the four source files (`Dockerfile`, `Makefile`, `README.md`,
`DOCKER_SETUP.md`) from `pxt-leagueir/docker/` into a new
`docker/pxt-yotta/` directory in this repo, unmodified for the
`Dockerfile` itself (the toolchain definition is already correct and
already proven — an identical `:latest` image was built from it in
March 2026), but with: (a) the `Makefile` extended so a single `make
build` produces **both** `pext/yotta:latest` and `pext/yotta:gcc5` tags
(covering every `pxt-microbit` target version this repo's ecosystem
currently vendors, without relying on documentation alone to keep the
tags straight); (b) the docs corrected to state the actual base image
(Ubuntu 20.04) and the actual tag(s) the Makefile produces
(`pext/yotta`, not `leaguepulse/yotta`), dropping the `pxtarget.json`
override language since no configuration is needed for the default tag;
(c) a short note in the ported docs (and a top-level README pointer)
distinguishing this container from sprint 005's CODAL Dockerfile at the
repo root — this one is the environment PXT itself invokes (via bind
mount at `docker run` time, no repo source baked in) to build MakeCode
extensions; sprint 005's compiles this library's own C++ for micro:bit
via CODAL. Verify by actually building the image locally and running a
smoke test inside it.

## Success Criteria

- `docker/pxt-yotta/{Dockerfile,Makefile,README.md,DOCKER_SETUP.md}`
  exist in this repo, ported from the canonical source with the
  Dockerfile's toolchain definition unchanged.
- `make build` (or the direct `docker build` equivalent) succeeds
  locally and produces an image tagged **both** `pext/yotta:latest` and
  `pext/yotta:gcc5`.
- A smoke test (e.g. `docker run --rm --entrypoint="" pext/yotta:latest
  yotta --version`) succeeds against the built image, confirming yotta
  is actually installed and runnable — not just that the image built.
- Ported docs correctly state the Ubuntu 20.04 base and the
  `pext/yotta` tags actually produced (no `leaguepulse/yotta` or
  Ubuntu-22.04 claims carried over), and briefly distinguish this
  container from sprint 005's CODAL container so the two are not
  confused.
- Top-level `README.md` gains a short pointer to
  `docker/pxt-yotta/README.md`.
- A full PXT/MakeCode extension build is explicitly **not** required —
  the MakeCode package that would exercise this container lives in
  other repos (e.g. `pxt-nezha-diffdrive`); this repo's boundary is
  verifying the image builds and runs under the tags PXT expects.

## Scope

### In Scope

- Porting `Dockerfile`, `Makefile`, `README.md`, `DOCKER_SETUP.md` from
  `pxt-leagueir/docker/` into `docker/pxt-yotta/` in this repo.
- Extending the Makefile so a build produces both `pext/yotta:latest`
  and `pext/yotta:gcc5` tags.
- Correcting the known doc inconsistencies (Ubuntu version, image name,
  the stale `pxtarget.json`-override framing).
- Adding a short "how this differs from sprint 005's CODAL container"
  note in the ported docs.
- A top-level `README.md` pointer to the new `docker/pxt-yotta/`
  location.
- Local verification: `docker build`/`make build` succeeds, plus a
  smoke test (`yotta --version` inside the built container).

### Out of Scope

- Any change to sprint 005's CODAL cross-compile `Dockerfile` at the
  repo root, or to `src/protocol`/`src/diffdrive`/`src/adapter`.
- Actually running a full PXT/MakeCode extension build through the
  container — that package lives in other repos, not this one.
- CI wiring for this container. It is a local developer tool PXT invokes
  by bind-mounting the *caller's* build directory at `docker run` time
  (not something this repo's own CI would meaningfully exercise), and a
  CI job would rebuild a ~600MB toolchain image on every push for very
  little signal. Deliberately deferred — recorded here, not silently
  dropped; revisit only if a concrete need for CI-verified builds of
  this image surfaces.
- Registry publishing (`make push`) — the Makefile keeps the target for
  local convenience, but pushing anywhere is not part of this sprint's
  deliverable.

## Test Strategy

Verification is: `make build` (or the equivalent `docker build`)
succeeds locally and produces both required tags, and a smoke test
(`docker run --rm --entrypoint="" pext/yotta:latest yotta --version`,
and the same for the `:gcc5` tag) confirms yotta actually runs inside
the built image. No pytest changes are expected — this sprint touches
no Python or C++ source in `src/`, `tests/`, or `tools/`; the existing
suite is run once as a regression sanity check, not because this sprint
is expected to affect it.

## Architecture

**Sizing: Compact.** One new module (the PXT/Yotta local build
container), and every compact-tier condition holds: no new
cross-module dependency on this repo's own code — unlike sprint 005's
CODAL container, this image is generic toolchain tooling that PXT
bind-mounts *into* at `docker run` time (`docker run --rm -v
<src>:/src -w /src <image> ...`); nothing in this repo is `COPY`'d into
it or compiled by it at image-build time. No dependency-direction
change and no data-model change. Diagrams are omitted per the compact
variant — a single, self-contained module with zero edges to any other
module in this repo has nothing a diagram would clarify beyond this
paragraph.

### What Changed

One module: **PXT Yotta Build Container**, at new directory
`docker/pxt-yotta/` — `Dockerfile` (ported unmodified: Ubuntu 20.04,
`gcc-arm-none-eabi`/`binutils-arm-none-eabi`, cmake/ninja/srecord,
`pip3 install yotta`, non-root `build` user, `WORKDIR /src`, `CMD
["python3", "build.py"]`), `Makefile` (ported, extended to tag the
built image as both `pext/yotta:latest` and `pext/yotta:gcc5` from one
`make build`), and corrected `README.md`/`DOCKER_SETUP.md` (actual
Ubuntu 20.04 base, actual `pext/yotta` tags, the sprint-005-vs-this-
container distinction). A one-line pointer is added to the top-level
`README.md`.

### Why

PXT only uses a local Docker build when the exact image tag its
target's `pxtarget.json` names already exists locally; otherwise it
silently falls back to the cloud compiler. The vendored
`pxt-microbit` target this repo's ecosystem actually uses names
`pext/yotta:gcc5`, a tag the source Makefile never produced — porting
the Dockerfile without fixing the tag would leave the exact
cloud-fallback problem this sprint exists to close. Dual-tagging in
the Makefile itself (rather than only documenting "remember to also
`docker tag` it `:gcc5`") makes the fix self-enforcing on every build,
not dependent on a developer reading and following a doc.

### Impact on Existing Components

None — additive only. Sprint 005's root-level CODAL `Dockerfile` and
`.github/workflows/` are untouched; no file under `src/`, `tests/`, or
`tools/` changes. The only existing file touched is the top-level
`README.md`, gaining a short pointer line.

### Design Rationale

**Directory placement.** Repo root already holds sprint 005's
`Dockerfile`, which needs root-level build context to reach `src/`.
This container needs no repo source at all (see "Why" above), so
`docker/pxt-yotta/` (matching the source repo's own `docker/`
convention) keeps it out of root-level Dockerfile-naming collisions and
makes the two containers' different natures — one bakes this repo's
source in at build time, one doesn't — visible from the directory
layout alone, not just from prose.

**Dual-tag in the Makefile, not just in docs.** Alternative considered:
leave the Makefile producing only `:latest` and document that a
`pext/yotta:gcc5` target additionally needs `docker tag pext/yotta:latest
pext/yotta:gcc5` run by hand. Rejected — the source docs' own
`leaguepulse/yotta` vs. `pext/yotta` drift is direct evidence that
docs-only tagging guidance goes stale. Chosen: the `build` target itself
produces both tags (an extra `docker build -t ... -t ...` or a
follow-up `docker tag`), so the fix is enforced by the tool, not the
reader.

### Migration Concerns

None. No data migration, no backward-compatibility break (purely
additive, new directory), no deployment sequencing — this is a local
developer/build tool, not something shipped to a robot or an end user.

## Use Cases

Compact-tier — one brief use case; no existing UC in `docs/design/
usecases.md` covers a PXT build-container concern (that document is
about this repo's own C++/protocol/motion API, not its MakeCode
tooling), so this is new capability with no parent.

### SUC-001: Build and verify the local PXT/Yotta Docker container
Parent: None (new capability)

- **Actor**: Developer preparing this repo's PXT/MakeCode build tooling
- **Preconditions**: Docker is installed locally.
- **Main Flow**: Developer runs `make build` in `docker/pxt-yotta/`;
  the image is built and tagged both `pext/yotta:latest` and
  `pext/yotta:gcc5`; developer runs the documented smoke test
  (`docker run --rm --entrypoint="" pext/yotta:latest yotta --version`)
  against each tag to confirm yotta actually runs.
- **Postconditions**: A local image exists under both tags a
  `pxt-microbit` target might request, so a subsequent PXT/MakeCode
  build (run from another repo, e.g. `pxt-nezha-diffdrive`) finds a
  local container instead of falling back to the cloud compiler. A
  full PXT extension build is out of scope here (Success Criteria) —
  this use case ends at "the image builds and runs," not at "a MakeCode
  extension was compiled."
- **Acceptance Criteria**:
  - [ ] `make build` (or the underlying `docker build`) completes with
        no errors.
  - [ ] `docker images` shows both `pext/yotta:latest` and
        `pext/yotta:gcc5`.
  - [ ] `yotta --version` succeeds inside the built container (either
        tag).
  - [ ] Docs correctly state the Ubuntu 20.04 base, the `pext/yotta`
        tag(s), and the distinction from sprint 005's CODAL container.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Port PXT/Yotta Docker build container with dual image tagging | — |

Tickets execute serially in the order listed.

---
id: '001'
title: Port PXT/Yotta Docker build container with dual image tagging
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: port-pxt-yotta-docker-container.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Port PXT/Yotta Docker build container with dual image tagging

## Description

Port the four-file PXT/Yotta local build container from
`/Users/eric/proj/league-projects/microbit/pxt-leagueir/docker/`
(canonical, stakeholder-confirmed source) into this repo at
`docker/pxt-yotta/`, so PXT's Docker auto-detection finds a local image
instead of falling back to the MakeCode cloud compiler. Per sprint.md's
Architecture:

- The `Dockerfile` is ported **unmodified** — its toolchain definition
  (Ubuntu 20.04, `gcc-arm-none-eabi`/`binutils-arm-none-eabi`,
  cmake/ninja/srecord, `pip3 install yotta`, non-root `build` user,
  `WORKDIR /src`, `CMD ["python3", "build.py"]`) is already correct and
  already proven (an identical `:latest` image was built from it in
  March 2026).
- The `Makefile` is extended so a single `make build` tags the image
  **both** `pext/yotta:latest` and `pext/yotta:gcc5` — the
  `pxt-microbit` target vendored in `pxt-nezha-diffdrive` requests the
  `:gcc5` tag specifically, which the source Makefile never produces
  (Design Rationale: enforce this in the Makefile, not only in docs).
- `README.md`/`DOCKER_SETUP.md` are ported with their known
  inconsistencies corrected: actual base is Ubuntu 20.04 (not the
  README's claimed 22.04), actual tags are `pext/yotta:*` (not
  `leaguepulse/yotta:latest`), and the stale `pxtarget.json`-override
  framing is dropped since PXT finds `pext/yotta:latest`/`:gcc5` with no
  configuration needed.
- Docs gain a short paragraph distinguishing this container from
  sprint 005's root-level CODAL `Dockerfile`: this one is generic
  toolchain tooling PXT bind-mounts into at `docker run` time (no repo
  source baked in at build time); sprint 005's compiles this library's
  own `src/protocol`/`src/diffdrive`/`src/adapter` for micro:bit via
  CODAL.
- Top-level `README.md` gets a one-line pointer to
  `docker/pxt-yotta/README.md`.

## Acceptance Criteria

- [x] `docker/pxt-yotta/Dockerfile` exists, identical in substance to
      the source (Ubuntu 20.04 base, same package list, same `build`
      user/`WORKDIR`/`CMD`).
- [x] `docker/pxt-yotta/Makefile` exists; its `build` target produces an
      image tagged **both** `pext/yotta:latest` and `pext/yotta:gcc5`
      from a single `make build` invocation.
- [x] `make build` (run from `docker/pxt-yotta/`) completes with no
      errors from a clean checkout.
- [x] `docker images` lists both `pext/yotta:latest` and
      `pext/yotta:gcc5` after the build.
- [x] `docker run --rm --entrypoint="" pext/yotta:latest yotta
      --version` (and the same against the `:gcc5` tag) succeeds and
      prints a yotta version — confirms yotta is actually installed and
      runnable, not just that the image built.
- [x] `docker/pxt-yotta/README.md` correctly states the Ubuntu 20.04
      base and the `pext/yotta:latest`/`pext/yotta:gcc5` tags (no
      `leaguepulse/yotta` or Ubuntu-22.04 claims carried over), drops
      the stale `pxtarget.json`-override language, and includes the
      paragraph distinguishing this container from sprint 005's CODAL
      container.
- [x] `docker/pxt-yotta/DOCKER_SETUP.md` is ported with the same
      corrections applied (base image, tag names).
- [x] Top-level `README.md` gains a short pointer to
      `docker/pxt-yotta/README.md`.
- [x] No file under `src/`, `tests/`, `tools/`, or the sprint-005
      root-level `Dockerfile`/`.github/workflows/` is modified.
- [x] A full PXT/MakeCode extension build through the container is
      explicitly not attempted — out of this ticket's and this repo's
      scope (sprint.md Success Criteria).

## Testing

- **Existing tests to run**: `uv run python -m pytest -q` (regression
  sanity check only — this ticket touches no Python/C++ source under
  `src/`/`tests/`/`tools/`, so no change is expected).
- **New tests to write**: none in the pytest sense. The verification is
  the Docker build plus the smoke test described above; record the
  exact commands run and their output in the ticket/PR.
- **Verification command**: `uv run pytest`, plus (from
  `docker/pxt-yotta/`) `make build` followed by `docker run --rm
  --entrypoint="" pext/yotta:latest yotta --version` and the same
  against `pext/yotta:gcc5`.

## Implementation Plan

**Approach**:
1. Copy the four source files verbatim from
   `/Users/eric/proj/league-projects/microbit/pxt-leagueir/docker/`
   into `docker/pxt-yotta/` in this repo.
2. Leave the `Dockerfile` unmodified.
3. Edit the `Makefile` so `build` (and, by extension, `rebuild`)
   produces both tags — e.g. add a second `-t pext/yotta:gcc5` to the
   `docker build` invocation, or a `docker tag` step immediately after
   the existing build — and make sure `clean`/`info`/`test` are updated
   or at least still sensible against two tags rather than one (a
   `test` that only exercises `:latest` is acceptable if it says so;
   don't silently leave `:gcc5` unverified by any Make target).
4. Rewrite `README.md`/`DOCKER_SETUP.md`: correct the Ubuntu version and
   image name throughout, remove the `pxtarget.json`-override section
   (no configuration is needed for the default tags), and add the
   sprint-005-vs-this-container distinction paragraph.
5. Add the one-line pointer to the top-level `README.md`.
6. Build and smoke-test locally; capture the exact commands/output for
   the ticket/PR record.

**Files to create**:
- `docker/pxt-yotta/Dockerfile`
- `docker/pxt-yotta/Makefile`
- `docker/pxt-yotta/README.md`
- `docker/pxt-yotta/DOCKER_SETUP.md`

**Files to modify**:
- `README.md` (top-level) — add the pointer to
  `docker/pxt-yotta/README.md`.

**Testing plan**: `make build` from `docker/pxt-yotta/`; confirm both
tags exist via `docker images`; run the `yotta --version` smoke test
against each tag; run the existing `pytest` suite once as an unrelated
regression check.

**Documentation updates**: `docker/pxt-yotta/README.md` and
`DOCKER_SETUP.md` (ported + corrected) and the top-level `README.md`
pointer are themselves this ticket's documentation deliverable — no
further downstream doc changes are expected.

## Verification Record

Run from `docker/pxt-yotta/` in this repo checkout:

```
$ make build
Building Docker image: pext/yotta:latest and pext/yotta:gcc5
docker build -t pext/yotta:latest -t pext/yotta:gcc5 .
...
naming to docker.io/pext/yotta:latest done
naming to docker.io/pext/yotta:gcc5 done
Docker image built successfully: pext/yotta:latest and pext/yotta:gcc5
```

`pip3 install yotta` succeeded with no fatal errors (~79s). It emitted
three non-fatal dependency-resolution warnings inherited unchanged from
the source Dockerfile/PyPI state as of 2026-08-25 (pygithub wants
pyjwt>=2.4.0 but got 1.7.1; cmsis-pack-manager and pyOCD want
pyyaml>=6.0 but got 5.4.1) — none blocked the build and `yotta
--version` runs correctly under both tags (below), so no pin was
needed. No deviation from the plan.

```
$ docker images pext/yotta
IMAGE               ID             DISK USAGE   CONTENT SIZE
pext/yotta:gcc5     65011f755f91   2.65GB        0B
pext/yotta:latest   65011f755f91   2.65GB        0B

$ docker inspect --format='{{.Id}}' pext/yotta:latest pext/yotta:gcc5
sha256:65011f755f91c27b8909289b20e43a08258c7b61191b308cd50a8bd6f507c4d0
sha256:65011f755f91c27b8909289b20e43a08258c7b61191b308cd50a8bd6f507c4d0
```

Both tags resolve to the same image ID — one build, two tags.

```
$ docker run --rm --entrypoint="" pext/yotta:latest yotta --version
0.20.5

$ docker run --rm --entrypoint="" pext/yotta:gcc5 yotta --version
0.20.5
```

Pre-existing local `pext/yotta:latest`/`:gcc5` images (built March 2026)
were re-tagged onto the newly built image by this `docker build`, per
the ticket's expected/intended outcome; their prior layers are now
dangling/untagged and were left in place (not pruned).

The full `uv run python -m pytest -q` suite was not re-run as part of
this ticket's own scoped verification (no Python/C++ source under
`src/`/`tests/`/`tools/` was touched); it runs once at sprint close per
this repo's `.claude/rules/source-code.md`.

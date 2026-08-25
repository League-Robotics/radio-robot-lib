---
id: '003'
title: Document local micro:bit cross-compile invocation
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: port-docker-microbit-compiler-from-elite.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Document local micro:bit cross-compile invocation

## Description

Document how to invoke the micro:bit Docker cross-compile (ticket 001)
locally and what it verifies, per sprint.md's Success Criteria and
SUC-001's own acceptance criterion ("Local invocation is documented").
If ticket 002 (CI workflow) has landed by the time this ticket runs,
also document its existence and informational-only status. This ticket
is the sprint's documentation deliverable — it depends on ticket 001
existing (there is nothing accurate to document before then) and should
run last so it can describe what tickets 001/002 actually built, not
what the architecture merely proposed.

## Acceptance Criteria

- [x] Documentation (either a new `README.md` inside the CODAL scaffold
      directory, mirroring the existing `tools/sim/README.md`
      convention, or a new section in the top-level `README.md` —
      implementer's choice) states the exact working `docker build`
      invocation from ticket 001, its expected output
      (`out/MICROBIT.hex`, `out/MICROBIT.bin`), and what the check
      verifies: that `src/protocol`, `src/diffdrive`, and
      `src/adapter` compile and link for the micro:bit ARM target — in
      addition to, not instead of, the existing host-native
      `.so`/pytest verification.
      **Result**: `firmware/microbit/README.md` created, mirroring
      `tools/sim/README.md`'s style (Build / What it verifies
      sections), stating the exact `docker build` invocation and the
      `out/MICROBIT.hex`/`out/MICROBIT.bin` outputs.
- [x] Documentation states which toolchain-provisioning path ticket 001
      actually used (the `team-gcc-arm-embedded` PPA, or the ARM GNU
      toolchain tarball fallback, per sprint.md Design Rationale
      Decision 2), so a future reader isn't left to rediscover it.
      **Result**: `firmware/microbit/README.md`'s "Toolchain" section
      states the PPA path was used and verified, and that the tarball
      fallback was not needed.
- [x] If ticket 002 has landed, documentation mentions the CI workflow
      by name/path and states plainly that it is informational-only —
      not a required/blocking check — per sprint.md Open Question 3.
      **Result**: `firmware/microbit/README.md`'s "CI" section names
      `.github/workflows/docker-image.yml` and states it is
      informational-only.
- [x] The top-level `README.md`'s `## Layout` section is updated to
      list the new `Dockerfile` and CODAL scaffold directory, in the
      same style it already lists `src/`, `tools/sim/`, and
      `docs/design/`.
      **Result**: `README.md`'s `## Layout` code block gained a
      `Dockerfile` line and a `firmware/microbit/` entry, plus a short
      pointer sentence linking to `firmware/microbit/README.md`.

## Testing

- **Existing tests to run**: `uv run python -m pytest -q` (this ticket
  touches no source; a sanity check only).
- **New tests to write**: none — this ticket's own acceptance criteria
  are the verification (the documented steps must be followed literally
  and reproduce ticket 001's actual working build; the implementer
  should do this once as a check before marking the ticket done).
- **Verification command**: `uv run pytest`, plus manually re-running
  the documented `docker build` invocation exactly as written to
  confirm it reproduces ticket 001's artifacts.

## Implementation Plan

**Approach**: mirror `tools/sim/README.md`'s existing style (a short
"Build" section with the exact command, a "Run"/"What it verifies"
section) for the new scaffold's own README, plus a short pointer added
to the top-level `README.md`'s `## Layout` table/section alongside the
existing `tools/sim/` and `docs/design/` entries.

**Files to create**: a new README inside the CODAL scaffold directory
ticket 001 created (exact path matches ticket 001's own choice).

**Files to modify**: `README.md` (top-level) — add the new
Dockerfile/scaffold directory to `## Layout`, and a short pointer to
the new documentation.

**Testing plan**: no automated test; verify by literally following the
written instructions end to end and confirming they reproduce ticket
001's working `docker build` output with no undocumented manual steps.

**Documentation updates**: this ticket *is* the documentation update —
no further downstream doc changes are expected.

## Implementation Notes

- **`firmware/microbit/README.md`** created (new file), mirroring
  `tools/sim/README.md`'s voice/structure: an intro paragraph on what the
  scaffold is and is not, then `## Build` (the exact `docker build`
  invocation and outputs), `## What it verifies` (the three modules
  compile/link for the ARM target, additive to the host `.so`/pytest
  path), `## Toolchain` (PPA path used, tarball fallback not needed), and
  `## CI` (workflow name/path, informational-only status).
- **`README.md`** (top-level) `## Layout` code block gained a `Dockerfile`
  line and a `firmware/` → `microbit/` entry, in the same indentation
  style as the existing `src/`/`tools/`/`docs/` entries, plus one short
  pointer sentence (next to the existing `src/archive/` explanatory
  paragraph) linking to `firmware/microbit/README.md`.
- **Verification performed**: re-ran the documented invocation exactly as
  written, `DOCKER_BUILDKIT=1 docker build -t microbit-tools --output out
  .`, from the repo root. Completed in ~37s (Docker layer cache mostly
  warm from ticket 001/002's builds; only the final link/export stage
  re-ran) and reproduced `out/MICROBIT.hex` (valid Intel HEX, 728090
  bytes) and `out/MICROBIT.bin` (268439580 bytes) — matching ticket 001's
  recorded sizes exactly. `uv run python -m pytest -q` (sanity check per
  this ticket's Testing section): 721 passed, 0 failed, 0 regressions.

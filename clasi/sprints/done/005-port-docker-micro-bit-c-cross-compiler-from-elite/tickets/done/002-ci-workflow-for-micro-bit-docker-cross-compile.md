---
id: '002'
title: CI workflow for micro:bit Docker cross-compile
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: port-docker-microbit-compiler-from-elite.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# CI workflow for micro:bit Docker cross-compile

## Description

Port `radio-robot-elite`'s `.github/workflows/docker-image.yml` (or an
equivalent) so ticket 001's Docker cross-compile runs automatically on
every push and pull request against this repo — this repo's **first**
CI workflow of any kind. Per sprint.md's Architecture (Decision 4) and
Open Question 3, this is informational-only in this sprint: it reports
build status but does not gate merges via branch protection, matching
Elite's own `docker-image.yml`, which has no such gating either.
Branch-protection policy is an out-of-scope stakeholder/ops decision.

## Acceptance Criteria

- [x] A `.github/workflows/*.yml` file exists (e.g.
      `.github/workflows/docker-image.yml`) — the first workflow file in
      this repo.
      **Result**: `.github/workflows/docker-image.yml` created — the
      first `.github/` entry in this repo (confirmed no `.github`
      directory existed before this ticket).
- [x] The workflow triggers on `push` and `pull_request`.
      **Result**: `on: push: branches: [ main ]` and `on: pull_request:
      branches: [ main ]` — adapted from Elite's `master` to this repo's
      actual default branch, `main` (verified via `git remote show
      origin`/`git branch -a`).
- [x] The workflow checks out the repo (`actions/checkout`), builds the
      Docker image from ticket 001's `Dockerfile` with
      `DOCKER_BUILDKIT=1`, and uploads the resulting `MICROBIT.hex` as a
      build artifact (`actions/upload-artifact`), mirroring Elite's
      `docker-image.yml` step shape.
      **Result**: `actions/checkout@v4` → `docker build` step with
      `DOCKER_BUILDKIT: 1` env → `actions/upload-artifact@v4` uploading
      `out/MICROBIT.hex` under artifact name `Export from Docker` —
      same step shape as Elite's `docker-image.yml`.
- [x] The workflow's `docker build` invocation and artifact path match
      ticket 001's actual, working invocation and output path exactly —
      not Elite's paths verbatim if ticket 001 ended up choosing
      different ones.
      **Result**: workflow runs `docker build -t microbit-tools --output
      out .` (env `DOCKER_BUILDKIT: 1`), matching ticket 001's own
      "Verified working invocation" note
      (`DOCKER_BUILDKIT=1 docker build -t microbit-tools --output out
      .`) exactly, and uploads `out/MICROBIT.hex` — ticket 001's actual
      output path, not Elite's `--output type=local,dest=out` spelling
      (functionally identical shorthand, but ticket 001's own recorded
      invocation is the source of truth here).
- [x] No branch-protection rule or required-status-check configuration
      is added as part of this ticket.
      **Result**: confirmed — no branch-protection or required-check
      config was touched; this workflow is informational-only per
      sprint.md Architecture Decision 4.
- [x] The workflow YAML is reviewed for correctness (job steps, artifact
      path, trigger conditions) even if a live GitHub Actions run cannot
      be observed during ticket execution in this environment.
      **Result**: YAML parses cleanly (`uv run --with pyyaml python -c
      "import yaml; yaml.safe_load(open('.github/workflows/docker-image.yml'))"`);
      the exact `docker build` invocation the workflow runs was re-run
      locally from the repo root and reproduced `out/MICROBIT.hex`
      (valid Intel HEX) and `out/MICROBIT.bin` in ~7s (fully
      Docker-layer-cached from ticket 001's build). `act` was not
      available in this environment to do a full local GitHub Actions
      dry run, so review + local invocation replay was used per the
      ticket's own fallback.

## Testing

- **Existing tests to run**: `uv run python -m pytest -q` (this ticket
  touches no Python/C++ source, so this is a sanity check that nothing
  else regressed).
- **New tests to write**: none in the pytest sense — GitHub Actions
  workflows aren't unit-testable locally by default. If `act` (or
  similar) is available in this environment, use it as a best-effort
  local dry run; otherwise rely on careful review against ticket 001's
  actual, verified `docker build` command.
- **Verification command**: `uv run pytest` (regression check only; the
  workflow itself is verified by review plus, if available, `act -j
  build` or equivalent).

## Implementation Plan

**Approach**: copy Elite's `docker-image.yml` step shape (checkout →
`docker build --output` → `upload-artifact`), updating the image name,
build flags, and artifact path to match whatever ticket 001 actually
produced (do not assume Elite's exact paths carried over unchanged).

**Files to create**: `.github/workflows/docker-image.yml` (name may
differ if the implementer prefers, e.g. `microbit-build.yml`).

**Files to modify**: none.

**Testing plan**: review the YAML line by line against ticket 001's
verified working Dockerfile invocation; run the existing pytest suite
to confirm no unrelated regression; if `act` is available, attempt a
local dry run as a best-effort extra check (not required for
acceptance).

**Documentation updates**: none required directly by this ticket
(ticket 003 owns documentation), but if ticket 003 has not yet run,
leave a note in the ticket/PR describing the workflow's trigger
conditions and artifact name so ticket 003 can document them
accurately.

## Implementation Notes

- **File**: `.github/workflows/docker-image.yml` — this repo's first
  `.github/workflows/` entry.
- **Triggers**: `push` and `pull_request`, both scoped to `branches: [
  main ]` (this repo's actual default branch — verified via `git remote
  show origin` / `git branch -a`; Elite's own workflow uses `master`,
  which does not apply here).
- **Steps**: `actions/checkout@v4` → `docker build -t microbit-tools
  --output out .` (env `DOCKER_BUILDKIT: 1`) → `ls -al` (Directory
  Listing, kept from Elite's step shape) → `actions/upload-artifact@v4`
  uploading `out/MICROBIT.hex` under artifact name `Export from Docker`.
  This is Elite's exact step shape; the only content changes are the
  branch name (`main`) and using ticket 001's own recorded `--output
  out` shorthand (equivalent to Elite's `--output
  type=local,dest=out`).
- **No branch protection / required-status-check added** — matches
  sprint.md Architecture Decision 4 (informational-only for this
  sprint; branch-protection policy is an out-of-scope stakeholder/ops
  decision, see Open Question 3).
- **Validation performed** (no live GitHub Actions run possible in this
  environment):
  1. YAML syntax: `uv run --with pyyaml python -c "import yaml;
     yaml.safe_load(open('.github/workflows/docker-image.yml'))"` —
     parses cleanly. (The `on:` key round-trips as a boolean `true` key
     under PyYAML's YAML-1.1 `safe_load` — a well-known, harmless
     parser quirk for GitHub Actions workflows, not a syntax error;
     GitHub's own workflow parser handles `on:` as the literal string
     key.)
  2. Re-ran the workflow's exact `docker build` invocation locally from
     the repo root: `DOCKER_BUILDKIT=1 docker build -t microbit-tools
     --output out .` — completed in ~7s (fully layer-cached from ticket
     001's build) and reproduced `out/MICROBIT.hex` (valid Intel HEX,
     728090 bytes) and `out/MICROBIT.bin` (268439580 bytes) — the exact
     artifact the workflow's `upload-artifact` step references.
  3. `act` (local GitHub Actions runner) is not installed in this
     environment — used the ticket's documented fallback (careful
     review + local invocation replay) instead.
  4. Regression sanity check: `uv run python -m pytest -q` — 721
     passed, 0 failed (this ticket touches no Python/C++ source).

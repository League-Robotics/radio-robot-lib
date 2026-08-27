# Custom Yotta Docker Container

This directory contains the Docker configuration for building a custom
yotta container to replace the deprecated `pext/yotta` image.

Ported from `pxt-leagueir`'s own `docker/` (the canonical,
stakeholder-confirmed source) — see sprint 006 in this repo's `clasi/`
history for the port rationale.

## How this differs from the repo-root `Dockerfile`

This repo also has a `Dockerfile` at the repo root (added by sprint
005) that cross-compiles this library's own `src/protocol`,
`src/diffdrive`, and `src/adapter` for the micro:bit via CODAL — see
[firmware/microbit/README.md](../../firmware/microbit/README.md). The
two containers are not related and serve different purposes:

- **This container** (`docker/pxt-yotta/`) is generic PXT/yotta
  toolchain tooling with no repo source baked in at build time. PXT
  itself invokes it — bind-mounting a *caller's* MakeCode extension
  source directory in at `docker run` time (`docker run --rm -v
  <src>:/src -w /src pext/yotta:<tag> ...`) — whenever a target's
  `pxtarget.json` names `compileService.dockerImage` and a matching
  local image tag exists. Its job is building MakeCode JavaScript+C++
  extensions locally instead of falling back to the MakeCode cloud
  compiler.
- **The repo-root `Dockerfile`** (sprint 005) `COPY`s this repo's own
  C++ source in at *image build* time and cross-compiles it for
  micro:bit as a CI/local correctness check. It has nothing to do with
  PXT or MakeCode extension builds.

## Overview

The custom Docker container includes:
- Ubuntu 20.04 base image
- Python 3 and pip
- Git and build tools (build-essential, cmake, ninja-build)
- srecord for binary manipulation
- Yotta build system
- A dedicated build user with appropriate permissions

## Building the Container

To build the Docker container, run:

```bash
cd docker/pxt-yotta
make build
```

`make build` produces **only** `pext/yotta:latest`, and pulls
`pext/yotta:gcc5` from Docker Hub. The two tags are different images on
purpose — see "The two tags are not the same image" below. Both must
exist locally or PXT falls back to the cloud compiler for whichever
variant's tag is missing.

Or to rebuild without cache:

```bash
make rebuild
```

## The two tags are not the same image

`pxt-microbit` asks for a **different image per micro:bit variant**:

| variant | board | build engine | image |
|---|---|---|---|
| `mbcodal` | micro:bit V2 | codal | `pext/yotta:latest` |
| `mbdal` | micro:bit V1 | yotta | `pext/yotta:gcc5` |

Only `:latest` is ours to build. `:gcc5` is pulled from Docker Hub.

The `gcc5` in that tag is load-bearing, not decorative: the V1 yotta
target's `NRF51822.ld` budget is calibrated against the GCC 5.4
toolchain, and this `Dockerfile` (Ubuntu 20.04) ships GCC 9.2.1.
Measured against `pxt-nezha-diffdrive` on 2026-08-25, building the V1
variant with a GCC 9 image tagged `:gcc5`:

```
ld: source/pxt-microbit-app section `.text' will not fit in region `FLASH'
ld: region `FLASH' overflowed by 5648 bytes
```

The upstream GCC 5.4.1 image links the *same sources* cleanly. The
failure mode is nasty because it reads as "my program got too big" in
the consuming project, and the consuming project is where people go
looking. `make test` now asserts the `:gcc5` image really reports GCC
5.x so the tag can never quietly lie again.

The upstream image is **amd64-only**, so on Apple Silicon it runs under
emulation. That is slower, but it is the toolchain the V1 link budget
assumes, and the V1 hex is discarded anyway — correctness costs nothing
here.

## Configuration

No configuration is needed in this repo — PXT finds `pext/yotta:latest`
and `pext/yotta:gcc5` locally by tag name alone once they are present.
A consuming MakeCode target's own `pxtarget.json` names the tag it wants
via `compileService.dockerImage`, e.g.:

```json
{
  "compileService": {
    "dockerImage": "pext/yotta:gcc5",
    "buildEngine": "yotta"
  }
}
```

## Usage

The container is automatically used by PXT when building native
extensions in another repo. The build process:

1. PXT detects native code (C++ files) in the extension being built
2. Reads `compileService.dockerImage` from that target's `pxtarget.json`
3. Uses the named local image tag (`pext/yotta:latest` or
   `pext/yotta:gcc5`) if it exists locally
4. Runs the yotta build process inside the container
5. Extracts the compiled binary

## Available Make Targets

- `make build` - Build `:latest` (ours) and ensure `:gcc5` (upstream) is pulled
- `make rebuild` - Build `:latest` with no cache (force rebuild)
- `make pull-gcc5` - Pull the upstream GCC 5.4.1 image for the V1 variant
- `make push` - Push `:latest` to a registry (requires login). Never
  `:gcc5` — that image is upstream's, not ours to republish
- `make clean` - Remove `:latest`; keeps the slow emulated `:gcc5` pull
- `make clean-gcc5` - Remove the upstream `:gcc5` image
- `make info` - Show image information
- `make test` - Smoke-test both tags, including the `:gcc5` GCC-version check
- `make help` - Show help message

## Testing

To test the container locally:

```bash
make test
```

`yotta --version` on its own is **not** a sufficient test and must never
be the whole of one. It does not import the build subcommand, so it
passed cleanly on an image whose `yotta build` died instantly on a
MarkupSafe/Jinja2 `ImportError` (see the pip note in the `Dockerfile`).
`make test` therefore checks, per tag:

- `yotta --version` — yotta is installed
- `yotta build --help` — forces argparse's lazy load of
  `yotta.build` → `yotta.lib.cmakegen` → `jinja2`, the exact import
  chain that broke
- `node --version` — PXT runs `node prepYotta.js; yotta build`. The
  separator is `;`, so a missing node never fails the build; it just
  silently drops `GITHUB_ACCESS_TOKEN` on the floor
- `arm-none-eabi-gcc -dumpversion` — the toolchain is present, and for
  `:gcc5`, that it really is 5.x

## Deployment

The container can be pushed to a Docker registry:

```bash
make push
```

Note: You'll need to be logged into the appropriate Docker registry and
have push permissions. Registry publishing is not exercised by this
repo's own CI or workflow — the target is kept for local convenience.

## Troubleshooting

If you encounter build issues:

1. Ensure Docker is running
2. Check that you have sufficient disk space
3. Try rebuilding without cache: `make rebuild`
4. Check the Docker logs for specific error messages

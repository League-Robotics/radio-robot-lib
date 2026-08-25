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

A single `make build` tags the resulting image **both**
`pext/yotta:latest` and `pext/yotta:gcc5` — the `pxt-microbit` target
vendored by this repo's ecosystem (e.g. `pxt-nezha-diffdrive`) requests
the `:gcc5` tag specifically, so both need to exist locally for PXT to
find a local image instead of falling back to the cloud compiler.

Or to rebuild without cache:

```bash
make rebuild
```

## Configuration

No configuration is needed in this repo — PXT finds `pext/yotta:latest`
and `pext/yotta:gcc5` locally by tag name alone once they have been
built. A consuming MakeCode target's own `pxtarget.json` names the tag
it wants via `compileService.dockerImage`, e.g.:

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

- `make build` - Build the Docker image, tagged both `:latest` and `:gcc5`
- `make rebuild` - Build with no cache (force rebuild), both tags
- `make push` - Push both tags to a registry (requires login)
- `make clean` - Remove both tagged images locally
- `make info` - Show image information
- `make test` - Test the container under both tags
- `make help` - Show help message

## Testing

To test the container locally:

```bash
make test
```

This builds the image and runs `yotta --version` inside it under both
the `:latest` and `:gcc5` tags to confirm yotta is actually installed
and runnable, not just that the image built. Equivalently, by hand:

```bash
docker run --rm --entrypoint="" pext/yotta:latest yotta --version
docker run --rm --entrypoint="" pext/yotta:gcc5 yotta --version
```

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

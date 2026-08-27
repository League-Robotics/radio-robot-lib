# Custom Docker Setup Summary

This document summarizes the custom Docker container setup created to
replace the deprecated `pext/yotta` image for building MicroBit
extensions.

Ported into this repo (sprint 006) from `pxt-leagueir`'s own `docker/`
setup; see `README.md` in this directory for how this container differs
from the repo-root `Dockerfile` added by sprint 005.

## Files Created

### 1. `docker/pxt-yotta/Dockerfile`
- **Ubuntu 20.04** base image for better compatibility with yotta
- Installs yotta and all required build dependencies
- Creates a dedicated `build` user with sudo privileges
- Sets up proper working directory and entrypoint

### 2. `docker/pxt-yotta/Makefile`
- **Image name**: `pext/yotta`. `make build` produces **only** `:latest`
  and *pulls* `:gcc5` — the two tags are different images (see below)
- **Build commands**: `build`, `rebuild`, `pull-gcc5`, `clean`,
  `clean-gcc5`, `push`, `info`, `test`, `help`
- Provides convenient interface for Docker container management

### 3. `docker/pxt-yotta/README.md`
- Comprehensive documentation for the Docker setup
- Usage instructions and troubleshooting guide
- Integration details with PXT build system

## Docker Container Specifications

- **Base Image**: Ubuntu 20.04 LTS
- **Build Tools**: cmake, ninja-build, gcc-arm-none-eabi, git, srecord
- **Python**: 3 (Ubuntu 20.04 default), yotta installed via `pip3`

## Usage

### Building the Container
```bash
cd docker/pxt-yotta
make build
```

`make build` produces `pext/yotta:latest` and pulls the upstream
`pext/yotta:gcc5`. They are **not** the same image: `:latest` is the
codal/V2 image built here, `:gcc5` is upstream's GCC 5.4.1 image for the
yotta/V1 variant, whose linker budget the newer GCC overflows. See
`README.md`.

### Testing the Container
```bash
make test
```

`yotta --version` alone is not a sufficient check — it passed on an
image whose `yotta build` was broken. `make test` also runs
`yotta build --help`, `node --version`, and `arm-none-eabi-gcc
-dumpversion`, and asserts `:gcc5` really is GCC 5.x.

### Using with PXT
The container is automatically used by PXT when building extensions
(in another repo) that contain C++ code, provided the consuming
target's own `pxtarget.json` names `pext/yotta:latest` or
`pext/yotta:gcc5` as `compileService.dockerImage` — PXT finds the
local image by tag with no further configuration needed in this repo.

## Benefits

1. **Stability**: No longer dependent on deprecated `pext/yotta` image
2. **Compatibility**: Ubuntu 20.04 provides better compatibility with yotta
3. **Maintainability**: Full control over the build environment
4. **Reproducibility**: Consistent builds across different environments
5. **Future-proof**: Can be updated and maintained as needed

## Integration

The setup integrates seamlessly with existing PXT workflows:
- `pxt build` in a consuming repo automatically uses the container once
  built locally under the tag its target requests
- No changes required to existing build scripts
- Compatible with existing PXT project structure
- Works with both local and remote builds

## Testing Results

- Docker container builds successfully
- Yotta installed and functional inside the built image — verified via
  `yotta build --help`, which loads the jinja2 import chain, not just
  `yotta --version`
- All build dependencies present, including `node` (PXT runs
  `node prepYotta.js` before `yotta build`)
- `pext/yotta:latest` built here; `pext/yotta:gcc5` pulled from upstream
  and asserted to be GCC 5.x
- Verified end to end on 2026-08-25 against `pxt-nezha-diffdrive` with
  `PXT_FORCE_LOCAL=1`: the codal/V2 variant builds a complete
  `mbcodal-binary.hex` locally through `:latest`, and the yotta/V1
  variant compiles and links through upstream `:gcc5`

See sprint 006's ticket record in this repo's `clasi/` history for the
exact build/smoke-test commands and their output as run against this
repo's checkout.

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
- **Image name**: `pext/yotta`, dual-tagged `latest` and `gcc5` from a
  single `make build`
- **Build commands**: `build`, `rebuild`, `clean`, `push`, `info`,
  `test`, `help`
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

A single `make build` produces an image tagged both
`pext/yotta:latest` and `pext/yotta:gcc5`.

### Testing the Container
```bash
make test
docker run --rm --entrypoint="" pext/yotta:latest yotta --version
docker run --rm --entrypoint="" pext/yotta:gcc5 yotta --version
```

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
- Yotta installed and functional inside the built image
- All build dependencies present
- Both `pext/yotta:latest` and `pext/yotta:gcc5` tags produced from one
  `make build`
- Documentation corrected: base image, tag names

See sprint 006's ticket record in this repo's `clasi/` history for the
exact build/smoke-test commands and their output as run against this
repo's checkout.

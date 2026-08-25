# Dockerfile -- micro:bit CODAL cross-compile check (sprint 005 ticket 001,
# "Port Docker micro:bit C++ Cross-Compiler from Elite").
#
# Ported from radio-robot-elite's own Dockerfile (Ubuntu 18.04 builder +
# gcc-arm-embedded via the team-gcc-arm-embedded PPA + `python3 build.py` +
# a `FROM scratch` export stage), adapted so the CODAL build compiles THIS
# repo's src/protocol, src/diffdrive, src/adapter -- via the thin scaffold
# at firmware/microbit/ -- instead of Elite's own robot application.
# src/archive/ is deliberately never copied (read-only provenance, sprint
# 005 sprint.md "What Changed").
#
# Architecture Decision 2 (toolchain provisioning): the team-gcc-arm-
# embedded PPA was verified against ubuntu:18.04 as the first concrete
# step of ticket 001 -- `add-apt-repository -y ppa:team-gcc-arm-embedded/
# ppa && apt-get update` resolves, and `apt-get install gcc-arm-embedded`
# installs cleanly (gcc-arm-embedded 7-2018q2-1, both amd64 and arm64).
# The PPA path is used as-is; the pinned ARM GNU toolchain tarball
# fallback named in sprint.md's Design Rationale was not needed.
#
# Architecture Decision 3: the three library modules are bridged into the
# scaffold's application source tree (firmware/microbit/app/) via plain
# COPY below, not a custom CMake include path -- CMakeLists.txt's existing
# recursive glob over CODAL_APP_SOURCE_DIR already picks them up.
FROM ubuntu:18.04 AS builder

RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
      software-properties-common ca-certificates && \
    add-apt-repository -y ppa:team-gcc-arm-embedded/ppa && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
      git make cmake python3 \
      gcc-arm-embedded && \
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/microbit-tools

# The CODAL scaffold: codal.json, CMakeLists.txt, build.py, and the
# vendored CODAL/yotta build tooling under src/utils/.
COPY firmware/microbit/ .

# Decision 3: bridge this repo's three library modules into the
# scaffold's application source tree. src/archive/ is not copied.
COPY src/protocol app/protocol
COPY src/diffdrive app/diffdrive
COPY src/adapter app/adapter

# Clones codal-microbit-v2 (and its own codal-core/codal-nrf52/
# codal-microbit-nrf5sdk dependencies) via CMake's INSTALL_DEPENDENCY,
# then configures and builds -- producing MICROBIT.hex/MICROBIT.bin at
# the scaffold root (CODAL_APP_OUTPUT_DIR default ".").
RUN python3 build.py

FROM scratch AS export-stage
COPY --from=builder /opt/microbit-tools/MICROBIT.bin .
COPY --from=builder /opt/microbit-tools/MICROBIT.hex .

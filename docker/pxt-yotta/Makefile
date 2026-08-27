# Makefile for building custom yotta Docker container
# Replaces the deprecated pext/yotta image

# Docker image name and tags. The two tags are DIFFERENT IMAGES and must
# not be built from one Dockerfile -- that was the original design here
# and it is wrong. pxt-microbit asks for a different image per variant:
#
#   mbcodal (micro:bit V2, codal engine) -> pext/yotta:latest
#   mbdal   (micro:bit V1, yotta engine) -> pext/yotta:gcc5
#
# Only :latest is ours to build. The V1/yotta target's linker budget is
# calibrated against the GCC 5.4 toolchain the `gcc5` tag NAMES, and this
# Dockerfile (Ubuntu 20.04) ships GCC 9.2.1. Measured against
# pxt-nezha-diffdrive on 2026-08-25: building the V1 variant with the
# GCC 9 image overflows the NRF51822 FLASH region by 5648 bytes and the
# build dies at link; the upstream GCC 5.4.1 image links the exact same
# sources cleanly. So :gcc5 is PULLED from Docker Hub, not built.
IMAGE_NAME := pext/yotta
IMAGE_TAG_LATEST := latest
IMAGE_TAG_GCC5 := gcc5
FULL_IMAGE_NAME := $(IMAGE_NAME):$(IMAGE_TAG_LATEST)
FULL_IMAGE_NAME_GCC5 := $(IMAGE_NAME):$(IMAGE_TAG_GCC5)

# Default target
.PHONY: all
all: build

# Build :latest (the codal/V2 image, which IS ours) and make sure the
# upstream :gcc5 (the yotta/V1 image, which is NOT) is present. Both
# have to exist locally or PXT silently falls back to the cloud compiler
# for the variant whose tag is missing.
.PHONY: build
build: pull-gcc5
	@echo "Building Docker image: $(FULL_IMAGE_NAME)"
	docker build -t $(FULL_IMAGE_NAME) .
	@echo "Docker image built successfully: $(FULL_IMAGE_NAME)"

# Fetch the upstream GCC 5.4.1 image for the V1/yotta variant. It is
# amd64-only, so on Apple Silicon it runs under emulation -- slower, but
# it is the toolchain the V1 link budget assumes, and correctness beats
# speed on a variant whose hex is discarded anyway. Never `docker build`
# over this tag.
.PHONY: pull-gcc5
pull-gcc5:
	@if docker image inspect $(FULL_IMAGE_NAME_GCC5) > /dev/null 2>&1; then \
		echo "Present: $(FULL_IMAGE_NAME_GCC5) ($$(docker image inspect $(FULL_IMAGE_NAME_GCC5) --format '{{.Architecture}}'))"; \
	else \
		echo "Pulling upstream $(FULL_IMAGE_NAME_GCC5) (amd64)"; \
		docker pull --platform linux/amd64 $(FULL_IMAGE_NAME_GCC5); \
	fi

# Build with no cache (force rebuild), still dual-tagged
.PHONY: rebuild
rebuild: pull-gcc5
	@echo "Rebuilding Docker image with no cache: $(FULL_IMAGE_NAME)"
	docker build --no-cache -t $(FULL_IMAGE_NAME) .
	@echo "Docker image rebuilt successfully: $(FULL_IMAGE_NAME)"

# Push both tags to a registry (optional)
# Only :latest is ours to publish. :gcc5 is upstream's image, pulled --
# pushing it back would republish someone else's bits under our build.
.PHONY: push
push: build
	@echo "Pushing Docker image: $(FULL_IMAGE_NAME)"
	docker push $(FULL_IMAGE_NAME)
	@echo "Docker image pushed successfully"

# Remove both tagged images
# `clean` removes only the image this Dockerfile produces. The upstream
# :gcc5 image is a separate, slow (emulated, ~300MB) pull and is not
# thrown away by a routine clean -- use `clean-gcc5` deliberately.
.PHONY: clean
clean:
	@echo "Removing Docker image: $(FULL_IMAGE_NAME)"
	docker rmi $(FULL_IMAGE_NAME) || true
	@echo "Docker image removed (kept $(FULL_IMAGE_NAME_GCC5); use clean-gcc5)"

.PHONY: clean-gcc5
clean-gcc5:
	@echo "Removing upstream image: $(FULL_IMAGE_NAME_GCC5)"
	docker rmi $(FULL_IMAGE_NAME_GCC5) || true

# Show image info for both tags
.PHONY: info
info:
	@echo "Docker images: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"
	@docker images | grep "$(IMAGE_NAME)" || echo "Image not found locally"

# Test the container by running it. Exercises :latest and :gcc5
# separately so neither tag is left unverified by any Make target.
#
# `yotta --version` is NOT sufficient and must never be the whole test:
# it does not import the build subcommand, so it passed cleanly on an
# image whose `yotta build` died instantly on a MarkupSafe/Jinja2
# ImportError (see the pip note in the Dockerfile). `yotta build --help`
# forces argparse's lazy load of yotta.build -> yotta.lib.cmakegen ->
# jinja2, which is the exact import chain that broke. `node --version`
# covers the other silent failure: PXT runs `node prepYotta.js` before
# `yotta build`, separated by `;`, so a missing node never fails the
# build -- it just drops GITHUB_ACCESS_TOKEN on the floor.
.PHONY: test
test: build
	@for img in $(FULL_IMAGE_NAME) $(FULL_IMAGE_NAME_GCC5); do \
		echo "Testing Docker container: $$img"; \
		docker run --rm --entrypoint="" $$img yotta --version || exit 1; \
		docker run --rm --entrypoint="" $$img yotta build --help > /dev/null \
			|| { echo "FAIL: $$img cannot load the yotta build subcommand"; exit 1; }; \
		docker run --rm --entrypoint="" $$img bash -lc 'node --version' \
			|| { echo "FAIL: $$img has no node; prepYotta.js would be skipped"; exit 1; }; \
		docker run --rm --entrypoint="" $$img bash -lc 'arm-none-eabi-gcc -dumpversion' \
			|| { echo "FAIL: $$img has no ARM toolchain"; exit 1; }; \
	done
	@# The tag must not lie about its toolchain. A :gcc5 tag carrying a
	@# newer GCC builds and smoke-tests clean, then overflows the V1
	@# NRF51822 FLASH region at link time in a consuming project -- which
	@# reads as "my code got too big", not "my container is wrong".
	@gccver=$$(docker run --rm --entrypoint="" $(FULL_IMAGE_NAME_GCC5) \
		bash -lc 'arm-none-eabi-gcc -dumpversion'); \
	case "$$gccver" in \
		5.*) echo "OK: $(FULL_IMAGE_NAME_GCC5) really is GCC $$gccver" ;; \
		*) echo "FAIL: $(FULL_IMAGE_NAME_GCC5) is GCC $$gccver, not 5.x --"; \
		   echo "      the V1 link budget assumes GCC 5.4; re-pull upstream"; \
		   echo "      with 'make clean-gcc5 pull-gcc5'."; exit 1 ;; \
	esac
	@echo "All container smoke tests passed."

# Help target
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  build      - Build :latest (ours) and ensure :gcc5 (upstream) is pulled"
	@echo "  rebuild    - Build :latest with no cache"
	@echo "  pull-gcc5  - Pull the upstream GCC 5.4.1 image for the V1 variant"
	@echo "  push       - Push :latest to registry (never :gcc5 -- upstream's)"
	@echo "  clean      - Remove :latest (keeps the slow :gcc5 pull)"
	@echo "  clean-gcc5 - Remove the upstream :gcc5 image"
	@echo "  info       - Show image information"
	@echo "  test       - Smoke-test both tags, incl. the :gcc5 GCC-version check"
	@echo "  help     - Show this help message"

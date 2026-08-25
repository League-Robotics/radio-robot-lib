# Makefile for building custom yotta Docker container
# Replaces the deprecated pext/yotta image

# Docker image name and tags. A single `make build` produces BOTH tags
# from one image so that whichever tag a given pxt-microbit target's
# pxtarget.json requests (compileService.dockerImage), PXT finds a local
# image and never falls back to the MakeCode cloud compiler.
IMAGE_NAME := pext/yotta
IMAGE_TAG_LATEST := latest
IMAGE_TAG_GCC5 := gcc5
FULL_IMAGE_NAME := $(IMAGE_NAME):$(IMAGE_TAG_LATEST)
FULL_IMAGE_NAME_GCC5 := $(IMAGE_NAME):$(IMAGE_TAG_GCC5)

# Default target
.PHONY: all
all: build

# Build the Docker image and tag it both :latest and :gcc5 from a
# single build (not a separate `docker tag` step run by hand) so the
# dual tag is enforced by the tool, not by documentation.
.PHONY: build
build:
	@echo "Building Docker image: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"
	docker build -t $(FULL_IMAGE_NAME) -t $(FULL_IMAGE_NAME_GCC5) .
	@echo "Docker image built successfully: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"

# Build with no cache (force rebuild), still dual-tagged
.PHONY: rebuild
rebuild:
	@echo "Rebuilding Docker image with no cache: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"
	docker build --no-cache -t $(FULL_IMAGE_NAME) -t $(FULL_IMAGE_NAME_GCC5) .
	@echo "Docker image rebuilt successfully: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"

# Push both tags to a registry (optional)
.PHONY: push
push: build
	@echo "Pushing Docker images: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"
	docker push $(FULL_IMAGE_NAME)
	docker push $(FULL_IMAGE_NAME_GCC5)
	@echo "Docker images pushed successfully"

# Remove both tagged images
.PHONY: clean
clean:
	@echo "Removing Docker images: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"
	docker rmi $(FULL_IMAGE_NAME) || true
	docker rmi $(FULL_IMAGE_NAME_GCC5) || true
	@echo "Docker images removed"

# Show image info for both tags
.PHONY: info
info:
	@echo "Docker images: $(FULL_IMAGE_NAME) and $(FULL_IMAGE_NAME_GCC5)"
	@docker images | grep "$(IMAGE_NAME)" || echo "Image not found locally"

# Test the container by running it. Exercises :latest and :gcc5
# separately so neither tag is left unverified by any Make target.
.PHONY: test
test: build
	@echo "Testing Docker container: $(FULL_IMAGE_NAME)"
	docker run --rm --entrypoint="" $(FULL_IMAGE_NAME) yotta --version
	@echo "Testing Docker container: $(FULL_IMAGE_NAME_GCC5)"
	docker run --rm --entrypoint="" $(FULL_IMAGE_NAME_GCC5) yotta --version

# Help target
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  build    - Build the Docker image, tagged both :latest and :gcc5"
	@echo "  rebuild  - Build the Docker image with no cache, both tags"
	@echo "  push     - Push both tags to registry"
	@echo "  clean    - Remove both tagged images"
	@echo "  info     - Show image information"
	@echo "  test     - Test the container under both tags"
	@echo "  help     - Show this help message"

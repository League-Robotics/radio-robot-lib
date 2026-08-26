FROM ubuntu:20.04

# Set non-interactive frontend to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive


# Update package list and install dependencies
# nodejs is NOT optional: PXT invokes the container as
# `node prepYotta.js; yotta build`, and prepYotta.js is what writes
# ~/.yotta/config.json and ~/.netrc from GITHUB_ACCESS_TOKEN. Without
# node the shell prints "node: command not found", the `;` carries on to
# `yotta build` regardless, and the token is silently never applied --
# so authenticated/rate-limited GitHub fetches fail for a reason the
# build log does not name.
RUN apt-get update && apt-get install -y \
    python3 python3-pip git build-essential cmake ninja-build srecord \
    gcc-arm-none-eabi binutils-arm-none-eabi nodejs \
    wget curl sudo python3-dev libssl-dev libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# yotta 0.20.5 pins Jinja2 2.x but not MarkupSafe, and MarkupSafe 2.1
# deleted `soft_unicode`, which jinja2/filters.py imports at module
# scope. pip resolves the unpinned dependency to 2.1.x, so the image
# builds clean and `yotta --version` works -- but `yotta build` dies in
# argparse's lazy load of the build subcommand:
#   ImportError: cannot import name 'soft_unicode' from 'markupsafe'
# Pinning MarkupSafe below 2.1 is what makes the compiler usable. The
# downgrade is a separate step so it is applied AFTER yotta's own
# resolution rather than racing it.
RUN pip3 install yotta && pip3 install "markupsafe<2.1"

ENV CC=arm-none-eabi-gcc
ENV CXX=arm-none-eabi-g++

# Create build user with sudo privileges
RUN useradd -m -s /bin/bash build && \
    echo "build ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to build user
USER build
WORKDIR /src

# Create a symbolic link from python to python3
RUN sudo ln -sf /usr/bin/python3 /usr/bin/python


CMD ["python3", "build.py"]

FROM ubuntu:20.04

# Set non-interactive frontend to avoid prompts during build
ENV DEBIAN_FRONTEND=noninteractive


# Update package list and install dependencies
RUN apt-get update && apt-get install -y \
    python3 python3-pip git build-essential cmake ninja-build srecord \
    gcc-arm-none-eabi binutils-arm-none-eabi \
    wget curl sudo python3-dev libssl-dev libffi-dev \
    && rm -rf /var/lib/apt/lists/*

# Try installing yotta with older Python/setuptools from Ubuntu 20.04
RUN pip3 install yotta

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

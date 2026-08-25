#!/usr/bin/env python3
"""build.py -- minimal CODAL build driver for this scaffold.

radio-robot-elite's own build.py (which this is ported from, sprint 005
ticket 001) is a large wrapper around the stock CODAL build.py: on top of
the same underlying CMake configure+build step, it also regenerates
protobuf message headers, bumps a project version, and builds a host-
simulation library -- none of which apply to this repo's library sources
or this scaffold's job (proving src/protocol, src/diffdrive, src/adapter
cross-compile for the micro:bit target). Architecture Decision 1 ("thin
scaffold + copy-in, not an application port") applies to build.py exactly
as it does to main.cpp: this file does only the one thing the scaffold
needs -- configure and build against codal.json's target -- by calling the
same codal_utils.build() Elite's own build.py ultimately calls for that
step.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _ROOT)

from src.utils.python.codal_utils import build  # noqa: E402

if not os.path.exists(os.path.join(_ROOT, "codal.json")):
    print("No target specified in codal.json, does codal.json exist?")
    sys.exit(1)

build_dir = os.path.join(_ROOT, "build")
if not os.path.exists(build_dir):
    os.mkdir(build_dir)

os.chdir(build_dir)
build(clean=False, verbose=False,
      parallelism=int(os.environ.get("BUILD_PARALLELISM", "4")))

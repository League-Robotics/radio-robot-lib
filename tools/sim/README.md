# tools/sim -- the compiled host version of the firmware

`sim_main.cpp` composes the real `Protocol::ProtocolHandler`
(`src/protocol/`) with `Protocol::FakeMotionAdapter`
(`tests/protocol/fake_motion_adapter.h`) into a standalone executable
that speaks protocol-v6 on a socket, a pipe, or stdio -- no robot, no
serial port, no CMake. This is a **host development tool**, not
firmware-targetable library code: it may use the full standard library
freely, unlike `src/protocol/`'s own no-allocation/no-`std::string`/
no-exception constraints, which this file does not relax anywhere else
in the tree -- it only *links against* `protocol_handler.cpp` unmodified.

## Build

Same `/usr/bin/c++` + explicit include-path pattern the test suite
already uses (see `tests/protocol/test_protocol_harness.py`'s own
`_compile_shared_lib`) -- no CMake, no build system:

```bash
/usr/bin/c++ -std=c++20 -O2 -Wall -Wextra \
  -I src/protocol -I tests/protocol \
  tools/sim/sim_main.cpp src/protocol/protocol_handler.cpp \
  -o /tmp/robot_sim
```

## Run

Two transport modes, identical line protocol on both:

```bash
# stdio -- preferred for tests and for piping through another program.
# No port allocation, no bind races, no leaked listener process.
/tmp/robot_sim --stdio

# TCP -- one client at a time; accepts a new one after the previous
# disconnects, so a dev session doesn't need to relaunch the process.
/tmp/robot_sim --listen 127.0.0.1:7654
```

`--period MS` (default 24) sets how often the simulated motion advances
one `step()` and telemetry (`thdr`/`t` plus the ack/nack reliability
piggyback) is emitted -- lower it in a test to make a multi-step move
complete sooner in wall-clock time.

```
$ /tmp/robot_sim --stdio
device NEZHA2 robot sim SIMHOST0001
WHEELS_V 100 100 500 #1
ack 1 0 none
thdr active kind id stepsleft
t 1 2 1 2
ack 1 0 none
t 1 2 1 1
ack 1 0 none
t 0 0 0 0
ack 1 1 stop
```

Ctrl-C (SIGINT), SIGTERM, or closing the input stream (EOF -- e.g. the
peer disconnects, or a test closes its subprocess's stdin) all shut a
session down the same way: a synthetic `ESTOP` is fed through the same
handler before the process exits, so whatever motion `FakeMotionAdapter`
still had "active" is stopped through the real wire verb (and a client
still listening sees the real `estop` confirmation line) rather than
the process just disappearing mid-move.

## Identity

The banner and `ID`/`VER` report a fixed identity (name `sim`, serial
`SIMHOST0001`, drivetrain `differential`, profile `sim`, version
`6.0.0`) -- there is exactly one adapter identity in this tool, no
`--name`/`--serial` flags, because nothing yet needs more than one
simulated robot per process. Add flags here if that changes.

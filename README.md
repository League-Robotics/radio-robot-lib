# radio-robot-lib

Common library code for the robot, shared by the deployment environments that
actually ship it — a MicroPython image in one repo, a JavaScript/MakeCode
package in another. Nothing in here is an application. Nothing in here talks to
a specific board.

Two things live here, and they are deliberately independent:

| | |
|---|---|
| **DiffDrive** | a self-contained differential-drive wheel kernel — the control law, four small ports, no chassis geometry, no sensors but the wheel encoders |
| **Protocol** | a protocol-v6 handler — one ASCII line grammar, parsed by a handler class, dispatched to an adapter class you implement |

They meet in exactly one place: an adapter that turns `WHEELS_V …` into
`DifferentialDrive::drive(...)`. That seam is the whole point of the repo, and
it is the thing this library exists to let us test.

A third thing lives here too, one layer up: **`robot_v6`**, a Python
protocol-v6 HOST client (the codec, a `Transport` abstraction over TCP/pipe/
serial, and the caller-side half of the reliability layer), plus **`tools/sim`**,
a compiled host binary that speaks the real wire grammar with no robot
attached — together these make the host side of this protocol testable
end to end with no hardware at all. See
[src/host/robot_v6/](src/host/robot_v6/) and
[tools/sim/README.md](tools/sim/README.md).

## Layout

```
src/
  archive/            reference copies from radio-robot-elite -- READ ONLY
    diffdrive/          the kernel + its fidelity gate, as extracted
    protocol-v6/        generated v6 tables + the generator (historical)
    protocol-v5-ref/    how parsing/config/telemetry are done today
  host/
    robot_v6/          the Python v6 HOST client -- codec, Transport, Session
tools/
  sim/                a compiled host binary speaking v6 with no robot attached
docs/
  design/
    diffdrive.md          what the kernel is and what it demands of a caller
    protocol.md            the wire grammar + the handler/adapter design
    motion-api.md          the layer above the kernel: six motion operations
```

`src/archive/` is provenance, not product. It is there so the new code can be
checked against the thing it descends from without a second checkout. **Nothing
in `src/archive/` is compiled or imported by the library.**

## Decisions

| | |
|---|---|
| **Language** | C++ for both libraries, exercised from Python through a `ctypes` shim |
| **Test structure** | two **independent** harnesses — protocol, and diffdrive — before anything is linked |
| **Configuration** | no storage in either library; each carries only its own configuration type. A config system may come later, separately |
| **Control law** | this repo is authoritative; `radio-robot-elite`'s two copies are deprecated |

## Status

**Built and tested.** Run the suite from the repo root:

```
uv run python -m pytest -q
```

| package | what | proves |
|---|---|---|
| DiffDrive | the kernel, lifted with its fidelity gate | byte-identical to what it was extracted from; held duty-for-duty against `golden_ref_drive` |
| Protocol | the handler -- feed()/dispatch/reply formatting | golden-vector conformance, adversarial input under ASan+UBSan, chunk-split equivalence |
| Adapter | `DiffDriveAdapter`, the link between them | end to end through bytes, no robot, no serial port |
| `robot_v6` | the Python v6 host client — codec, `Transport` (socket/pipe/stdio/serial), `Session` (sequencing, pipelining, cumulative ack/nack, resend-on-nack) | unit tests plus a lossy-transport square-tour scenario, and end to end against the real `tools/sim` binary over `--stdio` |

The acceptance runs with no robot and no serial port:

```
feed("WHEELS_V 100 100 1000 #1\n") ->  ack 1 0 none
step the kernel                    ->  t frames, counts climbing
step past 1000 ms                  ->  wheels at zero, lease expired
feed("ESTOP\n")                    ->  estop
```

The wire is protocol-v6 plus a reliability layer: every sequenced command
carries a mandatory, strictly incrementing `#<id>`, acknowledged
cumulatively (`ack <id> <lastDone> <reason>` / `nack <next> <lastDone>
<reason>`), and a decode failure NAKs and holds the sequence in place
rather than silently advancing past a garbled line. `WHEELS`/`onWheels`
was renamed `WHEELS_V`/`onWheelsV`, joined by five more motion verbs
(`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`) that decode and
dispatch correctly but have no real effect on `DiffDriveAdapter` (no
planner). See [docs/design/protocol.md](docs/design/protocol.md) §8-§9 for
the full reasoning, the resolved ambiguities, and the known gaps.

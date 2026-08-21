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

They meet in exactly one place: an adapter that turns `WHEELS …` into
`DifferentialDrive::drive(...)`. That seam is the whole point of the repo, and
it is the thing this library exists to let us test.

## Layout

```
src/
  archive/            reference copies from radio-robot-elite -- READ ONLY
    diffdrive/          the kernel + its fidelity gate, as extracted
    protocol-v6/        generated v6 tables + the generator (historical)
    protocol-v5-ref/    how parsing/config/telemetry are done today
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

The acceptance runs with no robot and no serial port:

```
feed("WHEELS 100 100 1000 #5\n")  ->  ok #5
step the kernel                    ->  t frames, counts climbing
step past 1000 ms                  ->  wheels at zero, lease expired
feed("ESTOP\n")                    ->  latched zero, no ack
```

`done` for `WHEELS` was settled: **no**. The handler stays a pure function of
the bytes fed to it -- no clock, no pending-id table, no state between calls
beyond a partial line. See [docs/design/protocol.md](docs/design/protocol.md)
§8-§9 for the reasoning and the full list of known gaps.

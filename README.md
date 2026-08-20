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

They meet in exactly one place: an adapter that turns `WHEELS:…` into
`DifferentialDrive::drive(...)`. That seam is the whole point of the repo, and
it is the thing this library exists to let us test.

## Layout

```
src/
  archive/            reference copies from radio-robot-elite -- READ ONLY
    diffdrive/          the kernel + its fidelity gate, as extracted
    protocol-v6/        generated v6 tables + the generator (sprint 137 work)
    protocol-v5-ref/    how parsing/config/telemetry are done today
docs/
  protocol-v6-spec.md       the wire specification
  protocol-v6-rationale.md  why v6 looks like this (the v5 simplification review)
  design/
    diffdrive.md            what the kernel is and what it demands of a caller
    protocol.md             the handler/adapter design -- the new code
  plan.md                   how these get linked into a testable library
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

Greenfield. `src/archive/` and `docs/` are populated; **no library code is
written yet.** [docs/plan.md](docs/plan.md) has the four-step sequence.

One design question is still open —
[docs/design/protocol.md](docs/design/protocol.md) §8, whether `WHEELS` emits
`done:` on lease expiry. It decides whether the handler is a pure function of
its input bytes or a thing with a clock and pending state, so it is worth
settling before step 3. It does not block steps 1 or 2.

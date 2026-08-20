# Plan — from archive to a library we can actually test

The goal is narrow and worth stating up front: **feed
`WHEELS:100:100:1000:5` in as a block of bytes and watch encoder counts climb,
then watch the wheels stop when the lease expires.** Everything below exists to
make that one sentence true, verifiably, with no robot attached.

Four steps. Each ends with something runnable — no step leaves the repo in a
state where the only evidence is that it compiled.

---

## Settled decisions (2026-08-20)

These were open in the first draft and are now closed. They are recorded here
so no step re-opens them:

| | |
|---|---|
| **Language** | C++ for both libraries, exercised from Python through a `ctypes` shim |
| **Test structure** | two **independent** harnesses — protocol, and diffdrive — before anything is linked |
| **Configuration storage** | none, in either library. Not core work; may become its own system later |
| **Config table** | the 80-row v6 table does not come across; each library carries only its own configuration type |
| **Authoritative control law** | this repo. `radio-robot-elite`'s two copies are deprecated |

One question remains, and it is small but shapes the handler:
[protocol.md](design/protocol.md) §8 — whether `WHEELS` emits `done:` on lease
expiry. It decides whether the handler is a pure function of its input bytes or
a thing with a clock and pending state. **Recommendation: no `done` for
`WHEELS`.** It does not block step 1 or 2.

---

## Proposed layout

```
src/
  diffdrive/            differential_drive.{h,cpp}      -- the kernel
  protocol/             protocol_handler.{h,cpp}, adapter.h
  archive/              read-only reference (unchanged)
tests/
  diffdrive/            fake ports, ctypes shim, python tests, fidelity gate
  protocol/             mock adapter, ctypes shim, python tests, golden vectors
```

Each `tests/*/` directory owns its own `extern "C"` shim. The shims are test
scaffolding, not library API — nothing in `src/` knows they exist.

---

## Step 1 — lift DiffDrive, unchanged, with its gate

Move `src/archive/diffdrive/` into `src/diffdrive/` and `tests/diffdrive/`.
Same namespace, same includes, **no edits to the law**.

Done when:
- the package compiles with an include path of **exactly its own directory** —
  a firmware include creeping in fails the build;
- the fidelity gate passes: the kernel held duty-for-duty against
  `golden_ref_drive` (feedforward exact, closed loop equal at steady state).

That gate is the entire justification for calling this "the same control law
that drives the robot", and this repo is now its authoritative home — so it
runs from day one, not at the end.

---

## Step 2 — the diffdrive harness: fake ports, ctypes, Python

Implement `Motor`, `Clock`, `Sleeper` as deterministic fakes and **decline
`FiberLauncher`** — drive the kernel by calling `step()` from the test's own
loop ([diffdrive.md](design/diffdrive.md) §2). Single-threaded and repeatable
beats realistic here.

Then the `extern "C"` shim: construct the kernel over the fakes, `drive()`,
`step()`, and read the `Output` snapshot out field by field so Python can
assert on it.

The fake `Motor` must honour the two obligations that are easy to miss and
expensive to debug ([diffdrive.md](design/diffdrive.md) §2.1): `sampleTime()`
stamps on collect **success only**, and `rebaseline()` issues no bus traffic. A
fake that stamps unconditionally makes every later bus-health assertion
vacuous.

Done when: a Python test commands `drive()`, steps the kernel, and sees
position accumulate, velocity settle, and **lease expiry stop the wheels** —
measured at the fake motor, not inferred from the kernel's own flags. No
protocol involved.

---

## Step 3 — the protocol harness: handler, mock adapter, ctypes, Python

`ProtocolHandler` on its own: line reassembly across arbitrary byte blocks,
split-in-place parsing, arity checks, reply formatting, delegation to the
adapter. No kernel, no motors, no config storage.

Its shim constructs a handler over a recording `Sink` and a mock adapter, feeds
a byte block, and lets Python read back both what the sink captured and which
adapter methods fired with which arguments.

Cover the cases from [protocol.md](design/protocol.md) §2.1 explicitly, because
a tidy bench never produces them:
- several lines in one block; a block ending mid-line; a fragment alone;
- an over-long line discarded to the next terminator, **not** truncated into
  something that still parses as a valid command;
- a **lowercase** inbound verb dropped silently and **not** counted malformed
  (this is the structural fix for the shared-channel reply flood — it needs a
  test, not a comment).

**The golden-vector fixture is written here, alongside the parser**, not after
it. It is the cross-language contract (spec §11.3) and it is worth nothing if
it is authored to match an implementation that already exists.

Done when: the handler round-trips every vector byte-for-byte from Python.

---

## Step 4 — link them

Only now. The `DiffDriveAdapter` that closes the seam the repo exists for:
`WHEELS` → mm/s → counts/s → `(velocity, twist)` → `drive(..., lease=duration)`,
plus `STOP`/`ESTOP` and `TLM` projecting `Output` into `thdr:`/`t:`.

This is where the robot's geometry (`countsPerLength`) enters, and it enters in
the adapter — not in either library.

The acceptance:

```
feed("WHEELS:100:100:1000:5\n")   →  sink contains "ok:5"
step the kernel                    →  t: frames show counts climbing
step past 1000 ms                  →  wheels at zero, lease expired
feed("ESTOP\n")                    →  latched zero, no ack (by design)
```

Plus the **twist-sign test** that fails if the two wheels are swapped — a bug
this project has shipped and then patched four times downstream before finding
it ([protocol.md](design/protocol.md) §4).

Done when that runs green from a single test command, with no robot and no
serial port.

---

## What this plan deliberately does not do

- **No `MOVE`, `GOTO`, `SEED`, or `CAL`.** They need a planner, a navigator, or
  odometry — none of which belong in a wheel-kernel library
  ([protocol.md](design/protocol.md) §5).
- **No configuration system.** Settled above.
- **No transport.** `feed()` takes bytes and `Sink` returns them. Serial, radio
  and UDP belong to the applications, which is what makes this testable without
  hardware.
- **No second-language implementation yet.** The golden vectors exist from step
  3 so that a MicroPython or JavaScript handler can be verified when one is
  wanted; writing one is not part of this plan.
- **No hardware bring-up.** Real motors and the radio relay come after this
  library is green, and belong to whichever deployment repo consumes it.

## Cost

Step 3 is the largest single piece of new code and is still small — the spec's
own claim is that the whole codec is a split on `':'` plus a verb table. If any
step starts sprawling, that is evidence the design has taken on something that
belongs in an application, and the right response is to cut it, not to schedule
it.

# Plan — from archive to a library we can actually test

The goal is narrow and worth stating up front: **feed `WHEELS:100:100:1000:5`
in as a block of bytes and watch encoder counts climb, then watch the wheels
stop when the lease expires.** Everything below exists to make that one
sentence true, verifiably, with no robot attached.

Six steps. Each ends with something runnable — no step leaves the repo in a
state where the only evidence is that it compiled.

---

## Step 0 — settle the open questions (blocking, no code)

[protocol.md](design/protocol.md) §7 has four, and the first one —
**what language is the library written in** — decides the build system, the
directory layout, and whether there is a binding layer at all. It cannot be
deferred past this point.

[diffdrive.md](design/diffdrive.md) §6 has one: which copy of the control law
becomes authoritative. That one *can* be deferred, but every week it is
deferred is another week of three-copy drift.

**Nothing else starts until these are answered.**

---

## Step 1 — lift DiffDrive, unchanged, with its gate

Move `src/archive/diffdrive/` into `src/diffdrive/` and `tests/diffdrive/`.
Same namespace, same includes, no edits to the law.

Done when:
- the package compiles with an include path of **exactly its own directory** —
  a firmware include creeping in fails the build;
- the fidelity gate passes: the kernel held duty-for-duty against
  `golden_ref_drive` (feedforward exact, closed loop equal at steady state).

That gate is the entire justification for calling this "the same control law
that drives the robot", so it runs from day one, not at the end.

---

## Step 2 — fake ports, and a kernel you can step

Implement `Motor`, `Clock`, `Sleeper` as deterministic fakes, and decline
`FiberLauncher` — drive the kernel by calling `step()` from the test's own
loop ([diffdrive.md](design/diffdrive.md) §2). Single-threaded and repeatable
beats realistic here.

The fake `Motor` must honour the two obligations that are easy to miss:
`sampleTime()` stamps on collect **success only**, and `rebaseline()` issues no
bus traffic. A fake that stamps unconditionally will make every later
bus-health assertion vacuous.

Done when: a test commands `drive()` and sees position accumulate, velocity
settle, and the lease expiry stop the wheels — **no protocol involved yet.**

---

## Step 3 — `ProtocolHandler`, against a mock adapter

The parsing half, on its own: line reassembly across arbitrary byte blocks,
split-in-place parsing, arity checks, reply formatting.

Test with a mock adapter that records calls and a `Sink` that records bytes —
no DiffDrive, no machine. Cover the cases from
[protocol.md](design/protocol.md) §2.1 explicitly, because they are the ones a
tidy bench never produces:
- several lines in one block; a block ending mid-line; a fragment alone;
- an over-long line discarded to the next terminator, not truncated into
  something that still parses;
- a **lowercase** inbound verb dropped silently and **not** counted malformed.

**The golden-vector fixture starts here**, not later. It is the cross-language
contract (spec §11.3), and it is worth nothing if it is written after the
implementation it is supposed to constrain.

Done when: the handler round-trips every vector byte-for-byte.

---

## Step 4 — the DiffDrive adapter

The seam the repo exists for. `WHEELS` → mm/s → counts/s → `(velocity, twist)`
→ `drive(..., lease=duration)`, plus `STOP`/`ESTOP`, `GET`/`SET` over the
`wheel_control.*` fields, and `TLM` projecting `Output` into `thdr:`/`t:`.

Two tests that must exist by name, because both failures are silent:
- **twist sign** — a test that fails if the two wheels are swapped. This
  project has shipped that bug and patched it four times downstream before
  finding it.
- **lease expiry** — commanded motion actually stops when the duration runs
  out, measured at the fake motor, not inferred from the kernel's own flags.

Done when: the adapter passes with the kernel from step 2 underneath it.

---

## Step 5 — end to end, through bytes

The acceptance:

```
feed("WHEELS:100:100:1000:5\n")   →  sink contains "ok:5"
step the kernel                    →  t: frames show counts climbing
step past 1000 ms                  →  wheels at zero, lease expired
feed("ESTOP\n")                    →  latched zero, no ack (by design)
```

Done when that runs green from a single test command, with no robot and no
serial port.

---

## Step 6 — the second implementation

Only meaningful once step 5 is green and the vectors exist. Whichever target
step 0 picked, the second language implements the same handler against the
**same fixture**, and the fixture is what proves they agree.

Spec §11.1 puts this at ~150 lines with zero dependencies. If it turns out to
be much more than that, the design has drifted from the spec and that is the
signal to stop and look, not to push through.

---

## What this plan deliberately does not do

- **No `MOVE`, `GOTO`, `SEED`, or `CAL`.** They need a planner, a navigator, or
  odometry — none of which belong in a wheel-kernel library.
  ([protocol.md](design/protocol.md) §5.)
- **No transport.** `feed()` takes bytes and `Sink` returns them. Serial,
  radio, and UDP are the applications' problem, which is what makes the library
  testable without hardware.
- **No hardware bring-up.** Real motors, a real robot, and the radio relay come
  after this library is green in simulation — and they belong to whichever
  deployment repo consumes it.

## Cost

Steps 1–5 are the library. Step 3 is the largest single piece of new code and
still small — the spec's own claim is that the whole codec is a split on `':'`
plus a verb table. If any step starts sprawling, that is evidence the design
took on something that belongs in an application, and the right response is to
cut it, not to schedule it.

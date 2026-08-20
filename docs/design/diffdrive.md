# DiffDrive — differential-drive wheel kernel

**Status:** design record for code being lifted, not invented. The control law
already exists and is already gated; this document says what it *is*, what it
demands of a caller, and what changes when it moves into this repo.

Source of truth for the extraction: `src/archive/diffdrive/` (1657 lines of
kernel, 1223 of gate). Provenance is in that directory's own `README.md` —
extracted 2026-08-18 from `src/firm/control/differential_drive.{h,cpp}`, the
law itself byte-identical, namespace and includes changed.

---

## 1. What it is

One class, `DiffDrive::DifferentialDrive`, owning two wheels end to end:

- the staged duty write schedule and split-phase encoder sampling,
- the `(velocity, twist)` control law — feedforward + Stage A wheel correction
  + fast PID + Stage C bias adaptation,
- stall, deficit and wedge latches,
- a **lease watchdog on every motion command**,
- an estop latch.

Its whole dependency list is `<cmath>`, `<cstdint>`, `<algorithm>`. It has its
own namespace and no inheritance relationship with any firmware header. That
independence is the reason it can move here at all, and it is worth defending:
the package gate compiles it with an include path of exactly its own directory,
so a firmware include creeping in fails immediately.

### 1.1 It speaks counts, not millimetres

The kernel's commands and outputs are in **native encoder counts** — `counts`,
`counts/s`. There are no millimetres in it, no track width, no wheel radius, no
chassis geometry of any kind.

This is not an oversight to be corrected. It is the boundary that keeps the
kernel portable: geometry is a property of a particular robot, and a wheel
control law is not. **Every mm ↔ counts conversion belongs above this class**,
in the adapter (see [protocol.md](protocol.md) §4).

---

## 2. The four ports

A caller implements four small interfaces, declared at the top of the header.
They are the package's own types, on purpose — the parent firmware connects
them to its HAL with one-line forwarding adapters; a MicroPython C module or a
MakeCode/PXT package implements them directly.

| port | methods | job |
|---|---|---|
| `Motor` | 13 | one wheel: stage duty, `tick()` to execute + collect, counts out, emergency stop, wedge reporting |
| `Clock` | 1 | `nowMicros()` — monotonic `[us]` |
| `Sleeper` | 2 | `sleepMillis()` settle/pace + `yield()` cooperative hand-off |
| `FiberLauncher` | 1 | start the kernel loop on its own thread — **optional** |

`FiberLauncher` is the one you can decline. If you would rather drive the
kernel from your own loop, call `step()` yourself and implement `launch()` to
fail loudly. For a first test harness that is the simpler choice: it makes the
whole thing single-threaded and deterministic.

### 2.1 `Motor` is the port with real obligations

Two of its 13 methods carry semantics that are easy to get wrong and expensive
to debug:

- **`sampleTime()` must stamp on collect SUCCESS only.** A failed collect
  *holds* the previous stamp. The kernel derives its `i2cFaultCount` purely
  from that stamp failing to advance — `requestSample()`/`tick()` return
  `void`, so stamp non-advance is the only observable "that collect did not
  land". A port that stamps unconditionally silently reports a healthy bus.
- **`rebaseline()` is a software re-anchor and must issue no bus traffic.**
  Encoders are never device-reset; position is accumulated and re-origined in
  software, and each rebaseline bumps that wheel's `positionEpoch` so a host
  can tell "the robot moved backwards" from "the origin moved".

---

## 3. The command surface a protocol adapter needs

Only a small part of the class is reachable from the wire, and for testing the
kernel it is smaller still:

```cpp
Status drive(float velocity, float twist, uint32_t lease);   // [counts/s] [counts/s] [ms]
Status driveDuty(float dutyLeft, float dutyRight, uint32_t lease);  // [%] [%] [ms]
void   neutral();      // commanded stop, through the full stop path
void   estop();        // latch: zero NOW; holds until estopClear()
void   estopClear();
Output output() const; // seq-consistent snapshot
```

`twist` is the measured/commanded **half-differential**, CCW-positive. So for
left/right wheel speeds:

```
velocity = (left + right) / 2
twist    = (right - left) / 2
```

### 3.1 The lease is the safety property, and it maps straight onto the wire

Every motion command carries a `lease` — a **duration in `[ms]` from now**,
clamped to `kLeaseMax`. When it expires the kernel stops. A dead caller cannot
mean a runaway.

This lines up exactly with protocol v6's `WHEELS:<left>:<right>:<duration>`,
where `duration` is a required `[ms]` field with a 5000 ceiling for precisely
the same reason. **`duration` becomes `lease` with no reinterpretation** — the
one place where the wire and the kernel already agree on a concept without
anyone having designed it that way.

That agreement is why `WHEELS` is the right verb to build the first end-to-end
test around, rather than `MOVE`.

### 3.2 `neutral()` and `estop()` are not synonyms

`neutral()` is a commanded stop through the full stop path. `estop()` latches
zero immediately and holds until `estopClear()`. The protocol keeps the same
distinction (`STOP` vs `ESTOP`), and the project has measured the difference on
hardware: a planned stop let a robot travel a full 400 mm leg, an estop stopped
it in 29 mm.

Any halt path — a test harness's Ctrl-C included — calls `estop()`.

---

## 4. The `Output` snapshot is the telemetry source

`output()` returns a seq-consistent snapshot with everything a `t:` telemetry
frame needs, already measured:

- **timing** — `now [ms]`, `nowFine [us]`, `cycleCount`, `cyclePeriodMeasured [us]`, `cycleBusy [us]`, `cycleOverrunCount`
- **per-wheel measurement** — `positionLeft/Right [counts]`, `velocityLeft/Right [counts/s]`, `sampleTimeLeft/Right [us]`, `positionEpochLeft/Right`
- **derived** — `velocity`, `twist` (both `[counts/s]`), `appliedDutyLeft/Right [%]`
- **learned state** — `lambda`, `biasLeft/Right [counts/s]`
- **health** — `ready`, `estopped`, `leaseExpired`, `stallHalted`, and per-wheel
  `sat`/`stall`/`wedge`/`wedgeSuspect`/`deficit`/`connected`
- **sticky diagnostics** — `leaseExpiryCount`, `i2cFaultCount`

`cycleBusy` is worth calling out: it is the number protocol v6 §11.2 wants
measured before the ASCII formatting cost can be trusted. The kernel already
publishes it, so the measurement is a subtraction, not a new instrument.

### 4.1 Age math

`sampleTime*` are `[us]` stamps from the same base as `nowFine`. Age is
`(int32_t)(nowFine - sampleTime)` — deliberately signed, deliberately wrapping.
Right is deterministically about one settle window younger than left, because
sampling is sequential split-phase. A telemetry consumer that treats the two
wheels as simultaneous will see a phantom twist at high speed.

---

## 5. What changes in this repo

Nothing about the law. Specifically:

1. **The files move as-is** from `src/archive/diffdrive/` into `src/diffdrive/`
   — same namespace, same includes, same behaviour.
2. **The fidelity gate comes with them.** `golden_ref_drive.{h,cpp}` +
   `fidelity_harness.cpp` hold the kernel duty-for-duty against the frozen
   pre-kernel law it descends from (feedforward exact; closed loop equal at
   steady state). That gate is the reason we can move this code and still claim
   it is the same code, so it is not optional cargo.
3. **The standalone-build check comes with them** — compile with an include
   path of exactly the package directory.

### 5.1 The dual-maintenance hazard, stated plainly

`radio-robot-elite` still contains its own copy at `src/firm/control/` **and**
the extracted package at `src/firm/diffdrive/`. Until a consumer actually cuts
over, a fix has to land in every live copy or they silently diverge. This repo
adds a third.

The fidelity suite is what keeps them honest, and it only works if it is
actually run. This is the single largest maintenance risk the repo takes on,
and it should be closed by making one copy authoritative as soon as an
environment consumes this library — not left to discipline.

---

## 6. Open question for review

**Which copy becomes authoritative, and when?** The options are to treat
radio-robot-lib as the source of truth immediately (elite consumes it as a
subtree/submodule and deletes its copies), or to keep elite authoritative until
a MicroPython or JavaScript environment ships against this library. The first
closes the divergence risk now and costs a change to elite's build; the second
defers both.

Nothing else in this document is uncertain — the code exists and is gated.

# Protocol handler — v6 line grammar, handler + adapter

**Status:** built. §1–§7 are the design as implemented; §8's question was
settled (**no `done:` for `WHEELS`**, 2026-08-20); **§9 is what the
implementation actually found** — resolved spec ambiguities, one deliberate
omission, and the gaps this work exposed. Read §9 before extending any of it.
The wire format itself is [protocol-v6-spec.md](../protocol-v6-spec.md).

---

## 1. The shape, in one picture

Three objects, each with one job, arranged so that the middle one is the only
thing that ever knows what a wire byte looks like.

```
   bytes from the port                                bytes to the port
           │                                                  ▲
           ▼                                                  │
  ┌────────────────────────────────────────────────────────────────────┐
  │  ProtocolHandler                                                   │
  │    feed(data, length)  →  reassemble lines  →  parse  →  dispatch  │
  │    reply formatting: ok / err / done / get / thdr / t / pong / …   │
  │    telemetry emission on a cadence the app drives                  │
  └────────────────────────────────────────────────────────────────────┘
       │ calls typed methods                     ▲ returns typed results
       ▼                                         │
  ┌────────────────────────────────────────────────────────────────────┐
  │  Adapter  (you implement — one class, all the callable methods)     │
  │    onWheels(left, right, duration, id) → Result                     │
  │    onSet(name, value, id) → Result     onGet(name, out) → bool      │
  │    onEstop()   onStop(id)   onPing() → now   …                      │
  └────────────────────────────────────────────────────────────────────┘
       │ calls
       ▼
     DifferentialDrive, config store, clock — the actual machine
```

**The adapter never writes a wire byte and never parses one.** It receives
decoded, typed arguments and returns a typed result. Everything textual lives
in `ProtocolHandler`. That is what makes the wire format single-sourced, the
adapter unit-testable without a socket, and a second language implementation a
matter of porting one class.

The transport is not an object here at all. `feed()` takes bytes from wherever
you got them and a `Sink` takes bytes back out — a serial port, a radio frame,
a UDP datagram, or a test's `std::vector`.

---

## 2. `ProtocolHandler`

```cpp
namespace Protocol {

class Sink {                       // where finished lines go
 public:
  virtual ~Sink() = default;
  virtual void write(const char* data, size_t length) = 0;
};

class ProtocolHandler {
 public:
  ProtocolHandler(Adapter& adapter, Sink& sink);

  // Feed an arbitrary block from the port. Partial lines are buffered
  // across calls; complete lines are parsed and dispatched immediately.
  void feed(const char* data, size_t length);

  // Unsolicited emissions the app drives, not the wire.
  void sendBanner();                        // device:NEZHA2:robot:<name>:<serial>
  void sendReady();                         // ready
  void emitTelemetry(const Snapshot& snapshot);  // thdr: once, then t: per frame

  uint32_t malformedCount() const;
};

}  // namespace Protocol
```

### 2.1 `feed()` must survive being handed anything

This is the method most likely to be got wrong, because on a bench it is always
handed one tidy line and in the field it never is. It must handle:

- a block containing **several** complete lines,
- a block **ending mid-line** (buffer the remainder, dispatch on the next feed),
- a block that is **only** a line fragment,
- `\r\n` — a lone `\r` before the terminator is stripped as a terminal
  artifact; `\r` appears nowhere else (spec §2),
- a line **longer than the 240-byte maximum** — discard to the next `\n` and
  count it malformed, rather than overflowing or truncating into a
  half-line that parses as something valid.

That last one matters more than it looks: a truncated line whose surviving
prefix is still a legal verb with legal arity is a command the host never sent.
Discard-to-terminator is the only safe recovery.

### 2.2 Parsing is split-in-place, no allocation

Per spec §11.1 the entire codec is a split on `':'`. Firmware constraints
(§11.2) mean no dynamic allocation and no `std::string`: the handler owns a
fixed `char[240]` line buffer, and parsing overwrites the separators with `\0`
to produce field pointers into that same buffer. Numbers come out with
`strtol`/`strtof`.

Fields are **positional with fixed arity per verb**. Wrong arity is a rejection,
not a best-effort parse.

### 2.3 Case is direction, and it is load-bearing

Commands (host→robot) are UPPERCASE; replies (robot→host) are lowercase; verb
lookup is case-**sensitive** (spec §2.1).

A lowercase verb arriving inbound is **another robot's reply** on a shared
radio channel. It is dropped silently and **not** counted malformed. Under v5 a
robot's own `DBG:` output was a syntactically valid `DBG` command to every
other robot on the channel, and the resulting flood was self-sustaining. v6
closes that structurally, and this handler is where that closure actually
happens — so it needs a test, not just a comment.

---

## 3. `Adapter` — one class, all the callable methods

```cpp
namespace Protocol {

enum class Result : uint8_t {      // maps 1:1 onto the wire outcome
  kOk,          // → ok:<id>
  kUnknown,     // → err:<id>:1
  kBadArg,      // → err:<id>:2
  kRange,       // → err:<id>:3
  kFull,        // → err:<id>:…
  kDuplicateId, // → err:<id>:11
  kNotReady,
};

class Adapter {
 public:
  virtual ~Adapter() = default;

  // ---- session ----
  virtual void identity(Identity& out) const = 0;   // name, serial, version, …
  virtual uint32_t now() const = 0;                 // [ms] for pong
  virtual void status(StatusFields& out) const = 0;

  // ---- motion (the minimal set: enough to exercise DiffDrive) ----
  virtual Result onWheels(float left, float right,      // [mm/s] [mm/s]
                          uint32_t duration,            // [ms]
                          uint32_t id) = 0;
  virtual Result onStop(uint32_t id) = 0;
  virtual void   onEstop() = 0;                     // never acked, never queued

  // ---- configuration ----
  virtual bool   onGet(const char* name, float& out) const = 0;
  virtual Result onSet(const char* name, float value, uint32_t id) = 0;
  virtual size_t fieldCount() const = 0;            // for bare GET
  virtual const char* fieldName(size_t index) const = 0;

  // ---- telemetry ----
  virtual Result onTlm(TlmMode mode) = 0;
};

}  // namespace Protocol
```

Returning a `Result` rather than writing a reply is the deliberate choice. It
means the adapter cannot emit a malformed reply, cannot forget to reply, and
cannot invent a reply shape — the handler does all three, once, for every verb.

`onEstop()` returns `void` on purpose: `ESTOP` never carries an id and is never
acked, because it must not queue behind anything, including an ack (spec §8.2).

---

## 4. The DiffDrive adapter is where geometry lives

The concrete adapter that closes the loop for testing:

```
WHEELS:<left>:<right>:<duration>
        [mm/s]  [mm/s]   [ms]
              │
              ▼   scale by countsPerLength [counts/mm]   ← the robot's geometry
        left, right  [counts/s]
              │
              ▼   velocity = (left+right)/2 ,  twist = (right-left)/2
   DifferentialDrive::drive(velocity, twist, lease=duration)
```

Three things fall out of this that are worth stating before anyone writes it:

1. **`duration` is `lease`, unchanged.** Both are `[ms]`, both mean "stop if
   nobody talks to me again within this window", both exist so a dead host
   cannot mean a runaway. No reinterpretation, no clamping surprise beyond
   DiffDrive's own `kLeaseMax` and the wire's 5000 ceiling.
2. **`countsPerLength` is the only geometry in the whole path**, and it lives
   in the adapter because DiffDrive deliberately has no millimetres in it
   ([diffdrive.md](diffdrive.md) §1.1). It is a config field, so it arrives by
   `SET` and reads back by `GET` like everything else.
3. **`twist` is the half-differential, CCW-positive.** Getting the sign wrong
   here is the single most repeated bug in this project's history — a robot
   whose "left" wheel was physically the right one negated every wheel-derived
   heading while leaving forward motion correct, so nothing surfaced it and it
   was patched four times downstream. The adapter is the one place that
   ordering is decided, and it needs a test that would fail if the two wheels
   were swapped.

### 4.1 Telemetry maps straight off `Output`

`DifferentialDrive::output()` already publishes everything a `t:` frame needs
(see [diffdrive.md](diffdrive.md) §4) — positions, velocities, applied duties,
timing, and the health flags. The adapter's telemetry job is a projection, not
a computation: pick the columns for the active `TLM` mode, convert counts to
the wire's units, and hand the handler an array.

`thdr:` is emitted once on subscribe and names the columns; `t:` carries the
values in that order. The frame is self-describing, so a consumer never
hardcodes a column index (spec §6.2).

---

## 5. Verb scope — what the first library implements

Enough to test DiffDrive over the wire, and no more.

| in scope | why |
|---|---|
| `HELLO` `PING` `VER` `ID` `STATUS` `HELP` | session and liveness; all trivial, all needed to bring a link up |
| `GET` `SET` | tune the kernel's gains live, and read back what was pushed |
| `TLM` + `thdr:`/`t:` | observe encoders and duties climbing — this is how you *see* the kernel work |
| `WHEELS` | the direct wheel primitive; maps onto `drive()` with no planner |
| `STOP` `ESTOP` | halt paths; `ESTOP` is mandatory, not optional |
| `ok` `err` `done` | outcomes for everything above |

| deferred | why |
|---|---|
| `MOVE` | needs a planner and a stop-condition evaluator — neither is in this library |
| `GOTO` | needs a navigator and world-frame odometry |
| `SEED` `CAL` | need OTOS/odometry that this library does not own |

`MOVE` is the interesting omission. It is the richer verb, but its `kind=w`
arm would still land on the same `drive()` call — the extra machinery is the
stop condition and the queue, which belong to an application, not a wheel
kernel. `WHEELS` reaches the kernel with the least intervening invention, which
is exactly what a first test wants.

---

## 6. Configuration — the library stores none

**Decision (stakeholder, 2026-08-20): neither the handler nor the kernel
implements configuration storage.** A configuration system may come later, as
its own thing; it is not core work and it is not in this library.

What that means concretely:

- **The 80-row v6 config table does not come across.** It stays in
  `src/archive/protocol-v6/` as reference. Carrying rows for subsystems this
  library does not contain is exactly the orphan-field problem the project's
  configuration-discipline rule says to delete rather than wire.
- **`GET`/`SET` are pure delegation.** The handler parses the line, decodes the
  value, calls `onGet`/`onSet`, and formats the reply. It holds no field table,
  no bounds, no storage. Which names are valid is entirely the adapter's
  business, and an unknown name is just `err:<id>:1` coming back from the
  adapter.
- **Each library carries only the configuration it needs, as its own type.**
  DiffDrive already has this: `DifferentialDrive::Config` plus the fluent
  setters, holding gains, limits, and the cycle period. The handler's own
  configuration is nearly nothing — the line-buffer ceiling and the default
  telemetry mode.
- **The robot's geometry is not in either library.** `countsPerLength` (§4)
  belongs to the adapter, because it is a property of a particular robot and
  neither a wheel control law nor a line parser is.

---

## 7. Testing — two independent C++ libraries, driven from Python by ctypes

**Decision (stakeholder, 2026-08-20).** Each library compiles to C++ and is
exercised from Python through a `ctypes` shim. The two are tested
**independently** — there is no combined harness in the first structure:

| harness | loads | exercises |
|---|---|---|
| **protocol** | the handler + a mock adapter | parsing, dispatch, reply formatting — no kernel, no motors |
| **diffdrive** | the kernel + fake ports | the control law, wired up and stepped — no protocol, no wire |

Independence is the point. A parsing bug and a control-law bug should never be
able to present as each other, and neither harness should be able to fail
because of the other's code.

### 7.1 Each harness needs a small `extern "C"` shim

`ctypes` cannot call C++ methods, so each library gets a thin C surface —
create, call, destroy, and enough accessors for a test to observe results. The
shim is test scaffolding, not library API: it lives beside the tests, and
nothing in the library knows it exists.

For the protocol harness that means roughly: construct a handler over a
recording sink and a mock adapter; `feed()` a byte block; read back what the
sink captured and which adapter methods fired with which arguments.

For the diffdrive harness: construct a kernel over fake `Motor`/`Clock`/
`Sleeper`; `drive()`; `step()`; read the `Output` snapshot out field by field.

### 7.2 The golden-vector fixture still binds implementations

Spec §11.3's ASCII line vectors — command in → expected line out, and the
reverse — remain the primary conformance gate for the protocol harness. They
are what will keep this C++ handler and any later MicroPython or JavaScript one
from drifting apart, and they are worth writing alongside the parser rather
than after it.

### 7.3 Two tests that must exist by name

Both failures are silent, and both have cost this project real time:

- **twist sign** — a test that fails if the two wheels are swapped (§4).
- **lease expiry** — commanded motion actually stops when the duration runs
  out, measured at the fake motor, not inferred from the kernel's own flags.

---

## 8. The one open question: should `WHEELS` emit `done:`?

This was badly explained last time, so here it is properly.

v6 has three outcome replies (spec §8.1): `ok:<id>` means *accepted*,
`err:<id>:<code>` means *rejected*, and `done:<id>:<reason>` means *the thing
you enqueued has now finished*, where `reason` is `stop` (its stop condition was
met) or `timeout` (its backstop fired).

`done` was designed for `MOVE`, which has a real stop condition — "drive until
400 mm of travel" genuinely finishes. **`WHEELS` has no stop condition.** It
holds a wheel speed for `duration` ms and then the lease expires. So "finished"
means only "the lease ran out", and the question is whether that is worth a
reply at all:

- **Emit `done:<id>:timeout`** — the host can await completion instead of
  timing the wheels itself, and `WHEELS` behaves like every other bounded
  command.
- **Emit nothing; `ok` is the whole story** — the host already knows the
  duration it asked for, so the reply carries no information it lacks.

**The reason this is a design question and not a preference:** emitting `done`
means the handler must remember outstanding ids and emit a reply *later, on its
own*, which means it needs a periodic entry point and a notion of time. Without
`done`, the handler is a pure function of the bytes fed to it — `feed()` in,
replies out, no state between calls beyond a partial line, nothing to tick.

That is a real difference in what the class *is*, and it is much cheaper to
decide now than to retrofit. **My recommendation: no `done` for `WHEELS`** —
keep the handler stateless and pure for the first library, and let `done` arrive
with `MOVE`, which is the verb that actually needs it.

---

## 9. As built — resolved ambiguities and known gaps

Everything above is design. This section is what the implementation actually
found, and it is the part to read before extending any of it.

### 9.1 Spec ambiguities resolved (not silently picked)

**The optional trailing `id` on `SET`/`WHEELS`.** Spec §7.1's worked example
shows `SET:wheel_control.pid_kp:0.03` → `ok:0` — an *omitted* id still acked
with id 0. But §8.2 says "id 0 means no ack wanted", which read literally
predicts no reply at all. Resolved by treating the two as different: **id
omitted → acked as id 0** (§7.1's literal text); **id written as literal `0` →
no ack** (§8.2's literal text). Applied only where the id is genuinely
optional; `STOP`'s id is required and is always acked.

**`GET` with an unknown field name.** `GET` never carries an id, so there is no
wire channel to carry an `err` on. Resolved as **fully silent** — no reply, and
not counted malformed.

**`WHEELS`'s 5000 ms ceiling** (§5.2) is prose at the verb level with no stated
owner. The handler holds no bounds table, so **the adapter enforces it** and
returns `kRange` above it.

### 9.2 Deliberately not implemented

**The 3× reply repeat.** Spec §8.1 says `ok`/`err`/`done` are each sent three
times on consecutive cycles, so an outcome survives the measured ~5% radio
frame loss without a ring or an eviction policy. This handler does **not** do
that, because "on consecutive cycles" needs a periodic entry point and pending
state — exactly what the no-`done`-for-`WHEELS` decision keeps out.

That is a real gap, not a rounding error: **v6's loss tolerance is currently
unimplemented.** It belongs at the app or transport layer that owns a loop, or
it comes back into the handler when `MOVE` and `done` do. Worth deciding
deliberately rather than discovering on a lossy link.

### 9.3 Gaps this step exposed

**Three `Config` fields the kernel needs to `begin()` — resolved: hard-coded,
not wired.** `maxDuty`, `fullDutyVelocity` and `cyclePeriod` are required to
start the kernel but do not appear in spec §7.3's 15-row `wheel_control`
group. This used to be an open gap ("wire-unreachable, armed only by the test
shim, and these documents do not say what a real boot path does"). Stakeholder
decision, 2026-08-20: "I don't see that max duty, full duty velocity, and
cycle period need to be configurable, so you can just hard code them." They
are now build-time constants on `Protocol::DiffDriveAdapter`
(`kMaxDuty`/`kFullDutyVelocity`/`kCyclePeriod`, `src/adapter/
diffdrive_adapter.h`), applied to the kernel at adapter construction — so
building a `DiffDriveAdapter` alone is sufficient for `begin()` to succeed,
with no external arming step and no per-robot variation. `countsPerLength`
remains the one field of real per-robot geometry, unaffected by this
decision, still a constructor parameter (§4 point 2). If a later robot
genuinely needs one of these three to vary, that is new work, not a bug fix:
it means giving the field a real wire home — a config system, or a new
spec row — the same way any other configurable value would need one.

**The `TLM` projection is reduced, not spec §6.3/§6.4's columns.** Those need
world-frame `x`/`y`/`h` fused from OTOS and encoder odometry, neither of which
this library owns (§5 defers odometry). The adapter emits a smaller, documented
column set. It deliberately does **not** reuse the spec's column names for
different data.

**The `flags` word uses a local bit layout**, not spec §6.5's numbers, for the
same reason — §6.5's bits are OTOS/line/colour/planner, none of which exist
here. Reusing those bit *numbers* for different meanings would actively
mislead anyone cross-referencing the spec.

### 9.4 Where the adapter lives, and why

`src/adapter/` — its own package, not inside `src/protocol/` or
`src/diffdrive/`. It is the one component required to depend on both, and each
of those two has a standalone-build gate ("compiles with an include path of
exactly its own directory") that a cross-dependency would break.

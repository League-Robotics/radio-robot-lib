# Protocol handler — v6 line grammar, handler + adapter

**Status:** new design, for review. Unlike [diffdrive.md](diffdrive.md), none of
this code exists yet. The *wire format* is settled — see
[protocol-v6-spec.md](../protocol-v6-spec.md) — but the object model below is
new, and §7 lists what needs deciding before implementation starts.

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

## 6. Testing

**The golden-vector fixture is the primary gate**, per spec §11.3: ASCII line
vectors, command in → expected line out and the reverse, asserted byte-for-byte
by every implementation. It is the only thing that will keep a C++ handler and
a MicroPython or JavaScript one from quietly drifting apart, and it is what
makes a second implementation verifiable at all.

Three layers, cheapest first:

1. **Handler unit tests, no machine.** A mock adapter that records calls, and a
   `Sink` that records bytes. Covers arity, unknown verbs, the split-block and
   over-long-line cases from §2.1, and the lowercase-is-not-a-command rule.
2. **Adapter tests against a fake motor.** DiffDrive with a simple simulated
   `Motor`/`Clock`/`Sleeper`, driven by `step()` rather than a fiber, so it is
   deterministic. Covers the mm→counts conversion, the twist sign, and lease
   expiry actually stopping the wheels.
3. **End-to-end through bytes.** Feed `WHEELS:100:100:1000:5` in as a byte
   block; assert `ok:5` comes out, encoder counts climb in the telemetry, and
   the wheels stop when the lease expires.

Layer 3 is the acceptance the whole repo is for.

---

## 7. Open questions — these need answering before implementation

1. **What language is the library written in?** The deployment targets are
   MicroPython and JavaScript, but DiffDrive is C++ and MakeCode/PXT compiles
   C++ underneath its JavaScript. **My recommendation: C++ is the reference
   implementation for both the kernel and the handler**, since both targets can
   bind C++ (MicroPython via a C module, PXT via C++ shims), with a small pure
   Python implementation of the handler for host-side testing and the golden
   vectors binding them. The alternative — writing the handler natively per
   language from the spec — is what §11.1 says is only ~150 lines each, and
   would avoid a binding layer entirely. This choice changes the repo's whole
   build story, so it should be settled first.

2. **Does the adapter own config storage, or does the library?** The v6 field
   table exists already (`src/archive/protocol-v6/wire_v6_config_fields.h`, 80
   rows with pre-resolved bounds). Either the library ships a config store keyed
   by that table, or the adapter implements `onGet`/`onSet` against its own.
   The former is more code here but makes `GET`/`SET` work identically in every
   environment; the latter keeps the library smaller.

3. **How much of the 80-row config table does this library need?** Testing
   DiffDrive needs the `wheel_control.*` block and little else. Carrying all 80
   rows means carrying fields for subsystems this library does not contain —
   which is exactly the orphan-field problem the configuration-discipline rule
   says to delete rather than wire.

4. **Is `done:` in scope?** `WHEELS` has a duration, not a stop condition, so
   "finished" is lease expiry. Whether that emits `done:<id>:timeout` or nothing
   is a real semantic choice, and the spec's `done` is written with `MOVE` in
   mind.

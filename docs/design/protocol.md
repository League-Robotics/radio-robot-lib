# Protocol handler — v6 line grammar, handler + adapter

**Status:** built and self-contained. This document is the wire format
*and* the design — there is no external spec file. §1–§8 are the design as
implemented; §9 is what the implementation actually found: resolved
ambiguities, one deliberate omission, and the gaps this work exposed,
including the 2026-08-20 space/`#id` grammar migration and the 2026-08-21
addition of `debug` and `RUN` (§6.2/§6.3).

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

## 2. The wire grammar

**One grammar. ASCII. No framing layer.**

```
line   ::= sp? verb ( sp field )* sp? '\n'
sp     ::= ' '+
verb   ::= [A-Za-z][A-Za-z0-9_]*
field  ::= any bytes except ' ' and '\n'
id     ::= '#' [0-9]+        (a field in trailing position — §2.3)
```

That is the entire wire format, both directions, every message. No COBS, no
CRC, no length prefix, no binary plane, no protobuf, no generated codec —
`readline()` is the transport. It replaced an earlier binary/cleartext-mixed
scheme whose complexity (COBS framing, an app-level CRC duplicating one the
radio hardware already computes, a nine-`FieldKind` protobuf walker, four
different cleartext sub-grammars) bought portability and integrity guarantees
this project does not use: endpoints are rev-locked by policy, and real
losses are whole packets, about which a CRC says nothing.

- **Terminator is `'\n'` (0x0A).** A lone `'\r'` before it is stripped
  (terminal artifact); `'\r'` appears nowhere else.
- **The first space ends the verb.** One or more spaces separate fields; a run
  of spaces is ONE separator, and leading/trailing whitespace on the line is
  ignored. `strtol`/`strtof`'s own whitespace-skipping implements most of this
  for free (§9.4's leading-whitespace finding covers the residue).
- **A blank or all-whitespace line is ignored silently** — a terminal
  artifact, not an error; it does not count malformed.
- **Fields are positional**, fixed arity per verb. Wrong arity is a rejection,
  not a best-effort parse.
- **Max line: 240 bytes** including the terminator. Chosen to sit inside a
  radio MTU and a safe UDP payload on the transports this format was designed
  for, so a message never fragments — a property this library's `feed()`
  relies on for its overflow-discard behavior (§3.1), even though this
  library itself is transport-agnostic.
- **Every wire value is a base-10 ASCII integer**, optionally signed, except
  config values (§7.2), which are decimal. No exponents, no `NaN`, no `inf`.
  `flags` is the one exception to base-10: lowercase hex, no `0x`.

### 2.1 Case is direction, and it is load-bearing

**Commands (host → robot) UPPERCASE. Replies (robot → host) lowercase.** Verb
lookup is case-**sensitive**.

Not cosmetic. On a shared radio channel a robot hears every other robot. A
robot's own debug output being a syntactically valid command to every other
robot on the channel produces a self-sustaining flood
(`.claude/rules/hardware-bench-testing.md`, radio-robot-elite). Under this
grammar a reply can never parse as a command, so that class is closed
structurally instead of by keeping the channel private.

A verb starting lowercase is dropped silently and does **not** count
malformed — that is another robot's reply, not an error.

### 2.2 Ids

An id is spelled **`#<n>`** and is always the **last token** of its line —
commands and replies alike.

- Host-assigned, unique for the session. The digits are bare and unsigned:
  `#+5`, `#-5`, and `# 5` are all malformed — §2's "optionally signed" applies
  to data fields, not the id, and needs a dedicated digits-only parser rather
  than the general integer one.
- Because it announces itself, the id never shifts position when an optional
  field is omitted, and it is recoverable even from a line that otherwise
  fails to parse (§2.3).
- **Required** on `STOP` — it completes and a caller needs a correlation key
  for the outcome. **Optional** on `SET`/`WHEELS`.
- **Omitted id** → the command still executes, and its `ok`/`err` is sent
  once, bare (§8.1). **`#0`** → executes silently, no reply — the
  ack-suppression spelling for a lossy link that doesn't want an ack for every
  line. `#0` is legal only where the id is optional; on `STOP` it is
  malformed.
- A reused id is `err #<id> 11` (`ERR_DUPLICATE_ID`).

### 2.3 Malformed-line recovery, and the one exception

Unknown verb, wrong arity, or an unparseable field → drop the line, increment
the malformed counter. If the line's last token is a well-formed nonzero
`#id`, reply `err #<id> <code>` — the self-marking id is trustworthy even on a
line that otherwise failed to parse, and this fires even for an unknown verb.
Otherwise, no reply.

**`ESTOP` is the one exception, and it wins.** `ESTOP` never emits a reply
under any circumstance, including this one: a malformed `ESTOP` line is
dropped and counted, silently. The reason is stronger than "consistency" — a
panic stop must never queue behind an outbound reply. Where the general
recovery rule and `ESTOP`'s own rule collide, `ESTOP`'s wins.

---

## 3. `ProtocolHandler`

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
  void sendBanner();                        // device NEZHA2 robot <name> <serial>
  void sendReady();                         // ready
  void sendDebug(const char* text);         // debug <text>  -- robot-to-host ONLY
  void emitTelemetry(const Snapshot& snapshot);  // thdr once, then t per frame

  uint32_t malformedCount() const;
};

}  // namespace Protocol
```

### 3.1 `feed()` must survive being handed anything

This is the method most likely to be got wrong, because on a bench it is always
handed one tidy line and in the field it never is. It must handle:

- a block containing **several** complete lines,
- a block **ending mid-line** (buffer the remainder, dispatch on the next feed),
- a block that is **only** a line fragment,
- `\r\n` — a lone `\r` before the terminator is stripped as a terminal
  artifact; `\r` appears nowhere else (§2),
- a blank or all-whitespace line (ignored silently — §2, NOT malformed),
- a line **longer than the 240-byte maximum** — discard to the next `\n` and
  count it malformed, rather than overflowing or truncating into a
  half-line that parses as something valid.

That last one matters more than it looks: a truncated line whose surviving
prefix is still a legal verb with legal arity is a command the host never sent.
Discard-to-terminator is the only safe recovery.

### 3.2 Parsing is split-in-place, no allocation

The entire codec is a tokenizer over runs of `' '`. Firmware constraints mean
no dynamic allocation and no `std::string`: the handler owns a fixed
`char[240]` line buffer, and tokenizing overwrites separator spaces with `\0`
to produce field pointers into that same buffer. Numbers come out with
`strtol`/`strtof`. The correlation id (§2.2), where a verb carries one, is a
trailing, self-marking `#<n>` token rather than a positional field — recovered
by a separate backward scan over the raw line, done before the forward
tokenizer mutates it, so the id stays recoverable even past the small
fixed field-token array's own storage cap
(`protocol_handler.cpp`'s `tokenizeLine()`).

---

## 4. `Adapter` — one class, all the callable methods

```cpp
namespace Protocol {

enum class Result : uint8_t {      // maps 1:1 onto the wire outcome
  kOk,             // → ok [#id]
  kUnknown,        // → err [#id] 1   ERR_UNKNOWN
  kBadArg,         // → err [#id] 2   ERR_BADARG
  kRange,          // → err [#id] 3   ERR_RANGE
  kFull,           // → err [#id] 4   ERR_FULL
  kUnimplemented,  // → err [#id] 6   ERR_UNIMPLEMENTED
  kNotReady,       // → err [#id] 8   ERR_NOT_CONFIGURED
  kBusy,           // → err [#id] 10  ERR_BUSY
  kDuplicateId,    // → err [#id] 11  ERR_DUPLICATE_ID
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

  // ---- invocation by name (§6.3) ----
  virtual Result onRun(const char* name,
                       const char* const* argv, size_t argc,
                       char* result, size_t resultCapacity,
                       bool& hasResult) = 0;
};

}  // namespace Protocol
```

`kUnimplemented` and `kBusy` are carried for completeness against the wire's
error-code space even though no verb in this library's scope currently
produces them — `kUnimplemented` is reserved for a recognized-but-unwired verb,
`kBusy` for a subsystem refusing because it is mid-motion (neither condition
arises in a wheel-kernel-only adapter with no queue to be busy about).

Returning a `Result` rather than writing a reply is the deliberate choice. It
means the adapter cannot emit a malformed reply, cannot forget to reply, and
cannot invent a reply shape — the handler does all three, once, for every verb.

`onEstop()` returns `void` on purpose: `ESTOP` never carries an id and is never
acked, because it must not queue behind anything, including an ack (§2.3).

---

## 5. The DiffDrive adapter is where geometry lives

The concrete adapter that closes the loop for testing:

```
WHEELS <left> <right> <duration> [#<id>]
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
   DiffDrive's own `kLeaseMax` and the wire's 5000 ceiling (enforced by the
   adapter — the handler holds no bounds table).
2. **`countsPerLength` is the only geometry in the whole path**, and it lives
   in the adapter because DiffDrive deliberately has no millimetres in it
   ([diffdrive.md](diffdrive.md) §1.1). It is a constructor argument, not a
   config field — it is a property of the robot's gearing/wheel, not a
   tunable control-law gain, so it is not reachable through `GET`/`SET`.
3. **`twist` is the half-differential, CCW-positive.** Getting the sign wrong
   here is the single most repeated bug in this project's history — a robot
   whose "left" wheel was physically the right one negated every wheel-derived
   heading while leaving forward motion correct, so nothing surfaced it and it
   was patched four times downstream. The adapter is the one place that
   ordering is decided, and it needs a test that would fail if the two wheels
   were swapped.

### 5.1 `STOP` and `ESTOP` — what they actually do here

**There is no queue in this library**, because there is no planner — `WHEELS`
reaches `drive()` directly. So neither verb "waits its turn behind an active
move"; that framing belongs to a full motion-planner robot (see
[motion-api.md](motion-api.md), which specifies that layer as something to
build, not something this library has). What `STOP #<id>` and `ESTOP`
actually do, traced to the kernel:

| verb | adapter call | kernel effect |
|---|---|---|
| `STOP #<id>` | `onStop()` → `drive_.neutral()` | writes a neutral command to the mailbox; duty zeroes **this cycle**, immediately — `stageStop()` is a bare `stageDuty(0, 0)`, no ramp |
| `ESTOP` | `onEstop()` → `drive_.estop()` | sets a **latch** that forces the same neutral path from the next cycle regardless of the mailbox's state, and additionally **refuses every new motion command** (`kRefusedEstopped`) until `estopClear()` |

**The two are not distinguished by how fast duty goes to zero — both are
immediate at the kernel level.** They are distinguished by persistence and
guarantee: `neutral()` is an ordinary mailbox write that the very next
`drive()` call overrides; `estop()` latches outside the normal command
handshake specifically so it is effective even if that handshake is wedged,
and it blocks new motion until explicitly cleared. `onStop()` always returns
`kOk` — `neutral()` has no refusal path of its own, even pre-`begin()` or
mid-estop.

This is a materially smaller distinction than a full motion-planner robot's
"planned stop queues behind the active move, ramps down at a decel ceiling"
versus "estop halts now" — that contrast (measured elsewhere at 39.8 cm/5.9 s
versus 2.9 cm/0.10 s) is about a *queue*, not about deceleration profile, and
it describes a planner this library does not have. Do not cite that
measurement as characterizing `DiffDriveAdapter::onStop()`; it does not apply
here.

### 5.2 Telemetry maps straight off `Output`

`DifferentialDrive::output()` already publishes everything a `t` frame needs
(see [diffdrive.md](diffdrive.md) §4) — positions, velocities, applied duties,
timing, and the health flags. The adapter's telemetry job is a projection, not
a computation: pick the columns for the active `TLM` mode, convert counts to
the wire's units, and hand the handler an array.

`thdr` is emitted once on subscribe and names the columns; `t` carries the
values in that order. The frame is self-describing, so a consumer never
hardcodes a column index.

**This is a reduced projection, not a world-frame pose.** A full robot's
`POSE`/`FULL` telemetry carries `x`/`y`/`h` fused from OTOS and encoder
odometry — neither of which lives in this library. What `DiffDrive` actually
publishes is per-wheel counts and counts/s, so that is what this adapter
projects: `posl`/`posr` `[mm]` and `vell`/`velr` `[mm/s ×10]`, converted
through the one geometry factor the adapter owns; `TLM FULL` adds
`lambda`/`biasl`/`biasr`/`cyc` — the kernel's own learned-state and heartbeat
fields. `flags` is a **local bit layout** (`computeFlags()` in
`diffdrive_adapter.cpp`), not any externally-numbered scheme — this library has
no OTOS/line/colour/planner, so reusing bit numbers that meant those things
elsewhere would actively mislead a reader.

---

## 6. Verb scope — what this library implements

Enough to test DiffDrive over the wire, and no more.

| verb | command | reply | notes |
|---|---|---|---|
| `HELLO` | — | `device NEZHA2 robot <name> <serial>` | |
| `PING` | — | `pong <now>` | `now` = robot clock `[ms]` |
| `ID` | — | `id <drivetrain> <profile> <version>` | |
| `VER` | — | `ver <version>` | |
| `STATUS` | — | `status ready=1 active=0 connL=1 connR=1 otos=0 wedge=0 flags=<hex> tlm=off` | `k=v`, order not guaranteed, unknown keys ignored |
| `HELP` | — | `help HELLO PING ID VER STATUS HELP GET SET TLM WHEELS STOP ESTOP RUN` | rest-of-line; generated from the same table `dispatch()` uses, so it cannot drift |
| `GET` | `[name]` | `get name value` (one field) or one `get` line per field (bare `GET`) | unknown name → silent, no reply, not counted malformed |
| `SET` | `name value [#id]` | `ok [#id]` / `err [#id] <code>` | |
| `TLM` | `mode` | — | `OFF`/`POSE`/`FULL`/`NOW`/`AUTO`/`BUFFER` decoded; mode-specific behavior beyond persisting the value is the calling application's job |
| `WHEELS` | `left right duration [#id]` | `ok [#id]` / `err [#id] <code>` | maps onto `drive()` with no planner |
| `STOP` | `#id` | `ok #<id>` / `err #<id> <code>` | required id; see §5.1 |
| `ESTOP` | — | — | mandatory, never acked; see §5.1 |
| `RUN` | `function [arg...] [#id]` | `ret <value> [#id]` / `ok [#id]` / `err [#id] <code>` | invocation by name; see §6.3 |
| — | — | `debug <text>` | robot-to-host ONLY, no inbound wire form; see §6.2 |

| deferred | why |
|---|---|
| `MOVE` | needs a planner and a stop-condition evaluator — neither is in this library |
| `GOTO` | needs a navigator and world-frame odometry |
| `SEED` `CAL` | need OTOS/odometry this library does not own |
| `done` | see §8 |

`MOVE` is the interesting omission. It is the richer verb, but its wheels-kind
arm would still land on the same `drive()` call — the extra machinery is the
stop condition and the queue, which belong to an application, not a wheel
kernel. `WHEELS` reaches the kernel with the least intervening invention,
which is exactly what a first test wants. See [motion-api.md](motion-api.md)
for what a layer that adds `MOVE`/`GOTO` back would look like.

### 6.1 Outcomes

| reply | meaning |
|---|---|
| `ok [#id]` | accepted — enqueued, or applied |
| `err [#id] <code>` | rejected, with a reason (§4's `Result` → the code table below) |
| `ret <value> [#id]` | `RUN` only (§6.3) — the invoked function returned a value |

An id-carrying reply, per the wire's own design, is meant to be sent three
times on consecutive cycles so an outcome survives packet loss without a ring
or an eviction policy. **This library does not do that** — see §8/§9.2 for
why and what it would take.

| code | name | meaning |
|---|---|---|
| 1 | `ERR_UNKNOWN` | no such verb or field name |
| 2 | `ERR_BADARG` | malformed/non-finite argument, wrong arity |
| 3 | `ERR_RANGE` | declared bound violated |
| 4 | `ERR_FULL` | queue full |
| 6 | `ERR_UNIMPLEMENTED` | recognized, not wired on this build |
| 8 | `ERR_NOT_CONFIGURED` | refused pre-`ready` |
| 10 | `ERR_BUSY` | subsystem in motion; retry at rest |
| 11 | `ERR_DUPLICATE_ID` | a reused id — refused out loud rather than silently dropped |

### 6.2 `debug` — robot-to-host only

Wire shape: **`debug <free text>`**, lowercase, a rest-of-line verb exactly
like `HELP`'s own reply — everything after the first space is one field.
`ProtocolHandler::sendDebug(const char* text)` is the only way this line is
ever emitted; it is an unsolicited emission the application drives
(alongside `sendBanner()`/`sendReady()`), never a reply to an inbound
command.

**The host never sends it, and there is no inbound wire form to reject.**
Because it is lowercase, an inbound `debug ...` line is dropped by the same
mechanism every other lowercase-led line is (§2.1) — silently, and not
counted malformed. This is the structural fix for the v5 `DBG:`-flood
incident (`.claude/rules/hardware-bench-testing.md` in the robot repo this
library was extracted from): under v5 a robot's own debug output was a
syntactically valid *command* to every other robot on a shared channel, and
the flood was self-sustaining. Under this grammar a `debug` line can never
parse as a command, closing that class structurally rather than by keeping
the channel private.

**Sanitization: strip, don't reject.** `text` is arbitrary and must not be
able to forge a second line — `'\n'` and `'\r'` bytes are stripped before
they can reach the sink. The alternative (reject the whole call) was
considered and rejected: `sendDebug()` is `void`, with no channel to report
a rejection back through, so discarding the entire message over one bad
byte would lose strictly more information than delivering everything else
in it. The whole line (`debug` + space + text + terminator) is also
truncated, never overflowed, to fit the 240-byte cap — the same posture
`feed()` itself takes on an overlong *inbound* line (§3.1).

**`sendDebug("")` and `sendDebug(nullptr)` are the same case.** Both emit
the bare line `debug\n` — no trailing space before the terminator. A text
that sanitizes down to nothing (e.g. it was entirely `'\n'`/`'\r'` bytes)
collapses onto this same bare shape, rather than leaving a dangling
separator space (`debug \n`) that no other field-less reply in this file
ever produces — consistent with the grammar's own "an empty token cannot
exist between spaces" rule.

### 6.3 `RUN` — invocation by name

Wire shape: **`RUN <function> [arg...] [#id]`**.

**Division of responsibility — this is the important design decision, and
it must not move.** The handler parses and nothing else: it extracts the
function name and the remaining raw argument tokens as `const char*`
pointers into its own line buffer, and hands them to
`Adapter::onRun(name, argv, argc, result, resultCapacity, hasResult)`. The
handler holds no function table, does no name resolution, and does no type
conversion — the same "the handler holds no tables" property that makes
`GET`/`SET` pure delegation (§7).

**The adapter owns resolution, type conversion, and invocation.** In
MicroPython or JavaScript that is `globals()[name]` plus argument
introspection — nearly free. **In C++ there is no lookup-by-name and no
parameter-type reflection**, so a C++ adapter needs an explicit
registration table declaring each function's name, arity, and per-argument
types to implement `onRun()` at all. Say this plainly to any porter: `RUN`
is the first verb where the C++ archetype does substantially *more* work
than the dynamic ports, not less — a porter reading this handler should not
conclude that registration machinery is itself part of the wire contract.
This library's own `DiffDriveAdapter` registers nothing and answers every
`RUN` with `ERR_UNKNOWN`, which is the correct behavior for an adapter with
an empty allowlist, not a stub left unfinished.

**The registration table IS the security boundary.** Whatever a concrete
adapter registers is invocable by name from the wire by anything that can
talk to the robot, including any other host on a shared radio channel.
Treat the table as an explicit allowlist, not an implementation detail —
a function should be registered because it is meant to be remotely
callable, not because it happened to be convenient to expose.

**Replies** — `ret` is a new lowercase reply verb:

| outcome | reply |
|---|---|
| function returned a value | `ret <value> [#id]` |
| function returned nothing (void) | `ok [#id]` |
| unknown function | `err [#id] 1` (`ERR_UNKNOWN`) |
| wrong arity, or an argument that will not convert | `err [#id] 2` (`ERR_BADARG`) |
| `RUN` with no function name at all | malformed (counted; `err #<id> <code>` if the line's last token is a well-formed nonzero `#id`, per §2.3's standard recovery rule) |

**The `#0` interaction:** `#0` means "no ack wanted, execute silently"
(§2.2). With `#0` the function **still runs**, but **nothing is emitted,
including `ret`** — a returned value is a reply, and `#0` suppresses
replies, full stop. Omitted id → the function runs, and its `ret`/`ok`/
`err` is sent once, bare, matching every other optional-id verb's own
omitted-id shape.

**A last field beginning with `'#'` is always the id slot, even against
RUN's own open arity.** Every other verb in this library has a *fixed*
arity, so whether an id slot exists at all is decided by field *count*
before any field's *content* is inspected. `RUN`'s arity is open-ended
(however many arguments the target function takes), so the handler instead
inspects the line's *last* field directly: if it begins with `'#'`, it is
the id (well-formed digits → a real id; anything else after the `'#'` →
the whole line is malformed, the same as `SET`/`WHEELS`'s own "trailing
token present but not a well-formed id" rule). This is a genuine
consequence worth stating on its own: **a function's own final argument
can never itself begin with `'#'`** under this wire grammar. A function
that needs a literal `'#'`-led value as its last argument cannot be called
that way — it would have to take that value as a non-final argument
instead, or the caller reorders the call.

**Two limitations, not implementation gaps:**

1. **An argument cannot contain a space.** The grammar makes a space the
   field separator, so string arguments are single-token only. This is a
   genuine constraint on what `RUN` can express, not an oversight.
2. **`onRun()` is called synchronously from `feed()`**, so a slow
   registered function stalls line processing for as long as it runs.
   Registered functions must return promptly; anything long-running must
   be deferred by the calling application.

**Sanitization of the return value.** The adapter's own `result` string is
sanitized by the handler exactly like `debug`'s text — `'\n'`/`'\r'`
stripped, truncated to fit the line cap — before it reaches the sink. A
concrete `onRun()` does not need to pre-sanitize its own output; the
handler treats it as untrusted content regardless, the same way it treats
every other free-form string it ever formats onto the wire.

---

## 7. Configuration — the library stores none

**Decision (stakeholder, 2026-08-20): neither the handler nor the kernel
implements configuration storage.** A configuration system may come later, as
its own thing; it is not core work and it is not in this library.

What that means concretely:

- **No config field table lives in this library.** `GET`/`SET` are pure
  delegation. The handler parses the line, decodes the value, calls
  `onGet`/`onSet`, and formats the reply. It holds no field table, no bounds,
  no storage. Which names are valid is entirely the adapter's business, and an
  unknown name is just `err [#id] 1` coming back from the adapter.
- **Each library carries only the configuration it needs, as its own type.**
  DiffDrive already has this: `DifferentialDrive::Config` plus the fluent
  setters, holding gains, limits, and the cycle period. `DiffDriveAdapter`
  maps 15 wire names 1:1 onto `Config`'s members — the whole field table is a
  name/member-pointer pair per row, in `diffdrive_adapter.cpp`.
- **`maxDuty`, `fullDutyVelocity`, `cyclePeriod` are hard-coded, not wired.**
  `DifferentialDrive::begin()` needs values for these to leave its fail-closed
  default, but they are not tuning gains — stakeholder decision, 2026-08-20:
  "I don't see that max duty, full duty velocity, and cycle period need to be
  configurable, so you can just hard code them." They are build-time constants
  on `DiffDriveAdapter` (`kMaxDuty`/`kFullDutyVelocity`/`kCyclePeriod`),
  applied to the kernel at adapter construction, so building a
  `DiffDriveAdapter` alone is sufficient for `begin()` to succeed with no
  external arming step.
- **The robot's geometry is not in either library.** `countsPerLength` (§5)
  belongs to the adapter, because it is a property of a particular robot and
  neither a wheel control law nor a line parser is.

---

## 8. The one open question: should `WHEELS` emit `done`?

The wire's outcome model has room for three reply verbs: `ok [#id]` means
*accepted*, `err [#id] <code>` means *rejected*, and `done #<id> <reason>`
would mean *the thing you enqueued has now finished*, where `reason` is `stop`
(its stop condition was met) or `timeout` (its backstop fired).

`done` was designed for `MOVE`, which has a real stop condition — "drive until
400 mm of travel" genuinely finishes. **`WHEELS` has no stop condition.** It
holds a wheel speed for `duration` ms and then the lease expires. So
"finished" means only "the lease ran out", and the question is whether that is
worth a reply at all:

- **Emit `done #<id> timeout`** — the host can await completion instead of
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
decide now than to retrofit. **Settled, 2026-08-20: no `done` for `WHEELS`** —
the handler stays stateless and pure for this library; `done` arrives with
`MOVE`, which is the verb that actually needs it, whenever that is built.

---

## 9. As built — resolved ambiguities and known gaps

Everything above is design. This section is what the implementation actually
found, and it is the part to read before extending any of it.

### 9.1 Design calls this file made

**The malformed-line `#id` recovery rule is verb-agnostic, with one
deliberate exception.** §2.3's own words — "if the line's last token is a
well-formed nonzero `#id`, reply `err #<id> <code>`" — carry no carve-out for
a verb whose own grammar has no id concept at all (`HELLO`/`PING`/`ID`/`VER`/
`STATUS`/`HELP`/`GET`/`TLM` in this library's scope), and "including unknown
verbs" confirms it fires even before a verb is identified. This handler
implements it that way, with exactly one exception: `ESTOP`, whose own rule
("never carries an id and is never acked … must not queue behind anything,
including an ack") is treated as the more specific rule winning over the
general one. This is a resolution the wire grammar does not spell out in one
place — it is this file's own call, recorded here and in
`protocol_handler.h`'s own file-header ambiguity note.

**`WHEELS`'s 5000 ms ceiling** is prose at the verb-definition level with no
stated owner in the grammar. The handler holds no bounds table, so **the
adapter enforces it** and returns `kRange` above it.

**The id's own numeric grammar (`'#' [0-9]+`) is stricter than an ordinary
wire integer field.** §2's general "every wire value is … optionally signed"
does not apply to the id itself — `#+5` is not a well-formed id, even though a
`+`-prefixed ordinary field elsewhere might parse. Implemented with a
dedicated digit-only pre-scan (`parseIdDigits()`) rather than reusing the
general unsigned-field parser.

### 9.2 Deliberately not implemented

**The 3× reply repeat.** The wire's own design has an id-carrying `ok`/`err`/
`done` sent three times on consecutive cycles, so an outcome survives radio
frame loss without a ring or an eviction policy. This handler does **not** do
that, because "on consecutive cycles" needs a periodic entry point and pending
state — exactly what the no-`done`-for-`WHEELS` decision (§8) keeps out. The
repeat is emission policy, owned by whatever drives a real per-cycle output
loop — NOT a property of the line codec. This handler stays a pure function of
its input bytes: it emits each id-carrying reply exactly once, and the
repeat, if and when it is wanted, belongs to whatever drives a real cycle
loop, not to this class.

That is a real gap, not a rounding error: **loss tolerance is currently
unimplemented** in this library. It belongs at the app or transport layer
that owns a loop, or it comes back into the handler when `MOVE` and `done` do.
Worth deciding deliberately rather than discovering on a lossy link.

### 9.3 Gaps this step exposed

**The `TLM` projection is reduced, not a literal world-frame pose** — see §5.2.
It deliberately does **not** reuse column names for different data than what
`DiffDrive` actually publishes.

**The `flags` word uses a local bit layout**, for the same reason — see §5.2.

### 9.4 Hardening sweep (2026-08-20) — bugs found, and what they mean for a port

Stakeholder direction: `src/protocol/` is going to be an **archetype**,
ported to MicroPython and JavaScript by reading it and running its fixture,
so this pass focused entirely on the handler's own robustness, not new
features. Full detail lives in the fix-site comments in
`protocol_handler.cpp` and in `tests/protocol/test_protocol_adversarial.py`'s
module docstring; this is the summary a future porter should read first.

**Three real bugs, all fixed, none a wire-format change:**

1. **`formatConfigValue()` cast a NaN straight to `uint32_t`** — undefined
   behavior, confirmed live by UBSan. A NaN can never arrive *over the wire*
   (`parseFloatField` already rejects it on input), so this was only
   reachable through the `Adapter` seam — an adapter's own stored config
   value being NaN, read back by `GET`. Fixed by clamping NaN to 0.0 before
   the cast; `+Inf`/`-Inf` were already handled correctly by the existing
   overflow clamp.
2. **Hex-float syntax (`SET name 0x1.8p3`) bypassed "no exponents" entirely.**
   The exponent guard only checked for `'e'`/`'E'`; a hex float's exponent
   marker is `'p'`, gated behind a `'0x'` prefix the guard never looked for,
   so `strtof` silently accepted it (`0x1.8p3` → 12.0).
   **Archetype-relevant on its own**: neither Python's `float()` nor
   JavaScript's `Number()`/`parseFloat()` accepts hex-float syntax, so this
   was a **C++-only divergence** — a straight port would not have this bug
   at all, and would need to actively decide whether to *add* hex-float
   rejection or simply rely on its own numeric parser already refusing it.
3. **A leading-whitespace numeric field was silently accepted** because
   `strtol`/`strtoul`/`strtof` all skip leading whitespace per the C
   standard. Under the space grammar a literal LEADING SPACE can no longer
   reach a field decoder at all — the tokenizer collapses every run of `' '`
   into one separator before a field pointer is ever produced. The guard is
   NOT dead code, though: the field grammar (`field ::= any bytes except ' '
   and '\n'`) still admits `'\t'`, `'\v'`, `'\f'`, and `'\r'` as ordinary,
   legal field bytes, and `strtol`/`strtoul`/`strtof` would silently skip any
   of those too — so the guard survives, now targeted at a narrower set of
   bytes. Every language's numeric parser has its own leniency here
   regardless (Python's `int()`/`float()` also strip whitespace **and**
   accept `_` digit separators; JavaScript's `Number(" ")` is `0`) — a port
   author should decide this deliberately per language, not inherit
   whichever behavior their host language's built-in parser happens to have.

**One characterization finding, not fixed — read this before porting
`dispatch()`:** every wire-touching comparison in this handler (verb
lookup, tokenizing) operates on NUL-terminated C strings, per the
no-allocation, no-`std::string` constraint. `strcmp()`/the tokenizer's own
forward scan both stop at the first NUL in a string, so `PING extra` compares
**equal** to `"PING"` and dispatches exactly like a bare `PING` — silently
discarding `extra` with no malformed-count increment. The grammar's verb
rule (`verb ::= [A-Za-z][A-Za-z0-9_]*`) does not admit NUL in a verb at all,
so the grammar-correct behavior would be rejection, not silent acceptance of
the truncated prefix. **This is NOT reproduced by a length-aware host
language**: Python `bytes`/JavaScript strings compare full length, embedded
NUL included, so `b"PING extra" == b"PING"` is `False` in Python. A faithful
line-by-line port of this C++ handler's *logic* would therefore behave
differently from this reference implementation on this one input class —
pinned as a characterization test
(`test_embedded_nul_immediately_after_verb_matches_bare_verb`) so it cannot
drift silently, not fixed, because a real fix means abandoning C-string
comparisons throughout the parser — a far larger, riskier change than this
pass's scope, and in tension with the explicit no-`std::string` firmware
constraint.

### 9.5 Where the adapter lives, and why

`src/adapter/` — its own package, not inside `src/protocol/` or
`src/diffdrive/`. It is the one component required to depend on both, and each
of those two has a standalone-build gate ("compiles with an include path of
exactly its own directory") that a cross-dependency would break.

### 9.6 The colon-to-space/`#id` grammar migration (2026-08-20)

Stakeholder decision, 2026-08-20: fields are separated by **spaces**, not
colons, and the correlation id returns to its historical **`#`-prefix**
spelling as a trailing, self-marking field. This section records what
changed in `src/protocol/` and `tests/protocol/`, for the same reason §9.4
records the hardening sweep — a future porter reading this file needs the
"why", not just a diff.

**What is a pure separator swap, and what is not.** Every wire example in
this document uses the space grammar; the underlying OBJECT MODEL —
`Adapter`, `Result`, `Sink`, `Snapshot`/`Column`, the handler/adapter split
itself — did not change at all. This was a rewrite of
`protocol_handler.{h,cpp}`'s parsing and formatting internals, not a
redesign.

**New mechanics this migration introduced:**

- **Tokenizing, not colon-splitting.** `tokenizeLine()` collapses runs of
  `' '` into one separator and trims leading/trailing line whitespace. A
  blank or all-whitespace line is now ignored SILENTLY (previously, under
  the colon grammar, an empty line dispatched as an unknown zero-length
  verb and counted malformed).
- **The id is self-marking and line-trailing**, not positional. Because it
  announces itself with `#`, an omitted optional field never shifts it into
  a data position — the reason `SET name value` and `SET name value #9` are
  both exactly two or three tokens, with no placeholder needed for the
  missing middle slot the old grammar would have required.
- **Bare vs id-carrying replies are now genuinely different wire shapes.**
  An omitted id → `ok`/`err <code>` (no `#id` token at all); an explicit
  nonzero id → `ok #<id>`/`err #<id> <code>`; an explicit `#0` (legal only
  where the id is optional) → no reply at all. This retired an old
  ambiguity where an omitted id and an explicit `0` looked equivalent but
  behaved differently with no sentence saying so.
- **The malformed-line `#id` recovery rule is new capability, not a
  reformatting of old behavior.** Under the old colon grammar an unknown
  verb's own arity was unknowable, so no field of its line could ever be
  trusted as an id; the new grammar's self-marking id makes it trustworthy
  regardless of whether the verb itself is known, or even well-formed.
  `ESTOP` is the one deliberate exception (§9.1). This inverted part of the
  old `test_unknown_verb_no_reply` test, which only covered the id-less
  case — now split into `test_unknown_verb_no_reply_when_no_recoverable_id`
  and `test_unknown_verb_with_recoverable_id_gets_err_unknown`.
- **The id's own numeric grammar is stricter than an ordinary integer
  field** — `'#' [0-9]+`, no sign at all, parsed with a dedicated
  digit-only pre-scan (`parseIdDigits()`) rather than reusing the general
  unsigned-field parser, so `#+5` is correctly NOT id 5.

**What did NOT change:** the `Adapter` interface (`adapter.h`) — every
method signature, `Result`/`TlmMode`/`Column`/`Snapshot` shape is untouched,
because none of them ever encoded a wire delimiter. `mock_adapter.h` and
`protocol_shim.cpp` needed zero changes for the same reason.
`tests/adapter/test_diffdrive_adapter.py`, which drives the real handler end
to end (not a mock), needed its wire literals updated for the same
mechanical reason `golden_vectors.txt` did.

**Golden-vector fixture:** every vector in `golden_vectors.txt` changed
SHAPE, not just separator — the old `ok:0` id-less arm is gone; there is no
new equivalent single spelling, because "id-less" now means literally "no
`#id` token in the reply", i.e. a bare `ok`. The fixture also grew new
vectors for rules the colon grammar never had: space-run collapsing,
`STOP #0` being malformed (required-id verb), and an unknown verb's
trailing `#id` recovering an `err` reply.

### 9.7 `debug` and `RUN` added (2026-08-21)

Two verbs added to the library: `debug` (robot-to-host only, §6.2) and
`RUN` (host-to-robot invocation by name, §6.3). `Adapter` gained one new
pure-virtual method, `onRun()` — both concrete implementations in this
repository (`MockAdapter`, `DiffDriveAdapter`) were updated; `DiffDriveAdapter`
registers nothing and answers every `RUN` with `ERR_UNKNOWN` (§6.3).
`kCommandTable` grew from 12 to 13 entries (`RUN` appended at the end), so
`HELP`'s generated reply grew by five bytes (` RUN`) — still comfortably
inside both its own local 160-byte formatting buffer and the wire's 240-byte
line cap; a porter should not assume this margin is infinite, only that it
held here.

**A real bug found and fixed while implementing this:** the first working
draft of `sendDebug()`/`handleRun()`'s final line-formatting buffer was sized
`char buf[kMaxLineBytes]` (240 bytes). `kMaxLineBytes` already counts the
wire content *including* the terminating `'\n'`, so a line that legitimately
reaches the full 240 bytes needs a **241-byte** buffer — `snprintf()` also
needs room for its own NUL terminator, and with only 240 bytes available it
silently truncated the *last* byte of the formatted string (the trailing
`'\n'` itself) to make room for the NUL it always writes. Caught by
`test_send_debug_exactly_240_bytes_is_not_truncated`, a boundary test
written specifically because the earlier hardening sweep (§9.4) had already
established that boundary-byte-count reasoning in this file is exactly where
bugs hide. Fixed by sizing the buffer `kMaxLineBytes + 1`. This is a
C++-only hazard in the same spirit as §9.4's hex-float finding: Python's
f-strings and JavaScript template literals have no equivalent "off by one
for a NUL the language forces you to reserve room for," so a straight port
would not reproduce this bug — but it WOULD need to get its own
line-length-cap arithmetic right by some other means, and this is exactly
the kind of boundary a porter should write a test for rather than reason
about by inspection.

**Design decisions made here, recorded for a future reader instead of only
living in code comments:**

- **`sendDebug("")` and `sendDebug(nullptr)` are the same case** (§6.2) —
  both emit the bare `debug\n` line. The alternative (making null a no-op
  that emits nothing at all) was rejected: `sendBanner()`/`sendReady()`
  never take a "should I even emit" argument, and giving `sendDebug()` a
  hidden suppression channel through its argument's nullness, distinct from
  the wire's own explicit `#0` suppression spelling used elsewhere in this
  file, would be a second, undocumented way to say "don't send this" with no
  wire vocabulary to describe it.
- **Sanitize, don't reject**, for both `debug`'s text and `RUN`'s returned
  value (§6.2/§6.3). `sendDebug()` is `void` with no return channel at all;
  `RUN`'s outcome channel (`Result`) is owned by the *adapter's* own
  resolution/conversion/invocation logic, not by whether its return value
  happens to contain a newline, so reusing that channel to signal "your
  return value had a bad byte in it" would conflate two unrelated failure
  modes. Stripping degrades gracefully; rejecting outright would silently
  drop legitimate content over one bad byte with no way for either caller to
  learn that happened.
- **A last field beginning with `'#'` is always the id slot, even against
  `RUN`'s own open arity** (§6.3) — resolved by content inspection rather
  than by field count, because `RUN` is the one verb in this library whose
  arity the handler cannot know in advance. The consequence — a function's
  own final argument can never itself begin with `'#'` — is a genuine
  expressiveness limit, not an oversight, and is stated as such rather than
  left for a porter to discover by testing.
- **`kMaxFieldTokens` raised from 8 to 20**, and a new `kMaxRunArgs` (16)
  added, both firmware resource limits with no wire meaning of their own.
  `RUN`'s open arity meant, for the first time in this file, that a verb's
  own field count could legitimately exceed what the fixed-size token array
  was sized for — every other verb's fixed arity had always been comfortably
  inside the old cap, so this never mattered before. `handleRun()` checks
  `fieldCount` against `kMaxFieldTokens` **before** indexing the field array
  at all, which no other handler in this file needs to do (`protocol_handler.h`'s
  own file-header ambiguity note #4 has the full reasoning). A line with more
  real arguments than `kMaxRunArgs` is rejected as `ERR_BADARG` before the
  adapter is ever called — a resource ceiling, not a claim about any real
  function's arity.

**What a MicroPython/JavaScript porter would get wrong, bluntly:**

- **Under-building `onRun()`, not over-building it.** The natural instinct
  in a dynamic language is `getattr(module, name)` / `globals()[name]` with
  no registration table at all — and that is *correct* for those languages,
  but it means "everything importable is remotely callable" unless the
  porter deliberately restricts it. §6.3's security framing ("the
  registration table IS the security boundary") is written for the C++
  archetype, where building a table is unavoidable and *therefore* an
  obvious place to enforce an allowlist; a dynamic-language port has to
  choose to build that same restriction on purpose, because its own
  language's ergonomics actively work against it.
- **Assuming the id and the last argument can't collide.** A JavaScript or
  Python port's own function-calling convention has no equivalent to "the
  last field might secretly be the correlation id" — a porter translating
  this handler's logic naively (e.g. "split on spaces, last token after the
  name list is the id if the caller says there's one") will get this wrong
  for a variadic function unless they re-derive the content-inspection rule
  from this document rather than from the C++ source's control flow alone.
- **Reproducing the buffer-sizing bug in spirit, if not in fact.** No
  dynamic language will hit an actual NUL-terminator off-by-one, but a port
  that computes "does this fit the 240-byte cap" by string concatenation
  length alone, without a boundary test at exactly 240 bytes, can still ship
  a fencepost error the same class of mistake produces — §9.4 already made
  this point about hex-floats and leading whitespace; this section's own
  finding is one more data point for the same lesson.
- **Forgetting `onRun()` must return promptly.** It is called synchronously
  from `feed()` in every implementation, dynamic or not; a JavaScript port
  built on an event loop is especially easy to get this wrong in, by
  registering an `async` function and awaiting it inline instead of
  deferring the actual work and returning immediately.

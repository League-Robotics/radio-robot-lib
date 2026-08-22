# Protocol handler — v6 line grammar, handler + adapter

**Status:** built and self-contained. This document is the wire format
*and* the design — there is no external spec file. §1–§8 are the design as
implemented; §9 is what the implementation actually found: resolved
ambiguities, one deliberate omission, and the gaps this work exposed,
including the 2026-08-20 space/`#id` grammar migration, the 2026-08-21
addition of `debug` and `RUN` (§6.2/§6.3), and the 2026-08-21 reliability
layer — mandatory sequence ids plus cumulative `ack`/`nack`, replacing the
undelivered 3×-reply-repeat idea outright (§8).

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
  │    reply formatting: ack / nack / err / estop / get / thdr / t / … │
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
id     ::= '#' [0-9]+        (mandatory, trailing, §2.2/§8)
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

### 2.2 Ids — now a mandatory sequence number (2026-08-21)

**Superseded, 2026-08-21.** This section used to describe the id as an
optional, host-assigned correlation token with an ack-suppression spelling
(`#0`) and a per-verb "required/optional" split. That design is gone,
replaced wholesale by §8's reliability layer: **every sequenced command now
carries a mandatory, strictly incrementing id, starting at 1**, and the id
*is* the sequence number the ack/nack scheme tracks — it is no longer a
free-standing correlation token a caller could pick arbitrarily.

An id is still spelled **`#<n>`** and is still always the **last token** of
its line — commands and replies alike.

- **Mandatory** on every sequenced verb (`PING ID VER STATUS HELP GET SET
  TLM WHEELS STOP RUN` — see §8.3 for the two exceptions, `HELLO` and
  `ESTOP`, which never carry one at all). A sequenced verb arriving with no
  id is malformed.
- The digits are bare and unsigned: `#+5`, `#-5`, and `# 5` are all
  malformed — §2's "optionally signed" applies to data fields, not the id,
  and needs a dedicated digits-only parser rather than the general integer
  one. This part is unchanged from the old design.
- Because it is always the line's last token, it never shifts position
  regardless of a verb's own field count, and it is recoverable even from a
  line that otherwise fails to parse — see §8.4 for how malformed-line
  recovery works under the new scheme.
- **`#0` is deleted as a special case.** It used to mean "no ack wanted,
  execute silently"; that spelling is incoherent once every message must be
  sequenced and acked (a suppressed ack is a hole in the stream by
  definition). Since ids start at 1 and the session's `expectedNext_`
  counter is always ≥ 1, an inbound `#0` simply always compares less than
  `expectedNext_` — it falls into the ordinary **stale/retransmit** bucket
  (§8.1) with no special-casing at all: acked against the highest
  already-accepted id, never executed. There is no way to suppress a reply
  any more, by design — suppression is incompatible with a scheme that must
  see every id to detect gaps.
- **`ERR_DUPLICATE_ID` (code 11) is now unreachable.** The old design had
  the *adapter* detect and reject a reused id. Under the new scheme the
  *handler* itself enforces strict monotonicity before an id ever reaches
  the adapter — an id is only ever dispatched when it exactly equals
  `expectedNext_`, which then advances past it, so the adapter can never be
  handed the same id twice. `Result::kDuplicateId` and its wire code remain
  declared (removing them is out of this change's scope), but no code path
  in this library can produce them any more. Flagged in §9.8.

### 2.3 Malformed-line recovery — superseded by §8.4

The old malformed-line recovery rule ("if the line's last token is a
well-formed nonzero `#id`, reply `err #<id> <code>`, with `ESTOP` the one
exception") is gone. It was designed for a world where the id was an
optional correlation token; under the mandatory-sequence-number design an
id is either present, well-formed, and sequence-checked, or the line simply
cannot be classified at all. See §8.4 for the replacement rule, and §8.3
for `ESTOP`'s own (still exceptional, but differently exceptional) status.

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

// Maps 1:1 onto the wire outcome (docs/design/protocol.md §8, updated
// 2026-08-21 for the reliability layer): kOk means nothing further is
// emitted beyond the ack dispatch() already sent (no more standalone
// `ok`); every other value means an `err <code> #<id>` follows that same
// ack (id now LAST, §8.6). kDuplicateId (code 11) is UNREACHABLE as of
// this change (§2.2/§9.8 item 8) -- kept declared, never produced.
enum class Result : uint8_t {
  kOk,             // → (ack alone; no further reply)
  kUnknown,        // → err 1 #<id>   ERR_UNKNOWN
  kBadArg,         // → err 2 #<id>   ERR_BADARG
  kRange,          // → err 3 #<id>   ERR_RANGE
  kFull,           // → err 4 #<id>   ERR_FULL
  kUnimplemented,  // → err 6 #<id>   ERR_UNIMPLEMENTED
  kNotReady,       // → err 8 #<id>   ERR_NOT_CONFIGURED
  kBusy,           // → err 10 #<id>  ERR_BUSY
  kDuplicateId,    // → err 11 #<id>  ERR_DUPLICATE_ID -- unreachable (§9.8)
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
  virtual void   onEstop() = 0;    // never sequenced, never queued -- the
                                   // handler replies `estop` itself (§8.3),
                                   // this method's own return is still void

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

`onEstop()` returns `void` on purpose: `ESTOP` never carries a sequence id and
is never part of the ack/nack scheme, because it must not queue behind
anything, including an ack (§8.3). The handler itself still replies `estop`
after calling this — that reply is not this method's own concern; it is
formatted and sent by `ProtocolHandler`, exactly like every other reply.

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

**`ESTOP`'s wire reply changed, 2026-08-21 (§8.3).** It used to never reply
at all; it now always replies the bare word `estop`, with the kernel call
(`onEstop()`) executed *before* that reply is written, so the reply can
never be mistaken for having queued ahead of the actual stop.

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

**Every row marked "sequenced" below now requires a mandatory `#<id>` —
see §8.** The reply column shows each verb's own *informational* reply
only; every sequenced verb ALSO emits the transport-layer `ack`/`nack`
described in §8, as a separate line, alongside whatever is shown here.

| verb | sequenced? | command | own reply | notes |
|---|---|---|---|---|
| `HELLO` | no | — | `device NEZHA2 robot <name> <serial>` | resets the sequence (§8.3) |
| `PING` | yes | `#id` | `pong <now>` | `now` = robot clock `[ms]` |
| `ID` | yes | `#id` | `id <drivetrain> <profile> <version>` | |
| `VER` | yes | `#id` | `ver <version>` | |
| `STATUS` | yes | `#id` | `status ready=1 active=0 connL=1 connR=1 otos=0 wedge=0 flags=<hex> tlm=off next=<n>` | `k=v`, order not guaranteed, unknown keys ignored; `next` added 2026-08-21 (§8.5) |
| `HELP` | yes | `#id` | `help HELLO PING ID VER STATUS HELP GET SET TLM WHEELS STOP ESTOP RUN` | rest-of-line; generated from the same table `dispatch()` uses, so it cannot drift |
| `GET` | yes | `[name] #id` | `get name value` (one field) or one `get` line per field (bare `GET`) | unknown name → no `get` line, but still acked (§8.1) |
| `SET` | yes | `name value #id` | — (accepted: none; rejected: `err <code> #<id>`) | `ok` is gone — an in-order `ack` **is** the acceptance (§8.2) |
| `TLM` | yes | `mode #id` | — | `OFF`/`POSE`/`FULL`/`NOW`/`AUTO`/`BUFFER` decoded; mode-specific behavior beyond persisting the value is the calling application's job; the adapter's own `Result` never surfaces on the wire, matching the pre-2026-08-21 behavior |
| `WHEELS` | yes | `left right duration #id` | — (accepted: none; rejected: `err <code> #<id>`) | maps onto `drive()` with no planner |
| `STOP` | yes | `#id` | — (accepted: none; rejected: `err <code> #<id>`) | see §5.1 |
| `ESTOP` | **no** | — | `estop` | never sequenced, never nacked, maximally forgiving; see §5.1/§8.3 |
| `RUN` | yes | `function [arg...] #id` | `ret <value> #<id>` (accepted, function returned a value) / — (accepted, void) / `err <code> #<id>` (rejected) | invocation by name; see §6.3 |
| — | — | — | `debug <text>` | robot-to-host ONLY, no inbound wire form; see §6.2 |
| — | — | — | `ack <n> <lastDone>` / `nack <n> <lastDone>` | transport layer, NEW 2026-08-21; see §8 |

| deferred | why |
|---|---|
| `MOVE` | needs a planner and a stop-condition evaluator — neither is in this library |
| `GOTO` | needs a navigator and world-frame odometry |
| `SEED` `CAL` | need OTOS/odometry this library does not own |

`MOVE` is the interesting omission. It is the richer verb, but its wheels-kind
arm would still land on the same `drive()` call — the extra machinery is the
stop condition and the queue, which belong to an application, not a wheel
kernel. `WHEELS` reaches the kernel with the least intervening invention,
which is exactly what a first test wants. See [motion-api.md](motion-api.md)
for what a layer that adds `MOVE`/`GOTO` back would look like.

### 6.1 Outcomes

**Rewritten 2026-08-21 — see §8 for the full design.** `ok` is deleted:
acceptance is now signaled by the transport-layer `ack` alone. `done` is
deleted as a standalone verb: the `lastDone` field carried by every
`ack`/`nack` is now the completion channel, for whichever future verb
(`MOVE`) produces a completion event this library's own verbs do not.

| reply | meaning |
|---|---|
| `ack <n> <lastDone>` | transport: the highest in-order id accepted so far arrived correctly (§8.1) |
| `nack <n> <lastDone>` | transport: `n` is the next id the robot actually needs — the stream has a gap (§8.1) |
| `err <code> #<id>` | application: an in-order command's *content* was rejected (§4's `Result` → the code table below). **Field order changed 2026-08-21** — the id is now always the LAST token, matching every other line in this grammar (§8.6); it used to be `err #<id> <code>`, an undocumented exception to the id-is-last rule. |
| `estop` | `ESTOP` only, NEW 2026-08-21 — confirms the stop executed (§8.3) |
| `ret <value> #<id>` | `RUN` only (§6.3) — the invoked function returned a value, emitted IN ADDITION to the `ack` |

A well-formed, in-order command is never "just" accepted or "just" rejected
on the wire in isolation — every sequenced verb's `ack` is unconditional on
arrival-in-order, and `err` (when it happens) is a *second*, additional
line layered on top, not a replacement for it (§8.2).

| code | name | meaning |
|---|---|---|
| 1 | `ERR_UNKNOWN` | no such verb or field name |
| 2 | `ERR_BADARG` | malformed/non-finite argument, wrong arity |
| 3 | `ERR_RANGE` | declared bound violated |
| 4 | `ERR_FULL` | queue full |
| 6 | `ERR_UNIMPLEMENTED` | recognized, not wired on this build |
| 8 | `ERR_NOT_CONFIGURED` | refused pre-`ready` |
| 10 | `ERR_BUSY` | subsystem in motion; retry at rest |
| 11 | `ERR_DUPLICATE_ID` | **unreachable as of 2026-08-21** — the handler's own sequencing now makes a repeated id structurally impossible to hand to the adapter (§2.2, §9.8) |

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

Wire shape: **`RUN <function> [arg...] #id`** — `id` is mandatory as of
2026-08-21 (§8), same as every other sequenced verb.

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

**Replies** — `ret` is a lowercase reply verb, always carrying the
(now-mandatory) id:

| outcome | reply |
|---|---|
| function returned a value | `ret <value> #<id>` — emitted IN ADDITION to the `ack` (§8.2) |
| function returned nothing (void) | nothing beyond the `ack` |
| unknown function | `ack` + `err 1 #<id>` (`ERR_UNKNOWN`) |
| wrong arity, or an argument that will not convert | `ack` + `err 2 #<id>` (`ERR_BADARG`) |
| `RUN` with no function name at all | malformed (§8.4) — if the id itself is present and well-formed but consumes the only field (`RUN #7`), still `ack` + `err 2 #<id>`; if there is no field at all (bare `RUN`), no reply of any kind |

**`#0` no longer suppresses anything (2026-08-21).** The old "`#0` means no
ack wanted, run silently" rule is deleted along with `#0` itself (§2.2) —
since ids start at 1, an inbound `#0` is always `< expectedNext_` and is
therefore always treated as a stale retransmit: **the function is never
invoked**, and the reply is the ordinary retransmit `ack` (§8.1), not
silence. There is no longer any way to run a `RUN` (or any other verb)
without a reply.

**The id is unconditionally the line's last token now — no content
inspection needed.** Before 2026-08-21, `RUN`'s open arity meant the
handler had to look at whether the *last field's content* began with `'#'`
to decide whether an id was even present. That branch is gone: every
sequenced verb's id is mandatory and is stripped from the line by the same
central step (§8.4), before any verb-specific parsing runs at all, so
`RUN`'s own handler never has to make that decision — it only ever sees
the function name and its real arguments, with the id already resolved.
The one genuine expressiveness limit this leaves is unchanged in spirit and
if anything sharper now: **a function's own final argument can never begin
with `'#'`, because that token position is unconditionally the id**, not
merely "usually" the id. A function needing a literal `'#'`-led value as
its logically-last argument cannot be called that way at all — it has to
take that value as a non-final argument, or the caller reorders the call.

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
  unknown SET name is just `err 1 #<id>` coming back from the adapter,
  layered on top of the ack every in-order `SET` gets regardless (§8.2).
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

## 8. The reliability layer — sequence numbers, cumulative ack/nack

**Stakeholder design, 2026-08-21, replacing §8's old "should `WHEELS` emit
`done`?" question outright** (the old text is preserved as §9.9's own
changelog entry, not repeated here). Where §2 through §7 describe the wire
grammar and per-verb payloads, this section describes a layer that sits
underneath every one of them: what it means for a command to *arrive*, as
opposed to merely being well-formed.

### 8.0 The problem this replaces

Before this change, the protocol had **no delivery guarantee at all**. §2.1
(pre-2026-08-21) promised a 3×-repeat of every id-carrying reply "on
consecutive cycles" so an outcome would survive packet loss — but nothing in
`ProtocolHandler` ever implemented it (§9.2, now historical), because doing
so honestly needs a periodic entry point and a notion of time, which the
handler deliberately does not have. Measured loss on the radio link this
protocol targets is real (~5%, `.clasi/knowledge/` in the robot repo this
library was extracted from) and nothing in this library protected against
it. **The 3×-repeat idea is deleted, not deferred** — this section is its
full replacement, not an addition alongside it.

### 8.1 The core idea — cumulative ack/nack over a mandatory sequence id

**Every command carries a mandatory sequential id, `#<n>`, starting at 1.**
The host may pipeline freely — it never has to wait for an ack before
sending its next command. The robot acknowledges **cumulatively**: one ack
covers every earlier id too, which is what lets the scheme survive loss
with no ring, no per-id storage, and no eviction policy — the entire
receiver-side state is two numbers.

Handler state, in full:

```cpp
uint32_t expectedNext_ = 1;   // next sequence id expected from the host
uint32_t lastDone_ = 0;       // most recent completed motion id (§8.5.1)
bool     gapOutstanding_ = false;  // a nack is currently owed (§8.5)
```

**Deliberately, there is no `tick()` and no clock anywhere in this list.**
The periodic half of the scheme (§8.5) rides on the telemetry cadence the
application already drives — it does not add one of its own. Keeping the
handler clock-free is load-bearing, not incidental: it is the property that
lets `feed()` stay a pure function of its input bytes plus this small,
explicit state, with nothing owed later "on its own."

Every inbound id, for every **sequenced** verb (§8.3's exemption set is the
only carve-out), is classified into exactly one of three cases:

| inbound id | action | reply |
|---|---|---|
| `== expectedNext_` | dispatch to the adapter; `expectedNext_ = id + 1` | `ack <id> <lastDone_>` |
| `< expectedNext_` | **do NOT re-execute** — a retransmit whose ack was lost | `ack <expectedNext_ - 1> <lastDone_>` |
| `> expectedNext_` | **discard, do NOT execute** — a gap | `nack <expectedNext_> <lastDone_>` |

The middle row is the one easy to get wrong: a resent `WHEELS` (the host
never saw the first ack, so it resends) must **not** drive the wheels a
second time. The reply for a retransmit echoes the *already-accepted*
id (`expectedNext_ - 1`), not the resent one — telling the host "I already
have everything through here," which is exactly what a resend needs to
hear to stop resending.

A gap **stalls the stream on purpose**: every subsequent command, however
well-formed, is discarded and nacked until the missing id arrives, giving
strict in-order delivery. Because every new command re-triggers the same
`nack <expectedNext_> ...`, a lost `nack` self-heals — the host will see
the next one along with the next command it sends.

`nack` carries **next-expected**, not "last good id": it tells the host
exactly what to resend with no `+1` inference on either side, and it avoids
overloading `0` as both "nothing accepted yet" and "resend from here."

### 8.2 Layering: `ack`/`nack` is transport, `err` is application

`ack`/`nack` answers one question only: *did the bytes arrive, in order?*
`err` answers a different one: *the command arrived fine — was its content
accepted?* A message can be perfectly in-order and still be garbage
(`WHEELS 99999 0 100 #7` is received correctly and rejected on range). So an
in-order command the adapter (or the handler's own field decode) rejects
emits **both**: the `ack` (it arrived, the sequence advances) **and**
`err <code> #<id>` (§6.1) — two lines, and errors are rare in practice. The
error code is never folded into `ack` itself; that would conflate a
transport signal with an application one.

This generalizes past the adapter: an **unknown verb**, or a known verb
with the wrong field count or an unparseable field, is *also* "arrived
fine, content rejected" as long as its id is in order — it gets an `ack`
plus an `err` (`ERR_UNKNOWN` or `ERR_BADARG` as appropriate), exactly like
an adapter-level rejection. See §8.4 for the full malformed-line story,
including the one case (missing or unparseable id) that genuinely cannot
be classified this way at all.

**Every reply so far bare `ok`/`err` gains a mandatory id, and `ok` itself
is gone.** `SET`/`WHEELS`/`STOP`/`RUN`(void) success now produces *nothing*
beyond the `ack` — the ack **is** the acceptance signal. `RUN`'s `ret` is
the one exception: a returned value is genuinely new information the `ack`
alone cannot carry, so it is still emitted, **in addition to** the ack, not
instead of it (§6.3).

### 8.3 `ESTOP` and `HELLO` — the exemption set (this library's own call)

**Sequenced:** `PING ID VER STATUS HELP GET SET TLM WHEELS STOP RUN`.
**Unsequenced:** `ESTOP` and `HELLO`.

The stakeholder's own framing was "every message must have an ID number" —
no exceptions stated. Exempting these two is **this file's own call, not
the stakeholder's**, made because the scheme is structurally unbootstrappable
and unsafe without them:

- **`HELLO` resets the sequence.** `expectedNext_ = 1`, `lastDone_ = 0`,
  `gapOutstanding_ = false`, then the banner is emitted — this is the
  session-start resync a host performs on (re)connect. A verb that resets
  the sequence cannot itself be *inside* the sequence without a
  chicken-and-egg problem (what id would the very first `HELLO` carry, and
  against what would it be checked?). `HELLO`'s own arity is unchanged from
  before this section (zero fields) — it does not accept a trailing id at
  all, and a `HELLO` with one is wrong arity, same as any other extra field.
- **`ESTOP` is outside the sequence entirely: no id, never sequenced, never
  nacked.** This is safety-critical: if `#4` goes missing and the stream is
  stalled waiting for it, an `ESTOP` sent as `#5` (or with no id at all)
  must still execute — it cannot be discarded as "out of order," and it
  cannot be made to wait behind the missing `#4` the way an ordinary
  sequenced command would.
- **`ESTOP` is maximally forgiving.** ANY line whose verb token is `ESTOP`
  executes the stop, regardless of trailing junk or arity — `ESTOP`,
  `ESTOP 1 2 3`, `ESTOP #5`, `ESTOP #abc` all stop the robot. A panic stop
  must never be refused over a syntax nit. (A verb that isn't the literal
  token `ESTOP` — e.g. `ESTOPXYZ`, no space — is a different, ordinary verb
  token under the tokenizer's own rules, not "ESTOP plus junk"; this
  forgiveness is about content AFTER the verb, not about fuzzy verb
  matching.)
- **`ESTOP` now REPLIES.** Stakeholder, verbatim: *"Agree about ESTOP, but
  if it is not acked, it should be acknowledged, with an `estop`
  response."* This **supersedes** the pre-2026-08-21 rule (recorded as
  defect D5 at the time) that `ESTOP` never emits a reply under any
  circumstance. The old rationale was "a panic stop must never queue
  behind an outbound reply" — satisfied here by **executing the stop
  before writing the reply**, so the silence was over-strict, not load
  -bearing. `ESTOP`'s reply is the bare word `estop`, no fields, ever.

This exemption set is deliberately narrow — flagged here prominently so the
stakeholder can find and overrule it easily: everything else in this
library's scope is sequenced, with no other carve-outs.

### 8.4 Malformed-line recovery under mandatory sequencing

The pre-2026-08-21 malformed-line recovery rule (§2.3, historical) is gone.
In its place, for any line whose verb is neither `ESTOP` nor `HELLO`
(§8.3) and does not start lowercase (§2.1, unchanged):

1. **No trailing field at all** (the line was just the verb) → malformed,
   no reply. There is nothing to sequence-check.
2. **A trailing field is present but is not a well-formed `#[0-9]+`**
   (missing `#`, non-digit content, `#+5`, digit overflow) → malformed, no
   reply. Same reasoning: nothing valid to compare against `expectedNext_`.
3. **A well-formed id is present** → classify it via §8.1's three-way
   table, using **only the id** — the verb's own identity and field
   content are not even inspected yet:
   - out of order (`<` or `>` `expectedNext_`) → `ack`/`nack` per §8.1,
     and **nothing else is examined or executed** — not even whether the
     verb is recognized. A stalled stream costs nothing but the id
     comparison itself.
   - in order (`== expectedNext_`) → the sequence advances and the `ack`
     is sent unconditionally; **then**, and only then, the verb is looked
     up and its fields validated. An unrecognized verb, wrong field count,
     or an unparseable field at this point behaves exactly like an
     adapter-level rejection (§8.2): `err <code> #<id>` follows the `ack`,
     using `ERR_UNKNOWN` (1) for an unrecognized verb or `ERR_BADARG` (2)
     for anything else. The malformed counter (`malformedCount()`) still
     increments for these — it tracks content/decode failures
     specifically, never sequencing outcomes (an out-of-order line is not
     "malformed", it is merely out of order, and does not increment it).

This is a strictly cleaner story than the old id-recovery rule it replaces:
there is no verb-specific carve-out to remember (the old rule needed one,
for `ESTOP`) because `ESTOP` is excluded at the top by verb identity, not
folded into the generic path with an exception bolted on.

### 8.5 Periodic emission — piggybacked on telemetry, still no timer

The scheme's loss-survival argument depends on `ack`/`nack` arriving
*regularly*, not only in direct response to a command — a host that sends
its last command and then goes quiet would otherwise never learn whether
that last id actually landed, or what `lastDone_` ended up being.

**`emitTelemetry()` now also emits the current reliability line** on every
call: `nack <expectedNext_> <lastDone_>` if `gapOutstanding_` is set,
`ack <expectedNext_ - 1> <lastDone_>` otherwise. Telemetry is already
periodic and application-driven (§3), so this rides that existing cadence
for free — **no timer, no clock, and no new entry point are added to the
handler** to make this happen, which is exactly the property §8.1
insisted on keeping. It doubles as the retransmit mechanism for a stalled
stream: as long as telemetry keeps flowing, a gap keeps producing fresh
`nack`s at the telemetry rate with no extra machinery.

#### 8.5.1 `lastDone_` — plumbed, not wired, in this library

`lastDone_` exists in the handler state because the *general* scheme needs
a completion channel (replacing the deleted `done` verb, §6.1) for whatever
future verb produces an asynchronous completion event. **This library has
no such verb.** `WHEELS` has no stop condition and no queue (§5.1); once
`onWheels()` returns, the command is fully handled from the protocol
layer's point of view — there is no later moment at which it "completes"
that this handler could observe without the timer/clock §8.1 explicitly
rules out. So in this library, **`lastDone_` is initialized to 0, reset to
0 by `HELLO`, included correctly in every `ack`/`nack`, and never written
anywhere else.** It is wire-correct (every ack/nack carries a real,
well-defined value) but functionally inert here. A future library that
adds `MOVE` (with a real stop condition or timeout) is where this field
would first be written — see §9.8 for why this is flagged as a resolved
ambiguity rather than a silent gap.

### 8.6 The `err` field-order fix

§2.2 (pre-2026-08-21) already stated the invariant "an id is always the
LAST token of its line, commands and replies alike" — but `replyErr()`
did not follow its own rule: it emitted `err #<id> <code>`, id first, code
last, undocumented as an exception. Nothing broke from this before now
because this library's own robot side never parses its own replies — but
an archetype (§9.4) must not carry an undocumented exception to its own
stated invariant, because a host parser written to that invariant
uniformly would silently mis-parse every `err` line. **Fixed: `err <code>
#<id>`.** The bare form (no id — which no longer exists at all now that
every `err` implies a prior `ack` for the same mandatory id) is retired
along with it.

### 8.7 Resync and wraparound

`status` gains a **`next=<expectedNext_>`** key (§6) so a reconnecting host
can resync its own tracking without forcing a full `HELLO` reset — useful
because a `HELLO` reset also clears `lastDone_`, which a host might not
want to lose. `status` does **not** also report `lastDone_` — flagged as a
gap in §9.8, not a considered omission.

Ids run **1 .. 999999** by host-side convention. **Modular wraparound is
explicitly out of scope and not implemented.** A session must reconnect
(`HELLO`) before exhausting the id space. Comparing `<`/`>` on a wrapping
counter is a classic bug source, and the space (999999 sequential
commands before a reconnect) is large enough in practice that wraparound
handling buys nothing but risk. The handler does not enforce the
999999 ceiling itself — ids are ordinary `uint32_t`s compared with plain
integer `<`/`==`/`>`, so nothing stops a host from counting past it, but
nothing in the design analysis above holds once it does; this is a
host-side discipline, not a wire-enforced limit.

---

## 9. As built — resolved ambiguities and known gaps

Everything above is design. This section is what the implementation actually
found, and it is the part to read before extending any of it.

### 9.1 Design calls this file made

**The first bullet below (the malformed-line `#id` recovery rule) is
historical, fully superseded by §8.4 (2026-08-21) — preserved for the
record, not current behavior.** The second and third bullets (the
5000 ms ceiling's ownership, and the id's stricter numeric grammar) are
still current and unaffected by the reliability layer.

**The malformed-line `#id` recovery rule is verb-agnostic, with one
deliberate exception.** §2.3 (historical)'s own words — "if the line's last token is a
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

### 9.2 Deliberately not implemented (historical — superseded 2026-08-21)

**This whole subsection describes a decision that no longer stands.** It is
kept, unedited below, as the changelog record of what this library used to
do and why §8 replaced it outright, rather than being deleted and leaving no
trace of the reasoning that came before.

> **The 3× reply repeat.** The wire's own design has an id-carrying `ok`/
> `err`/`done` sent three times on consecutive cycles, so an outcome
> survives radio frame loss without a ring or an eviction policy. This
> handler does **not** do that, because "on consecutive cycles" needs a
> periodic entry point and pending state — exactly what the
> no-`done`-for-`WHEELS` decision (§8) keeps out. The repeat is emission
> policy, owned by whatever drives a real per-cycle output loop — NOT a
> property of the line codec. This handler stays a pure function of its
> input bytes: it emits each id-carrying reply exactly once, and the
> repeat, if and when it is wanted, belongs to whatever drives a real cycle
> loop, not to this class.
>
> That is a real gap, not a rounding error: **loss tolerance is currently
> unimplemented** in this library. It belongs at the app or transport layer
> that owns a loop, or it comes back into the handler when `MOVE` and
> `done` do. Worth deciding deliberately rather than discovering on a lossy
> link.

**What actually happened:** the gap this subsection flagged was real, and
the stakeholder closed it 2026-08-21 — not by implementing the 3×-repeat
idea this subsection was written against, but by replacing the entire
delivery model with the cumulative sequence-id ack/nack scheme in §8. The
3× repeat is **deleted**, not implemented late: it would have meant sending
the SAME `ok`/`err` three times per id, which cannot coexist with a scheme
where the id itself is the sequence number and re-sending an old ack for a
stale id must NOT look like accepting a new command (§8.1's "do not
re-execute" case exists precisely to keep those two ideas from colliding).
Loss tolerance is no longer a gap: §8 IS this library's answer to it.

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

**Historical record, partially superseded by §8 (2026-08-21).** This
section is preserved as it was written, describing the separator/id-
spelling migration as it stood the day before the reliability layer
existed. Where it says an id is "optional" or describes `ok`/bare-`err`/
`#0`-suppression as current behavior, read those as **as of 2026-08-20
only** — §8/§8.6 replace all of it: every sequenced verb's id is now
mandatory, `ok` is deleted, and `#0` no longer suppresses anything. What
this section gets right and still holds: the SEPARATOR is still spaces,
the id is still spelled `#<n>` and still trails the line, and the
underlying object model (`Adapter`, `Result`, `Sink`, `Snapshot`/`Column`)
still did not change for THIS migration (§8 changed `Result`'s wire
mapping, not its own shape — see §9.8).

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
living in code comments** (the middle two bullets below are, like §9.6,
partially superseded by §8 the next day — `#0`-suppression is gone and
RUN's id is no longer detected by content inspection — annotated inline
rather than rewritten, since the REASONING each bullet records is still
sound even though the mechanism it describes changed):

- **`sendDebug("")` and `sendDebug(nullptr)` are the same case** (§6.2) —
  both emit the bare `debug\n` line. The alternative (making null a no-op
  that emits nothing at all) was rejected: `sendBanner()`/`sendReady()`
  never take a "should I even emit" argument, and giving `sendDebug()` a
  hidden suppression channel through its argument's nullness, distinct from
  the wire's own explicit `#0` suppression spelling used elsewhere in this
  file, would be a second, undocumented way to say "don't send this" with no
  wire vocabulary to describe it. **(`#0` no longer suppresses anything as
  of §8/§2.2 — the point about not inventing a SECOND suppression channel
  still stands, it just has one fewer sibling to be consistent with now.)**
- **Sanitize, don't reject**, for both `debug`'s text and `RUN`'s returned
  value (§6.2/§6.3). `sendDebug()` is `void` with no return channel at all;
  `RUN`'s outcome channel (`Result`) is owned by the *adapter's* own
  resolution/conversion/invocation logic, not by whether its return value
  happens to contain a newline, so reusing that channel to signal "your
  return value had a bad byte in it" would conflate two unrelated failure
  modes. Stripping degrades gracefully; rejecting outright would silently
  drop legitimate content over one bad byte with no way for either caller to
  learn that happened. (Unaffected by §8 -- still current.)
- **A last field beginning with `'#'` is always the id slot, even against
  `RUN`'s own open arity** (§6.3, as of 2026-08-20) — resolved by content
  inspection rather than by field count, because `RUN` is the one verb in
  this library whose arity the handler cannot know in advance. The
  consequence — a function's own final argument can never itself begin
  with `'#'` — is a genuine expressiveness limit, not an oversight, and is
  stated as such rather than left for a porter to discover by testing.
  **As of §8 (2026-08-21), content inspection is gone too: the id is
  UNCONDITIONALLY the last token for every sequenced verb, RUN included,
  so this is no longer RUN-specific machinery — but the consequence for a
  function's own final argument is unchanged, and if anything more
  absolute now (see §6.3's own updated text).**
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

### 9.8 The reliability layer (2026-08-21) — ambiguities this file resolved on its own

§8 is a stakeholder design brought to this library fresh; the brief settled
the shape but not every corner. Each item below is a real fork this
implementation hit, the choice made, and why — flagged explicitly rather
than picked silently, per this project's own stated practice.

1. **Does a handler-level decode failure (unknown verb, wrong arity,
   unparseable field) advance the sequence and get an `ack`, the same as an
   adapter-level rejection?** The brief's own worked example
   (`WHEELS 99999 0 100` rejected on range) only covers the adapter-Result
   case explicitly. **Resolved: yes, uniformly.** §8.2/§8.4 treat "the id
   was in order" as the ONLY gate for whether the sequence advances and an
   `ack` is sent; what happens after that (recognized verb vs. not,
   right arity vs. not, adapter accepts vs. rejects) all funnels into the
   same "maybe also emit `err`" step. The alternative — treating a
   handler-level decode failure as NOT consuming a sequence slot at all —
   would mean the host cannot tell "my malformed line was silently
   discarded, unsequenced" apart from "the wire ate it," which defeats the
   whole point of a delivery guarantee. Chosen for internal consistency:
   one rule, not two nearly-identical ones with a subtle carve-out.

2. **Does an out-of-order (`<` or `>` `expectedNext_`) line increment
   `malformedCount()`?** **Resolved: no.** `malformedCount()` tracks
   content/decode failures specifically (unchanged meaning from before
   2026-08-21); an out-of-order line's content is never even inspected
   (§8.4), so there is nothing to call malformed — it is a normal,
   expected occurrence on a lossy or reordering transport, not a protocol
   violation. Counting it would make `malformedCount()` climb on ordinary
   packet loss, which is precisely the noise this scheme exists to
   distinguish from real protocol violations.

3. **What does `lastDone_` actually track in a library with no queue and
   no completion event?** Addressed at length in §8.5.1: it is plumbed
   (correctly initialized, reset, and echoed) but never written past its
   initial 0, because nothing in this library's own verb set produces the
   asynchronous completion event the field exists to carry. This is
   flagged, not silently assumed, because it would be easy for a future
   reader to assume `lastDone_` tracks something (e.g. "the last accepted
   `WHEELS` id") when it deliberately does not — accepting a `WHEELS` is
   not the same event as it *completing*, and this library has no way to
   observe the latter without the timer §8.1 rules out.

4. **Reply ordering: does `ack` come before or after a verb's own
   informational/error reply?** Not stated in the brief. **Resolved:
   `ack` first, always**, emitted the instant an id is accepted as in
   order — before the verb is even looked up in the command table. Every
   other line the same command produces (`get`/`pong`/`id`/`ver`/`status`/
   `help`/`ret`/`err`) follows it. Chosen because it lets the accept
   decision and its wire evidence be emitted from one place
   (`dispatch()`), before delegating to per-verb logic that knows nothing
   about sequencing at all — simpler to implement and to reason about than
   threading "did I already ack this" state through every handler.

5. **Does `emitTelemetry()`'s piggybacked reliability line come before or
   after the `thdr`/`t` frame it accompanies?** Not stated. **Resolved:
   after** — `thdr` (if due), then `t`, then the `ack`/`nack` line. Purely
   a style choice with no behavioral consequence a test can observe
   differently either way; documented here so it does not read as an
   accident.

6. **Is `STATUS`'s wrong-arity case still recoverable against an `err`
   the way it implicitly was before?** `STATUS` is sequenced now, so a
   malformed `STATUS` (extra fields) still gets `ack` + `err 2 #<id>` as
   long as its id is in order (§8.4's item 3) — no special case needed,
   unlike `HELLO`'s.

7. **Does a malformed `HELLO` (wrong arity) get any reply at all?**
   **Resolved: no**, same as before this change — `HELLO` is outside the
   sequence entirely (§8.3), so there is no `ack` to anchor an `err`
   against, and inventing a bare `err` for an unsequenced verb would be a
   new, one-off wire shape with no counterpart anywhere else in this
   grammar. A malformed `HELLO` increments `malformedCount()` and produces
   no reply, exactly like a sequenced verb whose id cannot be determined
   at all (§8.4 item 1/2).

8. **Is `Result::kDuplicateId` (`ERR_DUPLICATE_ID`, code 11) still
   reachable?** **No — flagged, not removed.** §2.2 and §6.1 both call
   this out: the handler's own sequencing now guarantees the adapter is
   never handed a repeated id, so no code path in this library can produce
   code 11 any more. `Result::kDuplicateId` stays declared in `adapter.h`
   (removing an enumerator from a stakeholder-owned wire-outcome type is
   out of this change's scope), but it is dead as of this commit. A future
   caretaker should not spend time trying to find a test for it — there
   isn't one, and there cannot be one against this handler as designed.

### 9.9 §8, before this change (changelog, historical text)

Preserved verbatim for the record — this is what §8 said before the
2026-08-21 reliability-layer design replaced it outright (§8.0):

> ## 8. The one open question: should `WHEELS` emit `done`?
>
> The wire's outcome model has room for three reply verbs: `ok [#id]` means
> *accepted*, `err [#id] <code>` means *rejected*, and `done #<id> <reason>`
> would mean *the thing you enqueued has now finished*, where `reason` is
> `stop` (its stop condition was met) or `timeout` (its backstop fired).
>
> `done` was designed for `MOVE`, which has a real stop condition — "drive
> until 400 mm of travel" genuinely finishes. **`WHEELS` has no stop
> condition.** It holds a wheel speed for `duration` ms and then the lease
> expires. So "finished" means only "the lease ran out", and the question
> is whether that is worth a reply at all:
>
> - **Emit `done #<id> timeout`** — the host can await completion instead
>   of timing the wheels itself, and `WHEELS` behaves like every other
>   bounded command.
> - **Emit nothing; `ok` is the whole story** — the host already knows the
>   duration it asked for, so the reply carries no information it lacks.
>
> **The reason this is a design question and not a preference:** emitting
> `done` means the handler must remember outstanding ids and emit a reply
> *later, on its own*, which means it needs a periodic entry point and a
> notion of time. Without `done`, the handler is a pure function of the
> bytes fed to it — `feed()` in, replies out, no state between calls beyond
> a partial line, nothing to tick.
>
> That is a real difference in what the class *is*, and it is much cheaper
> to decide now than to retrofit. **Settled, 2026-08-20: no `done` for
> `WHEELS`** — the handler stays stateless and pure for this library;
> `done` arrives with `MOVE`, which is the verb that actually needs it,
> whenever that is built.

The "handler stays a pure function of the bytes fed to it, nothing to
tick" property this text worried about protecting is, notably, the SAME
property §8.1 protects in the new design (`expectedNext_`/`lastDone_`/
`gapOutstanding_` are ordinary state, not a clock) — the reliability layer
answers a different question (did the bytes arrive?) than `done` was going
to (did the motion finish?), and does so without reintroducing the timer
this section was written to keep out.

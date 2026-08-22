# Protocol handler — v6 line grammar, handler + adapter

**Status:** built and self-contained. This document is the wire format
*and* the design — there is no external spec file. §1–§8 are the design as
implemented; §9 is what the implementation actually found: resolved
ambiguities, one deliberate omission, and the gaps this work exposed,
including the 2026-08-20 space/`#id` grammar migration, the 2026-08-21
addition of `debug` and `RUN` (§6.2/§6.3), the 2026-08-21 reliability
layer — mandatory sequence ids plus cumulative `ack`/`nack`, replacing the
undelivered 3×-reply-repeat idea outright (§8) — and the 2026-08-22 changes
(§8.9): a decode failure now NAKs instead of acking, `ERR_DUPLICATE_ID` is
deleted, `PING` joins the unsequenced exemption set, `lastDone`/its reason
move from the handler to the Adapter, every `ack`/`nack` gains a reason
token, and the six [motion-api.md](motion-api.md) §9.1 verbs
(`WHEELS_X`/`WHEELS_V`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`, plus `STOP`'s
own `now` token) are implemented at the wire/handler layer (`WHEELS` is
renamed `WHEELS_V`).

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

- **Mandatory** on every sequenced verb (`ID VER STATUS HELP GET SET TLM
  WHEELS_X WHEELS_V MOVE_X MOVE_V GO_TO_R GO_TO_W STOP RUN` — see §8.3 for
  the three exceptions, `HELLO`, `ESTOP`, and (as of 2026-08-22) `PING`,
  none of which ever carry one at all). A sequenced verb arriving with no
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
- **`ERR_DUPLICATE_ID` (code 11) is DELETED (2026-08-22).** The old design
  had the *adapter* detect and reject a reused id. Under the reliability
  scheme the *handler* itself enforces strict monotonicity before an id
  ever reaches the adapter — an id is only ever dispatched when it exactly
  equals `expectedNext_`, which then advances past it, so the adapter can
  never be handed the same id twice. `Result::kDuplicateId` was kept
  declared-but-unreachable through 2026-08-21 (flagged in §9.8); the
  2026-08-22 pass removed the enumerator and its wire code entirely — it
  is structurally unreachable, not merely unused, and there is nothing left
  to keep it declared *for*. **Keep the duplicate-retransmit re-ack logic**
  (the `id < expectedNext_` → re-ack-without-re-executing row of §8.1's
  table) — that is a different thing, essential, and unaffected by this
  removal.

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

**Rewritten 2026-08-22** for the six stakeholder-directed changes. The
biggest structural change: `lastDone`/its completion reason MOVE here from
a `ProtocolHandler` field that nothing ever wrote (§8.8), `onWheels`
renames to `onWheelsV`, five new motion methods join it (one per
[motion-api.md](motion-api.md) §9.1 verb), `onStop` gains an `immediate`
argument, and `kDuplicateId` is deleted outright (§2.2).

```cpp
namespace Protocol {

// Maps 1:1 onto the wire outcome (§8.2): kOk means nothing further is
// emitted beyond the ack dispatch() already sent (no more standalone
// `ok`); every other value means an `err <code> #<id>` follows that same
// ack (id last, §8.6). kDuplicateId is GONE (2026-08-22) -- it was
// already structurally unreachable (§2.2), and there is nothing left to
// keep a deleted enumerator declared for.
enum class Result : uint8_t {
  kOk,             // → (ack alone; no further reply)
  kUnknown,        // → err 1 #<id>   ERR_UNKNOWN
  kBadArg,         // → err 2 #<id>   ERR_BADARG
  kRange,          // → err 3 #<id>   ERR_RANGE
  kFull,           // → err 4 #<id>   ERR_FULL
  kUnimplemented,  // → err 6 #<id>   ERR_UNIMPLEMENTED
  kNotReady,       // → err 8 #<id>   ERR_NOT_CONFIGURED
  kBusy,           // → err 10 #<id>  ERR_BUSY
};

// The reliability layer's completion-reason vocabulary (§8.8, motion-
// api.md §5.3): the four reasons a motion can finish, plus kNone for
// "nothing has completed yet" (lastDone() == 0's own pairing).
enum class DoneReason : uint8_t {
  kNone,     // → "none"
  kStop,     // → "stop"     -- the stop condition was met, or stop() ended it
  kTimeout,  // → "timeout"  -- the backstop fired
  kEstop,    // → "estop"    -- a panic stop ended it
  kAborted,  // → "aborted"  -- the caller abandoned it
};

class Adapter {
 public:
  virtual ~Adapter() = default;

  // ---- session ----
  virtual void identity(Identity& out) const = 0;   // name, serial, version, …
  virtual uint32_t now() const = 0;                 // [ms] for pong
  virtual void status(StatusFields& out) const = 0;

  // ---- motion: the six verbs (motion-api.md §9.1), plus STOP/ESTOP.
  // Angles (rotation, omega) arrive already decoded from the wire's
  // milliradian integers into float milliradians -- degrees-at-the-API
  // is a LANGUAGE BINDING's conversion, not this seam's.
  virtual Result onWheelsV(float left, float right,     // [mm/s] [mm/s]
                           uint32_t duration,            // [ms]
                           uint32_t id) = 0;             // renamed from onWheels
  virtual Result onWheelsX(float left, float right,     // [mm] [mm]
                           float cruise,                 // [mm/s]
                           uint32_t timeout,              // [ms]
                           uint32_t id) = 0;
  virtual Result onMoveX(float distance, float rotation,  // [mm] [mrad]
                         float cruise, uint32_t timeout,   // [mm/s] [ms]
                         uint32_t id) = 0;
  virtual Result onMoveV(float v_x, float omega,           // [mm/s] [mrad/s]
                         uint32_t duration, uint32_t id) = 0;  // [ms]
  virtual Result onGoToR(float x, float y, float speed,      // [mm] [mm] [mm/s]
                        float arrive, uint32_t timeout,       // [mm] [ms]
                        uint32_t id) = 0;
  virtual Result onGoToW(float x, float y, float speed,
                        float arrive, uint32_t timeout,
                        uint32_t id) = 0;
  virtual Result onStop(bool immediate, uint32_t id) = 0;  // STOP [now]
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

  // ---- the reliability layer's completion channel (§8.8, MOVED here
  // 2026-08-22 from a ProtocolHandler field nothing ever wrote) ----
  virtual uint32_t lastDone() const = 0;
  virtual DoneReason lastDoneReason() const = 0;

  // ---- invocation by name (§6.3) ----
  virtual Result onRun(const char* name,
                       const char* const* argv, size_t argc,
                       char* result, size_t resultCapacity,
                       bool& hasResult) = 0;
};

}  // namespace Protocol
```

`kUnimplemented` and `kBusy` are carried for completeness against the wire's
error-code space even though `DiffDriveAdapter` does not itself produce
them — `kUnimplemented` is reserved for a recognized-but-unwired verb,
`kBusy` for a subsystem refusing because it is mid-motion.

Returning a `Result` rather than writing a reply is the deliberate choice. It
means the adapter cannot emit a malformed reply, cannot forget to reply, and
cannot invent a reply shape — the handler does all three, once, for every verb.

`onEstop()` returns `void` on purpose: `ESTOP` never carries a sequence id and
is never part of the ack/nack scheme, because it must not queue behind
anything, including an ack (§8.3). The handler itself still replies `estop`
after calling this — that reply is not this method's own concern; it is
formatted and sent by `ProtocolHandler`, exactly like every other reply.

`onStop`'s `immediate` flag carries `STOP now`'s own request (motion-
api.md §3.7/§9.1) — a deceleration CHOICE, not a different verb.
`DiffDriveAdapter` accepts and ignores it (its `neutral()` was already
immediate either way, §5.1); an adapter that owns a real ramp is where the
flag first makes a behavioral difference.

### 4.1 `lastDone()`/`lastDoneReason()` — the reliability layer's
completion channel, now Adapter-owned

The handler POLLS these two methods fresh every time it formats an
`ack`/`nack` line (§8.1/§8.5) — no callback, no clock, no cached copy
anywhere in `ProtocolHandler`. Monotonic contract: a later `lastDone()`
value implies every earlier id has also completed, since this library's
motion runs one command at a time, in order. An adapter with no
completion event of its own (`DiffDriveAdapter`, §8.8.1) returns `0`/
`kNone` forever — wire-correct, functionally inert on that adapter
specifically. See §8.8 for the full story, including why this moved off
the handler.

---

## 5. The DiffDrive adapter is where geometry lives

The concrete adapter that closes the loop for testing:

```
WHEELS_V <left> <right> <duration> #<id>
          [mm/s]  [mm/s]   [ms]
              │
              ▼   scale by countsPerLength [counts/mm]   ← the robot's geometry
        left, right  [counts/s]
              │
              ▼   velocity = (left+right)/2 ,  twist = (right-left)/2
   DifferentialDrive::drive(velocity, twist, lease=duration)
```

**Renamed from `WHEELS`/`onWheels` (2026-08-22)** — same fields, same
meaning; [motion-api.md](motion-api.md) §9.2 confirms `WHEELS` *is*
`wheels_v`. `DiffDriveAdapter` implements only this one motion verb for
real. The other five (`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`)
all answer `Result::kUnknown` on `DiffDriveAdapter` specifically — it has
no planner, which is honest and already documented (this same posture
`RUN` already takes for an adapter with an empty registration table, §6.3)
— **not** `kUnimplemented`, a deliberate choice recorded in §9.8.

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

**There is no queue in this library**, because `DiffDriveAdapter` has no
planner — `WHEELS_V` reaches `drive()` directly, and the other five motion
verbs answer `kUnknown` on it (§5). So neither `STOP` nor `ESTOP` "waits its
turn behind an active move" on this adapter; that framing belongs to a full
motion-planner robot (see [motion-api.md](motion-api.md) §3.7/§9.1, which
specifies `stop`/`stop(immediate=True)`/`estop` as a richer three-way choice
this library's own `DiffDriveAdapter` collapses onto one behavior). What
`STOP [now] #<id>` and `ESTOP` actually do, traced to the kernel:

| verb | adapter call | kernel effect |
|---|---|---|
| `STOP [now] #<id>` | `onStop(immediate, id)` → `drive_.neutral()` | writes a neutral command to the mailbox; duty zeroes **this cycle**, immediately — `stageStop()` is a bare `stageDuty(0, 0)`, no ramp. `immediate` (the optional `now` token, motion-api.md §3.7/§9.1) is accepted but has no effect here — see §9.8 |
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

Enough to test DiffDrive over the wire, plus the full six-verb motion
surface [motion-api.md](motion-api.md) §9.1 specifies — the WIRE and
HANDLER side of all six is implemented; `DiffDriveAdapter` itself only
gives one of them (`WHEELS_V`) real effect (§5).

**Every row marked "sequenced" below requires a mandatory `#<id>` — see
§8.** The reply column shows each verb's own *informational* reply only;
every sequenced verb ALSO emits the transport-layer `ack`/`nack`
described in §8, as a separate line, alongside whatever is shown here.
**`PING` is unsequenced as of 2026-08-22** — it joins `HELLO`/`ESTOP` in
never carrying an id (§8.3).

| verb | sequenced? | command | own reply | notes |
|---|---|---|---|---|
| `HELLO` | no | — | `device NEZHA2 robot <name> <serial>` | resets the sequence (§8.3) |
| `PING` | **no** (2026-08-22) | — | `pong <now>` | `now` = robot clock `[ms]`; maximally forgiving, like `ESTOP` — see §8.3, §9.8 |
| `ID` | yes | `#id` | `id <drivetrain> <profile> <version>` | |
| `VER` | yes | `#id` | `ver <version>` | |
| `STATUS` | yes | `#id` | `status ready=1 active=0 connL=1 connR=1 otos=0 wedge=0 flags=<hex> tlm=off next=<n>` | `k=v`, order not guaranteed, unknown keys ignored |
| `HELP` | yes | `#id` | `help HELLO PING ID VER STATUS HELP GET SET TLM WHEELS_X WHEELS_V MOVE_X MOVE_V GO_TO_R GO_TO_W STOP ESTOP RUN` | rest-of-line; generated from the same table `dispatch()` uses, so it cannot drift |
| `GET` | yes | `[name] #id` | `get name value` (one field) or one `get` line per field (bare `GET`) | unknown name → no `get` line, but still acked (§8.1) |
| `SET` | yes | `name value #id` | — (accepted: none; rejected: `err <code> #<id>`) | an in-order `ack` **is** the acceptance; a value that fails to PARSE is a decode failure (§8.9), not a rejection |
| `TLM` | yes | `mode #id` | — | `OFF`/`POSE`/`FULL`/`NOW`/`AUTO`/`BUFFER` decoded; an unrecognized mode token is a decode failure (§8.9); the adapter's own `Result` never surfaces on the wire |
| `WHEELS_X` | yes | `left right cruise timeout #id` | — | per-wheel commanded DISTANCE, bounded by encoder travel + `timeout` (motion-api.md §3.1); `kUnknown` on `DiffDriveAdapter` (§5) |
| `WHEELS_V` | yes | `left right duration #id` | — | **renamed from `WHEELS`** (2026-08-22) — maps onto `drive()` with no planner (§5) |
| `MOVE_X` | yes | `distance rotation cruise timeout #id` | — | body displacement + heading (motion-api.md §3.3); `kUnknown` on `DiffDriveAdapter` |
| `MOVE_V` | yes | `v_x omega duration #id` | — | body twist held for `duration` (motion-api.md §3.4); `kUnknown` on `DiffDriveAdapter` |
| `GO_TO_R` | yes | `x y speed arrive timeout #id` | — | drive to a robot-frame point (motion-api.md §3.5); `kUnknown` on `DiffDriveAdapter` |
| `GO_TO_W` | yes | `x y speed arrive timeout #id` | — | drive to a world-frame point (motion-api.md §3.6); `kUnknown` on `DiffDriveAdapter` |
| `STOP` | yes | `[now] #id` | — (accepted: none; rejected: `err <code> #<id>`) | the optional `now` token (motion-api.md §3.7/§9.1) sits safely before the id since the id is self-marking; see §5.1 |
| `ESTOP` | **no** | — | `estop` | never sequenced, never nacked, maximally forgiving; see §5.1/§8.3 |
| `RUN` | yes | `function [arg...] #id` | `ret <value> #<id>` (accepted, function returned a value) / — (accepted, void) / `err <code> #<id>` (rejected) | invocation by name; see §6.3 |
| — | — | — | `debug <text>` | robot-to-host ONLY, no inbound wire form; see §6.2 |
| — | — | — | `ack <n> <lastDone> <reason>` / `nack <n> <lastDone> <reason>` | transport layer; the `<reason>` token is NEW 2026-08-22 — see §8.8 |

Angles (`rotation`, `omega`) are milliradian integers on the wire
(motion-api.md §9.1) — degrees only exist at a language binding, which is
not this library's concern; the handler decodes them with the ordinary
signed-integer field parser, the same as any other field.

`WHEELS_X` and the four body/positional verbs (`MOVE_X`/`MOVE_V`/
`GO_TO_R`/`GO_TO_W`) have no prior wire form before 2026-08-22 — they were
`motion-api.md`'s own proposal, now implemented at the wire/handler layer.
`SEED`/`CAL` remain deferred: they need OTOS/odometry this library does not
own.

### 6.1 Outcomes

**Rewritten 2026-08-21, updated 2026-08-22 — see §8 for the full design.**
`ok` is deleted: acceptance is signaled by the transport-layer `ack`
alone. `done` is deleted as a standalone verb: the `lastDone`/`<reason>`
pair carried by every `ack`/`nack` is the completion channel.

| reply | meaning |
|---|---|
| `ack <n> <lastDone> <reason>` | transport: the highest in-order id accepted so far arrived correctly (§8.1). `<reason>` (NEW 2026-08-22, §8.8) describes `lastDone` — `none` when `lastDone` is 0 |
| `nack <n> <lastDone> <reason>` | transport: `n` is the next id the robot actually needs — either a numeric gap, OR (2026-08-22, §8.9) an in-order id whose own content was a DECODE FAILURE and so was never accepted at all |
| `err <code> #<id>` | application: a command's *content* was rejected — either a MERITS rejection (arrived intact, the adapter's own `Result` refused it, §4) or the "content" half of a decode failure (§8.9). Field order: the id is always the LAST token, matching every other line in this grammar (§8.6). |
| `estop` | `ESTOP` only — confirms the stop executed (§8.3) |
| `ret <value> #<id>` | `RUN` only (§6.3) — the invoked function returned a value, emitted IN ADDITION to the `ack` |

**A command is never "just" accepted or "just" rejected in isolation** —
but which TRANSPORT reply it gets now depends on which of two kinds of
rejection it is (§8.9, the central 2026-08-22 change):

- **Decoded fine, refused on merit** (e.g. an out-of-range speed): `ack`
  (it arrived, the sequence advances) **plus** `err <code> #<id>`.
- **Decode failure** (unknown verb, wrong arity, unparseable field — the
  line did not arrive intact): `nack <n> <lastDone> <reason>` (the
  sequence does NOT advance — `n` names the SAME id that just failed)
  **plus** `err <code> #<id>`.

| code | name | meaning |
|---|---|---|
| 1 | `ERR_UNKNOWN` | no such verb or field name |
| 2 | `ERR_BADARG` | malformed/non-finite argument, wrong arity |
| 3 | `ERR_RANGE` | declared bound violated |
| 4 | `ERR_FULL` | queue full |
| 6 | `ERR_UNIMPLEMENTED` | recognized, not wired on this build |
| 8 | `ERR_NOT_CONFIGURED` | refused pre-`ready` |
| 10 | `ERR_BUSY` | subsystem in motion; retry at rest |

Code 11 (`ERR_DUPLICATE_ID`) is **deleted, not merely unreachable, as of
2026-08-22** — see §2.2/§9.8.

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

Handler state, in full (2026-08-22: `lastDone_` is GONE from this
list — see §8.8):

```cpp
uint32_t expectedNext_ = 1;   // next sequence id expected from the host
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
| `== expectedNext_` | decode the verb's own fields FIRST (§8.9); only if that succeeds, dispatch to the adapter and `expectedNext_ = id + 1` | `ack <id> <lastDone> <reason>` on a decode success; `nack <expectedNext_> <lastDone> <reason>` on a decode failure (§8.9) |
| `< expectedNext_` | **do NOT re-execute** — a retransmit whose ack was lost | `ack <expectedNext_ - 1> <lastDone> <reason>` |
| `> expectedNext_` | **discard, do NOT execute** — a numeric gap | `nack <expectedNext_> <lastDone> <reason>` |

`<lastDone>`/`<reason>` are read fresh off `Adapter::lastDone()`/
`lastDoneReason()` (§8.8) every time either reply is formatted — this
table's middle column used to read a handler-owned `lastDone_` field
before 2026-08-22.

The middle row is the one easy to get wrong: a resent `WHEELS_V` (the host
never saw the first ack, so it resends) must **not** drive the wheels a
second time. The reply for a retransmit echoes the *already-accepted*
id (`expectedNext_ - 1`), not the resent one — telling the host "I already
have everything through here," which is exactly what a resend needs to
hear to stop resending.

A gap **stalls the stream on purpose**: every subsequent command, however
well-formed, is discarded and nacked until the missing id arrives, giving
strict in-order delivery. Because every new command re-triggers the same
`nack <expectedNext_> ...`, a lost `nack` self-heals — the host will see
the next one along with the next command it sends. **A decode failure on
an in-order id (§8.9) holds the stream exactly the same way** — the failed
id itself becomes the thing every subsequent nack keeps asking for, until
a well-formed line finally supplies it.

`nack` carries **next-expected**, not "last good id": it tells the host
exactly what to resend with no `+1` inference on either side, and it avoids
overloading `0` as both "nothing accepted yet" and "resend from here."

### 8.2 Layering: `ack`/`nack` is transport, `err` is application

**Updated 2026-08-22 — see §8.9 for the change this section's own
"unknown verb" paragraph below is superseded by.** `ack`/`nack` answers
one question only: *did the bytes arrive, in order, INTACT?* `err`
answers a different one: *was the content accepted, once it was known to
have arrived?* A message can be perfectly in-order, decode fine, and
still be garbage on the merits (`WHEELS_V 99999 0 100 #7` decodes cleanly
and is rejected on range by the adapter). So an in-order command the
ADAPTER rejects on merit emits **both**: the `ack` (it arrived, decoded,
the sequence advances) **and** `err <code> #<id>` (§6.1) — two lines. The
error code is never folded into `ack` itself; that would conflate a
transport signal with an application one.

**This does NOT generalize to a handler-level decode failure any more
(2026-08-22, reversing the pre-2026-08-22 text this paragraph used to
carry).** An unknown verb, a known verb with the wrong field count, or an
unparseable field is a DIFFERENT case from a merits rejection — the line
did not "arrive fine", so it NACKs (§8.9), not acks. See §8.9 for the full
story and the stakeholder's own rationale for keeping the two distinct.

**Every reply so far bare `ok`/`err` gains a mandatory id, and `ok` itself
is gone.** `SET`/`WHEELS_V`/`STOP`/`RUN`(void) success now produces
*nothing* beyond the `ack` — the ack **is** the acceptance signal. `RUN`'s
`ret` is the one exception: a returned value is genuinely new information
the `ack` alone cannot carry, so it is still emitted, **in addition to**
the ack, not instead of it (§6.3).

### 8.3 `ESTOP`, `HELLO`, and `PING` — the exemption set

**Sequenced:** `ID VER STATUS HELP GET SET TLM WHEELS_X WHEELS_V MOVE_X
MOVE_V GO_TO_R GO_TO_W STOP RUN`.
**Unsequenced:** `ESTOP`, `HELLO`, and (as of 2026-08-22) `PING`.

The stakeholder's own framing was "every message must have an ID number" —
`PING`'s own exemption is a LATER, explicit stakeholder direction
(2026-08-22, verbatim: *"ESTOP, ping, and HELLO shouldn't require IDs"*),
not this file's own call the way `ESTOP`/`HELLO`'s exemption originally
was. All three are exempted because the scheme is structurally
unbootstrappable and unsafe without them:

- **`HELLO` resets the sequence.** `expectedNext_ = 1`,
  `gapOutstanding_ = false`, then the banner is emitted — this is the
  session-start resync a host performs on (re)connect. **It does NOT
  reset the Adapter's own `lastDone()`/`lastDoneReason()`** any more
  (2026-08-22, §8.8) — that state moved off this class entirely, and a
  handler-level reset has no business reaching into the Adapter to clear
  something it does not own. A verb that resets the sequence cannot
  itself be *inside* the sequence without a chicken-and-egg problem (what
  id would the very first `HELLO` carry, and against what would it be
  checked?). `HELLO`'s own arity is unchanged (zero fields) — it does not
  accept a trailing id at all, and a `HELLO` with one is wrong arity, same
  as any other extra field.
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
- **`ESTOP` REPLIES.** Stakeholder, verbatim: *"Agree about ESTOP, but
  if it is not acked, it should be acknowledged, with an `estop`
  response."* Executing the stop **before writing the reply** means a
  panic stop never queues behind an outbound reply. `ESTOP`'s reply is
  the bare word `estop`, no fields, ever.
- **`PING` is liveness and must answer even while the stream is stalled on
  a gap** (2026-08-22, the stakeholder's own words) — the same structural
  reason `ESTOP` is exempted: a command gated behind a missing id cannot
  serve as a liveness probe for the very link that id is missing on.
  `PING`'s own reply (`pong <now>`) is unchanged — it never carried an
  `ack`/id of its own even before this change, so the exemption is purely
  about the SEQUENCE GATING, not the reply shape.
- **Whether `PING` should be maximally forgiving (like `ESTOP`) or strict
  zero-arity (like `HELLO`) is THIS FILE'S OWN CALL**, not spelled out by
  the stakeholder's direction — resolved forgiving (§9.8), so an old-style
  host still appending `#<id>` to `PING` out of habit from before this
  change keeps working unchanged.

This exemption set is deliberately narrow — flagged here prominently so the
stakeholder can find and overrule it easily: everything else in this
library's scope is sequenced, with no other carve-outs.

### 8.4 Malformed-line recovery under mandatory sequencing (historical — superseded by §8.9)

**This whole section describes the 2026-08-21 design, replaced by §8.9's
2026-08-22 rewrite.** Preserved for the changelog record, the way §8.0/
§9.2 preserve the eras before them. The pre-2026-08-21 malformed-line
recovery rule (§2.3, historical) is gone. In its place, for any line whose
verb is neither `ESTOP` nor `HELLO` (§8.3) and does not start lowercase
(§2.1, unchanged):

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
   - in order (`== expectedNext_`) → **the sequence advances and the
     `ack` is sent UNCONDITIONALLY; then, and only then, the verb is
     looked up and its fields validated.** *(This is exactly the step
     §8.9 changes: as of 2026-08-22, decoding happens BEFORE the ack/nack
     decision, not after.)* An unrecognized verb, wrong field count, or
     an unparseable field at this point behaved exactly like an
     adapter-level rejection (§8.2, its own pre-2026-08-22 text): `err
     <code> #<id>` followed the `ack`, using `ERR_UNKNOWN` (1) for an
     unrecognized verb or `ERR_BADARG` (2) for anything else.

This was a strictly cleaner story than the id-recovery rule it replaced (no
verb-specific carve-out to remember, since `ESTOP` was excluded at the top
by verb identity) — but it could not tell a garbled square-tour leg apart
from a merits-rejected one on the wire, which is exactly the gap §8.9
closes.

### 8.5 Periodic emission — piggybacked on telemetry, still no timer

The scheme's loss-survival argument depends on `ack`/`nack` arriving
*regularly*, not only in direct response to a command — a host that sends
its last command and then goes quiet would otherwise never learn whether
that last id actually landed, or what the current `lastDone`/reason ended
up being.

**`emitTelemetry()` also emits the current reliability line** on every
call: `nack <expectedNext_> <lastDone> <reason>` if `gapOutstanding_` is
set, `ack <expectedNext_ - 1> <lastDone> <reason>` otherwise, where
`<lastDone>`/`<reason>` are read FRESH off `Adapter::lastDone()`/
`lastDoneReason()` (§8.8) on every single call — there is no cached copy
anywhere in `ProtocolHandler`. Telemetry is already periodic and
application-driven (§3), so this rides that existing cadence for free —
**no timer, no clock, and no new entry point are added to the handler** to
make this happen, which is exactly the property §8.1 insisted on keeping.
It doubles as the retransmit mechanism for a stalled stream: as long as
telemetry keeps flowing, a gap (numeric OR a decode failure, §8.9) keeps
producing fresh `nack`s at the telemetry rate with no extra machinery.

#### 8.5.1 The completion channel's own home — see §8.8

This subsection (as it stood through 2026-08-21) described `lastDone_` as
handler-owned state, "plumbed, not wired" in this library because
`WHEELS` had no completion event of its own. §8.8 below is the full
2026-08-22 replacement: the field moved OFF the handler entirely, onto
`Adapter`, and the "plumbed but inert" story now applies specifically to
`DiffDriveAdapter`, not to every adapter this library can host — a
step()-driven test adapter (`tests/protocol/fake_motion_adapter.h`) makes
it genuinely live for the first time.

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
can resync its own tracking without forcing a full `HELLO` reset. As of
2026-08-22 (§8.8), a `HELLO` reset no longer clears any completion state at
all — `lastDone()`/`lastDoneReason()` live on the Adapter and are
untouched by the handler's own reset — so the original motivation for this
key ("useful because a HELLO reset also clears lastDone_") is narrower
than it was, but `next=` remains useful on its own merits (resync without
re-establishing the session). `status` still does **not** also report
`lastDone`/its reason — flagged as a gap in §9.8, not a considered
omission.

### 8.8 `lastDone`/`lastDoneReason` move to the Adapter (2026-08-22)

**Stakeholder direction, verbatim:** *"[lastDone] is currently a handler
counter that nothing ever writes, so it is permanently 0 — wire-correct
and inert. Replace it with a virtual on `Adapter` that the handler
POLLS whenever it formats an `ack`/`nack`."*

```cpp
// Most recently completed motion, for the reliability piggyback. Monotonic:
// a later value implies every earlier one completed (motion runs one at a
// time, in order), which is what makes a dropped completion recoverable.
virtual uint32_t lastDone() const = 0;
virtual DoneReason lastDoneReason() const = 0;   // see §8.8.1
```

This removes handler state entirely (`ProtocolHandler` now carries only
`expectedNext_`/`gapOutstanding_`, §8.1), needs no callback and no clock,
and makes the field genuinely live once an Adapter that actually completes
motions drives it — which nothing in this library did before this change.
`replyAck()`/`replyNack()` call `adapter_.lastDone()`/`lastDoneReason()`
fresh every time they run; there is no cache.

**A real, undecided design fork this file resolves on its own:** the
brief settled the VALUE (`lastDone`) but not whether `HELLO`'s own reset
should reach into the Adapter to clear it too — the pre-2026-08-22 text
had HELLO reset `lastDone_ = 0` as part of the same call, back when it was
handler state. **Resolved: no.** `HELLO`'s reset now only touches
`expectedNext_`/`gapOutstanding_` — state the HANDLER owns. Reaching
across the seam to clear something the ADAPTER now owns would reintroduce
exactly the kind of handler-into-adapter coupling this whole move was
meant to avoid, and there is a real argument that a completed motion
SHOULD survive a reconnect (the host may be re-establishing a session
after a dropped link, not asking the robot to forget what it just did). An
adapter that wants HELLO to also clear its own completion state is free to
do so from wherever it observes HELLO itself.

#### 8.8.1 `DoneReason` — the conflict the reliability-layer brief didn't settle, and this file's resolution

The reliability change deleted standalone `done` and collapsed it into a
piggybacked *number* (`lastDone`). But [motion-api.md](motion-api.md) §5.3
defines FOUR completion **reasons**: `stop`, `timeout`, `estop`,
`aborted`. A bare number loses the reason, and `estop` vs `stop` is a
distinction that matters (a program that ended normally vs. one that got
panic-stopped mid-leg are very different facts for a host to learn).

**Resolved by piggybacking the reason too:** `ack <n> <lastDone> <reason>`
and `nack <n> <lastDone> <reason>`, where `reason` describes `lastDone`.
This keeps the loss-tolerance property (a later ack re-carries it) *and*
the reason vocabulary, at one extra token per ack/nack. `none` is the
reason when `lastDone` is 0 (nothing has completed yet).

```cpp
enum class DoneReason : uint8_t {
  kNone, kStop, kTimeout, kEstop, kAborted,
};
```

**This is this library's own resolution of a conflict the reliability-
layer discussion did not settle — flagged prominently so the stakeholder
can find and overrule it.** The alternative shapes considered and
rejected:

- *A separate reason-only reply, not piggybacked* (e.g. a standalone
  `done #<id> <reason>` line reintroduced) — rejected because it brings
  back exactly the "the handler must remember outstanding ids and emit a
  reply later, on its own" problem §8.0/§9.9 already killed the old
  `done` design over: it would need its own delivery guarantee, which is
  the whole thing this scheme already provides for `lastDone` itself.
  Piggybacking the reason onto the SAME line that already survives loss
  costs one token and buys nothing to re-engineer.
- *Fold the reason into a wider `lastDone` encoding* (e.g. high bits of a
  32-bit value) — rejected as needlessly clever for a text protocol whose
  whole design philosophy (§2) is "every wire value is a base-10 ASCII
  integer" with no bit-packing anywhere else in the grammar.

**A dependent design point, also resolved here:** does `WHEELS_X`/
`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W` on `DiffDriveAdapter` answering
`kUnknown` (§5) rather than `kUnimplemented` cost anything for
`lastDone`/`DoneReason`? No — a merits rejection never touches
`lastDone`/`lastDoneReason` at all; those two fields only move when an
Adapter's own motion genuinely COMPLETES, which never happens for a verb
that was refused outright.

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

### 8.9 Decode failure is a NAK (2026-08-22)

**Stakeholder, verbatim:** *"I think a decode failure is a NAK. The goal
here is that the movements don't work if you put them out of order... If
you're driving a square and you've got eight movements you send, and you
lose a turn, the whole square is wrong. The best thing to do there is to
NAK and resend from that point on, so we need to make decode failures be
NAK and err."*

This **reverses** §8.2/§8.4's own pre-2026-08-22 behavior, where a decode
failure acked and advanced the sequence exactly like a merits rejection.
Two distinct cases, kept distinct on the wire:

| case | meaning | sequence | reply |
|---|---|---|---|
| **decode failure** — unknown verb, bad arity, unparseable field | the line did not arrive intact | **does NOT advance** | `nack <expectedNext_> <lastDone> <reason>` (naming the SAME id, unchanged) **and** `err <code> #<id>` |
| **adapter rejection** — decoded fine, refused on merit (e.g. out-of-range speed) | arrived intact, refused | **advances** | `ack <id> <lastDone> <reason>` **and** `err <code> #<id>` |

The distinction is the whole point: a corrupted line should be resent
(resending the merits-rejected line would just be refused again,
identically, so THAT case still advances and moves on).

**Where the decode now happens.** `dispatch()` still resolves the
mandatory id first (§8.1's three-way compare, unaffected) — but for the
`id == expectedNext_` case, it now looks up the verb AND decodes its own
fields (arity + per-field parseability) **before sending any reply at
all**. Only once decoding succeeds does the sequence advance and the
`ack` go out; a decode failure at this point (verb not found, wrong
field count, an unparseable numeric field, an unrecognized `TLM` mode,
`STOP`'s trailing token being anything other than the literal `now`, a
bare `RUN` with no function name, or more raw `RUN` tokens than the
handler's fixed storage can hold) instead calls a dedicated path that
NACKs `expectedNext_` (still equal to the failed id, since it was never
accepted) and sets `gapOutstanding_` so a stalled stream keeps re-nacking
at the telemetry rate (§8.5) exactly like a numeric gap would, until a
well-formed line finally supplies the same id. `malformedCount()` still
increments for a decode failure, exactly as it did before this change.

**What does NOT change:** a numeric gap (`id > expectedNext_`) is still
classified and replied to WITHOUT ever looking up the verb at all — that
row of §8.1's table is untouched. A stale retransmit (`id < expectedNext_`)
is also untouched. Only the `id == expectedNext_` row's own internal
order changed (decode, then reply — not reply, then decode).

**The hazard this creates, stated plainly:** because a decode failure
never advances the sequence, **a host that genuinely CONSTRUCTS a
malformed line (a real bug on the host side, not packet corruption) will
be NACKed forever on that same id, and the stream wedges.** This is a
deliberate tradeoff — fail loud and stall rather than silently continue a
broken sequence — but it means **the host needs its own give-up path**:
a resend limit, a timeout, or an operator-visible stall detector that
eventually reconnects (`HELLO`) or aborts the sequence rather than
retrying the same bad line forever. This library does not, and structurally
cannot, supply that give-up path itself — it has no clock (§8.1) and no
notion of "how many times has this been retried," and inventing one would
reintroduce exactly the timer/pending-state machinery §8.0/§8.5 keep out
on purpose. Say this to every porter and every host implementation: a
decode failure is not a transient condition the wire protocol resolves on
its own; it needs an application-level backstop.

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

**`WHEELS_V`'s 5000 ms ceiling** (unchanged by the 2026-08-22 rename) is
prose at the verb-definition level with no stated owner in the grammar.
The handler holds no bounds table, so **the adapter enforces it** and
returns `kRange` above it. The same "handler holds no bounds table"
posture applies to every OTHER motion verb's own documented bound
(`WHEELS_X`/`MOVE_X`'s `timeout`, etc.) — none of them are enforced by
`ProtocolHandler` either.

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

### 9.10 The six stakeholder-directed changes (2026-08-22)

Six changes in one pass: decode failure is a NAK (§8.9), `ERR_DUPLICATE_ID`
deleted (§2.2/§6.1), `PING` unsequenced (§8.3), `lastDone`/its reason move
to the Adapter (§8.8), the ack/nack reason token (§8.8.1), and the six
motion-api.md §9.1 verbs implemented at the wire/handler layer (§6,
`WHEELS` renamed `WHEELS_V`). Ambiguities this pass resolved on its own,
in the same spirit as §9.8's own list for the pass before it:

1. **`DiffDriveAdapter`'s five unimplemented motion verbs answer
   `kUnknown`, not `kUnimplemented`.** The ticket driving this pass said
   so explicitly, and it matches an existing precedent this file already
   set: `RUN` on an adapter with an empty registration table answers
   `kUnknown` for every name, "the same wire outcome any name a real
   registration table would not recognize" (§6.3) — not `kUnimplemented`,
   even though a real registration table conceivably COULD recognize the
   name someday. The same reasoning applies here: `DiffDriveAdapter` has
   no planner at all, so `WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`
   are, from its own point of view, simply not verbs it knows anything
   about — `kUnimplemented` would suggest a build flag or a half-wired
   feature, when the honest description is "this adapter has no planner,
   period." A future adapter that DOES have a planner but deliberately
   ships with one of the six verbs turned off is where `kUnimplemented`
   would be the correct choice instead.
2. **`PING`'s own exemption is maximally forgiving, not strict
   zero-arity.** The stakeholder's direction ("ESTOP, ping, and HELLO
   shouldn't require IDs") settled that PING is unsequenced but not
   whether it should tolerate trailing content the way `ESTOP` does or
   reject it the way `HELLO` does. Resolved forgiving, matching `ESTOP` —
   see §8.3's own bullet for the reasoning (an old-style host still
   appending `#<id>` to `PING` out of habit keeps working; liveness must
   not itself be refusable over a syntax nit, echoing exactly why `ESTOP`
   is forgiving).
3. **`HELLO`'s reset no longer touches the Adapter's own completion
   state.** The pre-2026-08-22 text had `HELLO` reset `lastDone_ = 0` as
   part of the same call, back when it was handler state (§8.3's old
   text). Now that it lives on the Adapter (§8.8), `HELLO`'s reset only
   touches `expectedNext_`/`gapOutstanding_` — see §8.8's own reasoning
   for why reaching across the seam would be the wrong call.
4. **`ack`/`nack` is sent BEFORE the verb's own execute step runs, even
   for a verb (like `STOP`) whose OWN execution can synchronously
   complete a motion.** This means `STOP`'s own ack reflects
   `lastDone`/`lastDoneReason` as they stood immediately BEFORE that
   STOP executed — the completion IT just caused becomes visible
   starting with the NEXT reply (a later command's ack, or the next
   telemetry-piggybacked line), not on its own ack. This preserves
   dispatch()'s uniform "ack always precedes verb execution, for every
   verb" structure (§8.2/§9.8 item 4 of the pass before this one) rather
   than special-casing STOP to execute-then-ack. Nothing is ever LOST —
   the value is not lost, only delayed by one reply — but a host reading
   `STOP`'s own ack literally should not expect to see the completion it
   just caused reflected there yet.
5. **A decode failure's `nack` and `err` share the SAME id, by
   construction, not by a separate check.** Because a decode failure is
   only reachable via the `id == expectedNext_` branch (§8.9), the id
   that failed to decode IS `expectedNext_` at the moment the nack is
   formatted — there is no separate bookkeeping needed to keep the two
   numbers in sync, and no way for them to drift apart.
6. **The decode/execute split is per-verb, not global.** Rather than a
   single generic "parse this line, then maybe execute" pass,
   `ProtocolHandler` pairs each verb with its own `decode` function (pure
   arity/field-parseability check, no adapter call, no sink write) and
   `execute` function (runs only once decoding has already succeeded, and
   only after `dispatch()` has already sent the `ack`). The two re-derive
   the same fields independently rather than threading decoded values
   between them — a small, deliberate duplication (this is not a hot
   path) that keeps "what counts as decodable" defined in exactly one
   place per verb without a shared decoded-argument struct per verb.

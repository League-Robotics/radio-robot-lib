# Protocol handler — v6 line grammar, handler + adapter

**Status:** built. §1–§7 are the design as implemented; §8's question was
settled (**no `done` for `WHEELS`**, 2026-08-20); **§9 is what the
implementation actually found** — resolved spec ambiguities, one deliberate
omission, and the gaps this work exposed, including the 2026-08-20
colon-to-space/`#id` grammar migration (§9.6). Read §9 before extending any
of it. The wire format itself is
[protocol-v6-spec.md](../protocol-v6-spec.md).

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
  void sendBanner();                        // device NEZHA2 robot <name> <serial>
  void sendReady();                         // ready
  void emitTelemetry(const Snapshot& snapshot);  // thdr once, then t per frame

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

Per spec §11.1 the entire codec is a tokenizer over runs of `' '`
(originally a split on `':'` — see §9.6 for the 2026-08-20 grammar
migration). Firmware constraints (§11.2) mean no dynamic allocation and no
`std::string`: the handler owns a fixed `char[240]` line buffer, and
tokenizing overwrites separator spaces with `\0` to produce field pointers
into that same buffer. Numbers come out with `strtol`/`strtof`. The
correlation id, where a verb carries one, is a trailing, self-marking
`#<n>` token (spec §8.2) rather than a positional field — recovered by a
separate backward scan over the RAW line, done before the forward
tokenizer mutates it, so the id stays recoverable even past the small
fixed field-token array's own storage cap (protocol_handler.cpp's
`findLastFieldToken()`).

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
  kOk,          // → ok [#id]
  kUnknown,     // → err [#id] 1
  kBadArg,      // → err [#id] 2
  kRange,       // → err [#id] 3
  kFull,        // → err [#id] …
  kDuplicateId, // → err [#id] 11
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

`DifferentialDrive::output()` already publishes everything a `t` frame needs
(see [diffdrive.md](diffdrive.md) §4) — positions, velocities, applied duties,
timing, and the health flags. The adapter's telemetry job is a projection, not
a computation: pick the columns for the active `TLM` mode, convert counts to
the wire's units, and hand the handler an array.

`thdr` is emitted once on subscribe and names the columns; `t` carries the
values in that order. The frame is self-describing, so a consumer never
hardcodes a column index (spec §6.2).

---

## 5. Verb scope — what the first library implements

Enough to test DiffDrive over the wire, and no more.

| in scope | why |
|---|---|
| `HELLO` `PING` `VER` `ID` `STATUS` `HELP` | session and liveness; all trivial, all needed to bring a link up |
| `GET` `SET` | tune the kernel's gains live, and read back what was pushed |
| `TLM` + `thdr`/`t` | observe encoders and duties climbing — this is how you *see* the kernel work |
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
  business, and an unknown name is just `err [#id] 1` coming back from the
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

## 8. The one open question: should `WHEELS` emit `done`?

This was badly explained last time, so here it is properly.

v6 has three outcome replies (spec §8.1): `ok [#id]` means *accepted*,
`err [#id] <code>` means *rejected*, and `done #<id> <reason>` means *the thing
you enqueued has now finished*, where `reason` is `stop` (its stop condition was
met) or `timeout` (its backstop fired). An id-carrying reply spells the id as a
trailing, self-marking `#<n>` token; an id-less reply (an OMITTED optional id)
is sent bare — no `#id` token at all (spec §8.2, and see §9.6 below for the
2026-08-20 grammar migration that made this bare/id-carrying distinction the
literal wire shape rather than a positional placeholder).

`done` was designed for `MOVE`, which has a real stop condition — "drive until
400 mm of travel" genuinely finishes. **`WHEELS` has no stop condition.** It
holds a wheel speed for `duration` ms and then the lease expires. So "finished"
means only "the lease ran out", and the question is whether that is worth a
reply at all:

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
decide now than to retrofit. **My recommendation: no `done` for `WHEELS`** —
keep the handler stateless and pure for the first library, and let `done` arrive
with `MOVE`, which is the verb that actually needs it.

---

## 9. As built — resolved ambiguities and known gaps

Everything above is design. This section is what the implementation actually
found, and it is the part to read before extending any of it.

### 9.1 Spec ambiguities resolved (not silently picked)

**The optional trailing `id` on `SET`/`WHEELS` — historical, and now
resolved by the GRAMMAR itself, not by this file's own invention.** Under
the original colon grammar, spec §7.1's worked example (`SET:wheel_
control.pid_kp:0.03` → `ok:0`) directly contradicted §8.2's "id 0 means no
ack wanted" read literally, and this handler resolved it by treating an
*omitted* id and an *explicit* `0` as different wire forms (see
`clasi/issues/`-era `docs/spec-defects.md` D1 for the full contradiction).
The 2026-08-20 space/`#id` grammar switch (§9.6) closed this gap
structurally: an omitted id and a literal `#0` are now VISIBLY different
wire forms by construction (`SET name value` vs. `SET name value #0`), so
there is nothing left for this file to reconcile — omitted → executes,
bare `ok`/`err` once; `#0` → executes silently, no ack at all. Applied
only where the id is genuinely optional; `STOP`'s id is required, and a
literal `#0` there is itself malformed (spec §8.2's own words: "`#0` is
legal only where the id is optional").

**`GET` with an unknown field name.** `GET` never carries an id, so there is
no wire channel to carry an `err` on. Resolved as **fully silent** — no
reply, and not counted malformed. This is no longer a resolution this file
invented either: spec §7.1 now states it directly ("`GET` with an unknown
name is silent — no reply, and not counted malformed").

**`WHEELS`'s 5000 ms ceiling** (§5.2) is prose at the verb level with no
stated owner. The handler holds no bounds table, so **the adapter enforces
it** and returns `kRange` above it. Unaffected by the grammar migration.

**The malformed-line `#id` recovery rule (spec §2) is verb-agnostic, with
one deliberate exception.** Spec §2's own words — "if the line's last token
is a well-formed nonzero `#id`, reply `err #<id> <code>`" — carry no carve-out
for a verb whose own grammar has no id concept at all (`HELLO`/`PING`/`ID`/
`VER`/`STATUS`/`HELP`/`GET`/`TLM` in this library's scope), and the spec's
own "including unknown verbs" framing confirms it fires even before a verb
is identified. This handler implements it that way, with exactly one
exception: `ESTOP`, whose own §5.4/§8.2 text ("never carries an id and is
never acked … must not queue behind anything, including an ack") is treated
as the more specific rule winning over §2's general one. See
`protocol_handler.h`'s own file-header ambiguity note #2 for the fuller
reasoning; this is a resolution the SPEC text does not spell out in one
place, so it is recorded as this file's own call, the same as the id-grammar
entries above once were.

**The id's own numeric grammar (`'#' [0-9]+`) is stricter than an ordinary
wire integer field.** Spec §2.2's general "every wire value is … optionally
signed" does not apply to the id itself — `#+5` is not a well-formed id (no
sign of any kind is legal after the `#`), even though a `+`-prefixed
ordinary field elsewhere might parse. Implemented with a dedicated
digit-only pre-scan (`parseIdDigits()`) rather than reusing the general
unsigned-field parser.

### 9.2 Deliberately not implemented

**The 3× reply repeat.** Spec §8.1 says an id-carrying `ok`/`err`/`done` is
sent three times on consecutive cycles, so an outcome survives the measured
~5% radio frame loss without a ring or an eviction policy. This handler does
**not** do that, because "on consecutive cycles" needs a periodic entry point
and pending state — exactly what the no-`done`-for-`WHEELS` decision keeps
out. As of the 2026-08-20 grammar migration, spec §8.1 itself now says this
explicitly rather than leaving it as an implication this handler had to
infer: the repeat is "emission policy, owned by whatever drives the robot's
per-cycle output … NOT a property of the line codec." This handler stays a
pure function of its input bytes, matching that framing directly — it emits
each id-carrying reply exactly once, and the repeat, if and when it is
wanted, belongs to whatever drives a real cycle loop, not to this class. See
`radio-robot-lib`'s own `docs/spec-defects.md` D4 for the decision record
this text is downstream of.

That is a real gap, not a rounding error: **v6's loss tolerance is currently
unimplemented** in this library. It belongs at the app or transport layer
that owns a loop, or it comes back into the handler when `MOVE` and `done`
do. Worth deciding deliberately rather than discovering on a lossy link — and
now explicitly out of THIS component's contract, not merely unimplemented.

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

### 9.4 Hardening sweep (2026-08-20, pre-grammar-migration) — bugs found, and what they mean for a port

Stakeholder direction: `src/protocol/` is going to be an **archetype**,
ported to MicroPython and JavaScript by reading it and running its fixture,
so this pass focused entirely on the handler's own robustness, not new
features. Full detail lives in the fix-site comments in
`protocol_handler.cpp` and in `tests/protocol/test_protocol_adversarial.py`'s
module docstring; this is the summary a future porter should read first.
**All three findings below predate, and survive, the colon-to-space grammar
migration recorded in §9.6** — carried forward with their wire EXAMPLES
updated to the new syntax, and (for finding 3) a re-analysis of what changed
about its reachability, not a blind find-and-replace.

**Three real bugs, all fixed, none a wire-format change:**

1. **`formatConfigValue()` cast a NaN straight to `uint32_t`** — undefined
   behavior, confirmed live by UBSan. A NaN can never arrive *over the wire*
   (`parseFloatField` already rejects it on input, spec §2.2/§7.2's "no
   NaN, no inf"), so this was only reachable through the `Adapter` seam — an
   adapter's own stored config value being NaN (e.g. upstream
   divide-by-zero in a real firmware's config math), read back by `GET`.
   Fixed by clamping NaN to 0.0 before the cast; `+Inf`/`-Inf` were already
   handled correctly by the existing overflow clamp.
2. **Hex-float syntax (`SET name 0x1.8p3`) bypassed "no exponents"
   entirely.** The exponent guard only checked for `'e'`/`'E'`; a hex
   float's exponent marker is `'p'`, gated behind a `'0x'` prefix the guard
   never looked for, so `strtof` silently accepted it (`0x1.8p3` → 12.0).
   **Archetype-relevant on its own**: neither Python's `float()` nor
   JavaScript's `Number()`/`parseFloat()` accepts hex-float syntax, so this
   was a **C++-only divergence** — a straight port would not have this bug
   at all, and would need to actively decide whether to *add* hex-float
   rejection or simply rely on its own numeric parser already refusing it.
   Purely a property of `strtof()` parsing an already-extracted field's own
   content, so entirely unaffected by the grammar migration.
3. **A leading-whitespace numeric field was silently accepted** because
   `strtol`/`strtoul`/`strtof` all skip leading whitespace per the C
   standard — contradicting this file's own "strict, whole field consumed"
   doc comment, which (before the fix) was only actually true of
   *trailing* whitespace. Originally reproduced as `WHEELS: 100:100:1000`
   (a space right after the first `:` separator). **The grammar migration
   changed this finding's own reachability, and that had to be re-derived,
   not assumed:** under the space grammar a literal LEADING SPACE can no
   longer reach a field decoder at all — the tokenizer collapses every run
   of `' '` into one separator before a field pointer is ever produced, so
   `field[0] == ' '` is now structurally impossible. The guard is NOT dead
   code, though: spec §2's field grammar (`field ::= any bytes except ' '
   and '\n'`) still admits `'\t'`, `'\v'`, `'\f'`, and `'\r'` as ordinary,
   legal field bytes, and strtol/strtoul/strtof would silently skip any of
   those too — so the guard survives, now targeted at a narrower set of
   bytes than before (reachable and load-bearing for `'\t'`/`'\v'`/`'\f'`/
   `'\r'`; vestigial, but harmless to keep, for a literal space). Every
   language's numeric parser has its own leniency here regardless (Python's
   `int()`/`float()` also strip whitespace **and** accept `_` digit
   separators; JavaScript's `Number(" ")` is `0`) — a port author should
   decide this deliberately per language, not inherit whichever behavior
   their host language's built-in parser happens to have.

**One characterization finding, not fixed — read this before porting
`dispatch()`:** every wire-touching comparison in this handler (verb
lookup, tokenizing) operates on NUL-terminated C strings, per this file's
own "no allocation, no `std::string`" constraint (§2.2). `strcmp()`/the
tokenizer's own forward scan both stop at the first NUL in a string, so
`PING extra` compares **equal** to `"PING"` and dispatches exactly like
a bare `PING` — silently discarding `extra` with no malformed-count
increment. Spec §2's verb grammar (`verb ::= [A-Za-z][A-Za-z0-9_]*`) does
not admit NUL in a verb at all, so the grammar-correct behavior would be
rejection, not silent acceptance of the truncated prefix. **This is NOT
reproduced by a length-aware host language**: Python `bytes`/JavaScript
strings compare full length, embedded NUL included, so
`b"PING extra" == b"PING"` is `False` in Python. A faithful line-by-line
port of this C++ handler's *logic* would therefore behave differently from
this reference implementation on this one input class — pinned as a
characterization test
(`test_embedded_nul_immediately_after_verb_matches_bare_verb`) so it cannot
drift silently, not fixed, because a real fix means abandoning C-string
comparisons throughout the parser (a far larger, riskier change than this
pass's scope, and in tension with §2.2's explicit no-`std::string` firmware
constraint). **Re-verified, not just carried forward, at the 2026-08-20
grammar migration**: the ROOT CAUSE (C-string functions stop at the first
embedded NUL) is unchanged, but the specific MECHANISM shifted from
`strchr()`'s colon search to the new tokenizer's own forward scan — see the
test's own updated docstring for the full re-derivation.

**What did NOT change (at the TIME of the original, colon-era hardening
sweep):** the wire format itself. Every fix above tightened *rejection* of
inputs that were already meant to be rejected (per this file's own prior doc
comments) or were never legal per spec §2's grammar in the first place — no
previously-accepted, spec-legal input was rejected, and no reply shape
changed. The wire format DID change later the same day — §9.6 is that
migration, and it is a real, deliberate wire-format change, not a hardening
fix.

### 9.5 Where the adapter lives, and why

`src/adapter/` — its own package, not inside `src/protocol/` or
`src/diffdrive/`. It is the one component required to depend on both, and each
of those two has a standalone-build gate ("compiles with an include path of
exactly its own directory") that a cross-dependency would break.

### 9.6 The colon-to-space/`#id` grammar migration (2026-08-20)

Stakeholder decision, 2026-08-20 (`docs/protocol-v6-spec.md` §2, commit
5a5b6da): fields are separated by **spaces**, not colons, and the
correlation id returns to its historical **`#`-prefix** spelling as a
trailing, self-marking field. This section records what changed in
`src/protocol/` and `tests/protocol/`, for the same reason §9.4 records the
hardening sweep — a future porter reading this file needs the "why", not
just a diff.

**What is a pure separator swap, and what is not.** Every wire example in
this document up to §9.5 used the OLD colon grammar (`WHEELS:100:100:1000`,
`ok:5`, `get:name:value`); they have all been updated to the new spelling in
place, but the underlying OBJECT MODEL — `Adapter`, `Result`, `Sink`,
`Snapshot`/`Column`, the handler/adapter split itself — did not change at
all. This was a rewrite of `protocol_handler.{h,cpp}`'s parsing and
formatting internals, not a redesign.

**New mechanics this migration introduced:**

- **Tokenizing, not colon-splitting.** `tokenizeLine()` collapses runs of
  `' '` into one separator and trims leading/trailing line whitespace,
  matching spec §2's `sp ::= ' '+`. A blank or all-whitespace line is now
  ignored SILENTLY (previously, under the colon grammar, an empty line
  dispatched as an unknown zero-length verb and counted malformed).
- **The id is self-marking and line-trailing**, not positional. Because it
  announces itself with `#`, an omitted optional field never shifts it into
  a data position — the reason `SET name value` and `SET name value #9` are
  BOTH exactly two or three tokens, with no placeholder needed for the
  missing middle slot the old grammar would have required.
- **Bare vs id-carrying replies are now genuinely different wire shapes.**
  An omitted id → `ok`/`err <code>` (no `#id` token at all); an explicit
  nonzero id → `ok #<id>`/`err #<id> <code>`; an explicit `#0` (legal only
  where the id is optional) → no reply at all. See §9.1's rewritten first
  entry for how this retired an old ambiguity rather than merely
  reformatting it.
- **The malformed-line `#id` recovery rule is new capability, not a
  reformatting of old behavior.** Spec §2: "if the line's last token is a
  well-formed nonzero `#id`, reply `err #<id> <code>` … including unknown
  verbs." Under the old colon grammar an unknown verb's own arity was
  unknowable, so no field of its line could ever be trusted as an id; the
  new grammar's self-marking id makes it trustworthy regardless of whether
  the verb itself is known, or even well-formed. `ESTOP` is the one
  deliberate exception (§9.1's third entry). This inverted part of the old
  `test_unknown_verb_no_reply` test, which only covered the id-less case —
  now split into `test_unknown_verb_no_reply_when_no_recoverable_id` and
  `test_unknown_verb_with_recoverable_id_gets_err_unknown`
  (`tests/protocol/test_protocol_harness.py`).
- **The id's own numeric grammar is stricter than an ordinary integer
  field** — `'#' [0-9]+`, no sign at all, parsed with a dedicated
  digit-only pre-scan (`parseIdDigits()`) rather than reusing the general
  unsigned-field parser, so `#+5` is correctly NOT id 5.

**What did NOT change:** the `Adapter` interface (`adapter.h`) — every
method signature, `Result`/`TlmMode`/`Column`/`Snapshot` shape is untouched,
because none of them ever encoded a wire delimiter. `mock_adapter.h` and
`protocol_shim.cpp` (`tests/protocol/`) needed zero changes for the same
reason. `tests/adapter/test_diffdrive_adapter.py`, which drives the REAL
handler end to end (not a mock), needed its wire literals updated for the
same mechanical reason `golden_vectors.txt` did, even though it lives
outside `src/protocol/`/`tests/protocol/` proper — the handler it exercises
is the same one.

**Golden-vector fixture:** every vector in `golden_vectors.txt` changed
SHAPE, not just separator — the old `ok:0` id-less arm is gone; there is no
new equivalent single spelling, because "id-less" now means literally "no
`#id` token in the reply", i.e. a bare `ok`. The fixture also grew new
vectors for rules the colon grammar never had: space-run collapsing,
`STOP #0` being malformed (required-id verb), and an unknown verb's
trailing `#id` recovering an `err` reply.

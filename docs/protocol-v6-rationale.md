---
status: in-progress
priority: high
sprint: '137'
tickets:
- 137-001
- 137-002
- 137-003
- 137-004
- 137-005
- 137-006
- 137-007
- 137-008
- 137-009
- 137-010
- 137-011
- 137-012
- 137-013
- 137-014
- 137-015
- 137-016
- 137-017
---

# Protocol v5 and its implementation — simplification review

**Source:** stakeholder request, 2026-08-19: "review of our protocol and the
implementation for the protocol… recommendations for simplifying the protocol
and simplifying the implementation of it. I'd like to have something that we
can implement in C++, Python, and JavaScript. It's fairly complete, solves all
the problems we need, and is also fairly stingy… maybe there is a way to
simplify the configuration."

**Read for this review:** `docs/protocol-v5.md` (1070 lines),
`src/protos/*.proto` (13 files), `src/scripts/gen_messages.py` (3852 lines),
`src/scripts/gen_boot_config.py` (1345), `src/firm/core/comms.{h,cpp}`,
`src/firm/core/telemetry.h`, `src/firm/messages/*`,
`src/firm/config/*`, `src/host/robot_radio/io/wire_{codec,commands}.py`,
`src/host/robot_radio/robot/protocol.py`,
`src/host/robot_radio/config/robot_config.py`, `data/robots/tovez.json`.

---

## 1. Headline

**The wire is already stingy. The implementation is not.**

Measured: telemetry runs ~77 bytes/frame typical over USB
(`telemetry.h`'s own "31 fps is ~2400 B/s"), worst case 126 B for the `tlm`
arm. `CommandEnvelope` is 55 B worst case. Those are good numbers and the
`(scale)`/`(abs_max)` fixed-point work that produced them is the best design
decision in the protocol. **Do not chase wire bytes.**

The cost is all on the implementation side, and it is concentrated in three
places:

| # | Where the complexity is | Size today |
|---|---|---|
| 1 | A hand-written protobuf codec, machine-generated | 3852-line generator → 105 KB `wire.cpp` + ~40 KB headers |
| 2 | A verb plane that grew from 4 cleartext verbs to 25 verbs with 4 different data grammars and load-bearing parse order | `comms.cpp` 540 lines, growing per verb |
| 3 | Configuration expressed in **nine** representations, with a parity checker to police two of them | ~4200 lines across firmware + host + generators |

The three-language goal (C++/Python/JavaScript) is what makes #1 decisive
rather than merely untidy. Today: C++ has a 105 KB generated codec, Python
leans on the full protobuf runtime, and **JavaScript has nothing and cannot
easily be given anything** — `protobufjs` would be a third, independently
drifting codec, and it does not run on MicroPython either (which the
`micropython-first-rebuild` issue needs).

---

## 2. Findings

### F1 — protobuf is paying for a feature nobody uses

The schema uses a small subset: scalars, one `oneof` level, one nested
message level, one packed repeated field (`Telemetry.acks`). Presence
(`optional`) is used sparingly. That subset costs:

- `gen_messages.py`, 3852 lines, six output families (per-proto C++ POD
  headers, `wire.{h,cpp}`, `commands.h`, `layout_checks.{h,cpp}`, host
  `wire_commands.py`, host `robot_config_generated.py`,
  `robot_config.schema.json`).
- `src/firm/messages/wire.cpp`, 104,886 bytes — a `FieldDesc`/`MessageTable`
  runtime walker plus 31 generated field tables, handling nine `FieldKind`s
  (two of which the schema never reaches: `kMessage` is documented in the
  generated source as "unreached by this sprint's schema").
- `src/firm/messages/wire_runtime.cpp`, 505 lines of varint/zigzag/COBS/CRC.
- Host: the real protobuf runtime plus `gen_pb2.py`.

Protobuf's actual payoff is **tolerant schema evolution across independently
versioned endpoints**. This project does the opposite by policy: v2→v3→v4→v5
were atomic cutovers, the banner is "byte-frozen," the `micropython-first-rebuild`
issue specifies "byte-for-byte compatible," and host and firmware ship
rev-locked. We pay protobuf's whole bill and take none of its dividend.

### F2 — the verb plane sprawled, and its own invariants no longer hold

`docs/protocol-v5.md` §2.4 describes "the closed four-cleartext-verb registry
(§2.4) is the entire cleartext surface." Actual, from the generated registry:

```
25 verbs — 13 binary, 12 cleartext
binary:    CALIBRATE CFG CONFIG ERR ESTOP GET_CONFIG GO_TO MOVE OK SET_FIELD STOP TLM WHEELS
cleartext: DBG DEVICE HELLO HELP ID PING PONG POSE READY SEED STATUS VER
```

Three specific structural problems:

1. **The registry claims to be "the SOLE text/binary discriminator." It is
   not.** `TLM` is registered `(binary) = true`, but inbound it is
   intercepted *before* the binary branch and parsed as cleartext
   (`classifyTlmArg()` matching `NOW`/`ON`/`AUTO`/`OFF`). `SEED` and `DBG`
   are likewise intercepted ahead of the registry's own dispatch. **Parse
   order in `Comms::dispatchLine()` is load-bearing** — a fact the
   `micropython-first-rebuild` issue already had to write down for the
   reimplementer. The registry is missing a *direction* axis: `TLM`, `ID`,
   `VER`, `STATUS`, `SEED` and `POSE` all have different data shapes inbound
   vs outbound, and one `(binary)` bool cannot say that.

2. **Four cleartext data grammars.** Colon-joined (`ID:a:b:c`, `POSE:...`),
   `k=v` colon-separated (`STATUS:k=v:k=v`), comma-or-space separated
   (`SEED:<x>,<y>,<heading>`), and free-form (`DBG:<message>`,
   `DBG mark …` with its own tokenizer). Each is a separate hand-written
   parser, and each must be re-written in every target language.

3. **Arm names are load-bearing across a language boundary.** Both host
   senders derive the wire prefix — *and therefore the CRC scope* — from
   `WhichOneof("cmd").upper()`. `commands.proto`'s own comment says it:
   "Renaming an arm silently breaks every frame's CRC." This is why
   `SET_FIELD` had to be deliberately misspelled relative to the
   `SetConfigField` message it carries: the literal name would have pushed
   `kMaxLineBytes` to 256, over the 250-byte TX ring. **The protocol's
   naming is now constrained by a buffer size**, with 1 byte of headroom
   (249 vs 250). That is a design smell, not a tuning problem.

### F3 — the CRC-scope extension exists only because the type lives outside the payload

§2.2's CRC-over-`COMMAND ':' payload` required adding an incremental
`crcInit()`/`crcUpdate()` pair, `Comms::crcOverScope()`, and a mirrored
`_crc_over_scope()` on the host — solely so a bit-flip in the ASCII verb name
can't land on another valid verb. If the message type were the **first byte of
the payload**, it would be inside the CRC for free and this entire mechanism
would not exist in any language.

### F4 — configuration is expressed nine ways, with a parity checker

| # | Representation | Where |
|---|---|---|
| 1 | Authoring JSON | `data/robots/tovez.json` (155 leaves) |
| 2 | Schema | `src/protos/robot_config.proto` (51 KB, 12 groups, ~90 wire fields) |
| 3 | Runtime C++ object | `Config::Robot` (`src/firm/config/robot.h`, generated members) |
| 4 | **A second, independent C++ bake shape** | `Config::*BootConfig` via `gen_boot_config.py` (1345 lines of hand-written `*_for_config()` mappings) → `boot_config.cpp` (405 lines) |
| 5 | Flash shape | `PersistedTuning::TuningSnapshot` (`persisted_tuning.{h,cpp}`, 476 lines) |
| 6 | Host hand-written model | `robot_config.py` (729 lines pydantic) |
| 7 | Host generated model | `robot_config_generated.py` (174 lines) |
| 8 | Host protobuf | `pb2/robot_config_pb2.py` |
| 9 | JSON Schema | `data/robots/robot_config.schema.json` |

And `src/firm/config/config_parity_capi.{h,cpp}` (207 lines) exists **only to
check #3 against #4**. A parity checker is the standard tell that two
representations should have been one.

Two consequences already biting:

- **Boot-only fields can't be tuned without a reflash.** `tovez.json`'s own
  `_rotational_slip_note` says it: "a runtime CONFIG rotSlip patch acks
  `ERR_UNIMPLEMENTED`, so changing it needs a reflash." This is an artifact
  of #4 being a different code path from #3 — not a physical constraint.
- **Five JSON blocks never reach the robot** — `wheels`, `encoders`,
  `gripper`, `peripherals`, `perception` (17 leaves) have no proto group and
  no wire path; they are read only by the hand-written host model #6.
  `perception` appears to have no consumer at all. That violates
  `configuration-discipline.md`'s invariant 2 ("every value in the file
  reaches the robot… a value in the file that nothing consumes breaks
  invariant 2 as badly as a missing one. **Delete it, don't wire it**").

### F5 — `docs/protocol-v5.md` is materially stale

Already flagged by `clasi/issues/micropython-first-rebuild.md` ("`docs/protocol-v5.md`
is stale; truth = `src/protos/` + `src/firm/core/comms.cpp`+`telemetry.cpp`").
Confirmed drift:

| Doc says | Actual |
|---|---|
| ack ring depth 4 | `kAckRingDepth = 12`, plus `kAckRepeats = 3` (not documented at all) |
| `kPrimaryPeriod` 40 ms / ~25 Hz | `kPrimaryPeriod = 25` (~31 fps), with a documented half-duplex inbound-loss tradeoff |
| "four cleartext verbs are the entire cleartext surface" | 12 cleartext verbs |
| §1: `CommandEnvelope.cmd` "still carries exactly three arms" | 9 arms |
| §11: "no `HELP`/`SET`/`GET` text verb, and no free-form text command parser of any kind" | `HELP`, `STATUS`, `SEED`, `POSE` shipped; `DBG` has a free-form tokenizer |
| Telemetry field table ends at 16 | 22 fields (`duty_per_speed_*`, `bias_*`, `pid_*` added) |
| `ReplyEnvelope` worst case 130 B / 232 B | needs re-measure |

A doc that has to be cross-checked against source before use is not doing its
job. **This one should be generated from the schema.**

---

## 2a. Second pass — findings that supersede §3's R1/R2/R3

Stakeholder pushback, 2026-08-19: *"Do we really need binary telemetry? … if we
get rid of the binary scheme, then we don't really need COBS anymore. We don't
need CRCs either… Any of the errors we've actually had have been packet loss.
… maybe our entire configuration just goes down to a setter and a getter."*

Four measurements say this is right, and further than §3 went. **§3's R1/R2/R3
are superseded by §3a; R4 survives but shrinks.** The resulting wire format is
specified in [`docs/protocol-v6.md`](../../docs/protocol-v6.md).

### F6 — the app-level CRC duplicates a CRC the radio hardware already computes

`src/libraries/codal-nrf52/source/NRF52Radio.cpp:315-317`:

```c
NRF_RADIO->CRCCNF  = RADIO_CRCCNF_LEN_Two;   // 2 bytes
NRF_RADIO->CRCINIT = 0xFFFF;
NRF_RADIO->CRCPOLY = 0x11021;
```

That is **CRC-16/CCITT, init `0xFFFF`, poly `0x1021`** — bit-for-bit the
function `wire_runtime.cpp` computes in software. The nRF RADIO peripheral
computes it in hardware and drops the packet before CODAL sees it. USB CDC
carries its own CRC plus link-level retransmission.

So `telemetry.h`'s measured "zero unparseable and zero CRC mismatches at BOTH
rates" is not luck — the app CRC **cannot** fire on a corruption the layer
below did not already eat. It is the third redundant integrity check on a link
with two. The real failure mode is whole-packet loss, about which a CRC has
nothing to say.

COBS has exactly one job: keeping payload bytes from colliding with the `\n`
terminator. In an all-ASCII protocol it has no job. **Both delete.**

### F7 — the telemetry debug tail has no consumer outside bench scripts

| field | `f.<attr>` uses | consumers |
|---|---|---|
| `pid_left` / `pid_right` | 2 / 2 | `wheel_controller_ab_bench.py` |
| `bias_left` / `bias_right` | 5 / 5 | + `duty_sweep.py` |
| `duty_per_speed_left/right` | 5 / 4 | same |
| `cycle_busy` | 3 | `tlm_log.py`, `recorder.py` |
| `position_epoch` | 4 | one unit test |

against `pose` (87), `acks` (55), `otos` (52), `twist` (33). Nothing in the
control path, `rogo`, or the geofence reads the tail. It is opt-in
diagnostics riding an always-on frame.

### F8 — ASCII costs nothing on the radio, because loss is per-packet not per-byte

Radio MTU is 247 B and a message under it is **one on-air packet**
(`microbit_radio_link.h:96`, RadioRelay §5 `START|END`). Link runs at
1 Mbit/s (`NRF52Radio.cpp:286`).

| frame | bytes | on-air packets | airtime |
|---|---|---|---|
| today, binary, 22 fields | ~77 typical / 126 worst | 1 | ~0.7 ms |
| **v6 ASCII pose-only** | **~38** | 1 | ~0.3 ms |
| v6 ASCII full, 26 columns | ~160 | 1 | ~1.4 ms |

Pose-only ASCII is **half** today's binary. Even full-fidelity ASCII is one
packet, costing ~0.7 ms more airtime per frame — ~2% more RX blackout per
second at 31 fps. That is not what eats inbound commands (see F10).

Conclusion: **the field cull is what makes ASCII free.** ASCII *without* the
cull is a ~2× regression on outbound airtime; ASCII *with* a subscription
default of pose-only is a net improvement over today while deleting the entire
codec.

### F9 — the ack ring is an artifact of acks riding a periodic frame

`kAckRingDepth = 12`, `kAckRepeats = 3`, the packed `corr_id << 4 | err`
encoding, the eviction policy, and `wait_for_ack()`'s ring scan all exist for
one reason: an ack had to survive inside `Telemetry` until the host next
looked.

As its own line — `ok:<corr>` / `err:<corr>:<code>`, ~12 B, emitted 3× for
loss redundancy — an ack costs nothing on idle frames, and the ring, its
depth, its repeat counter, its packing and its scan **all collapse** to "read
lines until `ok:<corr>` arrives." It also removes a sizing bug ASCII would
otherwise hit: 12 inline acks would push a full frame past 247 B into
fragmentation.

### F10 — the robot's inbound radio path is a single-slot buffer

`microbit_radio_link.h:60-62`: *"a second message completing before
`readLine()` drains the first is dropped."* One `_msg` buffer, one
`_msgReady` flag.

This is a far better candidate for the measured inbound command loss than
outbound airtime — it is per-message and size-independent, which matches the
data exactly (same frame size, 25 ms vs 40 ms period, 5-of-6 commands lost).

**Nothing in this review fixes it, and none of these recommendations should be
sold as fixing it.** It needs its own ticket, alongside
`clasi/issues/later/inbound-command-loss-needs-retransmit-not-a-slower-telemetry-stream.md`.

---

## 3. Recommendations

**§3a below supersedes R1/R2/R3.** They are kept because R4/R5 reference them
and because the reasoning that led to the fixed-layout proposal is the
reasoning that led past it — R1 argued "stop generating protobuf wire code";
§3a's answer is "stop generating wire code at all."

Ordered by leverage. R1 and R4 are the two that matter; R2/R3 are cheap and
should ride along; R5 is hygiene.

### R1 — Keep the `.proto` files as the IDL. Stop generating protobuf wire code from them. Generate fixed-layout codecs instead.

This is the single change that makes C++/Python/JavaScript tractable.

**Keep:** `src/protos/*.proto` as the schema language, the custom options
(`scale`, `abs_max`, `max`, `min`, `req`, `max_count`, `units`), and
`gen_messages.py`'s descriptor-walking front end. All of that is good and
already paid for.

**Change:** what the back end emits. Replace varint/zigzag/tag-length
protobuf encoding with **fixed-offset, little-endian packed records**, one
byte of message type at the front.

Why this works *here* specifically:
- Endpoints are rev-locked by policy already (F1). Fixed layout's one real
  cost — no tolerant schema evolution — is a cost we are already paying
  voluntarily.
- Every field already declares its range (`abs_max`/`max`) and quantum
  (`scale`). Those two facts **determine the width** — the generator can pick
  `int16`/`uint8`/`int32` mechanically instead of falling back to varints.
- Fixed offsets make the size constant, which removes the whole class of
  "worst-case envelope budget vs. TX ring capacity" arithmetic that currently
  has 1 byte of headroom (F2.3).

What each language then needs, end to end:

| Piece | C++ | Python | JavaScript |
|---|---|---|---|
| COBS(0x0A) + CRC-16 | ~80 lines (have it) | ~80 lines (have it) | ~80 lines (new) |
| Record codec | `struct` + `memcpy`, generated | `struct.pack/unpack`, generated | `DataView`, generated |
| Verb/type tables | generated | generated | generated |
| **Total per language** | **~400 lines** | **~400 lines** | **~400 lines** |
| **External deps** | none | **none** (stdlib `struct` — MicroPython-clean) | **none** |

Against today: 105 KB of generated C++ deleted, `wire_runtime.cpp`'s
varint/zigzag half deleted, the protobuf runtime dependency dropped from the
host, `gen_pb2.py` deleted, and `gen_messages.py` shrinks substantially
(emitting fixed offsets is far simpler than emitting a nine-`FieldKind` field-table
walker with oneof/presence/packed handling).

**Expect roughly break-even on wire bytes, not a win.** A fixed telemetry
record lands near ~75–110 B against today's ~77 B typical / 126 B worst.
The win is *predictability* and *portability*, not size. Say so out loud so
nobody sizes this work on a byte-count promise it won't keep.

**Keep the golden-vector fixture** (`src/tests/fixtures/wire_golden_vectors.txt`).
Under a three-language target it becomes the primary conformance gate, and
it is the thing that makes a JS implementation verifiable at all.

### R2 — One binary sigil, message type in the payload

Replace the 13 binary verbs with **two**: one host→robot, one robot→host
(e.g. `C:` and `R:`). The message type becomes the first payload byte.

Deletes, in every language at once:
- `_envelope_command_name()` / `WhichOneof("cmd").upper()` and the
  "renaming an arm silently breaks every frame's CRC" hazard (F2.3);
- `bodyKindToVerb()`;
- **the entire CRC-scope extension** — `crcInit`/`crcUpdate`/`crcOverScope()`
  and its host mirror (F3). The type byte is inside the payload, so it is
  inside the CRC for free;
- the `kMaxCommandPrefixBytes`/`kMaxLineBytes`-vs-TX-ring squeeze, and with it
  the constraint that verb *names* must be short.

Cost: a wire sniffer sees `C:` instead of `MOVE:`. Mitigated — anything that
decodes at all reads the type byte first, and `rogo`/bench tooling prints the
decoded name either way.

### R3 — Split the cleartext plane cleanly, and give the registry a direction axis

- **One cleartext grammar**: `VERB[:field[:field…]]\n`, colon-separated,
  always. Fix `SEED` to use colons (it is the lone comma user). Keep
  `STATUS`'s `k=v` only because it is deliberately extensible — document it
  as the one exception.
- **Registry gains `direction` and drops the "sole discriminator" claim**, or
  keeps the claim by moving `TLM`'s inbound mode control to its own cleartext
  verb (e.g. `TLMMODE:AUTO`) so `TLM` is binary in both directions and the
  parse-order interception in `dispatchLine()` disappears. Prefer the latter:
  it restores the property the registry was built to guarantee.
- **`DBG`'s free-form tokenizer is bench-only** (`ROBOT_DEBUG`). Leave it,
  but move it out of the registry's "closed verb set" framing in the doc —
  it is a debug console, not protocol.

---

## 3a. The v6 recommendation — supersedes R1/R2/R3, shrinks R4

**Delete the binary plane entirely.** One grammar, ASCII, integers only:

```
<VERB>[':'<arg>]*'\n'
```

No COBS (F6), no CRC (F6), no protobuf, no generated codec, no framing layer —
`readline()` *is* the transport. Full wire format:
[`docs/protocol-v6.md`](../../docs/protocol-v6.md).

The four moves, and what each deletes:

| move | deletes |
|---|---|
| **V1. All-ASCII wire** | `wire.cpp` (105 KB), `wire.h`, `wire_runtime.cpp`'s varint/zigzag/COBS/CRC, `gen_pb2.py`, the host protobuf runtime dependency, `wire_codec.py`'s framing half. `gen_messages.py` (3852) becomes a ~200-line **lint** checking three hand-written tables against the spec, not a code generator. |
| **V2. Telemetry as a subscription** (`TLM:off\|pose\|full\|now`, default `pose`), with a `thdr:` column header emitted on every mode change | the always-on debug tail (F7); `Telemetry`'s 22-field message; the field-presence flags that gate `otos`/`line`/`color`. Default frame drops from ~77 B to ~38 B (F8). The `thdr:`/`t:` pair is self-describing, so `tlm_log.py` becomes "write the header, write the rows" — the CSV logger stops needing a schema. |
| **V3. Acks get their own line** (`ok:`/`err:`, sent 3×) | `kAckRingDepth`, `kAckRepeats`, ring packing, eviction, `wait_for_ack()`'s scan (F9). |
| **V4. Case separates direction** — commands UPPERCASE, replies lowercase | the shared-channel reflection class in `.claude/rules/hardware-bench-testing.md`, where a robot's own `DBG:` output is a syntactically valid `DBG` **command** to every robot on the channel and the flood self-sustains. A lowercase reply can never parse as a command. Channel 3 stops being load-bearing for correctness. |

| **V5. The REPL is a binding, not a second protocol** — `p(<line>)` prints the reply lines and returns `None`, so REPL stdout is byte-identical to the wire; `r.*` is the ergonomic wrapper over the same table | the need for any REPL-specific message format, and the mode switch the stakeholder ruled out ("I don't want to have to drop into another view"). Telemetry on the REPL defaults to `TLM:BUFFER` — the loop fills a deque, `r.frames()` drains it — which cannot corrupt the prompt and, unlike polling, does not perturb a running move. |

**Per-language implementation cost, complete:** split on `\n`, split on `:`,
`int()`, one name table. **~150 lines, zero dependencies** — in C++, Python,
JavaScript, *and* MicroPython (which `micropython-first-rebuild` needs).

**V5 is only reachable because of V1.** A COBS+CRC protobuf frame cannot be
typed at a REPL, cannot be a Python argument list, and cannot be read back as
output. The REPL requirement (stakeholder, 2026-08-19) and the JavaScript
requirement fall out of the same decision — which is the strongest single
argument for the ASCII wire.

**On "keep the protobufs as ideal, generate manually"** (stakeholder): agreed,
and it gets easier than that. The schema collapses to **two flat tables** —
config fields and telemetry columns. Keep them in `.proto` if the syntax is
useful; the tool becomes a lint that checks the hand-written tables against
the spec, not a generator that emits them.

**The one thing to measure before committing:** outbound formatting cost — 26
`snprintf("%ld")` per frame at 31 fps on newlib-nano. Estimated ~100 µs against
a 32 ms cycle (0.3%), but that is arithmetic, not measurement, and it is the
single number that could invalidate V1. One bench run comparing `cycle_busy`
settles it; if it bites, a hand-rolled `itoa` fixes it without touching the
design.

### R4 — Configuration: bake by default, one setter and one getter for tuning

**Revised by the 2026-08-19 second pass.** §3's "one blob" was still one blob
*shape* too many. The stakeholder's framing is stronger:

> *"We generally default to baking in configuration… what we're moving to is
> that in the end we're going to have Python and JavaScript environments. We
> send a Python or JavaScript program over that's got all the configuration,
> which means that the only thing we need for configuration is to be able to
> change variables for tuning in test… that means one at a time. Maybe our
> entire configuration just goes down to a setter and a getter."*

If the config travels **inside the uploaded program**, the robot never needs a
config schema on the wire at all. It needs exactly two verbs, one field at a
time, for bench tuning:

```
GET:<name>              ->  get:<name>:<value>
GET                     ->  get:<name>:<value>  (one line per field)
SET:<name>:<value>      ->  ok:<corr>  |  err:<corr>:<code>
```

**What replaces the nine representations of F4:** one generated table —
`name, offset, type, scale, min, max` — **80 rows** (measured:
`geometry` 6, `motors` 14, `drive` 11, `wheel_control` 15, `planner` 12,
`planner_shaper` 6, `navigator` 8, `otos` 5, `estimator` 3). `SET` looks up,
bounds-checks, assigns. `GET` prints. Bare `GET` dumps every row — which is
simultaneously the read-back-vs-file acceptance test *and* human-readable at
a plain terminal.

Deleted outright: `robot_config.proto` (51 KB) · `ConfigGroupTarget` ·
`SetConfigGroup` · `GetConfig` · `ConfigSnapshot` · `SetConfigField` · the
`CFG` reply arm · the live/boot-only gating table · `PersistedTuning`
(476 lines) · `config_parity_capi` (207) · `gen_boot_config.py` (1345) ·
`boot_config.cpp` (405) · hand-written `robot_config.py` (729) ·
`gen_pb2.py`. **≈3500 lines.**

Still true from §3's version, and still wanted:

- **Boot-only vs live collapses into "takes effect now" vs "next boot."**
  Every field becomes settable; only the *apply* is gated. `SET
  geometry.rotational_slip` becomes "stored, reboot to apply" instead of
  today's `ERR_UNIMPLEMENTED` → reflash, which `tovez.json`'s own
  `_rotational_slip_note` complains about by name.
- **Audit the five orphan blocks** (`wheels`, `encoders`, `gripper`,
  `peripherals`, `perception`, 17 leaves) — host-only or dead, per
  configuration-discipline invariant 2. `perception` looks fully dead; check
  before deleting, the stakeholder supplied those mount numbers 2026-08-08.

**The one objection, answered.** Sprint 132-012 deliberately moved *away* from
string keys to `(group, field-number)` addressing, citing the `pid.kff → kaff`
bug where "a wire key's name stopped matching what it set." That bug came from
a **hand-maintained name vocabulary sitting beside the thing it set** — two
copies, free to drift. If name and storage address are emitted from one
declaration into one table, there is no second copy and the failure mode is
structurally impossible. Field numbers fixed drift by deleting names; a
generated name table fixes it by deleting the duplicate — and keeps the wire
readable and the host free of descriptor lookups. Strictly better here.

Wire cost of string keys is real and irrelevant: config pushes are rare and
one-at-a-time. That is the whole point of the stakeholder's framing.

### R5 — Generate the protocol document

`docs/protocol-v5.md` drifted on at least seven load-bearing facts (F5). The
verb table, the field tables, the size budgets and the flags bit table are all
derivable from `src/protos/` + a handful of `constexpr`s. Generate those
sections; hand-write only the prose (execution model, the two-stops semantics,
the rationale sections). A hand-maintained transcription of a generated
schema is exactly the drift this project already fixed *inside* the code and
left unfixed in the doc.

---

## 4. What NOT to change

- **The `(scale)`/`(abs_max)` fixed-point scheme.** It is the reason
  telemetry is small and it is a genuine single source of truth for quanta.
  Carry it into the fixed-layout generator unchanged — it gets *more*
  useful there, because range + quantum then also select the field width.
- **COBS keyed on `0x0A` + CRC-16/CCITT-FALSE + one line per packet.** This
  is correct, cheap, and it closed a real defect class (the `0x0A`-embedding
  `move_wheels` failure). Port it verbatim to JS.
- **The ack ring** (packed `corr_id<<4|err`, depth 12, `kAckRepeats = 3`).
  Redundancy across three frames is what makes acks survive the measured
  ~5% radio loss. Document it; don't touch it.
- **`ESTOP` vs `STOP` as two verbs.** Hard-won semantics, measured
  (2.9 cm vs 39.8 cm).
- **Bounded moves — every `Move` carries a stop condition + required
  timeout.** The no-deadman safety property.

---

## 5. Sequencing

Revised for §3a. The v6 cutover is atomic on the wire — as every prior
protocol rev has been — but the work in front of it is separable.

0. **Measure the formatting cost** (§3a's one open number): 26
   `snprintf("%ld")` per frame at 31 fps, `cycle_busy` before/after on the
   stand. One bench run. Everything below assumes it comes back negligible.
1. **R5 + the F5 ledger** — repair the doc before anything else, so the
   cutover has a truthful v5 baseline to diff against. Cheap.
2. **R4 (config → `SET`/`GET`)** — largest line-count win (~3500), and it can
   land on the v5 wire as two new cleartext verbs *before* the v6 cutover,
   which de-risks both. Settles `configuration-discipline.md`'s open
   invariant.
3. **V2 + V3 (telemetry subscription + acks on their own line)** — also
   landable on v5. Once these two are in, the binary plane's only remaining
   users are the motion verbs.
4. **V1 + V4 (all-ASCII cutover, case-separated directions)** — the atomic
   rev. C++ and Python together, rev-locked; JavaScript as a third
   implementation of the same ~150 lines. Gate: the golden-vector fixture,
   rewritten as ASCII line vectors.

Sequencing this way means the binary plane is *emptied* before it is deleted,
rather than being cut over wholesale in one sprint.

## 6. Cross-references

- **[`protocol-v6-spec.md`](protocol-v6-spec.md)** — the wire format §3a
  specifies, written out in full. Kept in `clasi/issues/` because
  `docs/protocol-vN.md` is reserved for *shipped* wire truth; promote it to
  `docs/protocol-v6.md` when its §13 cutover lands, not before.
- `clasi/issues/micropython-first-rebuild.md` — needs exactly §3a's
  dependency-free, MicroPython-clean codec. Its "v5 byte-for-byte compatible"
  decision should be revisited against v6 before that rebuild starts: porting
  ~150 lines of ASCII is dramatically less work than porting COBS + CRC +
  a protobuf subset, and the v6 wire is the one that has a JavaScript
  implementation.
- `clasi/issues/later/inbound-command-loss-needs-retransmit-not-a-slower-telemetry-stream.md`
  — F10 (the single-slot inbound radio buffer, `microbit_radio_link.h:60-62`)
  is a concrete candidate cause. **Nothing in this review fixes it**; do not
  let v6 be sold as the fix.
- `clasi/sprints/done/132-…/issues/the-configuration-object.md` — R4 both
  completes and partly reverses it: the object is right, the wire schema
  around it is not.
- `.claude/rules/configuration-discipline.md` — R4's invariants 1 and 2. The
  uploaded-program model satisfies invariant 1 more directly than baking does
  (the program *is* the file), but note that it removes the single schema
  saying what a robot config *is* — the 80-row field table is what has to
  hold that line.
- `.claude/rules/hardware-bench-testing.md` — the shared-channel `DBG:`
  reflection flood that V4 (case-separated directions) structurally kills.

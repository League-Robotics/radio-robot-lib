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

# Protocol v6 — wire specification (PROPOSED)

**Status: proposal, not shipped.** The live wire is
[`docs/protocol-v5.md`](../../docs/protocol-v5.md) — itself stale against its
own implementation in seven places (see
[`protocol-and-implementation-simplification-review.md`](protocol-and-implementation-simplification-review.md)
§F5; read `src/protos/` and `src/firm/core/comms.cpp` for v5 truth).

This document lives in `clasi/issues/` deliberately: `docs/protocol-vN.md` is
reserved for shipped wire truth. **Promote it to `docs/protocol-v6.md` when the
§13 cutover lands**, not before.

Design authority — stakeholder, 2026-08-19:

> *"Do we really need to have binary telemetry? … if we get rid of the binary
> scheme, then we don't really need COBS anymore. We don't need CRCs either …
> Any of the errors we've actually had have been packet loss. … maybe our
> entire configuration just goes down to a setter and a getter."*
>
> *"When you connect to the serial port of a MicroPython device, you get the
> REPL. I don't want to have to drop into another view of that in order to
> implement the protocol, so it'd be nice to implement that by just calling
> Python functions in the REPL. … when we have a Wi-Fi connection, this
> interface will be on a UDP port. It could be on the radio, and then if it's
> on the serial port, it's probably on a MicroPython REPL."*

Separator and id decisions — stakeholder, 2026-08-20: fields are separated by
**spaces**, not colons ("I think we don't need strings with spaces in them");
correlation ids return to their historical **`#`-prefix** spelling; and a few
verbs (`dbg`, `help` — §2) are allowed to suppress the delimiter and take the
rest of the line as one field.

---

## 1. What v6 is

**One grammar. ASCII. No framing layer.**

```
<VERB>(' '<field>)*'\n'
```

That is the entire wire format, both directions, every message. No COBS, no
CRC, no length prefix, no binary plane, no protobuf, no generated codec.
`readline()` is the transport.

Two consequences drive everything else:

1. **A v6 line is human-typable**, so a plain terminal is a complete client.
2. **A v6 line is a Python argument list**, so the REPL is a complete
   transport (§10) — no second view, no mode switch.

Both properties are *lost* the moment any part of the wire goes binary. The
REPL requirement and the JavaScript requirement point the same direction, and
that is the strongest argument for this design.

### 1.1 Why each v5 layer left

| v5 layer | Why it is gone |
|---|---|
| **CRC-16/CCITT** | The nRF radio computes *the same CRC* in hardware (`NRF52Radio.cpp:315-317` — `CRCCNF=Two`, `CRCINIT=0xFFFF`, `CRCPOLY=0x11021`) and drops failing packets before CODAL sees them; USB CDC has its own CRC plus retransmission. The app CRC was a third redundant check on a link with two — measured "zero CRC mismatches" is structural, not luck. Real losses are whole packets, about which a CRC says nothing. |
| **COBS** | Existed only to keep binary payload bytes off the `\n` terminator. ASCII payloads never contain `\n`. |
| **CRC scope extension** (`crcOverScope()`, `crcInit`/`crcUpdate`) | Existed only to protect the verb name, which sat outside the CRC'd payload. |
| **protobuf + `wire.cpp`** | 105 KB generated C++ and a 3852-line generator, buying tolerant schema evolution this project never uses — every rev has been an atomic rev-locked cutover. |
| **The ack ring** (depth 12, 3× repeat, packed `corr_id<<4\|err`) | An artifact of acks riding a periodic frame. Acks are their own line now (§8). |
| **The config wire schema** (`robot_config.proto`, `ConfigGroupTarget`, `SetConfigGroup`, `GetConfig`, `ConfigSnapshot`, `SetConfigField`, `CFG`) | Config travels inside the uploaded program; the wire needs one setter and one getter (§7). |

### 1.2 Scope assumption, stated explicitly

The motion verbs (`MOVE`/`WHEELS`/`GOTO`/`STOP`/`ESTOP`) **stay on the wire**.
The uploaded-program future is treated as *additive*: a program running on the
robot calls a motion library directly, but the wire still needs teleop, bench
scripting and camera-in-the-loop control. If that is wrong and the robot should
eventually speak only `SET`/`GET`/`TLM`, §5 is the section that deletes;
nothing else depends on it.

---

## 2. Line grammar

```
line   ::= sp? verb ( sp field )* sp? '\n'
sp     ::= ' '+
verb   ::= [A-Za-z][A-Za-z0-9_]*
field  ::= any bytes except ' ' and '\n'
id     ::= '#' [0-9]+        (a field in trailing position — §8.2)
```

- **Terminator is `'\n'` (0x0A).** A lone `'\r'` before it is stripped
  (terminal artifact); `'\r'` appears nowhere else.
- **The first space ends the verb.** One or more spaces separate fields; a run
  of spaces is ONE separator, and leading/trailing whitespace on the line is
  ignored. This slack is deliberate — a human at a terminal gets forgiveness
  for free, and `line.split()` / `strtol`'s own whitespace-skipping implement
  it with no extra code (§11.1).
- **A blank or all-whitespace line is ignored silently** — a terminal
  artifact, not an error; it does not count malformed.
- **Fields are positional**, fixed arity per verb. Optional fields are
  trailing only, marked `[…]` in the tables. (The earlier draft's "empty
  field means use the default" is gone — an empty token cannot exist between
  spaces. The one middle-skip it enabled, `CAL`, stays expressible because
  the id is self-marking — §8.2.)
- **The id, where a verb carries one, is the last token and is spelled
  `#<n>`** (§8.2). Because it announces itself, omitting an optional field
  never shifts it into a data position.
- **Two verbs suppress the delimiter**: `dbg` and `help` take everything
  after the first space as ONE field, verbatim, spaces included
  (rest-of-line). They are the only two, both robot→host, and neither
  carries an id.
- **Max line: 240 bytes** including the terminator — inside the radio's 247 B
  MTU (§9.2) and inside a safe UDP payload (§9.3), so **no v6 message ever
  fragments on any transport**.
- **Unknown verb, wrong arity, or unparseable field** → drop the line,
  increment the malformed counter (`flags` bit 9). If the line's last token
  is a well-formed nonzero `#id`, reply `err #<id> <code>` — the self-marking
  id is trustworthy even on a line that otherwise failed to parse; otherwise
  no reply. **The one exception is `ESTOP`, which never emits a reply under
  any circumstance, including this one**: a malformed `ESTOP` line (stray
  field, stray `#id`) is dropped and counted, silently. The panic stop must
  never queue behind an outbound reply (§5.4/§8.2), and that rule outranks
  this recovery rule wherever the two meet.

### 2.1 Direction is carried by CASE, and that is load-bearing

**Commands (host → robot) UPPERCASE. Replies (robot → host) lowercase.** Verb
lookup is case-**sensitive**.

Not cosmetic. On a shared radio channel a robot hears every other robot. Under
v5 a robot's own `DBG:` *output* is a syntactically valid `DBG` *command* to
every robot on the channel, and the flood is self-sustaining — the incident in
[`.claude/rules/hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md).
Under v6 a reply can never parse as a command, so that class is closed
structurally instead of by keeping the channel private.

A robot receiving a lowercase verb drops it silently and does **not** count it
malformed — that is another robot's reply, not an error.

### 2.2 Numbers

**Every wire value is a base-10 ASCII integer**, optionally signed, except
config values (§7.2). No exponents, no `NaN`, no `inf`.

Unit and scale are fixed by the tables below and never appear on the wire. A
field documented `[mrad]` carries milliradians as a whole number: `1571` is
π/2.

Why integers: `newlib-nano` has no `printf` float support, so the firmware
cannot format `%f`. Inbound parsing is unaffected — `strtof` *is* available and
already used (`Comms::stageSeed`). The asymmetry is real and this spec is built
around it. (On MicroPython neither constraint applies, but the wire format
stays identical so one parser serves both.)

`flags` is the one exception to base-10: lowercase hex, no `0x`.

---

## 3. Verb tables

### 3.1 Commands (host → robot)

| Verb | Fields | § |
|---|---|---|
| `HELLO` | — | 4 |
| `PING` | — | 4 |
| `ID` | — | 4 |
| `VER` | — | 4 |
| `STATUS` | — | 4 |
| `HELP` | — | 4 |
| `GET` | `[name]` | 7 |
| `SET` | `name value [#id]` | 7 |
| `TLM` | `mode` | 6 |
| `MOVE` | `kind a b c stop limit timeout #id` | 5.1 |
| `WHEELS` | `left right duration [#id]` | 5.2 |
| `GOTO` | `x y frame speed arrive timeout #id` | 5.3 |
| `STOP` | `#id` | 5.4 |
| `ESTOP` | — | 5.4 |
| `SEED` | `x y h [#id]` | 5.5 |
| `CAL` | `[samples] [#id]` | 5.6 |

### 3.2 Replies (robot → host)

| Verb | Fields | § |
|---|---|---|
| `device` | `NEZHA2 robot <name> <serial>` | 4 |
| `ready` | — | 4 |
| `pong` | `<now>` | 4 |
| `id` | `<drivetrain> <profile> <version>` | 4 |
| `ver` | `<version>` | 4 |
| `status` | `k=v k=v …` | 4 |
| `help` | `<rest of line: the verb list>` | 4 |
| `get` | `name value` | 7 |
| `ok` | `[#id]` | 8 |
| `err` | `[#id] code` | 8 |
| `done` | `#id reason` | 8 |
| `thdr` | `col col …` | 6.2 |
| `t` | `val val …` | 6.2 |
| `dbg` | `<rest of line: free text>` | 4 |

30 verbs, one grammar, no interception order, and exactly one per-verb
framing decision — the two rest-of-line replies, `help` and `dbg` (§2).
Compare v5: 25 verbs across two planes, four cleartext sub-grammars, and a
`dispatchLine()` whose parse *order* is load-bearing.

---

## 4. Session verbs

| Command | Reply | Notes |
|---|---|---|
| `HELLO` | `device NEZHA2 robot <name> <serial>` | Also emitted unsolicited at boot, twice (power-on, preamble-done). |
| — | `ready` | Unsolicited, once, when the loop will actually accept a `MOVE`. **`pong` is liveness, not readiness** — the board answers `PING` ~5 s before it stops rejecting moves with `ERR_NOT_CONFIGURED` (measured: 5 of 6 fresh connections lost their first move to exactly this). Wait for `ready`. |
| `PING` | `pong <now>` | `now` = robot clock `[ms]`. Drives host clock-sync. |
| `ID` | `id <drivetrain> <profile> <version>` | Configured identity — "am I talking to the robot I think, calibrated how I think". |
| `VER` | `ver <version>` | Build identity. |
| `STATUS` | `status ready=1 active=0 connL=1 connR=1 otos=1 wedge=0 flags=<hex> tlm=pose` | Queryable counterpart to `ready`, which a late-connecting host has missed forever. Deliberately extensible: `k=v`, order not guaranteed, unknown keys ignored. |
| `HELP` | `help HELLO PING ID VER STATUS HELP GET SET TLM MOVE …` | Generated by walking the verb table at runtime, so it cannot drift from the dispatcher. Rest-of-line on the reply side (§2). |
| — | `dbg <text>` | Unsolicited firmware→host debug channel; compiled in only under `ROBOT_DEBUG`/`HOST_BUILD`. Rest-of-line: the text may contain spaces. **Safe on a shared channel under v6** (§2.1). |

### 4.1 The banner is the one byte-frozen string v6 breaks

v5's banner is `DEVICE:NEZHA2:robot:<name>:<serial>`, byte-frozen and matched by
host role detection and by the relay. v6 rewrites it twice over: the verb
lowercases to `device` AND the separators become spaces —
`device NEZHA2 robot <name> <serial>`. Nothing about the line is byte-frozen
any more. This is a deliberate, called-out break.

**Migration:** host role detection matches both shapes for one release —
either verb case, either separator (`^(DEVICE|device)[: ]` then tokenize on
the matched separator). Detection keys on the `NEZHA2` *token*, which
survives the rewrite in both spellings; verify at cutover that no matcher
grips a full byte-frozen prefix (`src/host/robot_radio/io/serial_conn.py`'s
role detection is the known site).

---

## 5. Motion

Semantics unchanged from v5. Every motion is **bounded** — a stop condition plus
a required timeout backstop, so a dead host can never mean a runaway.

### 5.1 `MOVE <kind> <a> <b> <c> <stop> <limit> <timeout> #<id>`

| field | meaning |
|---|---|
| `kind` | `t` twist (`a`=`v_x` `[mm/s]`, `b`=`v_y` `[mm/s]`, `c`=`omega` `[mrad/s]`) · `w` wheels (`a`=left `[mm/s]`, `b`=right `[mm/s]`, `c` unused, send `0`) |
| `stop` | `t` elapsed `[ms]` · `d` path length `[mm]` · `a` heading change `[mrad]` |
| `limit` | stop threshold in `stop`'s unit; `<= 0` → `ERR_BADARG` |
| `timeout` | `[ms]` REQUIRED backstop; `<= 0` → `ERR_BADARG` |
| `id` | `#`-prefixed, 1..999999, unique for the session (§8.2); required here |

`v_y` is accepted and ignored on a differential build — wire-forward for a
holonomic base. Queue is 1 active + 4 pending; arriving at a full queue is
`ERR_FULL`.

`MOVE` supersedes a `WHEELS` hold; `WHEELS` clears the planner. Exactly one
subsystem owns motion at a time, enforced at routing.

**v5's `replace` flag is dropped** — a boolean smuggling two different verbs.
Preemption is `ESTOP` then the new `MOVE`: explicit, and already what every
correct caller did.

```
MOVE t 150 0 0 d 400 5000 #7      forward 150 mm/s until 400 mm travelled
MOVE w 100 -100 0 a 1571 4000 #8  spin in place until 90° of heading change
```

### 5.2 `WHEELS <left> <right> <duration> [#<id>]`

Dumb teleop primitive, straight to the wheel controller, no planner.
`left`/`right` `[mm/s]`; `duration` `[ms]` **required**, ceiling 5000 — a dead
host cannot mean a runaway.

### 5.3 `GOTO <x> <y> <frame> <speed> <arrive> <timeout> #<id>`

`x`,`y` `[mm]`; `frame` `0`=world (OTOS/`SEED` frame), `1`=robot, resolved once
at acceptance; `speed` `[mm/s]` cruise, `0`=config default; `arrive` `[mm]`
tolerance, `0`=config default; `timeout` `[ms]` required.

### 5.4 `STOP #<id>` and `ESTOP`

**Not synonyms, and the distinction is measured.**

- **`ESTOP`** — halt now. Zeroes wheel targets *and* clears the planner's active
  + pending queue in the same cycle. Discarded entries get **no** `done`. No id,
  no ack — it must never queue behind anything.
- **`STOP #<id>`** — a *planned* stop: an ordinary queue entry that waits its
  turn, ramps down at the decel ceiling, completes at rest. `ok #<id>` on
  enqueue, `done #<id> stop` when the robot is actually stopped.

Measured on a 400 mm leg with the halt sent 0.5 s in: `STOP` travelled the
entire 39.8 cm and took 5.9 s to go inactive; `ESTOP` travelled 2.9 cm and went
inactive in 0.10 s. **Every geofence, Ctrl-C handler and panic path must use
`ESTOP`.**

**One `ESTOP` is not proof of a stop.** The Nezha brick latches its last
commanded speed and does not reset on an nRF reset, so a lost write is
permanent. Measured 2026-08-03 on `vevov`: a single `ESTOP` failed 5 of 6
attempts, and one issued by a then-silent host produced 936 mm of continued
travel with no decay. **A halt path must confirm the robot actually stopped
(`flags` bit 2 clear, encoders holding) and re-issue if it did not.**

### 5.5 `SEED <x> <y> <h> [#<id>]`

Seed world pose from an external fix, normally the overhead camera at run start.
`x`,`y` `[mm]`, `h` `[mrad]`. Writes **both** pose sources — the OTOS position
register (lever arm applied in firmware) and the encoder odometry — so the two
start agreed and their later divergence *is* the drift being measured.

`h` **must be wrapped to (−π, π]** before sending; an unwrapped heading corrupts
the position seed by ~91 mm.

Reply `ok #<id>` if an id was given; the applied pose is then visible in the
next `t` frame (both sources are pose-mode columns, §6.3).

v5's separate `POSE` query verb is **dropped** — `TLM NOW` returns the same
information in the same shape as the periodic frame.

### 5.6 `CAL [<samples>] [#<id>]`

Both fields optional, and — because the id is self-marking — independently so:
`CAL #9` is "default samples, ack as 9" with no placeholder needed.

Re-run OTOS gyro bias calibration on demand, robot parked. `ERR_BUSY` unless
both wheels are encoder-still and nothing commands velocity this cycle;
`ERR_NOT_CONFIGURED` with no OTOS. Bias only — tracking and a seeded pose
survive, unlike boot-time init which resets both.

Exists because boot calibration is unguarded: a robot booted while handled
carries a poisoned heading all session (measured on `tovez`: **+1.44 deg/s**
standstill drift after a mid-handling boot, **−0.006 deg/s** after one still
reboot).

---

## 6. Telemetry

### 6.1 `TLM <mode>` — telemetry is a subscription

| mode | effect |
|---|---|
| `OFF` | no `t` frames |
| `POSE` | **default** — 9 columns, ~38 B/frame (§6.3) |
| `FULL` | 35 columns, ~160 B/frame (§6.4) |
| `NOW` | emit one frame immediately in the current mode; does not change mode |
| `AUTO` | as `POSE`, silent while the robot is parked |
| `BUFFER` | do not push; accumulate frames for the REPL to drain (§10.4) |

Mode is per-connection, resets at boot (`POSE` on radio/UDP, `BUFFER` on the
REPL — §10.4). A mode change emits a fresh `thdr` before the next `t`.

**This is the change that makes ASCII free.** v5 carried its diagnostic tail
always; the only consumers of that tail are three bench scripts. Default `POSE`
is *half* the bytes of today's binary frame; `FULL` is what `tlm_log.py` and the
tuning benches subscribe to.

Rate is one frame per control cycle, floor 25 ms (~31 fps). Raising it trades
against inbound command loss on the half-duplex radio — §9.2.

### 6.2 `thdr` / `t` — the frame is self-describing

The robot emits a **column header** whenever the subscription changes, and
before the first frame after connect:

```
thdr seq now flags x y h ox oy oh
t 412 38472 d8 -1234 892 1571 -1240 889 1573
t 413 38504 d8 -1198 901 1571 -1205 898 1572
```

A reader zips `thdr` against each `t`. Nothing needs a schema, a field table
or version negotiation, and `tlm_log.py` becomes "write the header row, write
the data rows" — a valid whitespace-delimited table by construction, one
`s/ /,/g` from CSV. A host that reconnected mid-stream and missed the header
sends `TLM NOW`.

Column **order** is fixed per mode by §6.3/§6.4 — `thdr` is a convenience and a
drift check, not licence to reorder.

### 6.3 `POSE` columns (9)

| col | unit | meaning |
|---|---|---|
| `seq` | — | increments per sent frame, wraps at 128 |
| `now` | `[ms]` | robot clock at frame assembly |
| `flags` | hex | §6.5 |
| `x` `y` | `[mm]` | encoder-odometry pose |
| `h` | `[mrad]` | encoder-odometry heading |
| `ox` `oy` | `[mm]` | OTOS pose — valid iff `flags` bit 0 |
| `oh` | `[mrad]` | OTOS heading |

Both pose sources ride every frame deliberately: seeded from one fix (§5.5),
their divergence over a run *is* the drift being measured, so reporting them in
the same frame at the same instant is the measurement.

### 6.4 `FULL` columns (35)

`POSE`'s 9, then these 26 (the count was misstated as 30 in the first
draft — the table below is the authority; 9 + 26 = 35):

| col | unit | meaning |
|---|---|---|
| `mode` | — | 0 idle, 1 streaming, 2 timed, 3 distance, 4 navigating, 5 velocity |
| `elp` `elv` `ela` `ele` | `[mm]` `[mm/s ×10]` `[ms]` — | left encoder position, velocity, sample age, position epoch |
| `erp` `erv` `era` `ere` | same | right encoder |
| `ovx` `ovy` | `[mm/s ×10]` | OTOS velocity |
| `ow` | `[rad/s ×100]` | OTOS angular rate |
| `oa` | `[ms]` | OTOS sample age |
| `tvx` `tvy` | `[mm/s ×10]` | body twist, fused from both wheels |
| `tw` | `[rad/s ×100]` | body twist angular rate |
| `l1` `l2` `l3` `l4` | — | line sensor channels — valid iff `flags` bit 13 |
| `cr` `cg` `cb` `cc` | — | colour RGBC — valid iff `flags` bit 14 |
| `cyb` `cyp` | `[us]` | cycle busy, cycle period |

`×10`/`×100` mean the wire integer is the value times that factor — `elv=1500`
is 150.0 mm/s. These are v5's `(scale)` quanta carried forward unchanged, so no
precision is lost relative to v5.

**`age` is relative, not an absolute timestamp** — `now` minus that sample's own
collect time. Left and right differ by roughly the settle-collect separation;
never equal, never zero. An absolute clock could not fit a small integer.

Encoder `position` accumulates for the session and is **rebaselined** in
firmware at `|position| >= 30000` mm, incrementing `ele`/`ere`. A host seeing an
epoch change knows a discontinuity occurred; sum per-epoch totals if cumulative
travel matters.

### 6.5 `flags`

Lowercase hex, no prefix. Bits carried forward from v5 unchanged, so existing
decoders and bench scripts keep working:

| bit | meaning | bit | meaning |
|---|---|---|---|
| 0 | otos present | 13 | line valid |
| 1 | otos connected | 14 | colour valid |
| 2 | **active** (a move is running) | 15 | fault: move timeout |
| 3 | left wheel connected | 16 | fault: shaping disabled |
| 4 | right wheel connected | 17 | fault: position clamped |
| 5 | line re-read this cycle | 18 | fault: commands dropped |
| 6 | fault: I2C safety net | 19/20 | fault: wheel frozen L/R |
| 7 | fault: wedge latch | 21/22 | fault: wheel deficit L/R |
| 8 | fault: I2C NAK | 23 | colour re-read this cycle |
| 9 | fault: malformed line | 24/25 | fault: stall L/R |
| 10 | event: deadman expired | 26 | fault: kernel stalled (sticky) |
| 12 | event: config applied | 27-31 | reserved |

Bit 11 stays reserved — v5 declared `kFlagEventBootReady` and never set it;
`ready` (§4) carries that meaning now.

Bits 5/23 are *freshness*; bits 13/14 are *validity*. Line and colour tick on
alternate cycles so exactly one freshness bit is set per frame — but **both
readings ride every frame**. Want validity unless a sample must be
just-measured.

---

## 7. Configuration — one setter, one getter

Configuration is **baked by default**: the robot's operating configuration
arrives with the program it runs. The wire's job is bench tuning, one value at a
time. That is the whole surface.

### 7.1 `GET` / `SET`

```
GET wheel_control.pid_kp          ->  get wheel_control.pid_kp 0.020000
GET                               ->  get <name> <value>   (one line per field, 80 lines)
SET wheel_control.pid_kp 0.03     ->  ok          (no id: acked once, bare — §8.2)
SET wheel_control.pid_kp 0.03 #9  ->  ok #9       (or err #9 3 out of range)
```

Names are `<group>.<field>`, lowercase, exactly the 80 names in §7.3. A bare
`GET` dumps every field — simultaneously the read-back-vs-file acceptance test
and something a human can read at a plain terminal.

**`GET` with an unknown name is silent** — no reply, and not counted
malformed. `GET` carries no id, so there is no channel to `err` on, and a
`get`-with-empty-value reply would invent a shape; silence is the defined
behavior, stated here so no implementation picks a different one.

**`SET` applies immediately where the field is live, and is stored otherwise.**
v5's boot-only/live split (`ERR_NOT_LIVE`, and a `rotational_slip` needing a
*reflash*) collapses into "takes effect now" vs "takes effect next boot". Every
field is settable; only the *apply* is gated.

`SET` is `ERR_BUSY` while the target subsystem is in motion, `ERR_RANGE` outside
declared bounds, and `ERR_BADARG` for a non-finite value — checked **before**
bounds, since bounds compare with `<`/`>`, both false for NaN, so an unchecked
NaN would pass every bound.

### 7.2 Value format — the one place floats appear

Config values are **decimal**, not scaled integers, because a human types and
reads them.

- **Inbound** (`SET`): `strtof`, which `newlib-nano` provides.
- **Outbound** (`get`): a hand-rolled `formatFixed(value, decimals)` using
  integer arithmetic — `newlib-nano`'s `printf` has no `%f`. Six fractional
  digits, always present, no exponent: `0.020000`, `-51.500000`.

That ~15-line helper is the only float formatting in the firmware. Telemetry
(§6) stays pure scaled integers precisely so it never needs it at 31 fps.

### 7.3 The field table — 80 rows, one declaration

The firmware holds **one** generated table: `name, offset, type, scale, min,
max`. `SET` looks up, bounds-checks, assigns; `GET` prints. This table is the
single place a field's name and its storage are written, which is what makes the
v5-era `pid.kff → kaff` drift class structurally impossible — no second copy to
drift from.

| group | n | fields |
|---|---|---|
| `geometry` | 6 | `trackwidth` `rotational_slip` `rotation_gain_pos` `rotation_offset` `rotation_gain_neg` `rotation_offset_neg` |
| `motors` | 14 | `left_port` `right_port` `travel_calib_left` `travel_calib_right` `fwd_sign_left` `fwd_sign_right` `output_deadband` `reversal_dwell` `vel_kp` `vel_ki` `vel_kff` `vel_i_max` `vel_kaw` `vel_filt_alpha` |
| `drive` | 11 | `duty_per_speed_left` `duty_per_speed_right` `crawl_pulse` `wheel_gain_left_accel` `wheel_intercept_left_accel` `wheel_gain_left_decel` `wheel_intercept_left_decel` `wheel_gain_right_accel` `wheel_intercept_right_accel` `wheel_gain_right_decel` `wheel_intercept_right_decel` |
| `wheel_control` | 15 | `v_min` `bias_max` `tau_adapt` `a_steady` `deficit_threshold` `deficit_window` `pid_kp` `pid_ki` `pid_i_max` `pid_kaff` `pid_max` `pos_err_max` `stall_speed` `stall_demand` `stall_window` |
| `planner` | 12 | `v_max` `omega_max` `control_period` `actuation_delay` `settle_rest_velocity` `settle_rest_omega` `settle_epsilon_linear` `settle_epsilon_angular` `heading_hold_gain` `decel_plan_fraction` `align_tol` `align_max_nudges` |
| `planner_shaper` | 6 | `a_max` `a_decel` `alpha_max` `alpha_decel` `jerk_max` `yaw_jerk_max` |
| `navigator` | 8 | `speed` `max_wheel_step` `behind_angle` `turn_first_angle` `approach_radius` `approach_speed` `default_arrival_tolerance` `yaw_sign` |
| `otos` | 5 | `offset_x` `offset_y` `offset_yaw` `linear_scale` `angular_scale` |
| `estimator` | 3 | `weight_heading_otos` `weight_omega_otos` `staleness` |

Names are carried verbatim from v5's `robot_config.proto`, so no robot JSON,
bench script or note has to be rewritten.

### 7.4 What a tuning sweep still owes

`.claude/rules/configuration-discipline.md` relaxes "everything from the file"
for development, and that relaxation is safe only because of read-back. So a
sweep must **read back what it pushed** (`GET`, never the `ok` — config that
acks and lands nowhere is a live failure mode here), **record pushed values with
results**, and **promote the winner into the robot config** before anything is
baked or used as a baseline.

---

## 8. Outcomes

### 8.1 Three reply verbs, one meaning each

| reply | meaning |
|---|---|
| `ok [#id]` | accepted — enqueued, or applied |
| `err [#id] <code>` | rejected, with a reason |
| `done #<id> <reason>` | the enqueued thing **finished**; `reason` ∈ `stop` (stop condition met) or `timeout` (backstop fired) |

An **id-carrying** reply is sent **three times** on consecutive cycles. A
reply is ~12 bytes, so this costs nothing, and it makes an outcome survive
the measured ~5% radio frame loss without a ring, a depth, an eviction policy
or a scan. A host takes the first copy and ignores repeats by id. A reply
**without** an id (the bare `ok` to an id-less `SET`) is sent **once** —
there is no key to dedup repeats by; supply an id when the link is lossy.

**Who owns the repeat:** it is emission policy, owned by whatever drives the
robot's per-cycle output (`Core::Comms`'s cycle in the firmware; a
`tick()`-driven pending-reply table in a standalone handler port). It is NOT
a property of the line codec — a handler implemented as a pure function of
its input bytes emits each reply once, and the repeat is added where the
cycle lives. Specified target behavior, not yet implemented anywhere; see
`radio-robot-lib` `docs/spec-defects.md` D4 for the decision record.

**`done`'s `reason` replaces v5's flags-bit encoding.** v5 signalled
timeout-vs-stop-condition only through `flags` bit 15 on the same frame as a
completion ack whose `err` was always 0. It is a word now.

### 8.2 Ids

- An id is spelled **`#<n>`** and is always the **last token** of its line —
  commands and replies alike. (Restores the pre-v5 `#` correlation-id
  spelling — stakeholder, 2026-08-20.)
- Host-assigned, `1..999999`, **unique for the session**. The digits are
  bare and unsigned: `#+5`, `#-5`, and `# 5` are all malformed. §2.2's
  "optionally signed" applies to data fields, not to the id — parse it with
  a dedicated digits-only parser, not a general signed-integer one.
- Because the id announces itself, it never shifts position when an optional
  field is omitted — `CAL #9` needs no placeholder — and it is recoverable
  even from a line that otherwise fails to parse (§2's `err` rule).
- **Required** on `MOVE`/`GOTO`/`STOP`: they complete asynchronously and
  `done` needs a correlation key. **Optional** on `SET`/`WHEELS`/`SEED`/`CAL`.
- **Omitted id** → the command still executes, and its `ok`/`err` is sent
  once, bare (§8.1) — a human at a terminal gets confirmation without
  inventing ids. **`#0`** → "no ack wanted": execute silently. `#0` is legal
  only where the id is optional; on `MOVE`/`GOTO`/`STOP` it is malformed.
- A reused id is `err #<id> 11` (`ERR_DUPLICATE_ID`). Under v5 a reused move id
  was acked `err=0` and then **silently dropped** — a real, recorded footgun.
  v6 refuses it out loud.
- `ESTOP` never carries an id and is never acked — it must not queue behind
  anything, including an ack. This covers the malformed case too: `ESTOP #5`
  gets no `err`, overriding §2's recovery rule (stated there as well).

### 8.3 Error codes

| code | name | meaning |
|---|---|---|
| 1 | `ERR_UNKNOWN` | no such verb or field name |
| 2 | `ERR_BADARG` | malformed/non-finite argument, wrong arity |
| 3 | `ERR_RANGE` | declared bound violated |
| 4 | `ERR_FULL` | queue full (4 pending) |
| 6 | `ERR_UNIMPLEMENTED` | recognized, not wired on this build |
| 8 | `ERR_NOT_CONFIGURED` | refused pre-`ready` |
| 10 | `ERR_BUSY` | subsystem in motion; retry at rest |
| 11 | `ERR_DUPLICATE_ID` | **new in v6** — §8.2 |

Codes 5 (`ERR_DECODE`) and 7 (`ERR_OVERSIZE`) retire with the binary plane;
9 (`ERR_NOT_LIVE`) retires with the boot-only/live split (§7.1). Numbers are
never reused.

---

## 9. Transports — one line format, three carriers

**A v6 message is a line. A transport's only job is to deliver lines.** Nothing
in §2-§8 differs by transport; only the table below does.

| transport | carrier | telemetry default | notes |
|---|---|---|---|
| **serial (USB)** | MicroPython REPL — protocol as Python calls (§10) | `BUFFER` | REPL is the surface; no mode switch required |
| **WiFi** | UDP datagram, one line per datagram (§9.3) | `POSE` | REPL also mirrored on TCP :7654 |
| **radio** | RadioRelay §5 frame (§9.2) | `POSE` | primary untethered path |

### 9.1 Line delivery contract

Every transport must deliver **whole lines, in order within a message, with no
partial lines**. It may lose whole lines. That is the entire contract, and it is
why the `ok`/`err`/`done` 3× repeat (§8.1) and the telemetry `seq` (§6.3) exist:
they are the only loss-tolerance the protocol has, and they are enough.

### 9.2 Radio

RadioRelay §5 framing: `[SEQ:1][FLAGS:1][LEN:1][payload:LEN]`, MTU 247 B,
1 Mbit/s, group 10. **A message under 247 B is one on-air packet**, flagged
`START|END`. v6's 240 B line cap therefore guarantees no v6 message ever
fragments — partial reassembly disappears as a failure mode.

Both ends must be on the same channel or the robot is simply unreachable.

**Known: outbound rate trades against inbound reliability.** The link is half
duplex — outbound airtime eats the window in which the robot can hear the host.
Measured over the `getez` relay 2026-08-07 at v5 frame sizes:

| emit period | telemetry | inbound |
|---|---|---|
| 25 ms | 31.4 fps | a `WHEELS` command was **lost** |
| 40 ms | 15.8 fps | ok |

v6's default `POSE` roughly halves outbound bytes per frame, which helps but
**does not fix this**. Two honest caveats:

1. The loss is *inbound*; outbound measured 99.2% ok, zero unparseable frames at
   both rates.
2. A better candidate cause than airtime is that the robot's inbound radio path
   is a **single-slot buffer** — `microbit_radio_link.h:60-62`: *"a second
   message completing before `readLine()` drains the first is dropped."* That is
   per-message and size-independent, which fits the data (same frame size,
   different period, large difference) far better than bytes-on-air does. The
   MicroPython rebuild's `radio.config(queue=4)` addresses exactly this.

**Do not present v6 as the fix for inbound command loss** — tracked separately
in `clasi/issues/later/inbound-command-loss-needs-retransmit-not-a-slower-telemetry-stream.md`.

### 9.3 WiFi / UDP

**One v6 line per datagram**, UDP port 7654 (matching the MicroPython rebuild's
existing plane). The trailing `'\n'` is retained even though UDP already gives
message boundaries, so the same parser serves every transport unchanged.

- **240 B cap** keeps every datagram inside one IP packet — no fragmentation.
- **One `CIPSEND` per datagram.** Per-character AT sends flood the module; the
  one-line-per-datagram shape maps onto `CIPSEND` exactly.
- **Address discovery:** the robot listens; the first inbound datagram
  establishes the reply endpoint, and telemetry flows there. Send `HELLO` to
  register. A host that stops sending for 30 s is dropped and telemetry stops —
  a deliberate deadman, since UDP gives no disconnect signal.
- **UDP is lossy and unordered**, exactly like the radio, so §8.1's 3× repeat
  and §6.3's `seq` cover it with no extra mechanism.
- Measured RTT ~33 ms WiFi vs ~5 ms USB — do not read sub-100 ms timings
  through it.
- **The WiFi module persists passthrough/socket/static-IP state across an nRF
  reflash.** Power-cycle before believing a bring-up failure.

### 9.4 Serial

115200 baud CDC. A `POSE` frame at 31 fps is ~1200 B/s, ~10% of the link.

**Closing the port resets the robot** (macOS drops DTR on last close), rebooting
the MCU and wiping live-pushed config. Hold the port open for any session with
more than one command.

On a MicroPython build the serial port is the REPL — see §10.

### 9.5 Relay control plane

A line whose first byte is `#`, `!` or `?` is a radio-relay dongle control-plane
line. Drop it **before** verb lookup and do **not** count it malformed —
handshake fragments reach the robot before the dongle commits to pass-through,
and counting them trips a fault bit on a clean connect. No v6 verb starts with
these bytes, so this can never mask a real error.

The `#` that introduces an id (§8.2) never collides with this rule:
control-plane detection reads the FIRST byte of the line, and an id is never
first — a verb is.

---

## 10. The REPL binding

**Requirement (stakeholder, 2026-08-19):** connecting to the serial port of a
MicroPython device gives you the REPL, and the protocol must be reachable
*there* — by calling Python functions — without dropping into another view.

### 10.1 The design in one sentence

**A v6 line and a Python call are two spellings of one command table**, so the
REPL is not a second protocol — it is a second *binding* of the same table.

```
MOVE t 150 0 0 d 400 5000 #7              wire
p("MOVE t 150 0 0 d 400 5000 #7")         REPL, line-identical
r.move(v_x=150, stop_distance=400, timeout=5000, id=7)    REPL, ergonomic
```

The line binding is generated from the table; the ergonomic binding is a thin
hand-written wrapper. Neither needs a codec, because there is no codec.

**This only works because v6 is ASCII.** A COBS+CRC protobuf frame cannot be
typed at a REPL, cannot be a Python argument list, and cannot be read back as
output. The REPL requirement and the JavaScript requirement both fall out of the
same decision.

### 10.2 `p(line)` — the machine door

```python
>>> import v6; p = v6.p
>>> p("PING")
pong 38472
>>> p("GET wheel_control.pid_kp")
get wheel_control.pid_kp 0.020000
>>> p("MOVE t 150 0 0 d 400 5000 #7")
ok #7
```

`p()` **prints** each reply line and returns `None`. That is deliberate: a
returned string would be echoed by the REPL as `'ok #7'` — with quotes — and a
host would have to strip them. Printing means **the REPL's stdout is
byte-identical to the v6 wire**, so one host parser serves REPL, UDP and radio
with no special-casing.

A host driving this in **raw REPL mode** (Ctrl-A: no echo, no `>>> ` prompt,
`\x04`-delimited output) sees exactly the v6 byte stream it would see over UDP,
wrapped in raw-REPL framing it already has to handle to talk to MicroPython at
all. `mpremote`'s framing is the transport; v6 is the payload.

### 10.3 `r.*` — the human door

The ergonomic wrapper, for a person at a terminal and for on-robot programs:

```python
>>> from robot import r
>>> r.ready()                       # blocks until the robot will accept a move
True
>>> r.move(v_x=150, stop_distance=400, timeout=5000)
7
>>> r.wait(7)                       # -> 'stop' | 'timeout'
'stop'
>>> r.pose()
(-1234, 892, 1571)
>>> r.get('wheel_control.pid_kp')   # a float, not a string
0.02
>>> r.set('wheel_control.pid_kp', 0.03)
True
>>> r.estop()
```

`r.*` returns **Python values** (ints, floats, tuples, bools) and raises on
error. `p()` returns **wire lines**. Same table underneath; pick the door by
who is reading.

**The same `r` object is what an uploaded program imports.** A program running
on the robot calls `r.move(...)` directly with no wire involved; the identical
call from a host REPL goes over the wire. That is the property that makes the
uploaded-program future (§1.2) a *deployment* change rather than a rewrite.

### 10.4 Telemetry at a REPL — buffer, do not push

This is the one genuinely hard part, and it is why `TLM BUFFER` exists.

31 fps of asynchronous `t` lines printed into a REPL destroys it: they
interleave with the prompt, with the echo of what is being typed, and with any
other output. So **on the REPL transport, telemetry defaults to `BUFFER`**: the
control loop appends frames to a bounded deque and prints nothing.

```python
>>> r.tlm('full')            # subscribe; still buffered
>>> r.frames()               # drain -> list of tuples, newest last
[(412, 38472, 216, -1234, 892, 1571, ...), ...]
>>> r.frames(clear=False)    # peek without draining
```

Three reasons this is the right default rather than a workaround:

1. **It cannot corrupt the REPL**, because nothing is printed asynchronously.
2. **It does not perturb the robot.** Measured 2026-08-19: *polling* telemetry
   during a move over the relay cut travel from 197.5 mm to 0.3 mm — a
   request/reply round-trip inside a move is actively dangerous. Draining a
   buffer is not a round-trip; the frames were already collected.
3. **It is what the other transports already do**, viewed from the robot: the
   loop produces frames, and the transport decides how they leave. Push on
   radio/UDP, drain on REPL. The frame itself is identical.

`r.tlm('print')` opts into asynchronous printing for a human watching a
terminal. It is never the default and a host script should not use it.

### 10.5 Interrupts and safety at the REPL

- **Ctrl-C** raises `KeyboardInterrupt` in the REPL. The motion lease
  (`WHEELS.duration`, `MOVE.timeout`) is what actually stops the robot — **not**
  the interrupt. A `finally: r.estop()` around any driving loop is mandatory,
  and `r.estop()` must be re-issued until `flags` bit 2 clears (§5.4).
- **A CPU-bound Python loop starves the idle hook.** The motion kernel runs on
  its own fiber with a lease watchdog precisely so a wedged REPL cannot leave
  the wheels powered.
- **`ESTOP` must reach the robot even while the REPL is busy.** On radio and UDP
  it does, because those planes are pumped from a timer. On the serial REPL a
  blocked interpreter cannot process anything — which is a real, stated
  limitation of REPL-as-transport, and the reason the radio plane stays live in
  parallel rather than being replaced by it.

### 10.6 JavaScript

JS is a host language, not a robot language. It reaches the robot the same three
ways, with no new protocol surface:

| path | mechanism |
|---|---|
| UDP | Node `dgram`, one line per datagram — **~150 lines, zero dependencies** |
| serial | WebSerial (browser) or `serialport` (Node), driving the REPL exactly as §10.2 describes |
| radio | via the relay dongle's serial port, same as any other host |

The `p()`/`r.*` split ports directly: `p(line)` is the transport-agnostic core,
`r.*` a per-language ergonomic wrapper. Because the wire is text, the JS
implementation shares the conformance fixture (§11.3) with C++ and Python
byte-for-byte.

---

## 11. Implementing v6

### 11.1 The whole codec

```python
def encode(verb, *fields, id=None):
    parts = [verb] + [str(f) for f in fields]
    if id is not None:
        parts.append("#" + str(id))
    return (" ".join(parts) + "\n").encode()

def decode(line):
    parts = line.split()            # collapses space runs, strips '\r\n'
    id = None
    if parts and parts[-1][:1] == "#":
        id = int(parts[-1][1:])
        parts = parts[:-1]
    return parts[0], parts[1:], id
```

That is not a simplification for the document's sake — it is the implementation.
Add a verb table, an arity check, and the 80-row config table and you are done.
**~150 lines per language, zero dependencies**, in C++, Python, JavaScript, and
MicroPython. (`dbg` and `help` are rest-of-line — §2: decode those two with
`line.split(None, 1)`. They are the only exception, both robot→host, and
neither carries an id.)

For comparison, v5 needs: COBS with a parameterized delimiter, incremental
CRC-16 with a split scope, varint and zigzag codecs, a `FieldDesc`/`MessageTable`
walker over nine field kinds, and either 105 KB of generated C++ or a full
protobuf runtime.

### 11.2 Firmware notes

- **No dynamic allocation**, no `std::string`. Format with `snprintf` into a
  fixed `char[240]`; parse with `strtol`/`strtof` in place over the line
  buffer — both skip leading whitespace natively, so the space separator
  costs the parser nothing.
- **One float formatter**, `formatFixed()` (§7.2), used only by `get`.
- **Measure the formatting cost before committing.** 30 `snprintf("%ld")` per
  frame at 31 fps. Estimated ~100 µs against a 32 ms cycle — 0.3% — but that is
  arithmetic, not measurement, and it is the one number that could invalidate
  this design. Compare `cyb` (cycle busy) before and after on the stand. If it
  bites, a hand-rolled `itoa` fixes it without changing the wire.

### 11.3 Conformance

The cross-language golden-vector fixture
(`src/tests/fixtures/wire_golden_vectors.txt`) is rewritten as **ASCII line
vectors** — command in, expected line out, and the reverse — and every
implementation asserts against it byte-for-byte. Under a three-language target
this is the primary gate, and it is what makes a JavaScript implementation
verifiable at all.

**Add a fourth vector set for the REPL binding**: for each vector, the `p()`
form must produce byte-identical output to the wire form (§10.2). That is the
assertion that keeps the REPL from quietly becoming a second protocol.

The bench gate (`src/tests/bench/radio_bench_gate.py`) carries over unchanged in
intent: banner on connect, `HELLO`/`PING`/`ID`/`VER` answered, `WHEELS`
start/stop with climbing encoders, `ok` and `done` both observed, malformed
counter clear through a clean connect, and a wire-quality measurement against a
stated loss budget — run **over the relay**, not USB.

---

## 12. Deliberately not in v6

- **Any binary plane.** If a future need genuinely cannot be met in ASCII inside
  240 bytes, that is a new spec, not a second plane bolted onto this one. Two
  planes is the thing v5 got wrong.
- **CRCs and checksums** — §1.1.
- **Schema negotiation or version bytes.** Endpoints are rev-locked; `VER` tells
  a host what it is talking to and it can refuse.
- **Arc/segment moves, trajectory profiles, plan dumps, ring dumps.** The
  protocol is: **bounded velocity commands in, timestamped measurements out.**
- **A config schema on the wire** — §7.
- **`replace` / preemption flags** — §5.1.
- **A separate `POSE` query** — §5.5.
- **A REPL-specific message format.** §10 is a *binding*, not a dialect; the
  §11.3 vector set is what enforces that.

---

## 13. Migration from v5

Staged so the binary plane is *emptied* before it is deleted:

1. **`GET`/`SET`** land as two new cleartext verbs (already in v6's
   space-separated grammar); the binary config arms
   (`CONFIG`/`GET_CONFIG`/`SET_FIELD`/`CFG`) go unused, then away.
2. **`TLM` subscription + `thdr`/`t`** land alongside binary `TLM`; the binary
   telemetry arm goes unused, then away.
3. **`ok`/`err`/`done` lines** land alongside the ack ring; `wait_for_ack()`
   switches to line-reading; the ring goes away.
4. **Motion verbs + the case-and-separator flip** are the atomic cutover — C++ and Python
   together, JavaScript as a third implementation of the same fixture.

After step 3 the only binary users are the five motion verbs, so the final step
is small and its blast radius is one sprint's bench gate.

**Sequencing against the MicroPython rebuild:**
`clasi/issues/micropython-first-rebuild.md` currently fixes "v5 byte-for-byte
compatible" as a stakeholder decision. **That should be revisited against v6
before that rebuild starts.** Porting ~150 lines of ASCII is dramatically less
work than porting COBS + CRC + a protobuf subset to MicroPython; v6 is the wire
that has a JavaScript implementation; and §10's REPL binding is only reachable
from a text protocol. If the rebuild lands on v5 first, it pays the v5 port cost
and then the v6 migration cost.

---

## Appendix: superseded documents

- [`docs/protocol-v5.md`](../../docs/protocol-v5.md) — COBS(0x0A) + CRC-16
  binary framing, protobuf envelopes, 25 verbs across two planes, ack ring.
  Superseded by this document when §13 completes. Already stale against its own
  implementation in seven places (review §F5) — read `src/protos/` and
  `src/firm/core/comms.cpp` for v5 truth, not the doc.
- `docs/protocol-v4.md`, `v3`, `v2` — historical.

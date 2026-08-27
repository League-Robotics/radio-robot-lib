# radio-robot-lib — Consolidated Specification

**Purpose of this document.** This is an index and consolidation layer,
not a replacement for the four canonical design documents in this same
directory. Every normative rule lives in exactly one of them; this
document organizes what is required, states the load-bearing rules in
short form, and cites `<doc>#<section>` for the authoritative detail.
When this document and a canonical doc appear to disagree, the canonical
doc wins — that would be a defect in this document, not a design
decision. The canonical docs, in citation form used throughout:

| citation | file |
|---|---|
| `protocol` | [`protocol.md`](protocol.md) |
| `motion-api` | [`motion-api.md`](motion-api.md) |
| `diffdrive` | [`diffdrive.md`](diffdrive.md) |
| `wifi-link` | [`wifi-link.md`](wifi-link.md) |

Also covered here, without a canonical design doc of its own (specified
by source and docstring instead): the Python `robot_v6` host client
(`src/host/robot_v6/`) and the `tools/sim` host simulator.

---

## 1. System shape

Three layers, bottom to top, plus one host-side mirror of the top two:

```
robot side:  DiffDrive kernel  <-  DiffDriveAdapter  <-  ProtocolHandler  <-  bytes (radio/serial/wifi)
host side:   application code  ->  robot_v6.Session   ->  robot_v6.codec   ->  bytes (socket/pipe/serial)
```

`ProtocolHandler`/`Adapter` (`protocol#1`) is the one component required
to depend on both `DiffDrive` and the wire grammar; `DiffDrive`
(`diffdrive#1`) itself depends on neither the wire nor any chassis
geometry. `motion-api` (`motion-api#0`) is the layer a program actually
calls, sitting above the wheel kernel and expressed as six operations
that reduce to the two wheel primitives the wire already carries.

## 2. Wire grammar (protocol#2)

- One ASCII line grammar, no binary framing, no CRC, no length prefix
  (`protocol#2`, rationale in the same section for why the earlier
  binary/COBS/CRC scheme was dropped).
- `line ::= sp? verb ( sp field )* sp? '\n'`; a run of spaces is one
  separator; terminator is `'\n'`, a lone `'\r'` before it is stripped;
  a blank/all-whitespace line is ignored silently, not malformed
  (`protocol#2`).
- Fields are positional with fixed arity per verb (except `RUN`, see
  §6.3 below); wrong arity is a rejection. Max line 240 bytes including
  terminator; an overlong line is discarded to the next `\n` and counted
  malformed (`protocol#3.1`).
- Every wire value is base-10 ASCII, optionally signed, except `flags`
  (lowercase hex) and config values (decimal) (`protocol#2`).
- **Case is direction and is load-bearing**: commands (host→robot)
  UPPERCASE, replies (robot→host) lowercase, case-sensitive lookup. This
  structurally prevents a robot's own output from ever parsing as a
  command on a shared radio channel — the fix for the v5 debug-flood
  incident (`protocol#2.1`, `protocol#6.2`).
- **Sequence id**: `#<n>`, mandatory on every sequenced verb, always the
  line's last token, strictly incrementing from 1. `#0` is not special;
  `ERR_DUPLICATE_ID` does not exist — repeats/gaps are handled by the
  reliability layer instead (`protocol#2.2`).

## 3. Verb catalog and outcomes (protocol#6)

Full per-verb table (arity, sequenced/unsequenced, reply shape) is
`protocol#6`; do not duplicate it here. Load-bearing summary:

- **Sequenced** (require `#id`): `ID VER STATUS HELP GET SET TLM
  WHEELS_X WHEELS_V MOVE_X MOVE_V GO_TO_R GO_TO_W STOP RUN`.
- **Unsequenced** (never carry an id, maximally forgiving of trailing
  content): `HELLO` (resets the sequence), `ESTOP` (panic stop, always
  executes and always replies `estop`), `PING` (liveness, answers even
  while the stream is stalled on a gap) — full rationale for each
  exemption in `protocol#8.3`.
- **Outcome model** (`protocol#6.1`): `ok` no longer exists — acceptance
  is the transport-layer `ack` alone. `err <code> #<id>` reports an
  application-level rejection (see the error code table there). `ret
  <value> #<id>` is `RUN`'s own extra reply (§6.3 below). `debug <text>`
  is robot→host only, never a command (`protocol#6.2`).
- `WHEELS`/`onWheels` was renamed `WHEELS_V`/`onWheelsV`; the five other
  motion verbs (`WHEELS_X`/`MOVE_X`/`MOVE_V`/`GO_TO_R`/`GO_TO_W`) are new
  wire/handler-layer additions with no prior form before 2026-08-22
  (`protocol#6`, `motion-api#9`).

### 6.3 `RUN` — invocation by name

`RUN <function> [arg...] #id` is open-arity: the handler only parses and
delegates (name + raw argument tokens) to `Adapter::onRun`; the adapter
owns name resolution, type conversion, and — critically — **is the
security boundary**, since whatever it registers is remotely callable by
anything on the channel (`protocol#6.3`). Two hard limits: an argument
cannot contain a space, and `onRun()` must return synchronously/promptly
(`protocol#6.3`). `DiffDriveAdapter` registers nothing and answers every
`RUN` with `ERR_UNKNOWN` — this is correct behavior for an adapter with
an empty allowlist, not a stub (`protocol#6.3`, `protocol#9.10` item 1).

## 4. The reliability layer (protocol#8)

The core delivery guarantee for a wire with measured ~5% loss
(`motion-api#6`, `protocol#8.0`). Full state machine, wraparound policy,
and every resolved ambiguity are in `protocol#8`; required behavior:

- Every sequenced command carries `#<n>`; the host may pipeline freely
  without waiting for each ack. The robot acknowledges **cumulatively**
  — one ack covers every earlier id (`protocol#8.1`).
- Three-way classification of an inbound id against `expectedNext_`:
  in-order (decode, then dispatch, then `ack`), stale/retransmit
  (re-ack the last-accepted id, do NOT re-execute), and a numeric gap
  (discard, `nack` naming the next-needed id) — full table
  `protocol#8.1`.
- **A decode failure is a NAK, not an ack** (`protocol#8.9`, reversing
  the pre-2026-08-22 behavior): an unknown verb, wrong arity, or an
  unparseable field does not advance the sequence — it NACKs the same
  id, so a corrupted line in a sequence of motions (e.g. a lost turn in
  an eight-move square) is the thing that gets resent, not silently
  skipped. A **merits rejection** (decoded fine, refused by the
  adapter — e.g. an out-of-range speed) still `ack`s and advances,
  paired with `err <code> #<id>` (`protocol#8.2`, `protocol#8.9`).
- `ack <n> <lastDone> <reason>` / `nack <n> <lastDone> <reason>` piggyback
  the Adapter-owned completion channel (`lastDone()`/`lastDoneReason()`,
  polled fresh every time, no handler-side cache) — moved off the
  handler onto the Adapter in `protocol#8.8`; `DoneReason` vocabulary
  (`none/stop/timeout/estop/aborted`) resolved in `protocol#8.8.1` and
  matches `motion-api#5.1`'s completion reasons.
- **Reply-only** (2026-08-26, `protocol#8.5`): an `ack`/`nack` is emitted
  only in direct response to an inbound sequenced line — never
  periodically, never on the telemetry cadence, never as a beacon. A
  stalled stream keeps re-nacking because each subsequent command is
  itself nacked (`protocol#8.1`); a quiet host that wants confirmation
  polls with any sequenced verb (e.g. `STATUS`, which also reports
  `next=`). Still no timer or clock anywhere in the handler — a
  deliberate, load-bearing constraint (`protocol#8.1`, `protocol#8.0`).
- A decode failure the host itself keeps re-sending (a real host bug,
  not transient loss) wedges the stream forever by design; the host
  needs its own give-up path — this library cannot and does not supply
  one (`protocol#8.9`).
- Ids run 1..999999 by convention; wraparound is explicitly out of scope
  (`protocol#8.8.1`).

## 5. `Adapter` interface and `Result`/`DoneReason` (protocol#4)

One abstract class (`protocol#4`) a caller implements: session identity/
status, the six motion methods (one per `motion-api#9.1` verb, angles
already decoded to float milliradians), `onStop(immediate, id)`,
`onEstop()` (void, never sequenced), `onGet`/`onSet`/field enumeration,
`onTlm`, `lastDone()`/`lastDoneReason()`, and `onRun`. `Result` is an
enum mapped 1:1 onto the wire's error codes (`protocol#4`, code table
`protocol#6.1`); returning a `Result` rather than writing a reply is
deliberate — the adapter cannot emit a malformed reply, forget to reply,
or invent a reply shape (`protocol#4`).

## 6. `DiffDriveAdapter` — the one concrete, real adapter (protocol#5)

- Implements `WHEELS_V` with real effect: scales `[mm/s]` by
  `countsPerLength` and maps to `DifferentialDrive::drive(velocity,
  twist, lease)`, where `velocity=(left+right)/2`, `twist=(right-left)/2`
  half-differential, CCW-positive (`protocol#5`).
- The other five motion verbs answer `Result::kUnknown` on this adapter
  specifically — it has no planner. This is the deliberate, documented
  choice (not `kUnimplemented`; rationale `protocol#9.10` item 1).
- `STOP`/`ESTOP`: no queue exists in this adapter, so neither one waits
  behind an active move; `onStop` calls `neutral()` (immediate, ordinary,
  overridable), `onEstop` calls `estop()` (immediate, **latched**, blocks
  new motion until cleared) (`protocol#5.1`, `diffdrive#3.2`). A queued
  robot's own much larger stop-vs-estop contrast (measured 39.8 cm/5.9 s
  vs 2.9 cm/0.10 s) does **not** describe this adapter (`protocol#5.1`,
  `motion-api#6`).
- Telemetry is a projection of `DifferentialDrive::output()`
  (`diffdrive#4`) — per-wheel counts/velocities, not a world-frame pose;
  `flags` is a local bit layout, not an externally-numbered scheme
  (`protocol#5.2`).
- Configuration: 15 wire names map 1:1 onto `DifferentialDrive::Config`
  members; `maxDuty`/`fullDutyVelocity`/`cyclePeriod` are hard-coded
  build-time constants, not tunable fields (`protocol#7`).

## 7. Configuration — no storage in either library (protocol#7)

Stakeholder decision, 2026-08-20: neither the handler nor the kernel
stores configuration. `GET`/`SET` are pure delegation with no field
table, no bounds, in `ProtocolHandler` itself; each concrete library
carries only the config type it needs (`DifferentialDrive::Config`);
robot geometry (`countsPerLength`) lives in the adapter, not either
library (`protocol#7`, `diffdrive#6`).

## 8. DiffDrive kernel (diffdrive#1-#4)

- One class, `DiffDrive::DifferentialDrive`, owning both wheels:
  feedforward + Stage A/B/C control law, stall/deficit/wedge latches, a
  **lease watchdog on every motion command**, an estop latch
  (`diffdrive#1`). Dependency-free (`<cmath> <cstdint> <algorithm>`
  only) and speaks counts, never millimetres — geometry is entirely the
  caller's problem (`diffdrive#1.1`).
- Four small caller-implemented ports: `Motor` (13 methods, two with
  sharp semantics — `sampleTime()` stamps on collect success only,
  `rebaseline()` must issue no bus traffic), `Clock`, `Sleeper`,
  `FiberLauncher` (optional — a synchronous test harness can decline it)
  (`diffdrive#2`, `diffdrive#2.1`).
- Command surface reachable from the wire: `drive(velocity, twist,
  lease)`, `driveDuty(...)`, `neutral()`, `estop()`/`estopClear()`,
  `output()` (`diffdrive#3`). `lease` is a duration `[ms]` from now,
  clamped to `kLeaseMax`, and maps with no reinterpretation onto the
  wire's `WHEELS_V duration` field, 5000 ms ceiling enforced by the
  adapter (`diffdrive#3.1`, `protocol#9.1`).
- `output()` publishes everything a `t` telemetry frame needs: timing,
  per-wheel measurement, derived velocity/twist, learned state
  (`lambda`, bias), and health flags (`diffdrive#4`). Sample-time age
  math is deliberately signed and wrapping (`diffdrive#4.1`).
- This copy is the **authoritative** source for the control law by
  2026-08-20 stakeholder decision; prior copies elsewhere are deprecated
  (`diffdrive#5.1`).

## 9. Motion API — six operations, three modes (motion-api#1-#9)

- Two axes cross to give six operations: what you command (wheels/body/
  position) × how it's bounded (`x` displacement / `v` velocity), with
  `go_to_r`/`go_to_w` as the positional pair (`motion-api#1`). Full
  argument/unit table: `motion-api#1`.
- **All six reduce to two primitives** (`wheels_x`, `wheels_v`) via
  constant-ratio wheel segments; the algebra
  (`move_v == wheels_v(v_x∓omega·b/2)`, etc.) and the effective-track-
  width correction (`b = trackwidth / rotational_slip`) are
  `motion-api#2`, `motion-api#2.1`.
- Per-operation behavior detail — `wheels_x`/`wheels_v` (`motion-api#3.1`
  -`3.2`), `move_x`'s pivot-vs-blend thresholds and terminal-trim
  tuning (`motion-api#3.3`), `move_v` (`motion-api#3.4`), `go_to_r`'s
  arc solve and re-issue thresholds (`motion-api#3.5`), `go_to_w`'s
  pluggable pose source — OTOS when fitted, encoder odometry otherwise,
  with epoch-guarded rebaseline and unwrapped heading (`motion-api#3.6`).
- **Stopping is two verbs, not two flavors of one**: `stop()` (jerk-
  limited ramp — the default, ordinary control flow),
  `stop(immediate=True)` (zero now, accepts jerk, a legitimate choice
  when distance matters more than smoothness), `estop()` (zero now,
  **latched**, panic path only) (`motion-api#3.7`). Safety invariants
  that hold across every operation and mode — no unbounded form exists,
  a queued stop is not a stop, one estop is not proof of a stop, ~5% of
  moves are lost silently — are `motion-api#6`.
- **Three execution modes** (A background/fiber, B manual tick, C
  blocking) apply uniformly; over the wire, "tick" means drain telemetry
  already pushed and test completion — **it never means poll**, measured
  to matter (197.5 mm → 0.3 mm) (`motion-api#5`, `motion-api#5.3`).
- Wire mapping for all six verbs plus `STOP`/`ESTOP`, and the six-verbs-
  not-one-discriminated-verb design decision, are `motion-api#9`.
  Angles: degrees at the API, milliradian integers on the wire
  (`motion-api#9.1`, `protocol#6`).

## 10. WiFi link — dual-plane TCP-REPL + UDP-protocol (wifi-link#1-#11)

Specified and bench-proven (MicroPython reference, `nezha-upy`), a
porting authority for another language/host — **not yet implemented
against this repository's own protocol handler as a transport**. Key
requirements a port must satisfy:

- One ESP-AT module, one UART, two planes at the same port number 7654
  (TCP REPL mirror, UDP protocol plane); host's own UDP port fixed at
  7655 so a host restart needs no re-discovery (`wifi-link#2`).
- **`CIPMUX=1` command mode, never passthrough** — the two planes must
  coexist; the cost is one `AT+CIPSEND` exchange per outbound datagram
  (`wifi-link#3`, `wifi-link#7`).
- Bring-up state machine (`configure → join → address → server →
  ready`, `backoff` from any failure), with the poll-before-command join
  discipline (query `AT+CWJAP?` before an explicit join) called out as a
  measured landmine (`wifi-link#5`, `wifi-link#5.3`).
- Inbound demux by `+IPD` link id is "the heart of a port": link 4 is
  the protocol plane, handed one-datagram-is-one-line to this
  transport's own `ProtocolHandler` instance; peer address is learned
  from the header alone, forgotten after 60 s of silence
  (`wifi-link#6`, `wifi-link#6.1`).
- **Mandatory requirement, not optional guidance**: bound the outbound
  send queue or throttle periodic telemetry (≥ 50 ms floor) — the
  reference implementation's unbounded queue was measured to exhaust the
  heap under telemetry load (`wifi-link#7.1`).
- Security posture: the UDP plane has no authentication at this layer;
  the protocol layer's own containment (`RUN`'s registration allowlist)
  is the only gate (`wifi-link#11`).
- Full measured operational record (join time 6-170 s, first-datagram
  latency 108 ms, etc.) is `wifi-link#10`, cited before declaring any
  port "working".

## 11. Host v6 client — `robot_v6` (`src/host/robot_v6/`)

No canonical design doc; specified by source and module docstring.
Three modules, mirroring the robot-side split:

- **`codec.py`** — format/parse a protocol-v6 line. Deliberately tiny:
  no verb table, no per-verb arity knowledge, matching `protocol#2`'s
  grammar exactly (a generic caller needs neither to encode or decode a
  line). Commands/replies case convention (`protocol#2.1`) is the
  caller's responsibility, not enforced here.
- **`transport.py`** — one `Transport` abstraction (`SocketTransport`,
  `StdioTransport`/`PipeTransport`, `SerialTransport`), each implementing
  only `_read_chunk()`/`_write_bytes()`/`close()`; the base class
  supplies line-oriented `send_line()`/`read_lines()` with the same
  partial-line reassembly discipline as `ProtocolHandler::feed()`
  (`protocol#3.1`). Stakeholder requirement: the same client code must
  work unmodified against a real robot, `tools/sim` over stdio, or a
  socket-based relay server.
- **`reliability.py`** — the HOST half of `protocol#8`'s reliability
  layer: a `Session` assigns ids, pipelines sends without waiting for
  each ack, tracks the highest cumulative ack, and resends everything
  buffered from a nacked id forward, in order. Ordered execution across
  pipelined motion commands is guaranteed by the robot-side Motion
  Layer's own FIFO queue, not by the host holding commands back
  (stakeholder correction recorded in the module's own docstring); this
  module's job is delivery, not sequencing of motion effects.

## 12. Sim server — `tools/sim`

A compiled, standalone host binary (`sim_main.cpp`) composing the real
`Protocol::ProtocolHandler` with `Protocol::FakeMotionAdapter`
(`tests/protocol/fake_motion_adapter.h`) to speak real protocol-v6 with
no robot and no serial port, over stdio or a TCP listener
(`tools/sim/README.md`). `--period MS` controls the simulated `step()`/
telemetry cadence. This is what makes the host client's reliability
layer (§11) testable end to end against a real, if fake, motion queue.

## 13. Known gaps and deferred work

Called out explicitly in the canonical docs, not silently missing:

- `SEED`/`CAL` verbs remain deferred — they need OTOS/odometry this
  library does not own (`protocol#6`).
- Five of the six motion verbs have no kinematic effect on
  `DiffDriveAdapter` — wire/handler support exists, planner does not
  (`protocol#5`, `motion-api#0`).
- Sequence-id wraparound (beyond 999999) is explicitly unimplemented —
  host-side discipline to reconnect, not wire-enforced (`protocol#8.8.1`).
- The reliability layer supplies no give-up path for a host that
  genuinely, repeatedly sends a malformed line — an application-level
  backstop is required and not provided (`protocol#8.9`).
- The wifi-link design is not yet wired to this library's own
  `ProtocolHandler` as a transport (§10 above).
- Several C++-only parser hazards (hex-float syntax, embedded-NUL verb
  truncation, buffer off-by-one at the 240-byte boundary) are documented
  as characterization findings for any future porter, not defects in the
  current handler (`protocol#9.4`, `protocol#9.7`).

## 14. Planned near-term work — Rogo CLI import

Tracked in `clasi/issues/import-rogo-cli-adapt-robot-radio-to-v6-host.md`
(pending, unscheduled). Bring the `rogo` CLI entry point (relay-aware
drive/turn/goto/config/REPL/calibrate/sim/MCP-server tooling, currently
~108 Python files in `radio-robot-elite/src/host/robot_radio`) onto this
repository's `robot_v6` host client and protocol v6, rather than
vendoring the elite package wholesale — its transport/wire layers
(`robot_radio/io/wire_codec.py`, `wire_commands.py`, `client.py`,
`serial_conn.py`) are expected to be replaced by `robot_v6`'s equivalents
(§11 above), while higher layers (kinematics, nav, path planning,
calibration) may port more directly. Per-robot configuration this CLI
consumes (JSON configs, `robot_config.schema.json`, `active_robot.json`,
`devices.json`) is already staged in `config/robots/` in this repository
(see `config/MANIFEST.md`). See §15 (usecases.md UC-014 through UC-016)
for the use cases this work is expected to satisfy.

# radio-robot-lib — Use Cases

Extracted from [`protocol.md`](protocol.md), [`motion-api.md`](motion-api.md),
[`diffdrive.md`](diffdrive.md), [`wifi-link.md`](wifi-link.md), the
project README, and
`clasi/issues/import-rogo-cli-adapt-robot-radio-to-v6-host.md`. Each use
case cites the canonical section(s) it is drawn from rather than
restating their content; see [`specification.md`](specification.md) for
the consolidated cross-reference index.

**Actors**

- **Student / classroom programmer** — writes a program against the
  motion API (`robot.move_x(...)`, etc.) to drive a physical or
  simulated robot.
- **Host session** — the `robot_v6` client's reliability layer
  (`Session`, `src/host/robot_v6/reliability.py`) acting on the
  student's behalf at the wire level; called out separately from the
  student where the reliability layer's own behavior, not the student's
  intent, is what the use case is about.
- **Developer running the sim server** — runs `tools/sim` to develop or
  test host-side code with no robot attached.
- **Firmware developer** — implements or extends a concrete `Adapter`
  (`DiffDriveAdapter`, a new port, or a new transport) against the
  `ProtocolHandler` contract.
- **CLI / tooling user** — drives a robot through the planned Rogo CLI
  rather than writing a Python/JavaScript program directly.

---

## UC-001 — Command a timed wheel-velocity hold (`wheels_v`)

- **Actor:** Student / classroom programmer
- **Preconditions:** A `Session`/robot connection is established
  (`HELLO` has completed, `STATUS` shows `ready=1`); no motion is
  latched by `estop`.
- **Main flow:**
  1. Program calls `robot.wheels_v(left, right, duration)` (or the
     equivalent `WHEELS_V` wire call directly) — `motion-api#3.2`.
  2. Client encodes `WHEELS_V <left> <right> <duration> #<id>` and sends
     it — `protocol#6`, `protocol#9.1`.
  3. Robot's `ProtocolHandler` decodes the line, dispatches to
     `DiffDriveAdapter::onWheelsV`, which scales by `countsPerLength`
     and calls `DifferentialDrive::drive(velocity, twist, lease)` —
     `protocol#5`.
  4. Robot replies `ack <id> <lastDone> <reason>`.
  5. Wheels hold the commanded ratio through the ramp for `duration` ms,
     then the lease expires and the kernel returns to neutral —
     `diffdrive#3.1`.
- **Postconditions:** Wheels ran at the commanded ratio for the
  requested time; `expectedNext_` advanced past `id`.
- **Error flows:**
  - Speed out of range → `ack` (arrived, sequence advances) **plus**
    `err 3 #<id>` (`ERR_RANGE`) — a merits rejection, not a decode
    failure (`protocol#8.2`).
  - Line malformed (bad arity/unparseable field) → `nack` naming the
    same id, **no** sequence advance, plus `err <code> #<id>` —
    `protocol#8.9`.

## UC-002 — Command a body displacement and heading change (`move_x`)

- **Actor:** Student / classroom programmer
- **Preconditions:** Session established; robot at rest or already in
  motion (a `move_*` supersedes a `wheels_*` hold — `motion-api#6`).
- **Main flow:**
  1. Program calls `robot.move_x(distance, rotation, cruise=0,
     timeout)` — `motion-api#3.3`.
  2. Below the 50° pivot threshold the motion is one blended
     curvature segment; at or above it, the robot pivots to the new
     heading first, then travels straight — `motion-api#3.3`.
  3. Wire form `MOVE_X <distance> <rotation> <cruise> <timeout> #<id>`
     is sent and acked — `motion-api#9.1`.
  4. `timeout` acts as the required backstop if encoder travel never
     reaches the commanded distance — `motion-api#3.1` (shared with
     `wheels_x`).
- **Postconditions:** Heading changed by exactly `rotation`; position
  advanced along the resulting curve, or the timeout ended the move.
- **Error flows:**
  - **Current implementation gap:** on `DiffDriveAdapter` specifically
    (no planner), `MOVE_X` decodes and dispatches correctly but
    `onMoveX` returns `Result::kUnknown` — the command is acked (wire
    contract satisfied) but has no kinematic effect. This is documented,
    intended behavior for this one concrete adapter, not a bug —
    `protocol#5`, `protocol#9.10` item 1.
  - Timeout backstop fires before displacement is reached → motion ends
    with `DoneReason::kTimeout`, surfaced on the next `ack`/`nack` or
    telemetry line — `protocol#8.8.1`.

## UC-003 — Drive to a point with supervisory re-solve (`go_to_r`/`go_to_w`)

- **Actor:** Student / classroom programmer
- **Preconditions:** Session established. For `go_to_w`, a pose source
  is available (OTOS if fitted, else encoder odometry) — `motion-api#3.6`.
- **Main flow:**
  1. Program calls `robot.go_to_r(x, y, speed, arrive, timeout)`
     (robot-frame) or `go_to_w(x, y, speed, arrive, timeout)`
     (world-frame, which reads pose and transforms into `go_to_r` first)
     — `motion-api#3.5`, `motion-api#3.6`.
  2. The robot solves a constant-curvature arc tangent to current
     heading and drives it.
  3. The call is **supervisory**: the arc is re-solved as the robot
     proceeds and re-issued when the solution has materially changed
     (`|Δomega| > 0.05 rad/s`, `|Δ arc length| > 15 mm`, or half the arc
     already covered) — `motion-api#3.5`.
  4. Motion ends when the robot arrives within `arrive` tolerance, or
     `timeout` fires.
- **Postconditions:** Robot is within `arrive` of `(x, y)`; final
  heading is whatever the arc produced, not a chosen value —
  `motion-api#3.5`.
- **Error flows:**
  - Same current-adapter gap as UC-002: `GO_TO_R`/`GO_TO_W` decode and
    dispatch but answer `kUnknown` on `DiffDriveAdapter` — `protocol#5`.
  - `go_to_w` with no pose source fitted and no odometry seeded →
    application-level rejection (pose unavailable); a program needing a
    specific final heading must follow up with a pivot — `motion-api#3.5`.

## UC-004 — End a motion normally (`stop()`)

- **Actor:** Student / classroom programmer
- **Preconditions:** A motion is active (any of UC-001 through UC-003),
  or no motion is active (a `stop()` with nothing running is harmless).
- **Main flow:**
  1. Program detects its own end condition (found the line, operator
     pressed a button) and calls `robot.stop()` — `motion-api#3.7`.
  2. Wire form `STOP #<id>` is sent; `onStop(immediate=False, id)` calls
     `neutral()`, zeroing duty this cycle with no ramp on
     `DiffDriveAdapter` specifically — `protocol#5.1`.
  3. Robot replies `ack` (no `err` — `onStop` always returns `kOk` on
     this adapter) — `protocol#5.1`.
- **Postconditions:** Motion ends at the current position/heading, not
  queued behind whatever was in flight — `motion-api#6`.
- **Error flows:**
  - `stop(immediate=True)` variant (`STOP now #<id>`) — same acceptance
    path; the `immediate` flag is accepted but has no observable
    difference on `DiffDriveAdapter`, since `neutral()` was already
    immediate either way — `protocol#5.1`, `motion-api#9.1`.

## UC-005 — Panic-stop on a fault (`estop()`)

- **Actor:** Student / classroom programmer
- **Preconditions:** Program is in a motion loop (Mode B or A,
  `motion-api#5`) and detects a fault condition — a bumper press, a
  caught exception, an operator Ctrl-C.
- **Main flow:**
  1. Program catches the fault (`except BaseException` in the reference
     code) and calls `robot.estop()` before re-raising — `motion-api#7`.
  2. Wire form `ESTOP` is sent with **no id**, executes unconditionally
     regardless of trailing junk, and the latch is set **before** the
     handler's reply is written — `protocol#8.3`.
  3. Robot replies the bare word `estop`.
  4. Motion is refused until `estopClear()` is called — `diffdrive#3.2`.
- **Postconditions:** Kernel latched at zero duty; every subsequent
  motion command is refused (`kRefusedEstopped`) until explicitly
  cleared.
- **Error flows:**
  - `estop()` sent while the sequence is stalled on a numeric gap or a
    decode-failure NAK → still executes; `ESTOP` is exempt from
    sequencing entirely for exactly this reason — `protocol#8.3`.
  - **One `estop` is not proof of a stop**: the motor brick can retain
    its last commanded speed across a single latched estop (measured 5
    of 6 failures in one bench series, one producing 936 mm of
    continued travel). A caller must confirm the robot actually stopped
    (active flag clear, encoders holding) and re-issue if not —
    `motion-api#6`.

## UC-006 — Observe motion and decide to stop it (manual tick, Mode B)

- **Actor:** Student / classroom programmer
- **Preconditions:** A motion has been posted (e.g. `move_x`) without a
  callback.
- **Main flow:**
  1. Program iterates the returned object in a `for` loop; each
     iteration ticks the motion and yields a telemetry snapshot —
     `motion-api#5.1`.
  2. Program inspects a sensor field on each snapshot (e.g. `t.color.blue`,
     `t.range`) and calls `robot.stop()` (expected condition) or
     `robot.stop(immediate=True)` (must stop short) as appropriate —
     `motion-api#7`.
  3. Over the wire, "tick" only drains telemetry the robot already
     pushed and tests for completion — **it never polls** —
     `motion-api#5.3`.
- **Postconditions:** Loop exits with `m.reason` set to `stop`,
  `timeout`, `estop`, or `aborted` — `motion-api#5.1`.
- **Error flows:**
  - Program calls `tick`/iterates while a fiber (Mode A) already owns
    the loop → raises, rather than double-ticking — `motion-api#5.1`.
  - An unticked, posted move in-process does nothing (safe no-op); the
    same posted move over the wire runs regardless of whether the host
    ever looks again, bounded only by its own timeout/lease —
    `motion-api#5.2`.

## UC-007 — Recover a multi-move sequence from a lost command (NAK-and-resend)

- **Actor:** Host session (`robot_v6.reliability.Session`), on behalf of
  a student's program
- **Preconditions:** A program has queued several sequential motion
  commands (the canonical example: eight legs of a square) and the
  session is pipelining them without waiting for each ack —
  `protocol#8.1`, `reliability.py` module docstring.
- **Main flow:**
  1. Session assigns strictly increasing ids and sends commands without
     blocking.
  2. One command (e.g. a turn) is lost in transit.
  3. The robot's `nack <n> <lastDone> <reason>` names the next id it
     actually needs; the session resends everything buffered from `n`
     forward, in order.
  4. The robot's own Motion Layer executes the resent and subsequent
     commands in order from its FIFO queue — ordered execution is
     guaranteed structurally, not by the host holding commands back
     (`reliability.py` docstring, correcting an earlier design note).
- **Postconditions:** All eight motions complete exactly once, in order,
  despite the one loss — proven as an executable scenario in
  `tests/host/robot_v6/test_reliability.py`.
- **Error flows:**
  - The lost `nack` itself is lost too → self-heals, because every
    subsequent command re-triggers the same `nack` until the gap is
    filled — `protocol#8.1`.
  - Every command in the backlog is a genuine command (not a duplicate
    id) so none are silently dropped as stale.

## UC-008 — Detect and recover from a decode failure

- **Actor:** Host session
- **Preconditions:** A sequenced command is in flight.
- **Main flow:**
  1. A line is corrupted in transit such that it fails to decode
     (unknown verb, wrong arity, unparseable field) though it arrives
     with the expected next id.
  2. Robot classifies this as **decode failure, not a merits
     rejection**: it replies `nack <expectedNext_> <lastDone> <reason>`
     (naming the *same* id) plus `err <code> #<id>`, and does **not**
     advance the sequence — `protocol#8.9`.
  3. Session recognizes the nack and resends the same command (and
     anything buffered after it).
- **Postconditions:** The corrupted command is eventually delivered
  intact and the sequence advances normally.
- **Error flows:**
  - **The host itself constructs a genuinely malformed line (a real
    bug, not transient corruption)** → the robot NACKs the same id
    forever and the stream wedges permanently; this library supplies no
    give-up path. The session/application needs its own resend limit,
    timeout, or operator-visible stall detector — `protocol#8.9`,
    explicitly called out as a required application-level backstop.

## UC-009 — Establish or re-establish a session (`HELLO`)

- **Actor:** Host session
- **Preconditions:** A transport connection exists (socket, pipe, or
  serial — `transport.py`) but the protocol session is fresh or
  recovering from a suspected desync.
- **Main flow:**
  1. Host sends `HELLO` (no id, no fields).
  2. Robot resets `expectedNext_ = 1`, `gapOutstanding_ = false`, and
     emits its banner (`device NEZHA2 robot <name> <serial>`) —
     `protocol#8.3`.
  3. Robot's `lastDone()`/`lastDoneReason()` (Adapter-owned) are **not**
     reset by this — a reconnect does not erase what the robot already
     completed — `protocol#8.8`.
- **Postconditions:** Sequence numbering restarts at 1; session is ready
  for new sequenced commands.
- **Error flows:**
  - `HELLO` sent with extra fields (wrong arity) → no reply of any
    kind, `malformedCount()` increments; there is no `ack` to anchor an
    `err` against for an unsequenced verb — `protocol#9.10` item 7.
  - A host suspecting desync can instead read `STATUS`'s `next=` field
    to resync tracking without a full reset — `protocol#8.7`.

## UC-010 — Confirm liveness while the stream is stalled (`PING`)

- **Actor:** Host session / developer diagnosing a stuck link
- **Preconditions:** The sequence is stalled on a numeric gap or a
  decode-failure NAK (UC-007/UC-008 in progress).
- **Main flow:**
  1. Host sends `PING` (no id required, though a trailing `#<id>` from
     an old-style caller is tolerated) — `protocol#8.3`, `protocol#9.10`
     item 2.
  2. Robot replies `pong <now>` regardless of the stalled sequence —
     liveness must not be gated behind a missing id.
- **Postconditions:** Host confirms the link and robot are alive even
  though ordinary sequenced commands are not currently progressing.
- **Error flows:** None — `PING` is maximally forgiving of trailing
  content, matching `ESTOP`'s posture, specifically so it cannot itself
  be refused over a syntax nit — `protocol#8.3`.

## UC-011 — Develop and test host code with no hardware attached

- **Actor:** Developer running the sim server
- **Preconditions:** `tools/sim` is built (`tools/sim/README.md` build
  command); no robot or serial port available.
- **Main flow:**
  1. Developer launches `/tmp/robot_sim --stdio` (or `--listen
     127.0.0.1:7654` for TCP) with an optional `--period MS` to control
     the simulated telemetry/step cadence.
  2. `sim_main.cpp` composes the real `ProtocolHandler` with
     `Protocol::FakeMotionAdapter` — a fixed-capacity FIFO motion queue,
     not a stub — so pipelined motion commands behave as they would
     against a real planner-bearing robot.
  3. Developer's host code (or a test) connects via `robot_v6`'s
     `StdioTransport`/`SocketTransport` and drives the exact same
     protocol traffic UC-001 through UC-010 describe.
- **Postconditions:** Host-side behavior (codec, transport, reliability
  session) is validated end to end against a real wire grammar
  implementation with no physical robot.
- **Error flows:**
  - Ctrl-C, SIGTERM, or EOF on stdin all shut the simulator down
    cleanly — `tools/sim/README.md`.

## UC-012 — Implement and validate a new protocol adapter or port

- **Actor:** Firmware developer
- **Preconditions:** A target platform (e.g. MicroPython, JavaScript) or
  a new concrete robot needs a `ProtocolHandler`/`Adapter`
  implementation; `src/protocol/` is treated as the reference
  **archetype** to read and port, not a library to link against
  directly on that platform — `protocol#9.4`.
- **Main flow:**
  1. Developer implements the `Adapter` interface (`protocol#4`) for the
     target — session identity, the six motion methods, `onStop`/
     `onEstop`, `onGet`/`onSet`, `onTlm`, `lastDone`/`lastDoneReason`,
     and (if the target supports remote invocation) `onRun` with an
     explicit registration allowlist — `protocol#6.3`.
  2. Developer validates against `golden_vectors.txt` and the
     adversarial fixture (`tests/protocol/test_protocol_adversarial.py`)
     to confirm wire-level conformance — `protocol#9.4`.
  3. **Language-specific hazards to check deliberately, not inherit**:
     leading-whitespace/underscore numeric leniency differs by host
     language parser (`protocol#9.4`); a dynamic language's natural
     `onRun` implementation (`getattr`/`globals()`) is *more* permissive
     than the C++ archetype's registration table and must have an
     allowlist added deliberately (`protocol#9.7`); `onRun()` must still
     return promptly even in an event-loop language (`protocol#9.7`).
  4. If the target adapter is also a new **transport** (e.g. a WiFi
     dual-plane link rather than plain serial), the developer follows
     [`wifi-link.md`](wifi-link.md) as the porting authority: `CIPMUX=1`
     command mode, the demux-by-link-id design, mandatory send-queue
     bounding/telemetry throttling, and the bring-up state machine —
     `wifi-link#3`, `wifi-link#6`, `wifi-link#7.1`, `wifi-link#5`. This
     is itself unimplemented in this repository as of this writing and
     is available for a firmware developer to pick up.
- **Postconditions:** A new adapter or transport speaks conformant
  protocol-v6 and passes the golden-vector and adversarial suites.
- **Error flows:**
  - A straight logic port reproduces a C++-only bug class (e.g. treats
    an embedded NUL as verb-terminating) that a length-aware host
    language would not naturally exhibit — flagged as a pinned
    characterization test, not something to silently "fix away" from
    the reference behavior without recording the divergence —
    `protocol#9.4`.

## UC-013 — Extend an adapter's configuration surface (`GET`/`SET`)

- **Actor:** Firmware developer
- **Preconditions:** A concrete adapter (e.g. `DiffDriveAdapter`) exists;
  neither `ProtocolHandler` nor the kernel stores any configuration
  itself — `protocol#7`.
- **Main flow:**
  1. Developer adds a named field to the adapter's own config type
     (e.g. a new `DifferentialDrive::Config` member) and maps a wire
     name to it in the adapter's field table.
  2. Host sends `SET name value #<id>`; the handler delegates to
     `onSet`, which is acked with no separate `ok` reply — the ack
     itself is the acceptance signal — `protocol#8.2`.
  3. Host sends `GET name #<id>` (or bare `GET #<id>` for every field)
     to read it back — `protocol#6`.
- **Postconditions:** The new field is readable/writable over the wire
  with no changes needed to `ProtocolHandler` itself.
- **Error flows:**
  - Unknown `SET`/`GET` name → `err 1 #<id>` (`ERR_UNKNOWN`), layered on
    top of the `ack` every in-order command still gets — `protocol#7`.
  - A value that fails to parse at all is a decode failure (NAK), not a
    rejection — distinct from an unknown name — `protocol#6`.

## UC-014 — Drive a robot interactively via CLI (planned — Rogo import)

- **Actor:** CLI / tooling user
- **Preconditions:** The Rogo CLI has been imported and adapted onto
  `robot_v6`/protocol v6 per
  `clasi/issues/import-rogo-cli-adapt-robot-radio-to-v6-host.md`
  (not yet implemented as of this writing).
- **Main flow:**
  1. User invokes `rogo drive <args>` / `rogo turn <args>` / `rogo goto
     <args>` (or the interactive REPL) against a robot reachable through
     `robot_v6`'s `Transport` — a real robot, a relay server, or
     `tools/sim` (UC-011), all through the same client code (the
     stakeholder's own "either of those, the same way" requirement,
     `transport.py` docstring).
  2. CLI translates the command into the corresponding `robot_v6`
     motion-API call and reports the outcome/telemetry back to the
     terminal.
- **Postconditions:** Robot executes the requested motion; CLI reflects
  the reliability-layer outcome (ack/nack, completion reason) to the
  user.
- **Error flows:**
  - Relay/robot unreachable → CLI surfaces a transport-level error
    rather than hanging (per `TransportClosed` vs. an ordinary read
    timeout distinction already in `transport.py`).
  - A requested motion (e.g. `goto`) targets a verb with no kinematic
    effect on the connected adapter (UC-002/UC-003's current gap) → CLI
    surfaces the resulting `err`/`kUnknown` outcome rather than
    reporting silent success.

## UC-015 — Calibrate a robot via CLI (planned — Rogo import)

- **Actor:** CLI / tooling user
- **Preconditions:** Rogo's `calibration` subpackage has been ported;
  per-robot config the calibration flow reads/writes already exists in
  `config/robots/` (`config/MANIFEST.md`, per the issue's notes).
- **Main flow:**
  1. User invokes `rogo calibrate <robot>`.
  2. CLI drives a calibration routine (e.g. measuring effective track
     width / `rotational_slip`, `motion-api#2.1`) against the robot over
     `robot_v6`, and writes results back to that robot's JSON config.
- **Postconditions:** Robot's config file reflects newly measured
  calibration values (e.g. effective track width) for use by future
  motion commands.
- **Error flows:**
  - Measured value falls outside a sane range → CLI should reject the
    write and report rather than silently persisting a bad calibration
    (mirrors the specification's own caution against bending
    `trackwidth` to make turns land, `motion-api#2.1`).

## UC-016 — Expose robot control via an MCP server (planned — Rogo import)

- **Actor:** CLI / tooling user (or an external tool/agent acting as the
  MCP client)
- **Preconditions:** Rogo's `robot_mcp.py` has been ported and adapted
  to `robot_v6`.
- **Main flow:**
  1. User invokes `rogo mcp` (or equivalent) to start an MCP server
     process that exposes robot motion/config/telemetry operations as
     MCP tools.
  2. An external MCP client (an agent or another tool) calls those
     tools; the server translates each call into `robot_v6` traffic
     against the connected robot/relay/sim, the same way UC-014's direct
     CLI commands do.
- **Postconditions:** External tooling can drive and observe a robot
  without embedding `robot_v6` directly.
- **Error flows:**
  - Same transport/adapter-gap error flows as UC-014, surfaced through
    the MCP tool-call error channel instead of terminal output.

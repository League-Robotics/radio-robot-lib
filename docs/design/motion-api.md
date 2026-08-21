# Motion API — the six operations, and three ways to run them

**What this is.** The functions a person calls when they program motion on a
robot, and the ways those calls can be executed. It is the source the user
documentation will be written from.

**What this is not.** The protocol. [`protocol-v6-spec.md`](../protocol-v6-spec.md)
stays the wire authority; §9 here proposes a change to it and nothing more.

**Where it sits.** Above the wheel kernel, which is unchanged:

```
your program
    motion API          <- this document: six operations, three modes
    segments + profile   constant-ratio segments, stop conditions, the pivot rule
    wheel kernel         DifferentialDrive::drive(velocity, twist, lease)
    motors
```

The motion layer was deleted from `radio-robot-elite` firmware on 2026-08-16
(commit `88dd9ad8`), so `MOVE` and `GO_TO` currently answer
`ERR_UNIMPLEMENTED`. It is going to be rebuilt. This specifies what to build
rather than what was there — but every measured number below is recovered from
what was there, and is cited.

---

## 1. The six operations

Two axes: **what you command** — wheels, body, position — crossed with **how it
is bounded**, `x` for a displacement and `v` for a velocity.

`go_to` is a deliberate asymmetry. It is inherently positional, so there is no
`x`/`v` choice to make; its second letter names a **frame** instead — `r` for
robot-relative, `w` for world.

| method | wire verb | arguments | bounded by |
|---|---|---|---|
| `wheels_x` | `WHEELS_X` | `left` `right` `cruise` `timeout` | per-wheel encoder distance |
| `wheels_v` | `WHEELS_V` | `left` `right` `duration` | time — `duration` **is** the lease |
| `move_x` | `MOVE_X` | `distance` `rotation` `cruise` `timeout` | body displacement and heading |
| `move_v` | `MOVE_V` | `v_x` `omega` `duration` | time |
| `go_to_r` | `GO_TO_R` | `x` `y` `speed` `arrive` `timeout` | arrival within tolerance |
| `go_to_w` | `GO_TO_W` | `x` `y` `speed` `arrive` `timeout` | arrival within tolerance |

Units are `[mm]` for `left`/`right`/`distance`/`x`/`y`/`arrive`, `[mm/s]` for
`cruise`/`speed`/`v_x`, `[deg]` for `rotation`, `[deg/s]` for `omega`, and
`[ms]` for `timeout`/`duration`. They never appear in a name — see
`.claude/rules/naming-and-style.md`; a declaration carries `// [mm/s]` as the
first token of its trailing comment.

**One name per operation, everywhere.** The wire verb is the method name in
upper case. `move_x` in Python, `moveX` in JavaScript and C++, `MOVE_X` on the
wire — so a person reading a wire log and a person reading a program are reading
the same vocabulary.

### 1.1 Where the cruise speed lives, and why

- **A V-form's commanded velocity *is* its cruise.** There is no separate cruise
  argument, because the number you passed is the ceiling. It is still reached
  through the velocity profile — commanding `200` does not step to 200 mm/s, it
  ramps there — so "cruise" and "maximum" are the same thing throughout.
- **An X-form's commanded value is a displacement**, which says nothing about
  how fast to cover it, so `cruise` is its own argument. Pass `0` for the
  configured default.

`move_v` takes no cruise for `omega` for the same reason it takes no separate
profile: **omega is slaved to `v_x`.** The two are one ratio, and holding that
ratio through the ramp is exactly what keeps the commanded curve from changing
shape while the robot is speeding up. A separately-profiled yaw rate would bend
the path during acceleration.

---

## 2. Everything is constant-ratio wheel segments

This is the whole design in one section. The six operations are not six
mechanisms; they are four translations onto two primitives.

```
move_v(v_x, omega)     ==  wheels_v(v_x − omega·b/2,  v_x + omega·b/2)
move_x(distance, rot)  ==  wheels_x(distance − rot·b/2, distance + rot·b/2)
go_to_r(x, y)          ==  move_x(arcLength, 2·atan2(y, x))
go_to_w(x, y)          ==  read pose → world-to-body → go_to_r
```

`rot` is in radians for the arithmetic, and `b` is the **effective** track
width — not the measured one. The body forms *are* the wheel forms composed
with differential kinematics, which is why `wheels_x` and `wheels_v` are the
only primitives; everything else is a change of coordinates on top of them.

### 2.1 `b` is the effective track width

Ideal differential kinematics say `omega = (vR − vL) / b`, but a skid-steer
robot drags its wheels sideways through a turn and rotates **less** than that
for a given wheel differential. `rotational_slip` is the measured ratio of
actual rotation to ideal, so every kinematic use of the track wants:

```
b = trackwidth / rotational_slip          (rotational_slip == 0 → no correction)
```

The robot config stores only those two raw numbers, and the effective value is
derived at boot — `Config::Robot::effectiveTrackWidth()`, deliberately a
*method* rather than a stored field, so configuration read-back never reports a
derived number as though it had been measured. That one value is then handed to
the drive, the odometry and the planner limits, so **every `b` in this document
is the effective one**, including §3.3's pivot rate and §3.6's odometry.

**Never bend `trackwidth` to make turns land.** It is the one independently
verifiable number in the robot config — a caliper reaches it. Scrub belongs in
`rotational_slip`, which is separately measurable against camera truth, and
keeping them apart is what lets a bad turn be diagnosed instead of merely
compensated. On `tovez`, 13 camera-truth turns across ±90/±180/±360° at three
rates put the effective track at 136.59 ± 0.58 mm against a caliper-measured
128 mm.

Useful things that fall straight out:

- `wheels_x(+d, −d)` is a pivot in place; `wheels_x(d, d)` is a straight line.
- `move_x(d, 0)` is a straight line; `move_x(0, θ)` is a pivot. The two most
  common motions in any program are special cases of one call, not two verbs.
- Sign convention is the project's, unchanged: **CCW-positive**, so a positive
  `omega` or `rotation` turns left and increases camera yaw. The left wheel is
  the slower one. Do not re-derive this from a cable order — the project has
  shipped that bug and patched it four times downstream
  (`.claude/rules/playfield-testing.md`).

One qualification. A `move_x` whose rotation is large is **not** one segment —
it is a pivot segment followed by a straight one (§3.3). So the general
statement, and the sentence an implementer should hold onto:

> **Every motion is one or more constant-ratio segments, each bounded by a
> displacement or by a time.**

---

## 3. The operations in detail

### 3.1 `wheels_x(left, right, cruise, timeout)`

Move each wheel a commanded distance. Both wheels finish together: the ratio
`left:right` is what defines the path, so the faster wheel is not allowed to
arrive early and wait.

`cruise` is the **dominant** wheel's **maximum** speed — the dominant wheel
being the one with the larger magnitude. The other wheel's speed follows from
the ratio, which is what makes `cruise` a single number for a two-wheel command.

It is a ceiling, not a speed that is held: the profile ramps up to it and
decelerates out of it, and a short move may never reach it at all.

`timeout` is a required backstop, not the stop condition. If the robot is
blocked and the encoders never reach the commanded distance, the timeout is what
ends the move.

### 3.2 `wheels_v(left, right, duration)`

Command each wheel a maximum velocity, held for `duration`. The ratio is
maintained through the ramp, so the commanded curvature is the same at 20 mm/s
and at 200 mm/s.

`duration` is **required** and it *is* the kernel's lease — the same field, the
same meaning, no reinterpretation
([`diffdrive.md`](diffdrive.md) §3.1). A dead caller cannot mean a runaway,
because the wheels stop when the lease expires whether anyone is still talking
or not.

This is the one operation that already exists end to end today, as `WHEELS`
mapping onto `drive(velocity, twist, lease)`.

### 3.3 `move_x(distance, rotation, cruise, timeout)`

Travel `distance` along the path while the heading changes by `rotation`. At the
end the heading has changed by exactly `rotation`, and the position has moved
along whatever curve carried it there.

**How the two combine depends on how big the rotation is**, and the thresholds
are measured, not chosen. Recovered from `navigator.cpp:237-240` at `88dd9ad8^`:

| condition | behaviour |
|---|---|
| \|rotation\| `>=` **50°** (`turn_first_angle`, 0.8726646 rad) | stop, pivot to the new heading, then travel |
| \|rotation\| `<` 50° | one blended segment — steer it out with curvature alone |
| \|bearing\| `>` **90°** (`behind_angle`, π/2) | no finite-radius tangent arc reaches the target; pivot |

Two rules that are not obvious and were learned the hard way:

- **Never replace an in-flight arc with a pivot at speed.** Doing so ratio-locks
  a hard brake onto the reversing wheel. Ramp to rest through an ordinary
  planned stop first, then pivot from rest.
- **The pivot rate is derived, not configured**: `2 · speed / b`, with `b` the
  effective track width (§2.1). There is no pivot-speed knob, and adding one
  would let a program request a pivot the wheels cannot deliver.

**Fine alignment is not a tuning knob.** The terminal trim (`align_tol` 1.0°,
`align_max_nudges` 6) exists to land the last fraction of a degree. 333 measured
nudges show the corrective pivot is bimodal with a roughly 1.8° quantum;
tightening the tolerance to 0.3° drops convergence from 93% to 64% and makes
some corners worse. Leave it alone
(`radio-robot-elite/docs/bench-reports/motion-planning-lab-2026-08-04.md` §5.2).

### 3.4 `move_v(v_x, omega, duration)`

Command a body twist — forward velocity and yaw rate — held for `duration`.
This is teleop's natural verb, and the one a joystick maps onto.

`duration` is required and is the lease, exactly as in `wheels_v`.

A holonomic base would take `v_y` as well. It is not in the signature because
adding an argument later is cheap and carrying a permanently-zero one is not;
the wire's own `v_y` field is accepted and ignored on a differential build, and
this API declines to expose a parameter that does nothing.

### 3.5 `go_to_r(x, y, speed, arrive, timeout)`

Drive to a point expressed in the robot's own frame — `x` forward, `y` left —
along the constant-curvature arc that leaves tangent to the current heading.

**The final heading is a consequence, not an argument.** It is:

```
turn angle   φ = 2 · atan2(y, x)
chord        c = hypot(x, y)
arc length   s = c · φ / (2 · sin(φ/2))        → c as φ → 0
```

If `y` is not zero the robot curves, and where it ends up pointing is whatever
that curve produced. A program that needs a specific arrival heading must use
`move_x`, or follow the `go_to_r` with a pivot.

`go_to_r` is **supervisory**: it re-solves the arc as the robot proceeds rather
than committing once. Re-issue when the solution has materially changed —
\|Δomega\| > 0.05 rad/s, \|Δ arc length\| > 15 mm, or half the commanded arc
already covered. Re-issuing on every cycle floods the planner; never re-issuing
means driving a stale solve.

`arrive` is the arrival tolerance; `0` takes the configured default, which is
10 mm on `tovez` (moved there from 100 mm after camera-truth measurement cut
mean arrival error from 76 mm to 12.7 mm).

### 3.6 `go_to_w(x, y, speed, arrive, timeout)`

The same, in world coordinates. It is `go_to_r` plus a pose read: transform the
world-frame delta into the body frame using the current pose, then delegate.

**The pose source is pluggable, because the fleet is not uniform.** `gopiv` has
no OTOS fitted at all — telemetry `flags` 216, bits 0/13/14 clear — so an API
that assumes one is an API that does not run on the whole fleet.

- **OTOS when fitted.** Read position and heading; the lever arm is applied in
  firmware.
- **Encoder odometry otherwise.** Integrate the wheels. The update is midpoint-arc
  (recovered from `odometry.cpp:17-51`):

  ```
  Δleft, Δright → (distance, headingDelta)     forward kinematics, effective b
  midTheta  = theta + headingDelta / 2
  x        += distance · cos(midTheta)
  y        += distance · sin(midTheta)
  theta    += headingDelta
  ```

  Two details are load-bearing and easy to lose in a rewrite. **A changed
  `positionEpoch` credits that wheel exactly zero delta for that pass** —
  encoders are never device-reset, they are re-anchored in software, and
  differencing across the ~30,000 mm rebaseline jump would corrupt the pose.
  Left and right rebaseline independently. And **heading is unwrapped**, never
  reduced to (−π, π]; wrapping it here is how a position seed acquires ~91 mm of
  error (`.clasi/knowledge/seed-heading-must-be-wrapped-*`).

Pose drifts. A camera fix or an external seed is what corrects it, and seeding
writes **both** sources so their later divergence is the drift being measured.

---

## 4. The control block

Every operation, however high-level, reduces to one structure. It is what gets
posted, and what the loop consumes:

```
uLeft, uRight   the ratio, normalized so max(|uLeft|, |uRight|) == 1
cruise          [mm/s] the dominant wheel's maximum speed — a ceiling, not a hold
stop            Displacement | Time
limit           in stop's own unit
deadline        [ms] timeout backstop, or the lease
id              correlation id, unique for the session
```

This is not new. It is the recovered `Motion::Move` plus `MoveShape`, where
`shapeOf()` normalizes by the dominant wheel and `cruise` is that wheel's own
maximum speed.

**The profiler plans one scalar.** Each wheel commands `λ · u_w`, so the
commanded left:right ratio — and therefore the heading the segment sweeps —
cannot drift no matter what the velocity profile does. That is the mechanical
guarantee behind §2's claim, and it is why the ratio is normalized into the
command rather than recomputed downstream.

Ratio preservation already has four implementations in the live kernel
(`src/firm/diffdrive/differential_drive.cpp`), and this layer builds on them
rather than adding a fifth: λ authority-headroom scaling, the twist-integral
hold (`twistHoldGain` — the encoder-only ratio keeper), the ratio-preserving
speed floor, and the Stage A/B/C per-wheel pipeline.

`Arrival` is deliberately not a stop kind. `go_to_*` supervises — it watches the
pose and issues fresh segments — so arrival is a decision made above the control
block, not a condition inside it.

---

## 5. The three modes

Every mode is the same two operations, **post** and **tick**. They differ only
in *who calls tick*.

| mode | who ticks | what you owe |
|---|---|---|
| **A — background** | the fiber or timer | yield in your own loop; call `stop`/`estop` yourself |
| **B — manual** | you | tick (or iterate) every pass |
| **C — blocking** | the library | nothing; pass a callback to observe or abort |

**Posting always happens when you call the method.** What varies is who advances
the loop afterwards.

### 5.1 The object model

- **No callback → you get an object you drive.** Iterating it ticks it and
  yields telemetry.
- **A callback → the library drives the loop**, calling you back each tick.
  Calling `stop` or `estop` from the callback ends the move.
- **A fiber is running → the fiber drives it**, and the object you get back is
  for observing.

The object reports `done` and a `reason`. The reasons are the wire's own
(`done #<id> <reason>` — [`protocol-v6-spec.md`](../protocol-v6-spec.md) §8.1),
plus two that only exist locally:

| reason | meaning |
|---|---|
| `stop` | the stop condition was met |
| `timeout` | the backstop fired |
| `estop` | a panic stop ended it |
| `aborted` | the caller abandoned it — callback said so, or the generator was closed |

**The loop has exactly one owner.** Calling `tick` while a fiber owns it raises
rather than double-ticking. Iteration works in both: with a fiber it waits for
the next frame instead of advancing the loop.

### 5.2 What an unticked move does

In-process, a posted move that nobody ticks **does nothing** — the loop never
advances, so the wheels never turn. That is a programming error, and a safe one.

Over the wire it is the opposite: a posted move runs on the robot whether or not
the host ever looks again. `nowait` is genuinely fire-and-forget, and the
`timeout` is the only thing that ends it. This asymmetry is the reason every
form carries a deadline.

### 5.3 Over the wire

The same three modes, with one hard rule:

> **Over the wire, tick means "drain the telemetry the robot already pushed and
> test for completion." It never means send a poll.**

This is not a style preference. Polling during a move over the relay was
measured cutting travel from **197.5 mm to 0.3 mm** — a request/reply round-trip
inside a move is actively dangerous, and it looks exactly like a dead robot.
Draining is passive; those frames were sent already.

Mode C is the default over the wire — it is what `RogoClient.cmd("drive 200")`
already does — and mode A is already spelled `nowait` in the repl grammar.

---

## 6. Safety invariants

These hold for every operation and every mode. They are stated once here and
assumed everywhere else.

- **No unbounded form exists.** A V-form is bounded by its `duration`, which is
  the lease. An X-form is bounded by its displacement **plus** a required
  `timeout` backstop. There is no call that means "go until I say stop."
- **`stop` is planned; `estop` is the panic stop.** Measured on a 400 mm leg
  with the halt sent 0.5 s in: `stop` travelled the full 39.8 cm and took 5.9 s
  to go inactive, `estop` travelled 2.9 cm and cleared in 0.10 s. Every halt
  path — geofence, Ctrl-C, callback abort, `finally` — calls `estop`.
- **One `estop` is not proof of a stop.** The motor brick latches its last
  commanded speed and does not reset when the microcontroller does. A single
  `estop` failed 5 of 6 attempts in measurement, and one issued by a
  then-silent host produced **936 mm of continued travel with no decay**.
  Confirm the robot actually stopped — active flag clear, encoders holding —
  and re-issue if it did not.
- **Roughly 5% of moves are lost silently over the radio.** The enqueue
  acknowledgement is generated locally, so it proves the host spoke, not that
  the robot heard. Confirm motion actually started before believing it.
- **Ids are unique for the session.** A reused id is `err #<id> 11`
  (`ERR_DUPLICATE_ID`); under the older protocol it was acknowledged and then
  silently dropped, which is the footgun that rule exists to close.
- **Exactly one subsystem owns motion at a time.** A `move_*` supersedes a
  `wheels_*` hold; a `wheels_*` clears the planner.

---

## 7. Code — Python

```python
from robot import robot, sleep

# ---- Mode C — the library drives the loop --------------------------------
robot.move_x(400, 0).wait()                  # travel 400 mm straight, block
robot.move_x(0, 90).wait()                   # pivot 90 deg CCW in place
robot.go_to_w(-150, 400, arrive=10).wait()   # drive to a world point

def watch(t):                                # observer; may end the move
    if t.line[0] < 200:                      # crossed a line
        robot.estop()

robot.move_x(400, 0, on_tick=watch)          # a callback blocks, and observes


# ---- Mode B — you drive the loop -----------------------------------------
try:
    for t in robot.move_x(400, 0):           # iterating is what ticks it
        if t.color.blue > 300:
            robot.estop()
            break
finally:
    robot.estop()                            # every driving loop owes this


# ---- Mode A — a fiber drives the loop ------------------------------------
robot.start()                                # launch the tick fiber, once

m = robot.move_x(400, 0)                     # posts; the fiber runs it
try:
    while not m.done:
        if bumper.pressed():
            robot.estop()
            break
        sleep(0.02)                          # you MUST yield, or you starve it
finally:
    robot.estop()

print(m.reason)                              # stop | timeout | estop | aborted
```

Composing the primitives directly, when the body forms are not what you want:

```python
robot.wheels_x(-120, 120, cruise=100).wait()   # pivot, expressed at the wheels
robot.wheels_v(150, 150, 800).wait()           # both wheels, 150 mm/s, 800 ms
```

## 8. Code — JavaScript

The returned object is both awaitable and async-iterable, so the same three
modes fall out of the language rather than out of three different methods.

```js
// ---- Mode C — the library drives the loop -------------------------------
await robot.moveX({distance: 400, rotation: 0});
await robot.goToW({x: -150, y: 400, arrive: 10});

await robot.moveX({distance: 400, rotation: 0}, (t) => {
  if (t.line[0] < 200) robot.estop();          // a callback blocks, and observes
});


// ---- Mode B — you drive the loop ----------------------------------------
try {
  for await (const t of robot.moveX({distance: 400, rotation: 0})) {
    if (t.color.blue > 300) { robot.estop(); break; }
  }
} finally {
  robot.estop();
}


// ---- Mode A — a timer drives the loop -----------------------------------
robot.start();

const m = robot.moveX({distance: 400, rotation: 0});
try {
  while (!m.done) {
    if (bumper.pressed()) { robot.estop(); break; }
    await robot.sleep(20);                     // yield
  }
} finally {
  robot.estop();
}

console.log(m.reason);                         // stop | timeout | estop | aborted
```

---

## 9. Harmonizing with the wire

### 9.1 The mapping

| method | wire |
|---|---|
| `wheels_x(left, right, cruise, timeout)` | `WHEELS_X <left> <right> <cruise> <timeout> #<id>` |
| `wheels_v(left, right, duration)` | `WHEELS_V <left> <right> <duration> #<id>` |
| `move_x(distance, rotation, cruise, timeout)` | `MOVE_X <distance> <rotation> <cruise> <timeout> #<id>` |
| `move_v(v_x, omega, duration)` | `MOVE_V <v_x> <omega> <duration> #<id>` |
| `go_to_r(x, y, speed, arrive, timeout)` | `GO_TO_R <x> <y> <speed> <arrive> <timeout> #<id>` |
| `go_to_w(x, y, speed, arrive, timeout)` | `GO_TO_W <x> <y> <speed> <arrive> <timeout> #<id>` |

Angles are **degrees at the API and milliradian integers on the wire**. The API
is what a person types; the wire is what a parser reads, and it carries base-10
integers only. The conversion lives in the binding, in one place.

`STOP #<id>` and `ESTOP` are unchanged.

### 9.2 The verb split, proposed

Today's wire has three motion verbs plus four discriminator fields — `kind`,
`stop`, `limit` and `frame`. Under this API those four disappear into the verb
name, and six verbs replace three:

```
                      before                                    after
MOVE <kind> <a> <b> <c> <stop> <limit> <timeout> #<id>    MOVE_X / MOVE_V
WHEELS <left> <right> <duration> [#<id>]                  WHEELS_X / WHEELS_V
GOTO <x> <y> <frame> <speed> <arrive> <timeout> #<id>     GO_TO_R / GO_TO_W
```

The argument is the one this protocol has already made twice — for case carrying
direction, and for the self-marking `#id`: **put the discriminator in the verb,
where it cannot be mismatched.** `MOVE w 100 -100 0 a 1571 4000 #8` is spellable
today and means something odd — a wheels-kind command with an angular stop
condition. After the split, every invalid combination is unspellable rather than
merely discouraged.

It also closes two gaps that are not stylistic. `wheels_x` **has no wire form at
all** today. And `move_x` cannot be expressed: `MOVE` carries one stop condition,
so a command that is a distance *and* a rotation has nowhere to go.

The adapter seam moves with it — `Protocol::Adapter`'s single `onWheels` becomes
six methods, one per verb, each receiving decoded typed arguments and returning
a `Result` exactly as today ([`protocol.md`](protocol.md) §3).

### 9.3 Decisions worth naming

Stated here so they are easy to reverse rather than buried in prose.

1. **Degrees at the API, milliradians on the wire** (§9.1).
2. **Underscored verbs** (`MOVE_X`, `GO_TO_R`) so the wire verb is exactly the
   method name in upper case. Flipping to `MOVEX`/`GOTOR` costs one table.
3. **`go_to_w`'s pose source is pluggable** rather than assuming an OTOS is
   fitted (§3.6).
4. **No `v_y` argument** on `move_v` until a holonomic base exists (§3.4).

---

## 10. Sources

Every measured number above comes from one of these.

| claim | source |
|---|---|
| 50° pivot-first, 90° behind-guard, `2·speed/trackWidth` | `navigator.cpp:237-240`, `arc_solver.h:159/172` at `88dd9ad8^` |
| 1.8° align quantum, 93%→64% at 0.3° | `docs/bench-reports/motion-planning-lab-2026-08-04.md` §5.2 |
| midpoint-arc odometry, epoch guard, unwrapped heading | `odometry.cpp:17-51` at `88dd9ad8^` |
| ratio lock, one-scalar profile, `shapeOf()` | `planner/shape.h`, `planner/profile.h` at `88dd9ad8^` |
| four live ratio-preservation mechanisms | `src/firm/diffdrive/differential_drive.cpp` |
| 39.8 cm / 5.9 s versus 2.9 cm / 0.10 s | `.claude/rules/playfield-testing.md` |
| `estop` failing 5 of 6; 936 mm of travel | measured on `vevov`, 2026-08-03 |
| polling a move: 197.5 mm → 0.3 mm | measured over the relay, 2026-08-19 |
| ~5% of moves lost, ack proves nothing | `src/tests/bench/radio_move_reliability.py` |
| arrival tolerance 100 mm → 10 mm, error 76 mm → 12.7 mm | `tovez.json`, camera-truth measurement |

Paths without a repository are in `radio-robot-elite`.

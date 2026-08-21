---
status: pending
---

# Write the motion API design document

## Description

`docs/design/motion-api.md` does not exist. It should specify the motion
functions a person calls when programming a robot — six operations across two
axes — and the three ways those calls can be executed, in a form that harmonizes
with protocol v6 without being the protocol.

The six operations, two axes (what you command × how it is bounded), with
`go_to`'s second letter being a *frame* rather than a bound:

| method | wire verb | arguments | bounded by |
|---|---|---|---|
| `wheels_x` | `WHEELS_X` | `left right cruise timeout` | per-wheel encoder distance |
| `wheels_v` | `WHEELS_V` | `left right duration` | time (duration **is** the lease) |
| `move_x` | `MOVE_X` | `distance rotation cruise timeout` | body displacement + heading |
| `move_v` | `MOVE_V` | `v_x omega duration` | time |
| `go_to_r` | `GO_TO_R` | `x y speed arrive timeout` | arrival within tolerance |
| `go_to_w` | `GO_TO_W` | `x y speed arrive timeout` | arrival within tolerance |

## Cause

Two concrete gaps make the document necessary now.

**The motion layer no longer exists to document.** It was deleted from firmware
`master` on 2026-08-16 (commit `88dd9ad8` — `planner/`, `navigator/`,
`arc_solver`, `odometry`); `MOVE` and `GO_TO` answer `ERR_UNIMPLEMENTED`. What
survives is the wheel kernel,
`DiffDrive::DifferentialDrive::drive(velocity, twist, lease)`. The layer is
going to be rebuilt, so it should be specified cleanly rather than restored.

**The wire cannot express two of the six operations.** `wheels_x` has no wire
form at all, and `move_x` (a distance *and* a rotation) cannot be said with
`MOVE`'s single stop condition.

**"Post a move, then wait for it to finish" is reimplemented four times**, each
slightly different and none public or composable: `tour.py`'s
`_wait_for_move_terminal`, `pathplan/planner.py`'s `_gotoAndWait`,
`testgui/transport.py`'s `_await_move_completion`, `square_tour.py`'s
`Tour._awaitMove` (all in `radio-robot-elite/src/host/robot_radio/`). A
generator/callback API for exactly the three execution modes already exists at
`robot/robot.py` (`speed()`, `go_to(on_tick=…)`, `Nezha._run_until_done`) but is
dead — it calls `NezhaProtocol` methods that no longer exist.

## Proposed fix

Write `docs/design/motion-api.md` with these sections and this substance.

### The central claim: everything decomposes to constant-ratio wheel segments

```
move_v(v_x, omega)      ==  wheels_v(v_x − omega·b/2, v_x + omega·b/2)
move_x(distance, rot)   ==  wheels_x(distance − rot·b/2, distance + rot·b/2)
go_to_r(x, y)           ==  move_x(arcLength, 2·atan2(y, x))
go_to_w(x, y)           ==  read pose → world-to-body → go_to_r
```

The body forms *are* the wheel forms composed with differential kinematics;
`wheels_x`/`wheels_v` are the only primitives. `wheels_x(+d, −d)` is a pivot,
`wheels_x(d, d)` a straight line. A `move_x` with a large rotation is not one
segment but a pivot followed by a straight, so the general statement is: **every
motion is one or more constant-ratio segments, each bounded by displacement or
by time.**

### The X/V rule for cruise

- **V-forms: the commanded velocity *is* the cruise** — a target reached through
  the velocity profile, not an instantaneous jump. No separate cruise argument.
  `move_v` needs no cruise for `omega` because omega is slaved to `v_x`; that
  coupling is what preserves curvature.
- **X-forms: the commanded value is a displacement**, so cruise is its own
  argument (`0` = configured default).

### The pivot-first rule (recover the real numbers from `88dd9ad8^`)

Decision site was `navigator.cpp:237-240`, taken before any arc solve.
`turn_first_angle = 0.8726646 rad` (**50°**), compared `>=` → stop, pivot, then
arc. `behind_angle = π/2` (**90°**), compared `>` inside the geometry — no
finite-radius tangent arc reaches beyond it. Never replace an in-flight arc with
a pivot at speed (it ratio-locks a hard brake onto the reversing wheel): ramp to
rest via a planned stop first, then pivot from rest. Pivot rate is derived, not
configured: `2·speed / trackWidth`.

Fine-align (`align_tol` 1.0°, `align_max_nudges` 6) must carry its existing
warning rather than being presented as a tunable — 333 measured nudges show a
bimodal ~1.8° quantum, and tightening to 0.3° drops convergence 93% → 64%.

### `go_to` geometry and pose sources

`go_to_r(x, y)` drives the constant-curvature arc tangent to the current
heading. **Final heading is a consequence, not an argument** — it is
`2·atan2(y, x)`; state the formula so the constraint is concrete.

`go_to_w` is `go_to_r` plus a pose read, from a **pluggable source** because the
fleet is not uniform (`gopiv` has no OTOS): OTOS when fitted, otherwise the
midpoint-arc encoder integration recovered from `odometry.cpp:17-51` — keeping
its two load-bearing details, a per-wheel `positionEpoch` guard that credits zero
delta across a software rebaseline, and an **unwrapped** heading.

### The control block

One structure that every operation posts and the loop consumes:

```
uLeft, uRight    ratio, normalized so max(|uLeft|,|uRight|) == 1
cruise           [mm/s] dominant-wheel target speed
stop             Displacement | Time     (Arrival is supervisory)
limit            in stop's own unit
deadline         [ms] timeout backstop / lease
id               correlation id, unique per session
```

Not invented: this is the recovered `Motion::Move` plus `MoveShape` (`shapeOf()`
normalized by the dominant wheel, `cruise` = that wheel's speed). Ratio
preservation already has four implementations in the live kernel to build on — λ
authority scaling, the twist-integral hold (`twistHoldGain`, the encoder-only
ratio keeper), the ratio-preserving speed floor, and Stage A/B/C. The profiler
plans one scalar; each wheel commands `λ · u_w`, so commanded curvature cannot
drift no matter what the profile does.

### The three execution modes

All three are the same two operations — **post** and **tick** — differing only in
who calls tick.

| mode | who ticks | you must |
|---|---|---|
| **A — background** | the fiber/timer | yield in your loop; call `stop`/`estop` yourself |
| **B — manual** | you | call `tick` (or iterate) every pass |
| **C — blocking** | the library | nothing; pass a callback to observe or abort |

Object model: **no callback → you get an object you drive** (iterating ticks it
and yields telemetry); **callback → the library drives the loop** and calls you
back each tick, and calling `stop` from the callback ends it; **fiber running →
the fiber drives it** and the object is for observing. Completion reasons match
the wire's `done #<id> <reason>` exactly (`stop`, `timeout`) plus two local-only
(`estop`, `aborted`). The loop has exactly one owner: calling `tick` while a
fiber owns it raises rather than double-ticking.

**Over the wire, tick means "drain the pushed telemetry and test for completion"
— never a poll round-trip.** Not stylistic: polling during a move over the relay
was measured cutting travel from 197.5 mm to 0.3 mm. Mode C is the default over
the wire (what `RogoClient.cmd("drive 200")` already does); mode A is already
spelled `nowait` in the repl grammar.

### Safety invariants

- **No unbounded form exists.** V-forms bounded by duration (which *is* the
  lease); X-forms by displacement **plus** a required timeout backstop.
- **`stop` is planned, `estop` is the panic stop** — measured on a 400 mm leg
  with the halt 0.5 s in: 39.8 cm / 5.9 s versus 2.9 cm / 0.10 s. Every halt
  path calls `estop`.
- **One `estop` is not proof of a stop.** The brick latches its last commanded
  speed; a single `estop` failed 5 of 6 attempts, and one from a then-silent host
  produced 936 mm of continued travel. Confirm and re-issue.
- **~5% of moves are lost silently over the radio** — the enqueue ack is
  generated locally, so it proves nothing. Confirm motion actually started.
- **Ids are unique per session**; a reused id is `err #<id> 11`.
- **Exactly one subsystem owns motion at a time.**

### The wire split, as a proposal

Six verbs replace three verbs plus four discriminator fields — `kind`, `stop`,
`limit` and `frame` all disappear into the verb name:

```
WHEELS_X <left> <right> <cruise> <timeout> #<id>
WHEELS_V <left> <right> <duration> #<id>
MOVE_X   <distance> <rotation> <cruise> <timeout> #<id>
MOVE_V   <v_x> <omega> <duration> #<id>
GO_TO_R  <x> <y> <speed> <arrive> <timeout> #<id>
GO_TO_W  <x> <y> <speed> <arrive> <timeout> #<id>
```

Same argument already made for case-carries-direction and the self-marking
`#id`: **put the discriminator in the verb, where it cannot be mismatched.**
`MOVE w … a …` is spellable today and means something odd; after the split every
invalid combination is unspellable. Written as a proposal against
`docs/protocol-v6-spec.md`, which stays the wire authority and is not edited.

### Decisions to state explicitly (so they are easy to reverse)

1. **Angles in degrees at the API**, milliradian integers on the wire —
   conversion lives in the binding.
2. **Verb spelling `MOVE_X` / `GO_TO_R`** so the wire verb is exactly the method
   name uppercased. Trivially flipped to `MOVEX`/`GOTOR`.
3. **`go_to_w`'s pose source is pluggable** rather than assuming OTOS.

### Code examples

Python and JavaScript, all three modes each, every driving example carrying an
`estop` path.

## Verification

- Every operation appears in the per-operation reference with matching argument
  names and units, and in the wire table with the same argument order.
- Every numeric claim traces to a cited source — the 50°/90° thresholds, the
  1.8° align quantum, the 39.8 cm / 2.9 cm halt contrast, the 197.5 / 0.3 mm
  polling measurement, the ~5% loss.
- No identifier carries a unit; every quantity carries a `[unit]` comment tag
  (`radio-robot-elite/.claude/rules/naming-and-style.md`).
- Round-trip: each of the six methods maps to exactly one wire verb and back,
  with no field left over and no discriminator argument surviving.

## Related

- `docs/protocol-v6-spec.md` — the wire authority; §5 motion, §8 outcomes
- `docs/design/protocol.md` — the adapter seam (`onWheels`/`onStop`/`onEstop`)
- `docs/design/diffdrive.md` — `drive()`, `step()`, `FiberLauncher`, the lease
- `radio-robot-elite` at `88dd9ad8^` — recoverable `Motion::Planner`,
  `Navigator`, `ArcSolver`, `Odometry`, `profile`/`shape`
- `radio-robot-elite/src/firm/diffdrive/differential_drive.cpp` — the live
  ratio-preservation mechanisms

"""motion.py -- the motion-API convenience layer (docs/design/
motion-api.md), a thin binding of its six operations plus stop/estop
and GET/SET config delegation onto `reliability.Session`.

This is the fourth module in `robot_v6`, added purely as a convenience
layer over the existing three (`codec`, `transport`, `reliability`) --
see sprint.md's Architecture Step 3 and Design Rationale Decision 1 for
why it lives here rather than inside the `rogo` CLI package that first
consumes it: any future host-side caller gets `wheels_v()`/`move_x()`/
etc. for free, instead of re-deriving the unit conversion and verb
encoding motion-api.md#9.1 specifies.

**One name per operation, everywhere** (motion-api.md#1): the wire verb
is the method name in upper case, so `wheels_v` in Python sends
`WHEELS_V` on the wire, `get`/`set` send bare `GET`/`SET` -- the same
vocabulary a person reading a wire log and a person reading this module
are both reading. `get`/`set` deliberately shadow the builtins of the
same name; that is the point, not an oversight (motion-api.md's own
"one name per operation" rule leaves no other spelling available for
protocol.md#7's `GET`/`SET` verbs).

**Unit conversion is the one thing this module actually does**
(motion-api.md#9.1): "degrees at the API, milliradians on the wire."
`rotation_deg` (`move_x`) and `omega_deg_s` (`move_v`) are the only two
arguments across all six operations expressed in degrees; every other
argument is already mm, mm/s, or ms and passes straight through
unconverted -- there is no other unit boundary anywhere else in this
module.

**No change to `codec.py`, `transport.py`, or `reliability.py`.** Every
function below is built entirely on `Session.send()` (returns the
assigned sequence id) and `Session.send_unsequenced()` (for the three
verbs protocol.md#8.3 exempts from sequencing -- only `estop()` uses
that here; `stop()` IS sequenced, per protocol.md#6's own verb table).
"""

from __future__ import annotations

import math

from .reliability import Session

# motion-api.md#9.1: "Angles are degrees at the API and milliradian
# integers on the wire." One conversion factor, one place -- every
# degree-valued argument in this module funnels through it.
_MRAD_PER_DEG = math.pi / 180.0 * 1000.0


def _deg_to_mrad(degrees: float) -> int:
    """Convert one degree-valued API argument to the integer
    milliradians the wire carries (motion-api.md#9.1). The wire has no
    fractional-field syntax at all (protocol.md#2: "every wire value is
    a base-10 ASCII integer"), so this always returns an `int`, rounded
    to the nearest milliradian."""
    return round(degrees * _MRAD_PER_DEG)


# ---------------------------------------------------------------------------
# The six motion-api operations (motion-api.md#1, #9.1).
# ---------------------------------------------------------------------------

def wheels_x(session: Session, left: float, right: float,  # [mm] [mm]
             cruise_mm_s: float, timeout_ms: int) -> int:
    """`WHEELS_X <left> <right> <cruise> <timeout> #<id>` -- move each
    wheel a commanded distance, bounded by encoder travel plus the
    required `timeout` backstop (motion-api.md#3.1). `kUnknown` on
    `DiffDriveAdapter` today (no planner) -- this call still encodes
    and dispatches correctly; see protocol.md#5."""
    return session.send("WHEELS_X", left, right, cruise_mm_s, timeout_ms)


def wheels_v(session: Session, left: float, right: float,  # [mm/s] [mm/s]
             duration_ms: int) -> int:
    """`WHEELS_V <left> <right> <duration> #<id>` -- command each wheel
    a maximum velocity, held for `duration`, which IS the kernel's
    lease (motion-api.md#3.2). The one operation `DiffDriveAdapter`
    gives real kinematic effect to today (protocol.md#5)."""
    return session.send("WHEELS_V", left, right, duration_ms)


def move_x(session: Session, distance_mm: float, rotation_deg: float,
           cruise_mm_s: float, timeout_ms: int) -> int:
    """`MOVE_X <distance> <rotation> <cruise> <timeout> #<id>` -- travel
    `distance_mm` while the heading changes by `rotation_deg`
    (motion-api.md#3.3). `rotation_deg` is converted to integer
    milliradians on the wire (motion-api.md#9.1); `kUnknown` on
    `DiffDriveAdapter` today."""
    return session.send(
        "MOVE_X", distance_mm, _deg_to_mrad(rotation_deg), cruise_mm_s, timeout_ms)


def move_v(session: Session, v_x_mm_s: float, omega_deg_s: float,
           duration_ms: int) -> int:
    """`MOVE_V <v_x> <omega> <duration> #<id>` -- command a body twist
    (forward velocity, yaw rate) held for `duration`, the lease exactly
    as in `wheels_v` (motion-api.md#3.4). `omega_deg_s` is converted to
    integer milliradians/s on the wire; `kUnknown` on `DiffDriveAdapter`
    today."""
    return session.send(
        "MOVE_V", v_x_mm_s, _deg_to_mrad(omega_deg_s), duration_ms)


def go_to_r(session: Session, x_mm: float, y_mm: float, speed_mm_s: float,
            arrive_mm: float, timeout_ms: int) -> int:
    """`GO_TO_R <x> <y> <speed> <arrive> <timeout> #<id>` -- drive to a
    point in the robot's own frame along the tangent constant-curvature
    arc (motion-api.md#3.5). No degree-valued argument here -- the
    final heading is a consequence, not an argument. `kUnknown` on
    `DiffDriveAdapter` today."""
    return session.send("GO_TO_R", x_mm, y_mm, speed_mm_s, arrive_mm, timeout_ms)


def go_to_w(session: Session, x_mm: float, y_mm: float, speed_mm_s: float,
            arrive_mm: float, timeout_ms: int) -> int:
    """`GO_TO_W <x> <y> <speed> <arrive> <timeout> #<id>` -- the same as
    `go_to_r`, in world coordinates: transform through the current pose,
    then delegate (motion-api.md#3.6). `go_to_w`'s pose source is
    pluggable and not this module's concern (motion-api.md#9.3 item 3);
    `kUnknown` on `DiffDriveAdapter` today."""
    return session.send("GO_TO_W", x_mm, y_mm, speed_mm_s, arrive_mm, timeout_ms)


# ---------------------------------------------------------------------------
# Stopping (motion-api.md#3.7, #9.1) -- two verbs, not two flavours of
# the same thing: `stop` ends a motion the program meant to end; `estop`
# is the panic path.
# ---------------------------------------------------------------------------

def stop(session: Session, immediate: bool = False) -> int:
    """`STOP [now] #<id>` -- acts on the CURRENT motion, not queued
    behind it (motion-api.md#3.7/#6). `STOP` is sequenced (protocol.md#6
    verb table), unlike `ESTOP` -- the optional `now` token requests a
    non-ramped stop and sits before the id, since the id is always the
    line's own last token regardless (protocol.md#9.1)."""
    if immediate:
        return session.send("STOP", "now")
    return session.send("STOP")


def estop(session: Session) -> None:
    """`ESTOP` -- the panic path, not a general-purpose halt
    (motion-api.md#3.7). Unsequenced per protocol.md#8.3's exemption
    set: no id, never acked/nacked, must execute even while the stream
    is stalled on a gap. Returns nothing -- there is no sequence id to
    hand back."""
    session.send_unsequenced("ESTOP")


# ---------------------------------------------------------------------------
# GET/SET -- protocol.md#7's config delegation. The library stores no
# field table of its own; these are pure wire wrappers.
# ---------------------------------------------------------------------------

def get(session: Session, name: str | None = None) -> int:  # noqa: A001 -- see module docstring
    """`GET [name] #<id>` -- `name=None` (the default) sends the bare
    `GET` that asks the adapter for every field it exposes, one `get`
    reply line per field; a `name` asks for just that one (protocol.md
    #6/#7). An unknown name still gets acked, just with no `get` reply
    line."""
    if name is None:
        return session.send("GET")
    return session.send("GET", name)


def set(session: Session, name: str, value: float) -> int:  # noqa: A001 -- see module docstring
    """`SET <name> <value> #<id>` -- protocol.md#7's config delegation:
    the handler holds no field table, so an unknown `name` is an
    adapter-level merits rejection (`err 1 #<id>`, `ERR_UNKNOWN`),
    layered on top of the in-order `ack` every `SET` gets regardless
    (protocol.md#8.2)."""
    return session.send("SET", name, value)

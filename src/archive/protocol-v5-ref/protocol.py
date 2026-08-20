"""NezhaProtocol — binary wire-protocol adapter for the P4 single-loop
Nezha firmware (103-001 onward).

Owns the SerialConnection and is the only code that touches the serial port.
All command encoding and response parsing lives here; higher-level objects
(NezhaState, Nezha) delegate every wire operation to this class.

Wire format — P4 (single-loop firmware, 103-001)
-------------------------------------------------
The command plane is binary-only: one ``CommandEnvelope`` (protobuf,
``protos/envelope.proto``) per outbound command, framed as a COBS+CRC frame
(sprint 123 tickets 001/002/003; was armored as a `*B<base64>` line pre-123
-- see ``io/serial_conn.py``'s ``send_envelope()``/``send_envelope_fast()``
and ``io/wire_codec.py`` for the current framing). ``CommandEnvelope``'s
``cmd`` oneof carries exactly THREE arms —
``move``/``config``/``stop`` — every earlier arm (ping/echo/id/hello/ver/
help/get/drive/segment/replace/motion/pose_fix/otos/stream/plan_dump) was
pruned by 103-001's schema prune and is `reserved`, not reused (see
``envelope.proto``'s own header comment). 116-001 (MOVE protocol cutover)
replaced the interim ``twist`` arm (103-001) with ``move`` — a single
bounded motion command (velocity variant + stop condition + required
``timeout`` backstop + a ``replace`` flag against a small on-chip queue);
``twist`` (field 19) is `reserved`, not reused — see ``move_twist()``/
``move_wheels()`` below. There is no per-command synchronous reply for
``move``/``config``/``stop`` — a command's outcome rides the ack ring
inside the next ``Telemetry`` push (``wait_for_ack()``).

Telemetry is always-on (no STREAM arm to arm first): the firmware pushes a
``ReplyEnvelope{tlm: Telemetry}`` frame unconditionally every loop iteration
(primary period == cycle period, 20 ms — frame v2, 115-003) — see
``read_binary_tlm_frames()``/``read_pending_binary_tlm_frames()``.

Telemetry frame v2 (115-003, gut-to-minimal-firmware S1 — implements
``telemetry-frame-tightening-amendment-to-gut-s1.md``): a clean, incompatible
rewrite of the ``Telemetry`` message. Per-source reading objects
(``EncoderReading``/``OtosReading``) replace the old flat ``enc_*``/``vel_*``
floats and the bare ``Pose2D otos``; one ``flags`` bit-string replaces every
standalone status bool plus the ``fault_bits``/``event_bits`` masks; a single
``ack_corr``/``ack_err`` slot replaces the depth-3 ``AckEntry`` ring; packed
``line``/``color`` sensor words are new. See ``TLMFrame``'s own docstring for
the host-side adaptation.

104-002 deleted every method targeting a now-reserved arm (ping/echo/get_id/
get_ver/get_help/get_config/get_config_binary/pose_fix/drive/timed/distance/
arc/vw/turn/go_to/grip/zero_*/otos_*/port_*/stream/stream_fields/snap/
stream_drive/wait_for_evt_done/cancel and the ``Stop`` stop-clause-token
builder) — see this ticket's completion notes for the full disposition
table.

132-011 (GetConfig/ConfigSnapshot wire read-back) reopens the read-back gap
104-002's note above described as "permanent": ``get_config()`` (below) is
a NEW method, not a resurrection of the deleted 104-002 one of the same
name -- it targets a genuinely new wire arm
(``CommandEnvelope.cmd.get_config`` / ``ReplyEnvelope.body.cfg``,
``GET_CONFIG``/``CFG`` in ``commands.proto``), not the reserved pre-104
``get`` arm. Unlike ``move``/``config``/``stop`` (whose outcome rides the
ack ring, §7.1 ``docs/protocol-v5.md``), ``get_config()`` sends via the
BLOCKING ``send_envelope()``/``_send_envelope()`` and reads a genuinely
synchronous ``ReplyEnvelope{cfg: ConfigSnapshot}`` back -- the one CONFIG
binary-arm outcome that needs a real reply body, not just an ack.

132-014 (migrate host NezhaProtocol onto the new config surface): 132-013
deleted ``config.proto`` wholesale (the ``*ConfigPatch`` messages,
``PatchKind``, ``ConfigDelta`` itself) and retargeted ``envelope.proto``'s
``config`` arm (field 6) from ``ConfigDelta`` onto ``robot_config.proto``'s
``SetConfigGroup``. This ticket retargets every host method that used to
build a ``*ConfigPatch`` onto the surviving/new group-and-field primitives:
``set_config_group()``/``set_config_field()``/``get_config()``.
``config()``/``otos_config()``/``estimator_config()`` -- each a "build
exactly ONE envelope carrying exactly ONE ``*ConfigPatch``" builder for a
message type that no longer exists -- are DELETED outright, not
retargeted: their job (single-envelope, single-target pushes) is now
``set_config_field()``'s job, and ``otos_config(init=True)``/``OI``'s
underlying ``OtosConfigPatch.init`` trigger has NO successor field in
``robot_config.proto``'s ``Otos`` message (offset/scale values only -- see
``configurator.h``'s own OTOS re-appliability row: "its 6th field, init,
was a fire-and-forget trigger with no ``Config::Robot``-shaped successor
and was never persisted either") -- a real, flagged capability gap, not an
oversight (see ``binary_bridge.py``'s ``_handle_otos_patch()`` for where
this lands on the wire-verb surface). ``set_config()``/``set_config_binary()``
also referenced the deleted ``ConfigDelta``/``*ConfigPatch`` shapes;
``set_config_binary()`` (a raw-``ConfigDelta``-in, ``AckEntry``-out sender
with no successor shape to wrap) is DELETED, while ``set_config()`` -- kept
as the flat ``SET key=value`` text-verb vocabulary's host entry point,
still used by ``binary_bridge.py``'s/``SimTransport``'s ``SET`` verb and by
``robot_radio.robot.nezha.Nezha`` -- is REWRITTEN as a thin fan-out over
``set_config_field()``, one round trip per key, via the new
``_SET_KEY_TARGETS`` table (below) mapping each flat key to a
``(ConfigGroupTarget, field_name)`` pair instead of the old
``_DRIVETRAIN_KEYS``/``_MOTOR_PID_KEYS`` Patch-field tables.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, NamedTuple

from robot_radio.io.serial_conn import SerialConnection

# Binary-plane pb2 bindings (096-007, M6 Host Config/Telemetry Client). Safe
# to import at module level here (unlike robot_radio.io.serial_conn.py --
# see that module's own _get_envelope_pb2() docstring for the circular-
# import hazard it avoids): robot_radio.robot.pb2 has no dependency back onto
# robot_radio.robot or robot_radio.io, so importing it while
# robot_radio.robot's own __init__.py is still mid-execution (which is
# always the case when this module is first loaded -- __init__.py imports
# this module itself) never re-enters a partially-initialized module.
from robot_radio.robot.pb2 import envelope_pb2, robot_config_pb2, telemetry_pb2
# config_pb2 -- DELETED, 132-013 (patch-surface retirement, sprint 132
# "configuration discipline"): config.proto (DrivetrainConfigPatch/
# MotorConfigPatch/OtosConfigPatch/EstimatorConfigPatch's only source) no
# longer exists, replaced by robot_config.proto's group/field wire arms.
# Dropped from this import (not merely left broken) because this module's
# __init__-time import chain (robot_radio.robot/__init__.py imports this
# module eagerly) means an ImportError here would poison every caller of
# `robot_radio.robot`, including firmware-side tests with nothing to do
# with config. 132-014 migrates every method that used it
# (config()/otos_config()/estimator_config()/set_config()/set_config_binary())
# onto robot_config_pb2's group/field messages -- see this module's own
# header comment for the full disposition.

# robot_config_generated -- the GENERATED pydantic group model (132-002),
# NOT robot_radio.config.robot_config's hand-written loader/validator
# classes (a different, larger surface -- get_robot_config()/list_robots()/
# derived fields/env-var resolution). get_config() (132-011, below) returns
# ONE of these flat per-group models (Geometry/Motors/Drive/WheelControl/
# Planner/Otos/Estimator), field-for-field identical to the wire group it
# decodes -- matching the-configuration-object.md's "the object holds RAW
# file values" rule on the host side too. Safe to import at module level
# for the same reason config_pb2/envelope_pb2 above are (robot_radio.config
# has no dependency back onto robot_radio.robot/robot_radio.io -- confirmed
# by reading robot_config_generated.py/robot_config.py/config/__init__.py
# in full: pydantic only).
from robot_radio.config import robot_config_generated


# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------

# kAngleScale mirror (source/telemetry/tlm_frame.cpp): 18000/pi, converting
# radians (telemetry.proto's Pose2D.h, common.proto) to centidegrees (the
# int this dataclass's pose/otos fields carry, matching the historical
# text-plane TLM parser's own units -- see TLMFrame.from_pb2()'s docstring).
# Same scale factor, same truncate-toward-zero int() cast the firmware's
# static_cast<int> applies -- see TLMFrame.from_pb2().
_ANGLE_SCALE = 5729.5779513  # [cdeg/rad]

# Fixed-point wire scales (124-008, issue §B3) -- mirror telemetry.proto's/
# common.proto's own (scale) field option declarations EXACTLY (options.proto's
# own doc comment: "generated conversion, not hand-transcribed" -- host-side,
# the schema is the single source via the compiled descriptor; these
# constants transcribe that same schema value once, here, rather than at
# every call site). The host decodes these fields via REAL protobuf, which
# already reverses zigzag automatically (sint32 is a native proto3 scalar
# type) -- these constants are the remaining real = raw * scale step.
_POSITION_SCALE = 1.0     # [mm] EncoderReading.position, OtosReading.x/y, common.proto Pose2D.x/y
_VELOCITY_SCALE = 0.1     # [mm/s] EncoderReading.velocity, OtosReading.v_x/v_y, common.proto BodyTwist3.v_x/v_y
_HEADING_SCALE = 0.001    # [rad] OtosReading.heading, common.proto Pose2D.h
_OMEGA_SCALE = 0.01       # [rad/s] OtosReading.omega, common.proto BodyTwist3.omega

# modeChar() mirror (source/telemetry/tlm_frame.cpp): maps msg::DriveMode
# (telemetry.mode, telemetry.proto -- DriveMode relocated in from the deleted
# planner.proto by 115-003, unchanged shape) to a single-character mode= wire
# value the historical text-plane TLM parser read off a text STREAM/SNAP
# frame's "mode=" token. Any DriveMode value with no entry here falls back to
# "I" via .get()'s own default in from_pb2(), so a future DriveMode value
# added to telemetry.proto without a matching entry here falls back safely
# instead of raising.
#
# 116-007: VELOCITY (set by RobotLoop while a MOVE is actively driving,
# `driving_ ? VELOCITY : IDLE`) previously had no entry and silently fell
# back to "I" -- the SAME character IDLE produces -- so a host-side reader
# (e.g. tlm_log.py's `mode` column) could never distinguish "driving" from
# "idle" by this column alone (confirmed on the sim dry-run, see
# docs/bench-checklists/sprint-115-gut-s1.md). "V" is unused by every other
# entry below (I/S/T/D/G) and is now VELOCITY's own dedicated character.
_DRIVE_MODE_CHAR = {
    telemetry_pb2.IDLE: "I",
    telemetry_pb2.STREAMING: "S",
    telemetry_pb2.TIMED: "T",
    telemetry_pb2.DISTANCE: "D",
    # 135-004: DriveMode value 4 renamed GO_TO -> NAVIGATING (number
    # unchanged) -- resolved a protoc enum-value-scope collision with
    # commands.proto's new Verb.GO_TO. Display character unchanged ("G",
    # for "going"/navigating).
    telemetry_pb2.NAVIGATING: "G",
    telemetry_pb2.VELOCITY: "V",
}


@dataclass(frozen=True)
class AckEntry:
    """One command's ack outcome, adapted onto a plain host-side shape the
    same way ``TLMFrame`` adapts ``Telemetry`` itself — either the single
    "freshest ack" scalar slot (``Telemetry.ack_corr``/``ack_err``,
    ``from_telemetry()`` below) or one entry from the bounded ack ring
    (``Telemetry.acks``, 120, ``from_ring_entry()`` below).

    Reports the outcome of ONE previously-sent command (matched by
    ``corr_id``) — the P4 wire has no per-command synchronous
    ``ReplyEnvelope`` for ``move``/``stop``/``config``, so telemetry is the
    ONLY place their outcome is reported. ``err_code == 0`` (``ok=True``)
    means OK; nonzero is the raw ``ErrCode`` (envelope.proto) value — the
    same two-value shape every command outcome has always produced.

    115-003 frame v2 deleted the pre-115 depth-3 wire ``AckEntry``
    ring/``AckStatus`` enum (OK/ERR/DONE/TRIVIAL/SUPERSEDED/FLUSHED/
    TIMEOUT/SOLVE_FAIL, the deleted executor's own completion taxonomy) in
    favor of one scalar slot. 120 (bench-single-ack-slot-observability-
    collapses-at-40ms.md) brought a wire ``AckEntry`` message back — a
    bounded, depth-4, corr_id+err ring. 124-008 (issue §B4) DELETED both
    the wire ``AckEntry`` message AND the single "freshest ack" scalar slot
    (``ack_corr``/``ack_err``, ``flags`` bit 5) it duplicated — ring
    membership already means "really acked." ``Telemetry.acks`` is now
    ``repeated uint32``, PACKED, each element ``corr_id<<4 | err`` — the
    ring is this dataclass's ONLY remaining origin.
    """
    corr_id: int
    ok: bool
    err_code: int  # raw ErrCode (envelope.proto) value when ok is False, else 0 (ERR_NONE)

    @classmethod
    def from_ring_entry(cls, entry: int) -> "AckEntry":
        """Build an ``AckEntry`` from one packed word of the bounded ack
        ring (``Telemetry.acks``, 120, repacked 124-008) — ``entry`` is a
        plain python ``int`` (the real protobuf decoder hands back a bare
        int for a packed-scalar repeated field; there is no ``AckEntry``
        wire message any more, issue §B4), not the whole ``Telemetry``
        frame it rode in on. `corr_id`` is the upper bits, ``err`` the low
        4 (mirrors ``Core::Telemetry::pushAckRing()``'s own packing,
        telemetry.cpp)."""
        corr_id = entry >> 4
        err = entry & 0xF
        return cls(corr_id=corr_id, ok=(err == 0), err_code=err)


# ---------------------------------------------------------------------------
# Reading objects (115-003 frame v2) -- host-side adapters for the wire's
# per-source EncoderReading/OtosReading messages (telemetry.proto), mirroring
# AckEntry's own "plain dataclass adapted from a pb2 message" shape. Named the
# SAME as their telemetry_pb2 counterparts; no collision since telemetry_pb2
# is always accessed as a qualified module attribute (telemetry_pb2.
# EncoderReading), never imported bare.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class EncoderReading:
    """One wheel's encoder sample -- position AND velocity together.
    Adapts ``telemetry_pb2.EncoderReading``.

    124-008 (issue §B3/§B2): ``position``/``velocity`` are sint32+scale on
    the wire now -- protobuf reverses zigzag automatically (sint32 is a
    native proto3 type), ``from_pb2()`` applies the remaining
    ``real = raw * scale`` step (``_POSITION_SCALE``/``_VELOCITY_SCALE``).
    ``time`` is RENAMED ``age`` -- an absolute robot-clock value can't be
    packed small; ``age`` is the delta this sample's own collect time is
    BEHIND ``TLMFrame.t``, bounded to 255ms. Production firmware always
    emits ``age=0`` as of this ticket (genuine per-sample skew is ticket
    009's own work) -- the field decodes correctly regardless.
    ``position_epoch`` (ADDITIVE) is the position-rebaseline policy's own
    counter (sprint 124 architecture Decision 6): increments each time
    firmware software-rebaselines this wheel's position."""
    position: float  # [mm] accumulated
    velocity: float  # [mm/s] signed, measured
    age: int         # [ms] delta behind TLMFrame.t at sample collect
    position_epoch: int  # wraps; +1 each firmware-side rebaseline (Decision 6)

    @classmethod
    def from_pb2(cls, reading: "telemetry_pb2.EncoderReading") -> "EncoderReading":
        return cls(position=float(reading.position) * _POSITION_SCALE,
                    velocity=float(reading.velocity) * _VELOCITY_SCALE,
                    age=int(reading.age), position_epoch=int(reading.position_epoch))


@dataclass(frozen=True)
class OtosReading:
    """Everything the OTOS supplies in one burst read: position, heading,
    AND the measured velocities (v_x/v_y/omega -- previously read by the
    driver and dropped on the floor). Adapts ``telemetry_pb2.OtosReading``.
    Valid iff ``TLMFrame.otos_present`` (flags bit 0) -- see
    ``TLMFrame.otos_reading``.

    124-008: x/y/heading/v_x/v_y/omega are sint32+scale on the wire now --
    see ``EncoderReading``'s own docstring for the zigzag/scale split.
    ``time`` is RENAMED ``age`` -- same rationale as ``EncoderReading.age``."""
    x: float        # [mm]
    y: float        # [mm]
    heading: float  # [rad]
    v_x: float      # [mm/s]
    v_y: float      # [mm/s]
    omega: float    # [rad/s]
    age: int        # [ms] delta behind TLMFrame.t at burst read

    @classmethod
    def from_pb2(cls, reading: "telemetry_pb2.OtosReading") -> "OtosReading":
        return cls(x=float(reading.x) * _POSITION_SCALE, y=float(reading.y) * _POSITION_SCALE,
                    heading=float(reading.heading) * _HEADING_SCALE,
                    v_x=float(reading.v_x) * _VELOCITY_SCALE, v_y=float(reading.v_y) * _VELOCITY_SCALE,
                    omega=float(reading.omega) * _OMEGA_SCALE,
                    age=int(reading.age))


# ---------------------------------------------------------------------------
# flags bit layout (telemetry.proto Telemetry.flags -- 115-003). Mirrors the
# proto's own bit-table comment exactly; TLMFrame's presence/status/fault/
# event properties below are computed from these constants, never a second
# hand-copied numbering.
# ---------------------------------------------------------------------------
_FLAG_OTOS_PRESENT = 1 << 0
_FLAG_OTOS_CONNECTED = 1 << 1
_FLAG_ACTIVE = 1 << 2
_FLAG_CONN_LEFT = 1 << 3
_FLAG_CONN_RIGHT = 1 << 4
# Bit 5 / bit 11 pair with LINE_PRESENT / COLOR_PRESENT below: Present
# means a reading is on the wire, Fresh means it was re-read on that
# cycle. The firmware samples only ONE perception leaf per cycle, so the
# other sensor's value is up to one alternation stale but still sent.
_FLAG_LINE_FRESH = 1 << 5
# bit 5 -- RESERVED (124-008: formerly _FLAG_ACK_FRESH, deleted with the
# single "freshest ack" scalar slot it gated).
_FLAG_FAULT_I2C_SAFETY_NET = 1 << 6
_FLAG_FAULT_WEDGE_LATCH = 1 << 7
_FLAG_FAULT_I2C_NAK_TIMEOUT = 1 << 8
_FLAG_FAULT_MALFORMED_FRAME = 1 << 9
_FLAG_EVENT_DEADMAN_EXPIRED = 1 << 10
_FLAG_EVENT_BOOT_READY = 1 << 11
_FLAG_EVENT_CONFIG_APPLIED = 1 << 12
_FLAG_LINE_PRESENT = 1 << 13
_FLAG_COLOR_PRESENT = 1 << 14
_FLAG_FAULT_MOVE_TIMEOUT = 1 << 15
_FLAG_FAULT_SHAPING_DISABLED = 1 << 16
# bit 17 (kFlagFaultPositionClamped) / bit 18 (kFlagFaultCommandsDropped) --
# declared in telemetry.h, not yet decoded here (no host consumer needed one
# until now) -- NOT skipped, reserved for a future ticket to fill in the gap
# rather than renumbered around.
_FLAG_FAULT_WHEEL_FROZEN_LEFT = 1 << 19
_FLAG_FAULT_WHEEL_FROZEN_RIGHT = 1 << 20
# 130-005 (wheel-speed-controller-moves-into-drive.md, issue 04's
# folded-in observability mandate): Core::DifferentialDrive's deficit-flag policy --
# a sustained large speed error while BOTH Stage C's bias and Stage B's
# fast PID sit pinned at their configured authority. See telemetry.h's
# own kFlagFaultWheelDeficitLeft/Right doc comment.
_FLAG_FAULT_WHEEL_DEFICIT_LEFT = 1 << 21
_FLAG_FAULT_WHEEL_DEFICIT_RIGHT = 1 << 22
_FLAG_FAULT_STALL_LEFT = 1 << 24
_FLAG_FAULT_STALL_RIGHT = 1 << 25
# NOT bit 11 -- that is _FLAG_EVENT_BOOT_READY above.
_FLAG_COLOR_FRESH = 1 << 23


def _unpack_channels4(word: "int | None") -> "tuple[int, int, int, int] | None":
    """Unpack a packed 4-channel sensor word (``telemetry.proto``'s ``line``/
    ``color`` fields share this exact packing: one byte per channel, channel
    1 in the low byte) into a ``(ch1, ch2, ch3, ch4)`` tuple. Returns
    ``None`` for ``None`` (not-present) input -- callers gate on ``line``/
    ``color`` returning None."""
    if word is None:
        return None
    return (word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF, (word >> 24) & 0xFF)


@dataclass
class TLMFrame:
    """Parsed TLM telemetry frame from the firmware (frame v2, 115-003).

    All fields are optional — a frame built without going through
    ``from_pb2()`` (a hand-built test double, e.g.) leaves every field at
    this dataclass's own ``None`` default, distinguishing "never decoded"
    from "decoded as the wire's zero value". ``t`` is the robot clock in
    milliseconds at frame-assembly time. ``seq`` is the D10 sequence
    counter (uint16, wrapping at 65535). Use ``tlm_drop_rate(frames)`` to
    estimate packet loss. ``pose``/``otos`` heading is in centi-degrees
    (integer), positions in mm — this dataclass's own historical unit
    convention, kept unchanged by the frame v2 rewrite so every existing
    downstream reader (e.g. ``testgui/telemetry_panel.py``) keeps working.

    ``enc``/``vel`` are ``(left, right)`` — position [mm] / velocity [mm/s]
    per wheel, now DERIVED from the wire's own ``EncoderReading`` messages
    (``enc_left``/``enc_right``, each carrying position+velocity+its own
    collect time together — see ``enc_left``/``enc_right`` below for the
    full reading including ``time``). Always present on the wire (no
    presence gate), so always populated by ``from_pb2()``.

    ``twist`` is fused body-frame velocity, 2-tuple ``(v_mmps,
    omega_mradps)`` — the wire's ``BodyTwist3`` always zero-fills ``v_y``
    for this differential build (``tlm_frame.cpp``), so ``v_y`` is dropped
    here exactly as before. Always present on the wire.

    ``otos`` is the raw OTOS pose, ``(x, y, heading)`` in (mm, mm, cdeg) —
    valid iff ``otos_present`` (flags bit 0); ``otos_reading`` (below)
    carries the SAME burst's fuller shape (velocities + its own read
    time) for a caller that wants more than the legacy 3-tuple.

    ``line``/``color`` are ``(ch1..ch4)`` / ``(r, g, b, c)`` — NEWLY wired
    this ticket: frame v2 packs both sensors into one ``uint32`` word each
    (one byte per channel); previously these fields existed on this
    dataclass but were never populated by the binary decode path (only the
    retired text-plane parser ever set them). Valid iff ``line_present``/
    ``color_present`` (flags bits 13/14).

    ``ekf_rej``, ``wedge``, ``encpose``, ``otos_health`` remain permanent
    gaps for the binary decode path — telemetry.proto never declared
    matching fields even before this rewrite (see the retired text-plane
    parser, ``robot_radio.robot._legacy_tlm_text``, for the only place
    these were ever populated) — frame v2 does not change that.
    ``cmd_vel``/``acc_*``/``glitch_*``/``ts_*`` remain on
    ``TelemetrySecondary`` (own cadence, own decode path — 103-001,
    untouched by this ticket) — same permanent gap on the PRIMARY frame
    this class decodes.

    ``cycle_busy``/``cycle_period`` (123-004) are loop-timing diagnostics,
    ALWAYS populated (no presence gate — plain ``uint32`` fields on the
    wire, proto3 zero-value default when genuinely zero) — MIGRATED here
    from ``TelemetrySecondary`` (122-003's interim placement, forced by
    the pre-123 base64-armored envelope budget having no room on the
    primary frame) now that COBS+CRC framing (123-001/123-002) restored
    that headroom. Fresh every cycle now, not just at
    ``TelemetrySecondary``'s own ~5 Hz cadence.

    ``active`` is ``bb.drivetrain.busy`` (flags bit 2) — TRUE while a
    motion is in progress. The one reliable motion-complete signal
    (``mode`` does not track it for every drive path).

    ``flags`` is the raw wire bit-string (``telemetry.proto``
    ``Telemetry.flags`` — see that message's own bit-table comment for the
    authoritative numbering) — always populated. Every other
    presence/status/fault/event signal below is a ``@property`` DERIVED
    from ``flags``, never a second field to keep in sync:
      - ``otos_present`` (bit 0), ``otos_connected`` (bit 1) — OTOS
        freshness/connectivity.
      - ``conn_left``/``conn_right`` (bits 3/4) — per-motor bus
        connectivity.
      - bit 5 — RESERVED (124-008: formerly ``ack_fresh`` -- deleted with
        the single "freshest ack" scalar slot it gated; ring membership
        in ``acks`` below already means "really acked").
      - ``fault_i2c_safety_net``/``fault_wedge_latch``/
        ``fault_i2c_nak_timeout``/``fault_malformed_frame`` (bits 6-9) —
        the four fault bits.
      - ``event_deadman_expired``/``event_boot_ready``/
        ``event_config_applied`` (bits 10-12) — the three (one-shot,
        transition-cycle) event bits.
      - ``line_present``/``color_present`` (bits 13/14) — packed-word
        freshness.
      - ``fault_move_timeout`` (bit 15) — declared now, not wired until
        sprint 116's MOVE protocol lands (S1 has no MOVE command to time
        out); always False until then.
      - ``fault_shaping_disabled`` (bit 16) — a MOVE is active AND both
        angular and linear ``ShaperLimits`` axes are disabled (119 ticket
        001, kill-the-silent-off-shaping-config-boundary.md) — the loud
        off-state for the shaping/anticipation silent-off config
        boundary: with no taper, the land-at-zero completion path can
        never fire and the threshold/timeout backstop is the ONLY
        completion path.
      - ``fault_wheel_frozen_left``/``fault_wheel_frozen_right`` (bits
        19/20, 129-002, wheel-frozen-fault-flag-in-telemetry.md) — that
        wheel was commanded a nonzero duty for N consecutive cycles with
        NO encoder change (``Hardware::MotorArmor::wedgeSuspect()``, the
        GATED, motion-qualified stall detector) — deliberately NOT the
        same signal as ``fault_wedge_latch`` (bit 7, the raw,
        unconditional stuck-encoder latch that also fires on a healthy
        robot merely parked at rest). The TestGUI shows a red banner
        naming the frozen wheel; ``planner.tour.run_tour()`` aborts the
        active leg the instant either bit is observed rather than
        driving on.
      - ``fault_wheel_deficit_left``/``fault_wheel_deficit_right`` (bits
        21/22, 130-005, wheel-speed-controller-moves-into-drive.md) —
        Core::DifferentialDrive's deficit-flag policy: a sustained large speed error
        while BOTH Stage C's bias and Stage B's fast PID sit pinned at
        their configured authority ceiling — no more correction to give,
        so the robot runs slow, loudly, rather than silently.
      - ``duty_per_speed_left``/``duty_per_speed_right``/``bias_left``/
        ``bias_right``/``pid_left``/``pid_right`` (fields 17-22, 130-005)
        — Core::DifferentialDrive's unified wheel-speed controller's installed
        conversion scale and live per-wheel Stage C/B state, always
        populated (no presence gate).
    These properties are the ticket's own "existing downstream consumer
    keeps working unchanged" surface — grep ``src/host/robot_radio/`` for
    every attribute name the pre-115 standalone bool/bitmask fields
    exposed before renaming or removing any of them.

    ``ack_corr``/``ack_err``/``ack`` — DELETED (124-008, issue §B4): the
    single "freshest ack" scalar slot they adapted is gone; ring membership
    in ``acks`` below already means "really acked."

    ``acks`` (120, ADDITIVE) is the bounded ack ring
    (``telemetry.proto``'s ``Telemetry.acks``, depth 4) decoded as a plain
    ``list[AckEntry]``, oldest-pushed first, ALWAYS populated (may be
    empty) — independent of ``ack_fresh``, since a ring entry needs no
    freshness gate (see ``AckEntry.from_ring_entry()``'s own docstring).
    ``wait_for_ack()`` (``SerialConnection``/``NezhaProtocol``) scans this
    ring, not the single slot, to find a specific ``corr_id`` reliably
    across a bounded-but-real burst of rapid-fire commands; ``acks`` is
    exposed here too for a caller (bench scripts, ``tlm_log.py``) that
    wants to inspect the whole ring directly, per-frame.

    ``enc_left``/``enc_right`` (``EncoderReading | None``) and
    ``otos_reading`` (``OtosReading | None``, valid iff ``otos_present``)
    are the full per-source reading objects the wire now carries — richer
    than ``enc``/``vel``/``otos`` above (they add each reading's OWN
    collect/burst time), for a caller (e.g. ticket 008's ``tlm_log.py``)
    that wants the raw per-sample stamps rather than the legacy tuples.

    ``recvTime`` (127-004, ADDITIVE) is the HOST's own monotonic clock
    (``time.monotonic()``, not wall clock) at the instant the host drained
    this frame off the wire — ``t`` above is the ROBOT's clock, and the two
    are never the same timebase (``ClockSync`` would reconcile them; it is
    not activated by this field — see ``pathplan.world_pose`` for why).
    Combined with a reading's own ``age`` (``EncoderReading``/
    ``OtosReading``, both ``# [ms] behind TLMFrame.t``), a caller can
    recover that reading's own approximate HOST-clock capture instant —
    ``recvTime - age / 1000.0`` — the frame-age extrapolation pattern
    ``src/tests/bench/hil_drive.py``'s ``ingestTelemetry()``
    already uses (there, entirely within the ROBOT's own clock via
    ``t - age``; here, anchored onto the HOST's clock so it can be compared
    against a camera fix's own host-clock capture time). Populated ONLY by
    ``read_pending_binary_tlm_frames()`` at the point each frame is drained
    — NOT by ``from_pb2()`` itself (decode time is not receive time) and
    NOT by any other frame source (``read_binary_tlm_frames()`` included);
    every other builder of a ``TLMFrame`` leaves this at its ``None``
    default, the same "never decoded" convention as every other field.
    """
    t: int | None = None
    mode: str | None = None
    seq: int | None = None                       # D10 sequence counter (uint16, wraps at 65535)
    flags: int | None = None                      # raw bit-string -- see telemetry.proto Telemetry.flags (115-003)
    enc: tuple[int, int] | None = None          # (left, right) [mm] -- derived from enc_left/enc_right.position
    pose: tuple[int, int, int] | None = None    # (x, y, heading) [mm, mm, cdeg]
    vel: tuple[int, int] | None = None          # (left, right) [mm/s] -- derived from enc_left/enc_right.velocity
    cmd_vel: tuple[int, int] | None = None      # (left, right) COMMANDED per-wheel velocity (PID setpoint) mm/s -- permanent gap, TelemetrySecondary only
    twist: tuple[int, int] | None = None        # (v, omega_mrad)
    otos: tuple[int, int, int] | None = None    # (x, y, heading) [mm, mm, cdeg] — raw OTOS pose; valid iff otos_present
    line: tuple[int, int, int, int] | None = None   # (ch1, ch2, ch3, ch4); valid iff line_present, just-sampled iff line_fresh
    color: tuple[int, int, int, int] | None = None  # (r, g, b, c); valid iff color_present, just-sampled iff color_fresh
    ekf_rej: int | None = None                   # cumulative EKF gate rejection count -- permanent binary-decode gap
    wedge: tuple[int, int] | None = None         # (left, right) wedge-latch state, 0/1 each -- permanent binary-decode gap
    encpose: tuple[int, int, int] | None = None  # (x, y, heading) [mm, mm, cdeg] -- permanent binary-decode gap
    otos_health: tuple[int, bool] | None = None  # (raw STATUS byte, fusion_blocked) -- permanent binary-decode gap
    active: bool | None = None                   # bb.drivetrain.busy — motion in progress (flags bit 2)
    # ack_corr/ack_err/ack -- DELETED (124-008, issue §B4): the single
    # "freshest ack" scalar slot is gone; use `acks` below.
    acks: "list[AckEntry]" = field(default_factory=list)  # bounded ack ring (120), oldest-first, ALWAYS populated (may be empty), no freshness gate
    enc_left: "EncoderReading | None" = None      # full per-wheel reading (position/velocity/time) -- always present on the wire
    enc_right: "EncoderReading | None" = None
    otos_reading: "OtosReading | None" = None      # full OTOS burst (adds v_x/v_y/omega/time over `otos`); valid iff otos_present
    cycle_busy: int | None = None                 # [us] cycleStart -> frame-staging instant, THIS cycle (123-004, migrated from TelemetrySecondary)
    cycle_period: int | None = None               # [us] this cycle's cycleStart minus the previous cycle's (123-004)
    # Core::DifferentialDrive's unified wheel-speed controller (130-005, issue 04's
    # folded-in observability mandate): the installed conversion scale,
    # Stage C's adapted bias, and Stage B's last-computed fast-PID output,
    # per wheel. Always populated (no presence gate) -- Drive always has
    # SOME value for these (0.0 if uncalibrated / no gains configured, per
    # the fail-closed contract drive.h documents). The deficit-flag policy
    # rides flags bits 21/22 instead -- see fault_wheel_deficit_left/right.
    duty_per_speed_left: float | None = None       # [duty/(mm/s)]
    duty_per_speed_right: float | None = None      # [duty/(mm/s)]
    bias_left: float | None = None                 # [mm/s] Stage C's adapted parameter
    bias_right: float | None = None                # [mm/s]
    pid_left: float | None = None                  # [mm/s] Stage B's last-computed output
    pid_right: float | None = None                 # [mm/s]
    recvTime: float | None = None                 # [s] HOST monotonic clock at frame decode (127-004) -- NOT populated by from_pb2() itself; set by read_pending_binary_tlm_frames() at the point each frame is drained off the wire. See that method's own docstring; every other builder of a TLMFrame (from_pb2() alone, or a hand-built test double) leaves this at its None default, same "never decoded" convention as every other field.

    # ------------------------------------------------------------------
    # flags-derived properties (115-003) -- see this class's own docstring.
    # ------------------------------------------------------------------

    def _flag(self, bit: int) -> bool:
        return bool(self.flags is not None and (self.flags & bit))

    @property
    def otos_present(self) -> bool:
        return self._flag(_FLAG_OTOS_PRESENT)

    @property
    def otos_connected(self) -> bool:
        return self._flag(_FLAG_OTOS_CONNECTED)

    @property
    def conn_left(self) -> bool:
        return self._flag(_FLAG_CONN_LEFT)

    @property
    def conn_right(self) -> bool:
        return self._flag(_FLAG_CONN_RIGHT)

    @property
    def fault_i2c_safety_net(self) -> bool:
        """Known-benign boot one-shot (telemetry.proto's own bit-6 comment)
        -- only a bit that flips DURING driving, not just at boot, is
        actionable."""
        return self._flag(_FLAG_FAULT_I2C_SAFETY_NET)

    @property
    def fault_wedge_latch(self) -> bool:
        return self._flag(_FLAG_FAULT_WEDGE_LATCH)

    @property
    def fault_i2c_nak_timeout(self) -> bool:
        return self._flag(_FLAG_FAULT_I2C_NAK_TIMEOUT)

    @property
    def fault_malformed_frame(self) -> bool:
        return self._flag(_FLAG_FAULT_MALFORMED_FRAME)

    @property
    def fault_move_timeout(self) -> bool:
        """Bit 15 -- declared now, wired by sprint 116's MOVE protocol; S1
        has no MOVE command to time out, so this is always False today."""
        return self._flag(_FLAG_FAULT_MOVE_TIMEOUT)

    @property
    def fault_shaping_disabled(self) -> bool:
        """Bit 16 (119 ticket 001) -- a MOVE is active AND both angular and
        linear ``ShaperLimits`` axes are disabled -- see this class's own
        docstring and ``telemetry.h``'s ``kFlagFaultShapingDisabled`` doc
        comment for the full rationale."""
        return self._flag(_FLAG_FAULT_SHAPING_DISABLED)

    @property
    def fault_wheel_frozen_left(self) -> bool:
        """Bit 19 (129-002) -- the LEFT wheel was commanded a nonzero duty
        for N consecutive cycles with NO encoder change
        (``Hardware::MotorArmor::wedgeSuspect()``, GATED/motion-qualified --
        see this class's own docstring for why this is deliberately NOT
        ``fault_wedge_latch``)."""
        return self._flag(_FLAG_FAULT_WHEEL_FROZEN_LEFT)

    @property
    def fault_wheel_frozen_right(self) -> bool:
        """Bit 20 (129-002) -- same as ``fault_wheel_frozen_left``, RIGHT
        wheel."""
        return self._flag(_FLAG_FAULT_WHEEL_FROZEN_RIGHT)

    @property
    def fault_wheel_deficit_left(self) -> bool:
        """Bit 21 (130-005) -- Core::DifferentialDrive's LEFT-wheel deficit-flag
        policy: a sustained large speed error while BOTH the Stage C bias
        and the Stage B fast PID sit pinned at their configured authority
        -- there is no more correction to give, so the robot runs slow,
        loudly, rather than silently (issue 04's fail-loud observability
        mandate)."""
        return self._flag(_FLAG_FAULT_WHEEL_DEFICIT_LEFT)

    @property
    def fault_wheel_deficit_right(self) -> bool:
        """Bit 22 (130-005) -- same as ``fault_wheel_deficit_left``, RIGHT
        wheel."""
        return self._flag(_FLAG_FAULT_WHEEL_DEFICIT_RIGHT)

    @property
    def fault_stall_left(self) -> bool:
        """Bit 24 -- the LEFT wheel was commanded to move and did not, so the
        firmware HALTED the robot (Core::RobotLoop::haltOnStall). This is the
        robot jammed against something: a rail, a wall, the table edge.

        Distinct from the two neighbouring wheel faults, and the distinction
        is the whole point -- only this one means the robot stopped itself:

        - ``fault_wheel_frozen_left`` (19) is an ENCODER fault; the wheel may
          be spinning perfectly well.
        - ``fault_wheel_deficit_left`` (21) means the wheel IS turning, just
          under its commanded speed.
        - this means the wheel is being driven and is not turning at all.

        LATCHED: it survives the halt that ends the stall condition, and
        clears when the host commands a new motion (MOVE/WHEELS/GO_TO/ESTOP).
        So it always describes the motion that was just stopped, and a halt
        can never erase its own explanation before the host sees it.
        """
        return self._flag(_FLAG_FAULT_STALL_LEFT)

    @property
    def fault_stall_right(self) -> bool:
        """Bit 25 -- same as ``fault_stall_left``, RIGHT wheel."""
        return self._flag(_FLAG_FAULT_STALL_RIGHT)

    @property
    def stalled(self) -> bool:
        """Either wheel stalled -- the robot halted itself on an obstacle."""
        return self.fault_stall_left or self.fault_stall_right

    @property
    def event_deadman_expired(self) -> bool:
        return self._flag(_FLAG_EVENT_DEADMAN_EXPIRED)

    @property
    def event_boot_ready(self) -> bool:
        return self._flag(_FLAG_EVENT_BOOT_READY)

    @property
    def event_config_applied(self) -> bool:
        return self._flag(_FLAG_EVENT_CONFIG_APPLIED)

    @property
    def line_present(self) -> bool:
        return self._flag(_FLAG_LINE_PRESENT)

    @property
    def color_present(self) -> bool:
        return self._flag(_FLAG_COLOR_PRESENT)

    @property
    def line_fresh(self) -> bool:
        """``line`` was re-read on THIS cycle (vs. carried over).

        ``line_present`` says the value is meaningful; this says it is
        brand new. The firmware ticks line and colour on alternate cycles,
        so exactly one of ``line_fresh``/``color_fresh`` is set per frame
        and the other reading is up to one alternation old. Most consumers
        want ``line_present`` -- reach for this only when a sample must be
        just-measured (e.g. edge timing off the line sensor).
        """
        return self._flag(_FLAG_LINE_FRESH)

    @property
    def color_fresh(self) -> bool:
        """``color`` was re-read on THIS cycle -- see ``line_fresh``."""
        return self._flag(_FLAG_COLOR_FRESH)

    @classmethod
    def from_pb2(cls, telemetry: "telemetry_pb2.Telemetry") -> "TLMFrame":
        """Build a TLMFrame from a binary-plane ``pb2.Telemetry`` message
        (``ReplyEnvelope.body.tlm``, envelope.proto/telemetry.proto,
        frame v2 -- 115-003).

        Adapts telemetry.proto's wire shape onto this SAME dataclass shape
        pre-115 callers already read (``t``/``mode``/``seq``/``enc``/
        ``vel``/``pose``/``otos``/``twist``/``active``/``line``/``color``) —
        the decode INTERNALS move (nested readings, one flags bit-string,
        one ack slot), the dataclass's own public field names do not. This
        is an ADAPTER, not a redesign.

        Truncation matches the firmware's own text formatter exactly
        (``buildTlmFrame()``'s ``static_cast<int>``, i.e. truncate-toward-
        zero) — Python's ``int()`` on a float does the same.

        ``enc_left``/``enc_right``/``pose``/``twist`` are ALWAYS present on
        the wire (no presence gate, message-typed fields with proto3
        zero-value defaults when genuinely absent) — populated
        unconditionally, unlike pre-115's ``has_enc``/``has_vel``/
        ``has_pose``/``has_twist``-gated decode. ``otos``/``otos_reading``/
        ``line``/``color`` stay gated, now on ``flags`` bits (0/13/14)
        instead of ``has_otos``-style bool fields.

        Permanent gaps unchanged by this rewrite (telemetry.proto declares
        no matching field): ``wedge``, ``encpose``, ``otos_health``,
        ``ekf_rej``, ``cmd_vel`` (lives on ``TelemetrySecondary`` — own
        cadence, own decode path, untouched by 115-003).
        """
        frame = cls()
        frame.t = telemetry.now
        frame.mode = _DRIVE_MODE_CHAR.get(telemetry.mode, "I")
        frame.seq = telemetry.seq
        frame.flags = int(telemetry.flags)
        frame.active = bool(frame._flag(_FLAG_ACTIVE))

        frame.enc_left = EncoderReading.from_pb2(telemetry.enc_left)
        frame.enc_right = EncoderReading.from_pb2(telemetry.enc_right)
        frame.enc = (int(frame.enc_left.position), int(frame.enc_right.position))
        frame.vel = (int(frame.enc_left.velocity), int(frame.enc_right.velocity))

        # 124-008 (issue §B3): pose/twist/otos are sint32+scale on the wire
        # now -- apply the wire scale FIRST (raw -> real mm/rad/s, matching
        # EncoderReading/OtosReading.from_pb2()'s own convention), then this
        # dataclass's own historical mm/cdeg/mrad-per-s int convention.
        frame.pose = (
            int(telemetry.pose.x * _POSITION_SCALE),
            int(telemetry.pose.y * _POSITION_SCALE),
            int(telemetry.pose.h * _HEADING_SCALE * _ANGLE_SCALE),
        )
        frame.twist = (
            int(telemetry.twist.v_x * _VELOCITY_SCALE),
            int(telemetry.twist.omega * _OMEGA_SCALE * 1000.0),
        )

        if frame.otos_present:
            frame.otos_reading = OtosReading.from_pb2(telemetry.otos)
            frame.otos = (
                int(telemetry.otos.x * _POSITION_SCALE),
                int(telemetry.otos.y * _POSITION_SCALE),
                int(telemetry.otos.heading * _HEADING_SCALE * _ANGLE_SCALE),
            )

        if frame.line_present:
            frame.line = _unpack_channels4(int(telemetry.line))
        if frame.color_present:
            frame.color = _unpack_channels4(int(telemetry.color))

        # 123-004 (migrated from TelemetrySecondary, 122-003): always
        # populated, no presence gate.
        frame.cycle_busy = int(telemetry.cycle_busy)
        frame.cycle_period = int(telemetry.cycle_period)

        # Core::DifferentialDrive's unified wheel-speed controller (130-005) -- always
        # populated, no presence gate (see this dataclass's own field
        # comments above).
        frame.duty_per_speed_left = float(telemetry.duty_per_speed_left)
        frame.duty_per_speed_right = float(telemetry.duty_per_speed_right)
        frame.bias_left = float(telemetry.bias_left)
        frame.bias_right = float(telemetry.bias_right)
        frame.pid_left = float(telemetry.pid_left)
        frame.pid_right = float(telemetry.pid_right)

        # Bounded ack ring (120, ADDITIVE) -- ALWAYS populated (may be
        # empty), oldest-pushed first, matching the wire's own push/evict
        # order (telemetry.cpp's ack() doc comment). 124-008 (issue §B4)
        # deleted the single "freshest ack" scalar slot (ack_corr/ack_err)
        # this used to also populate -- the ring is the only ack source
        # now, and each element is a plain packed int (AckEntry deleted).
        frame.acks = [AckEntry.from_ring_entry(entry) for entry in telemetry.acks]

        return frame


def wheel_frozen_reason(frame: "TLMFrame") -> "str | None":
    """Which wheel(s), if any, `frame`'s wheel-frozen fault flags (129-002,
    wheel-frozen-fault-flag-in-telemetry.md; ``TLMFrame.
    fault_wheel_frozen_left``/``fault_wheel_frozen_right``, flags bits
    19/20) report frozen RIGHT NOW -- ``None`` if neither is set. One of
    ``"LEFT"``/``"RIGHT"``/``"LEFT + RIGHT"``, naming the wheel(s) exactly
    the way the source issue's own acceptance criterion asks the GUI
    banner to ("the GUI shows a red banner naming which wheel").

    The single shared helper for both host-side consumers of this signal
    (the TestGUI banner and ``planner.tour.run_tour()``'s abort-on-flag
    check) -- kept HERE, not in ``testgui/``, because ``planner/`` must
    never import ``testgui/`` (see ``tour.py``'s own module docstring) and
    both already import ``TLMFrame`` from this module.
    """
    left = bool(frame.fault_wheel_frozen_left)
    right = bool(frame.fault_wheel_frozen_right)
    if left and right:
        return "LEFT + RIGHT"
    if left:
        return "LEFT"
    if right:
        return "RIGHT"
    return None


@dataclass
class ParsedResponse:
    """Structured representation of a single text-plane response line.

    Retained as generic line-parsing infrastructure (``parse_response()``
    below) — not itself a "verb" method targeting a specific
    ``CommandEnvelope`` oneof arm, so out of 104-002's dead-verb-deletion
    scope. No ``NezhaProtocol`` method in this file constructs one any
    more (the binary-only P4 command plane has no per-command
    ``ReplyEnvelope`` line to parse this way) — surviving callers are
    outside this file (e.g. relay-transport EVT/keepalive line handling).
    """
    tag: str          # "OK", "ERR", "EVT", "TLM", "CFG", "ID"
    tokens: list[str] = field(default_factory=list)  # plain tokens after tag
    kv: dict[str, str] = field(default_factory=dict) # key=value pairs
    corr_id: str | None = None                       # trailing #<id>, if any
    raw: str = ""                                    # original stripped line
    tlm: "TLMFrame | None" = None                     # binary-sourced frame, if any


# ---------------------------------------------------------------------------
# Module-level parse functions (can be used without a NezhaProtocol instance)
# ---------------------------------------------------------------------------

_RESPONSE_TAGS = frozenset(("OK", "ERR", "EVT", "TLM", "CFG", "ID"))


def _strip_relay(line: str) -> str:
    """Strip relay prefix characters and surrounding whitespace."""
    return line.strip().lstrip("<# ").strip()


def parse_response(line: str) -> ParsedResponse | None:
    """Parse one text-plane response line into a ParsedResponse, or None if
    unrecognised.

    Handles relay prefix stripping, optional trailing '#<id>' correlation
    token, and key=value pair extraction. See ``ParsedResponse``'s own
    docstring for why this parser survives 104-002's dead-verb sweep.
    """
    s = _strip_relay(line)
    if not s:
        return None

    parts = s.split()
    if not parts:
        return None

    tag = parts[0].upper()
    if tag not in _RESPONSE_TAGS:
        return None

    rest = parts[1:]

    # Extract trailing corr_id: '#' followed by digits only.
    corr_id: str | None = None
    if rest and rest[-1].startswith("#") and rest[-1][1:].isdigit():
        corr_id = rest[-1][1:]
        rest = rest[:-1]

    # Parse key=value pairs; remainder are plain positional tokens.
    kv: dict[str, str] = {}
    plain: list[str] = []
    for tok in rest:
        if "=" in tok and not tok.startswith("="):
            k, _, v = tok.partition("=")
            kv[k] = v
        else:
            plain.append(tok)

    return ParsedResponse(
        tag=tag,
        tokens=plain,
        kv=kv,
        corr_id=corr_id,
        raw=s,
    )


def tlm_drop_rate(frames: "list[TLMFrame]") -> float:
    """Estimate the TLM frame drop rate from a sequence of TLMFrame objects.

    Uses the ``seq`` field (D10, firmware 028-005+) to detect gaps.  The
    uint16 seq counter wraps at 65535; wrap-around is handled correctly.

    Returns the fraction of expected sequence numbers that are absent:
      0.0 — no drops detected (or fewer than 2 frames, or no seq fields).
      1.0 — every possible intermediate frame was dropped.

    Returns 0.0 for fewer than 2 frames or when all ``seq`` fields are None
    (pre-D10 firmware).

    Args:
        frames: List of TLMFrame objects (in order received).
    """
    seq_frames = [f for f in frames if f.seq is not None]
    if len(seq_frames) < 2:
        return 0.0

    expected_span = 0
    drops = 0
    for i in range(1, len(seq_frames)):
        prev = seq_frames[i - 1].seq
        curr = seq_frames[i].seq
        # Gap accounting with uint16 wrap-around (modulo 65536).
        gap = (curr - prev) & 0xFFFF  # type: ignore[operator]
        expected_span += gap
        if gap > 1:
            drops += gap - 1

    if expected_span == 0:
        return 0.0
    return drops / expected_span


# ---------------------------------------------------------------------------
# Config key <-> (ConfigGroupTarget, field name) mapping (132-014, replacing
# 097-002's _DRIVETRAIN_KEYS/_MOTOR_PID_KEYS Patch-field tables). NezhaProtocol.
# set_config() keeps the same flat "wire key" vocabulary the retired text
# plane and the retired ConfigDelta/*ConfigPatch surface both used -- but the
# binary plane now addresses a value by (ConfigGroupTarget, protobuf field
# number) via SetConfigField (robot_config.proto, 132-012), not a per-SLICE
# Patch message. This table is the translation: flat key -> the
# (ConfigGroupTarget, field_name) pair set_config_field() resolves to a wire
# field number.
#
# ekfQxy/ekfQtheta/ekfROtosXy/ekfROtosTheta -- DROPPED, not migrated: the old
# DrivetrainConfigPatch's EKF process/measurement-noise fields have NO
# successor field anywhere in robot_config.proto (confirmed by reading the
# whole schema -- Geometry carries trackwidth/rotational_slip/rotation
# calibration only, Estimator carries the fusion weights only). They were
# already permanently non-functional before this ticket (DrivetrainConfigPatch
# had no Configurator::apply() branch at all -- sprint.md's own Problem
# section, "trackwidth/rotational_slip/the EKF noise pair have had no working
# wire path for some unknown span of sprints") -- this migration cannot invent
# a field the schema does not declare, so these four keys simply stop being
# recognized (the same "unknown key -> no wire traffic" outcome any other
# bogus key already produced, not a behavior regression).
#
# tw/rotSlip -- KEPT, now targeting GEOMETRY (boot-only per configurator.h's
# own re-appliability table: "trackWidth has no post-construction setter
# anywhere"). A push still round-trips and gets the honest ERR_NOT_LIVE this
# sprint's whole point is to surface, rather than silently dropping the key
# host-side -- the SAME non-functional outcome the old DrivetrainConfigPatch
# arm always had (ERR_UNIMPLEMENTED, per the Problem section above), now
# reported by name instead of by omission. GEOMETRY is also one of the sim's
# own justified BootOverrides divergences (trackWidth) -- see sim_loop.py's
# configure_from_robot(), which deliberately does NOT select "tw"/"rotSlip"
# into its own live-push field set for exactly this reason, even though
# set_config()/the SET verb still recognize them for a human/hardware caller.
#
# pid.kp/ki/kff/iMax/kaw -- REPOINTED (130-005, unchanged by this ticket) onto
# Core::DifferentialDrive's unified wheel-speed controller's Stage B fast-PID gains
# (WheelControl.pid_kp/pid_ki/pid_kaff/pid_i_max/pid_max), a LIVE,
# PERSISTED target (configurator.h's own re-appliability table) -- these are
# genuinely wired, unlike tw/rotSlip above.
#
# ml/mr -- MOTORS.travel_calib_left/travel_calib_right. Disambiguated by
# FIELD now, not by a `side` sub-message field the way the old
# MotorConfigPatch was -- each is its own independent SetConfigField push.
_SET_KEY_TARGETS: dict[str, "tuple[int, str]"] = {
    "tw": (robot_config_pb2.GEOMETRY, "trackwidth"),
    "rotSlip": (robot_config_pb2.GEOMETRY, "rotational_slip"),
    "ml": (robot_config_pb2.MOTORS, "travel_calib_left"),
    "mr": (robot_config_pb2.MOTORS, "travel_calib_right"),
    "pid.kp": (robot_config_pb2.WHEEL_CONTROL, "pid_kp"),
    "pid.ki": (robot_config_pb2.WHEEL_CONTROL, "pid_ki"),
    "pid.kff": (robot_config_pb2.WHEEL_CONTROL, "pid_kaff"),
    "pid.iMax": (robot_config_pb2.WHEEL_CONTROL, "pid_i_max"),
    "pid.kaw": (robot_config_pb2.WHEEL_CONTROL, "pid_max"),
    # pid.posErrMax -- 133-002. Stage B's I term is a POSITION term, and
    # this is the clamp on its INPUT, in millimetres. It is a SEPARATE
    # domain from pid.iMax above (which clamps the same term's OUTPUT, in
    # mm/s) -- both are live and both matter; setting one is not setting
    # the other. See robot_config.proto's WheelControl.pos_err_max.
    "pid.posErrMax": (robot_config_pb2.WHEEL_CONTROL, "pos_err_max"),
    # Stall detection (2026-08-08). stall.speed is the measured-speed ceiling
    # below which a wheel counts as not turning, stall.demand the commanded
    # floor above which we are genuinely asking for motion, stall.window the
    # sustain time before the firmware HALTS the robot. See
    # robot_config.proto's WheelControl for why this is a different fault
    # from deficit/wheelFrozen.
    "stall.speed": (robot_config_pb2.WHEEL_CONTROL, "stall_speed"),
    "stall.demand": (robot_config_pb2.WHEEL_CONTROL, "stall_demand"),
    "stall.window": (robot_config_pb2.WHEEL_CONTROL, "stall_window"),
}

# PlannerConfigPatch/CONFIG_PLANNER -- DELETED (115-003, gut-to-minimal-
# firmware S1 motion-stack excision): minSpeed/headingKp/headingKd/
# distanceKp/arriveDwell all patched PlannerConfigPatch (config.proto),
# deleted wholesale alongside Motion::Executor/Core::Pilot, the subsystems
# that read them. There is no live config target left for any of the five
# -- they are simply no longer valid set_config() keys (returns the same
# "unknown key" outcome as any other bogus key).
#
# sTimeout -- DELETED (116-001, MOVE protocol cutover): patched ConfigDelta's
# bare `watchdog` oneof arm (uint32 sTimeout, the pre-116 StreamingDrive
# Watchdog window), which is itself deleted -- every Move is now
# self-bounding (its own stop condition or required `timeout`), so the
# separate deadman/watchdog window this key configured is gone along with
# `Core::Deadman`. `sTimeout` is simply no longer a valid set_config() key --
# returns the same "unknown key" outcome any other bogus key already
# produced.
_ALL_SET_KEYS = frozenset(_SET_KEY_TARGETS)


# _CONFIG_GROUP_NAMES (132-011) -- ConfigGroupTarget wire value -> the
# group's own name, shared verbatim by BOTH robot_config_pb2 (the real
# compiled message class, e.g. robot_config_pb2.Drive) and
# robot_config_generated (the generated pydantic class, e.g.
# robot_config_generated.Drive) -- ticket 002's own field-descriptor walk
# emits both from the SAME robot_config.proto message names, so one name
# resolves the class on either side; get_config() (below) is the only
# caller. robot_config_pb2.ConfigGroupTarget's CONFIG_GROUP_UNSPECIFIED (0)
# is deliberately absent -- it is never a real target.
_CONFIG_GROUP_NAMES: dict[int, str] = {
    robot_config_pb2.GEOMETRY: "Geometry",
    robot_config_pb2.MOTORS: "Motors",
    robot_config_pb2.DRIVE: "Drive",
    robot_config_pb2.WHEEL_CONTROL: "WheelControl",
    robot_config_pb2.PLANNER: "Planner",
    # PLANNER_SHAPER (132-017): split out of PLANNER -- see
    # robot_config.proto's PlannerShaper message header comment.
    robot_config_pb2.PLANNER_SHAPER: "PlannerShaper",
    robot_config_pb2.OTOS: "Otos",
    robot_config_pb2.ESTIMATOR: "Estimator",
}


# ---------------------------------------------------------------------------
# Config provenance and verified push (133-006,
# A-live-config-push-is-wiped-by-the-next-reconnect.md).
# ---------------------------------------------------------------------------


class ConfigNotVerified(RuntimeError):
    """A ``verify=True`` config push did not land as sent.

    Raised by ``set_config_field()``/``set_config_group()`` when the push was
    rejected outright (no ack / NAK) *or* when the ack said OK but the
    read-back disagrees with what was sent. Both are the same failure from a
    caller's point of view -- the robot is not running what the caller thinks
    it is running -- and both used to be reported by returning ``None``, which
    is easy to not check.

    132-019 caught exactly this class of defect (a whole bench measurement
    computed against config the robot had silently discarded) and only because
    that script happened to read back by hand. Raising is what stops the next
    one depending on the same luck. ``wheel_control_tuning.TuningNotConfirmed``
    is the same idea, hand-rolled for one group before this existed.
    """


# Provenance (133-006): the ConfigSource enum lives on the ConfigSnapshot
# REPLY, never inside a config group -- a group field would be emitted into
# the pydantic model and data/robots/robot_config.schema.json too, and a file
# can never carry a runtime-assigned value. See robot_config.proto's own
# ConfigSource comment for the full reasoning.
CONFIG_SOURCE_NAMES: dict[int, str] = {
    robot_config_pb2.CONFIG_SOURCE_UNSPECIFIED: "UNSPECIFIED",
    robot_config_pb2.CONFIG_SOURCE_BAKED: "BAKED",
    robot_config_pb2.CONFIG_SOURCE_LIVE: "LIVE",
    robot_config_pb2.CONFIG_SOURCE_PERSISTED: "PERSISTED",
}


class ConfigReadback(NamedTuple):
    """One ``GetConfig`` answer: a group's current values AND where they came
    from (``get_config_snapshot()``).

    ``source`` is a ``robot_config_pb2.ConfigSource`` value -- ``BAKED`` (the
    robot-JSON values compiled in at boot), ``LIVE`` (a wire push landed on
    this group in the current power cycle), or ``PERSISTED`` (restored at boot
    from the flash tuning snapshot). ``UNSPECIFIED`` means the firmware
    predates provenance: proto3 omits a zero-valued field from the wire, so an
    older robot's reply simply carries no source at all rather than lying.

    Provenance is per GROUP, not per robot: ``DRIVE`` can be ``LIVE`` while
    ``PLANNER`` is still ``BAKED``, which is why a single global flag was
    rejected -- it would have to lie about one of them.
    """

    target: int
    values: Any
    source: int

    @property
    def source_name(self) -> str:
        """``source`` as a human-readable string, for logs and gate output."""
        return CONFIG_SOURCE_NAMES.get(self.source, f"UNKNOWN({self.source})")


def _format_config_value(value: Any) -> str:
    """Format a set_config() kwarg value into the SAME string shape the
    text plane's set_config() already produced -- floats to 6 significant
    digits, everything else via str(). Reused as-is from the pre-097-002
    text implementation (the formatting rule itself did not change)."""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


# ---------------------------------------------------------------------------
# Move builder support (116-001, MOVE protocol cutover) -- shared by
# NezhaProtocol.move_twist()/move_wheels(), which differ only in which
# Move.velocity oneof arm (twist/wheels) they build. Move.stop is itself a
# oneof (time/distance/angle, envelope.proto) -- both host builders expose it
# as three separate, mutually-exclusive keyword-only args (stop_time/
# stop_distance/stop_angle) rather than a single generic "kind+value" arg, so
# each carries its own unit as a `# [unit]` tag on its own parameter (project
# naming convention, .claude/rules/coding-standards.md) instead of one
# ambiguous-unit parameter whose meaning depends on a second value.
# ---------------------------------------------------------------------------

def _build_move_stop_kwargs(*, stop_time: float | None, stop_distance: float | None,
                            stop_angle: float | None) -> dict[str, float]:
    """Validate and translate a move_twist()/move_wheels() caller's
    stop_time/stop_distance/stop_angle kwargs into the single
    ``{"time"|"distance"|"angle": value}`` kwarg ``envelope_pb2.Move()``'s
    own ``stop`` oneof constructor expects.

    Raises ``ValueError`` (no wire traffic) unless EXACTLY ONE of the three
    is given -- ``Move.stop`` is a oneof, so zero is an underspecified Move
    and more than one is unrepresentable on the wire."""
    candidates = {"time": stop_time, "distance": stop_distance, "angle": stop_angle}
    given = {k: v for k, v in candidates.items() if v is not None}
    if len(given) != 1:
        raise ValueError(
            "move requires exactly one stop condition (stop_time/"
            f"stop_distance/stop_angle), got {sorted(given)!r}")
    (key, value), = given.items()
    return {key: float(value)}


# GoTo.frame (envelope.proto) -- 0=WORLD (OTOS/SEED frame), 1=ROBOT (resolved
# once, firmware-side, at acceptance). Matches Core::RobotLoop::handleGoto()'s
# own goTo.frame check and src/tests/bench/goto_otos.py's own
# FRAME_WORLD/FRAME_ROBOT exactly -- see NezhaProtocol.go_to() below.
GOTO_FRAME_WORLD = 0
GOTO_FRAME_ROBOT = 1


# ---------------------------------------------------------------------------
# NezhaProtocol
# ---------------------------------------------------------------------------

class NezhaProtocol:
    """Binary wire-protocol adapter for the P4 single-loop Nezha firmware.

    Owns a SerialConnection and exposes one method per firmware command group
    (``move``/``stop``/``config`` — ``move_twist()``/``move_wheels()`` build
    the ``move`` arm's two velocity variants) plus telemetry read accessors.
    All response parsing delegates to module-level parse_* functions so
    callers can reuse them on lines received through other paths (streaming
    generators).
    """

    def __init__(self, conn: SerialConnection) -> None:
        self._conn = conn

    # ------------------------------------------------------------------
    # Connection delegation
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._conn.is_open

    @property
    def mode(self) -> str | None:
        return self._conn.mode

    def _send_envelope(self, envelope: "envelope_pb2.CommandEnvelope",
                       read_timeout: int = 500,  # [ms]
                       ) -> "envelope_pb2.ReplyEnvelope | None":
        """Send ``envelope``; return the decoded ``ReplyEnvelope`` (or
        ``None`` on timeout/not-connected), normalizing the two different
        ``send_envelope()`` return shapes this tree's two connection
        backends use.

        ``SerialConnection.send_envelope()`` (``robot_radio/io/
        serial_conn.py``) returns a dict --
        ``{"sent": ..., "mode": ..., "reply": ReplyEnvelope | None}`` --
        because a real serial link's request/reply is genuinely
        asynchronous (a background reader thread fills a corr-id-keyed
        queue that could just as easily be filled by an unrelated frame
        first). A historical ctypes sim connection backend used to return
        the decoded ``ReplyEnvelope`` (or ``None``) DIRECTLY instead, since
        that sim call was already synchronous -- see this method's own git
        history for the reconciliation this reflects. 108-006: that
        backend is deleted; its ctypes successor
        (``robot_radio.io.sim_loop.SimLoop``) is a ``TwistTransport``
        implementation, not a ``SerialConnection``-shaped object
        ``NezhaProtocol`` wraps, so this dict-vs-direct reconciliation is
        now purely a ``SerialConnection`` implementation detail this method
        still normalizes defensively.
        """
        result = self._conn.send_envelope(envelope, read_timeout=read_timeout)
        if isinstance(result, dict):
            return result.get("reply")
        return result

    def send(self, cmd: str, read_timeout: int = 500) -> dict:  # [ms]
        """Send a text-plane command, return raw response dict (for ad-hoc /
        pass-through). NOTE: the P4 firmware has no text-plane command
        parser (``main.cpp``'s dispatch switch decodes binary
        ``CommandEnvelope`` only) -- this passthrough survives as generic
        transport plumbing (``SerialConnection.send()``), not a verb this
        ticket's dead-arm sweep targets, but a text line sent through it
        reaches no live firmware handler."""
        return self._conn.send(cmd, read_timeout)

    def send_fast(self, cmd: str) -> None:
        """Fire-and-forget send with no response reading."""
        self._conn.send_fast(cmd)

    def read_lines(self, duration: int) -> list[str]:  # [ms]
        """Blocking read for up to duration milliseconds."""
        return self._conn.read_lines(duration)

    def read_pending_lines(self) -> list[str]:
        """Drain the pending queues without blocking."""
        return self._conn.read_pending_lines()

    # ------------------------------------------------------------------
    # Static parse helpers (reusable on raw lines from streaming callers)
    # ------------------------------------------------------------------

    @staticmethod
    def parse_response(line: str) -> ParsedResponse | None:
        """Parse a text-plane response line. Delegates to module-level
        parse_response()."""
        return parse_response(line)

    # ------------------------------------------------------------------
    # Config: SET one flat key (132-014, rewritten off the retired
    # ConfigDelta/*ConfigPatch surface). No longer a thin wrapper over a
    # deleted set_config_binary() -- fans directly over set_config_field()
    # below, one round trip per key, via the _SET_KEY_TARGETS table.
    # ------------------------------------------------------------------

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Send each ``key=value`` in *kwargs* as its own ``SetConfigField``
        push (``set_config_field()`` below), resolved through the flat
        ``_SET_KEY_TARGETS`` vocabulary (module level) -- the SAME ``SET
        key=value`` text-verb surface this method has always presented
        (``binary_bridge.py``'s ``SET`` verb, ``SimTransport``'s SET path,
        ``robot_radio.robot.nezha.Nezha.set_config()``), rebuilt on the new
        per-field wire primitive now that the ``ConfigDelta``/``*ConfigPatch``
        surface it used to build (``set_config_binary()``, 096-007) is
        deleted (132-013).

        Any kwarg key outside ``_ALL_SET_KEYS`` fails the WHOLE call
        (returns ``None``, no wire traffic at all) -- unchanged contract.
        Each recognized key becomes its OWN ``set_config_field()`` round trip
        (never batched into a single envelope -- there is no whole-group
        shape here to batch into: a caller wanting an atomic whole-group push
        from a complete source wants ``set_config_group()`` instead, not this
        method). If EVERY touched key's round trip Acks, the returned dict
        echoes the kwargs actually sent (formatted the same way the retired
        text plane formatted them) -- the binary Ack carries no per-key echo
        of its own, so this is the closest same-shape substitute, not a wire
        round trip of the applied value. Returns ``None`` the moment any one
        key's push fails (NAK or timeout) -- the keys already pushed before
        the failure are NOT rolled back (this was already true of the old
        multi-target ConfigDelta fan-out; unchanged posture, see this
        project's "transcribe, never re-derive; flag genuine gaps"
        discipline for why no new atomicity is invented here).
        """
        if not kwargs:
            return None
        if any(k not in _SET_KEY_TARGETS for k in kwargs):
            return None

        for key, value in kwargs.items():
            target, field_name = _SET_KEY_TARGETS[key]
            if self.set_config_field(target, field_name, float(value)) is None:
                return None

        return {key: _format_config_value(value) for key, value in kwargs.items()}

    # ------------------------------------------------------------------
    # Config: SET a whole group (132-014, Configurator::applyGroup()'s host
    # arm -- landed firmware-side by 132-008/009/013, never wired to a host
    # method until this ticket). The whole-group counterpart of
    # set_config_field() below: ONE envelope carries an entire group's worth
    # of values, decoded straight into Config::Robot with NO patch, no
    # presence flags, no merge (Configurator::applyGroup()'s own doc
    # comment) -- so a caller must supply EVERY field of *target*'s own
    # generated message, sourced from a COMPLETE object (the robot JSON,
    # via robot_config_generated.<Group>, or a prior get_config() read-back),
    # never a curated subset: an omitted field decodes to that field's
    # proto3 zero value and OVERWRITES whatever the group held before,
    # exactly the "development-mode ad-hoc single push" case
    # .claude/rules/configuration-discipline.md reserves for
    # set_config_field() instead. This is why set_config() (above), whose
    # whole job is a curated flat-key SUBSET, is built on set_config_field(),
    # never on this method.
    # ------------------------------------------------------------------

    def set_config_group(self, target: int, *,
                         read_timeout: int = 500,  # [ms]
                         verify: bool = False,
                         **fields: Any) -> "AckEntry | None":
        """Send ``SetConfigGroup{target, body}`` -- push *target*'s ENTIRE
        group in one frame, ``**fields`` supplying every field of that
        group's own generated message (``robot_config_pb2.<Group>``, e.g.
        ``proto.set_config_group(robot_config_pb2.OTOS, offset_x=-47.7,
        offset_y=0.0, offset_yaw=0.0, linear_scale=1.0275,
        angular_scale=0.987)``).

        ``target`` is a ``robot_config_pb2.ConfigGroupTarget`` value, the
        SAME vocabulary ``get_config()``/``set_config_field()`` use.
        ``body`` is built by constructing the target group's real compiled
        protobuf message from ``**fields`` and serializing it
        (``pb_cls(**fields).SerializeToString()``) -- the SAME wire
        encoding ``get_config()`` decodes on the read-back side, so a
        caller can round-trip a ``get_config()`` result's own field values
        straight back through this method.

        Rides the ack ring like every other CONFIG-arm SET (fires via
        ``send_envelope_fast()``, then polls for completion the SAME
        duck-typed way ``set_config_field()`` does). Returns the matched
        ``AckEntry`` on success, or ``None`` on an unknown ``target``, a
        field name ``pb_cls(**fields)`` does not recognize (``TypeError``
        from the compiled protobuf constructor), a timeout, or a NAK reply
        -- check ``ERR_NOT_LIVE`` (``target`` is a boot-only group --
        GEOMETRY/PLANNER) or ``ERR_BUSY`` (MOTORS, guarded while that side
        is in motion) by polling the ack directly instead, if the
        distinction matters to the caller.

        ``verify=True`` (133-006) makes the push SELF-CHECKING: after the ack,
        the group is read back and every field in ``**fields`` compared against
        what the robot reports. Anything short of "the robot is running exactly
        what was sent" raises ``ConfigNotVerified`` -- including the no-ack and
        NAK cases, which otherwise return ``None`` and are easy to not check.
        Bench tooling should pass it; see ``set_config_field()``'s own note.
        """
        group_name = _CONFIG_GROUP_NAMES.get(target)
        if group_name is None:
            if verify:
                raise ConfigNotVerified(
                    f"set_config_group({target}): unknown target, nothing sent")
            return None
        pb_cls = getattr(robot_config_pb2, group_name)
        try:
            group_msg = pb_cls(**fields)
        except (TypeError, ValueError) as exc:
            if verify:
                raise ConfigNotVerified(
                    f"set_config_group({group_name}): {exc}, nothing sent") from exc
            return None

        request = robot_config_pb2.SetConfigGroup(
            target=target, body=group_msg.SerializeToString())
        envelope = envelope_pb2.CommandEnvelope(config=request)
        corr_id = self._conn.send_envelope_fast(envelope)
        poll_ack = getattr(self._conn, "poll_ack", None)
        ack = poll_ack(corr_id, timeout=read_timeout) if poll_ack is not None \
            else self.wait_for_ack(corr_id, timeout=read_timeout)
        if ack is None or not ack.ok:
            if verify:
                raise ConfigNotVerified(
                    f"set_config_group({group_name}) was REJECTED "
                    f"(ack={ack!r}) -- the robot is not running these values")
            return None
        if verify:
            # Only the fields actually supplied are checked. A whole-group push
            # replaces the group, so an omitted field lands at its proto3 zero
            # default -- real, but not something this caller asserted.
            self._verify_pushed(
                target, {name: float(value) for name, value in fields.items()},
                read_timeout=read_timeout, what=f"set_config_group({group_name})")
        return ack

    # ------------------------------------------------------------------
    # Config: GET (132-011, GetConfig/ConfigSnapshot wire read-back) --
    # the machinery ``set_config_binary()``'s own docstring flags as
    # long-standing gaps: unlike every other CONFIG-arm call above (whose
    # outcome rides the ack ring, since the current firmware never sends a
    # synchronous ``ReplyEnvelope{ok:...}``/``{err:...}``), GetConfig's
    # firmware handler (``RobotLoop::handleGetConfig()``) genuinely DOES
    # reply synchronously -- a group's worth of values has no room in a
    # 4-deep ack-ring entry. ``get_config()`` therefore uses the BLOCKING
    # ``send_envelope()``/``_send_envelope()`` path instead, exactly the
    # machinery this class's own docstring already names
    # (``SerialConnection.send_envelope()`` + its corr-id-keyed
    # ``_reply_queues``, built for -- per that class's own docstring --
    # "OK/ERR/CFG replies").
    # ------------------------------------------------------------------

    def get_config(self, target: int, *,
                    read_timeout: int = 500,  # [ms]
                    ) -> Any | None:
        """Send ``GetConfig{target}``, return the target group's CURRENT
        value as a typed, GENERATED pydantic model (``robot_config_
        generated.<Group>`` -- ticket 002's own generated model, never a
        raw dict), or ``None`` on an unknown target, a timeout, a
        not-connected connection, or an ``err`` reply (e.g.
        ``ERR_BADARG`` for a malformed target -- see ``RobotLoop::
        handleGetConfig()``'s own doc comment).

        ``target`` is a ``robot_config_pb2.ConfigGroupTarget`` value
        (``robot_config_pb2.DRIVE``, ``robot_config_pb2.GEOMETRY``, ...).
        Read-back is NOT gated by re-appliability the way a SET would be
        (a future ticket's ``set_config_group()``) -- every robot-config
        group reads back, including the boot-only ones (GEOMETRY/
        PLANNER): ``Configurator::encodeSnapshot()`` (firmware) is
        deliberately not gated by the same ``isLiveConfigurable()`` table
        ``applyGroup()``/``install()`` use for writes.

        The returned pydantic instance is built generically from the
        REAL protobuf message's own field descriptor
        (``robot_config_pb2.<Group>.FromString(reply.cfg.body)``) --
        every field the wire group carries copies straight across, since
        ticket 002's pydantic model and the real compiled protobuf
        message are generated from the exact same robot_config.proto
        field list and therefore always agree field-for-field.
        """
        snapshot = self.get_config_snapshot(target, read_timeout=read_timeout)
        return None if snapshot is None else snapshot.values

    def get_config_snapshot(self, target: int, *,
                            read_timeout: int = 500,  # [ms]
                            ) -> "ConfigReadback | None":
        """``get_config()`` plus PROVENANCE (133-006): returns a
        ``ConfigReadback(target, values, source)``, or ``None`` on exactly the
        same failure set ``get_config()`` documents.

        ``get_config()`` is implemented on top of this and returns only
        ``.values``, so every existing caller is unaffected -- there is one
        wire round trip and one implementation, not two.

        ``source`` answers "is the robot running what I pushed":
        ``CONFIG_SOURCE_LIVE`` for a group a wire push landed on during this
        power cycle, ``CONFIG_SOURCE_BAKED`` for the compiled-in robot-JSON
        values, ``CONFIG_SOURCE_PERSISTED`` for one restored at boot out of
        the flash tuning snapshot. ``CONFIG_SOURCE_UNSPECIFIED`` means the
        firmware predates provenance rather than that the answer is unknown.

        Scope, and it is deliberately narrow: for the groups that are not
        flash-persisted (``DRIVE``, ``PLANNER_SHAPER``), ``LIVE`` survives
        only while the robot stays powered. After a power cycle they read
        ``BAKED`` by definition -- which is the honest report of a value that
        is genuinely gone, and the point is that the loss is now VISIBLE
        instead of silent.
        """
        group_name = _CONFIG_GROUP_NAMES.get(target)
        if group_name is None:
            return None

        request = robot_config_pb2.GetConfig(target=target)
        envelope = envelope_pb2.CommandEnvelope(get_config=request)
        reply = self._send_envelope(envelope, read_timeout=read_timeout)
        if reply is None or reply.WhichOneof("body") != "cfg":
            return None

        pb_cls = getattr(robot_config_pb2, group_name)
        pydantic_cls = getattr(robot_config_generated, group_name)
        pb_obj = pb_cls.FromString(reply.cfg.body)
        values = pydantic_cls(**{
            field.name: getattr(pb_obj, field.name) for field in pb_obj.DESCRIPTOR.fields
        })
        return ConfigReadback(target=reply.cfg.target, values=values,
                              source=reply.cfg.source)

    # ------------------------------------------------------------------
    # Verified push (133-006, part 3) -- shared by set_config_field() and
    # set_config_group() below.
    # ------------------------------------------------------------------

    def _verify_pushed(self, target: int, expected: "dict[str, float]", *,
                       read_timeout: int,  # [ms]
                       what: str) -> None:
        """Read `target` back and raise ``ConfigNotVerified`` unless every
        name in `expected` matches what the robot reports.

        Compared with ``math.isclose`` at a tolerance sized for the wire, not
        for the arithmetic: every config value crosses as a 32-bit float, so a
        host ``float`` (a double) that round-trips through the wire is not
        bit-identical to what was sent and an ``==`` comparison would fail
        every push. ``rel_tol=1e-6`` is comfortably inside float32's ~1e-7
        resolution while still catching a value that landed nowhere (the
        actual failure being guarded against -- a discarded push reads back as
        the BAKED value, which is not a rounding distance away).
        """
        import math

        snapshot = self.get_config_snapshot(target, read_timeout=read_timeout)
        if snapshot is None:
            raise ConfigNotVerified(
                f"{what}: push acked but the read-back failed -- get_config"
                f"({_CONFIG_GROUP_NAMES.get(target, target)}) returned nothing, "
                f"so what the robot is running is UNKNOWN")

        mismatches: list[str] = []
        for name, sent in expected.items():
            got = getattr(snapshot.values, name, None)
            if got is None or not math.isclose(float(got), float(sent),
                                               rel_tol=1e-6, abs_tol=1e-9):
                mismatches.append(f"{name}: sent {sent!r}, robot reports {got!r}")
        if mismatches:
            raise ConfigNotVerified(
                f"{what}: push acked OK but did NOT land -- "
                + "; ".join(mismatches)
                + f" (group source: {snapshot.source_name})")

    # ------------------------------------------------------------------
    # Config: SET one field (132-012, SetConfigField / Configurator::
    # applyField()) -- the development-mode single-value push
    # ``.claude/rules/configuration-discipline.md`` carves out for bench
    # tuning sweeps ("we're going to do a sweep, so we should allow that").
    # Addressed by (``ConfigGroupTarget``, protobuf field number), never a
    # string key -- see ``robot_config.proto``'s own ``SetConfigField``
    # doc comment for the wire rationale (~11 B vs ~25 for a string key
    # like ``"wheel_gain_left_decel"``, and no hand-maintained name
    # vocabulary that can drift from what it names, the
    # ``pid.kff -> kaff`` class of bug).
    # ------------------------------------------------------------------

    def set_config_field(self, target: int, field_name: str, value: float, *,
                         read_timeout: int = 500,  # [ms]
                         verify: bool = False,
                         ) -> "AckEntry | None":
        """Send ``SetConfigField{target, field, value}`` -- write exactly
        ONE field inside ONE already-live robot-config group.

        ``field_name`` is resolved to its wire field NUMBER via the REAL
        compiled protobuf descriptor for ``target``'s own group message
        (``<Group>.DESCRIPTOR.fields_by_name[field_name].number``), so a
        human still types a name (``"wheel_gain_left_decel"``) and the wire
        still carries only a number — the resolution happens HERE, host-side,
        never as a hand-maintained string-to-number table that could drift
        from the schema it names.

        Rides the ack ring like every other CONFIG-arm SET
        (``set_config_binary()``'s own docstring — ``move``/``config``/
        ``stop``/``wheels``/``estop`` never get a synchronous
        ``ReplyEnvelope``) — fires via ``send_envelope_fast()``, then polls
        for the completion the SAME duck-typed way ``set_config_binary()``
        does (``poll_ack()`` for a ``Sim``-backed connection,
        ``wait_for_ack()`` otherwise).

        Returns the matched ``AckEntry`` (``ack.ok`` True, ``err_code`` 0)
        on success, or ``None`` on an unknown ``target``, an unknown
        ``field_name`` (no wire traffic in either case), a timeout, or a
        NAK reply — check ``ERR_BADARG`` (unknown field number on the
        firmware's own table, or a non-finite ``value``), ``ERR_RANGE``
        (a declared (min)/(max)/(abs_max) bound violated), ``ERR_NOT_LIVE``
        (``target`` is a boot-only group — GEOMETRY/PLANNER), or
        ``ERR_BUSY`` (MOTORS, guarded while that side is in motion) by
        calling ``wait_for_ack()``/``poll_ack()`` directly instead, if the
        distinction matters to the caller.

        ``verify=True`` (133-006) reads the value back via ``get_config()``
        and raises ``ConfigNotVerified`` unless the robot reports exactly what
        was sent -- a rejection, a timeout, and an ack-OK-but-landed-nowhere
        all raise, rather than returning a ``None`` the caller may not check.

        This is what ``.claude/rules/configuration-discipline.md`` is relying
        on when it permits ad-hoc single-value pushes for bench tuning: the
        relaxation is safe *because* you can interrogate the robot and see
        what you pushed. Bench tooling should therefore pass ``verify=True``
        -- ``calibration.push._push_via_proto()`` and
        ``wheel_control_tuning.push_gains()`` do. The default stays False so
        that library/sim/REPL callers keep their existing return-code
        contract, and so a push costs one round trip unless a caller asks for
        two.
        """
        group_name = _CONFIG_GROUP_NAMES.get(target)
        if group_name is None:
            if verify:
                raise ConfigNotVerified(
                    f"set_config_field({target}, {field_name}): unknown target, "
                    f"nothing sent")
            return None
        pb_cls = getattr(robot_config_pb2, group_name)
        field_desc = pb_cls.DESCRIPTOR.fields_by_name.get(field_name)
        if field_desc is None:
            if verify:
                raise ConfigNotVerified(
                    f"set_config_field({group_name}, {field_name}): no such field, "
                    f"nothing sent")
            return None

        request = robot_config_pb2.SetConfigField(
            target=target, field=field_desc.number, value=float(value))
        envelope = envelope_pb2.CommandEnvelope(set_field=request)
        corr_id = self._conn.send_envelope_fast(envelope)
        poll_ack = getattr(self._conn, "poll_ack", None)
        ack = poll_ack(corr_id, timeout=read_timeout) if poll_ack is not None \
            else self.wait_for_ack(corr_id, timeout=read_timeout)
        if ack is None or not ack.ok:
            if verify:
                raise ConfigNotVerified(
                    f"set_config_field({group_name}, {field_name}={value!r}) was "
                    f"REJECTED (ack={ack!r}) -- the robot is not running this value")
            return None
        if verify:
            self._verify_pushed(
                target, {field_name: float(value)}, read_timeout=read_timeout,
                what=f"set_config_field({group_name}, {field_name})")
        return ack

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def move_twist(self, v_x: float, v_y: float, omega: float, *,
                   stop_time: float | None = None,       # [ms]
                   stop_distance: float | None = None,   # [mm]
                   stop_angle: float | None = None,       # [rad]
                   timeout: float,                        # [ms]
                   replace: bool = True,
                   move_id: int = 0) -> int:
        """Enqueue (or preempt-and-start) a bounded body-frame twist MOVE —
        one of the P4 wire's two ``Move`` velocity variants
        (``CommandEnvelope{move: Move{twist: MoveTwist{v_x, v_y, omega},
        ...}}}``, envelope.proto arm 21, 116-001 MOVE protocol cutover).
        Supersedes the deleted 103-era ``twist()`` (bare ``v_x``/``omega`` +
        deadman-arming ``duration``): every ``Move`` is now bounded by its
        own stop condition and a required ``timeout`` backstop instead of a
        separate watchdog module (``Core::Deadman`` no longer exists).

        ``v_y`` is accepted and wire-forwarded but ignored server-side on
        this differential build (``MoveTwist.v_y``'s own doc comment) —
        pass ``0.0`` unless a future holonomic drivetrain needs it.

        Exactly ONE of ``stop_time``/``stop_distance``/``stop_angle`` selects
        this Move's stop condition (``Move``'s ``stop`` oneof) — elapsed
        time since activation, |path arc length| since activation (encoder
        odometry), or |heading change| since activation (encoder odometry),
        respectively. Passing zero or more than one raises ``ValueError``,
        no wire traffic sent — the oneof can carry only one.

        ``timeout`` is the REQUIRED safety backstop (envelope.proto: "<=0 ->
        ERR_BADARG") that fires the Move even if the stop condition can
        never be reached (e.g. stalled wheels) — validated host-side
        (``ValueError`` for a non-positive value) to avoid a wasted wire
        round trip for a command the firmware would reject anyway.

        ``replace`` selects queue semantics against ``Core::MoveQueue`` (1
        active + 4 pending): ``True`` (the default — matches every existing
        caller's own pre-Move "just drive this now" usage) flushes pending
        and preempts the active Move, starting this one immediately;
        ``False`` enqueues behind the active Move (``ERR_FULL`` if 4 already
        pending). TWO HARDWARE FACTS about ``move_id`` (measured 2026-08-14):
        the firmware DEDUPS accepted ids in a 16-entry ring that OUTLIVES the
        host session -- a resent id is acked err=0 and silently ignored, which
        is idempotent-resend behaviour, so a NEW session reusing an old id has
        its Move swallowed with a success ack. Use ids unique across sessions
        (or 0, which is never deduped). And the id survives the wire only
        mod 2**28: ids >= 268,435,456 come back truncated in the completion
        ack and will never match what was sent.

        ``move_id`` is echoed back in this Move's own COMPLETION
        ack (``Move.id`` — distinct from the enqueue ack, which echoes this
        envelope's ``corr_id`` as usual); the default ``0`` is fine for a
        caller that does not need to distinguish completion acks.

        Fire-and-poll, NOT fire-and-wait (103-009, Decision 2's
        "telemetry-only return path", unchanged by the Move cutover): the
        P4 wire has no per-command synchronous ``ReplyEnvelope`` for
        ``move`` — this call's own ENQUEUE outcome arrives later, riding the
        ack ring inside a subsequent ``Telemetry`` push (see
        ``wait_for_ack()``). This method returns as soon as the bytes reach
        the wire; it never blocks waiting for a reply that will not come.

        Returns the corr_id assigned to this command — pass it to
        ``wait_for_ack()`` to confirm the firmware accepted it. Raises
        ``ConnectionError`` if not connected (``send_envelope_fast()``'s own
        not-open contract).
        """
        stop_kwargs = _build_move_stop_kwargs(
            stop_time=stop_time, stop_distance=stop_distance, stop_angle=stop_angle)
        if timeout <= 0:
            raise ValueError(f"move_twist(): timeout must be > 0, got {timeout!r}")
        move = envelope_pb2.Move(
            twist=envelope_pb2.MoveTwist(v_x=v_x, v_y=v_y, omega=omega),
            timeout=timeout, replace=replace, id=move_id, **stop_kwargs)
        envelope = envelope_pb2.CommandEnvelope(move=move)
        return self._conn.send_envelope_fast(envelope)

    def move_wheels(self, v_left: float, v_right: float, *,
                    stop_time: float | None = None,       # [ms]
                    stop_distance: float | None = None,   # [mm]
                    stop_angle: float | None = None,       # [rad]
                    timeout: float,                        # [ms]
                    replace: bool = True,
                    move_id: int = 0) -> int:
        """Enqueue (or preempt-and-start) a bounded per-wheel-speed MOVE —
        the ``Move`` velocity variant's OTHER branch alongside
        ``move_twist()`` (``CommandEnvelope{move: Move{wheels: MoveWheels{
        v_left, v_right}, ...}}}``, envelope.proto arm 21). Stages directly
        through ``Drive::setWheels()`` firmware-side — never translated
        through a twist round trip (sprint 116's architecture-update.md
        Decision 3) — the bench rig's own per-motor-pair driving idiom
        (``.clasi/knowledge/bench-test-rig-layout.md``).

        ``stop_time``/``stop_distance``/``stop_angle``/``timeout``/
        ``replace``/``move_id`` share the SAME contract as ``move_twist()``'s
        own (exactly one stop condition; ``timeout`` required and validated
        > 0 host-side; ``replace`` defaults ``True``) — see that method's
        docstring for the full rationale; not re-derived here.

        Fire-and-poll, the SAME shape as ``move_twist()``/``stop()`` (103-009
        Decision 2's "telemetry-only return path"): this call writes the
        bytes and returns immediately; its ENQUEUE outcome rides the ack
        ring (``wait_for_ack()``).

        Returns the corr_id assigned to this command. Raises
        ``ConnectionError`` if not connected; raises ``ValueError`` for a
        missing/ambiguous stop condition or a non-positive ``timeout``.
        """
        stop_kwargs = _build_move_stop_kwargs(
            stop_time=stop_time, stop_distance=stop_distance, stop_angle=stop_angle)
        if timeout <= 0:
            raise ValueError(f"move_wheels(): timeout must be > 0, got {timeout!r}")
        move = envelope_pb2.Move(
            wheels=envelope_pb2.MoveWheels(v_left=v_left, v_right=v_right),
            timeout=timeout, replace=replace, id=move_id, **stop_kwargs)
        envelope = envelope_pb2.CommandEnvelope(move=move)
        return self._conn.send_envelope_fast(envelope)

    def move(self, *, v_x: float = 0.0, v_y: float = 0.0, omega: float = 0.0,
             v_left: float | None = None, v_right: float | None = None,
             stop_time: float | None = None,       # [ms]
             stop_distance: float | None = None,   # [mm]
             stop_angle: float | None = None,       # [rad]
             timeout: float,                        # [ms]
             replace: bool = True, id: int | None = None) -> int:
        """Single-entry-point ``Move`` builder mirroring
        ``robot_radio.io.sim_loop.SimLoop.move()``'s own kwargs exactly
        (testgui-motion-paths-dead-after-move-cutover fix, planner.tour
        revival) -- ``planner.tour``'s ``MoveTransport`` Protocol calls
        `.move(**kwargs)` on whatever `.protocol` a transport exposes;
        ``_HardwareTransport.protocol`` returns THIS class, so without this
        method a live hardware connection could not run a tour (only
        ``SimTransport.protocol`` -- a ``SimLoop``, which already had
        ``.move()`` -- could). A thin dispatcher over the two methods this
        class already has: a velocity variant of ``v_left``/``v_right``
        (BOTH given) calls ``move_wheels()``; the default (``v_x``/``v_y``/
        ``omega``, ``v_left``/``v_right`` both ``None``) calls
        ``move_twist()``. Raises ``ValueError`` if only one of
        ``v_left``/``v_right`` is given -- mirrors ``SimLoop.move()``'s own
        guard.

        ``id`` maps to ``move_twist()``/``move_wheels()``'s own
        ``move_id`` parameter (``Move.id`` -- the key THIS Move's own
        COMPLETION ack echoes, per ``docs/protocol-v4.md`` section 7.2);
        defaults to ``0`` (their own default) when omitted. UNLIKE
        ``SimLoop.move()``, this does NOT also become the envelope's own
        ``corr_id`` -- ``move_twist()``/``move_wheels()`` auto-assign that
        separately (``send_envelope_fast()``'s own connection-scoped
        counter), so the RETURNED value here is that auto-assigned
        envelope ``corr_id`` (the ENQUEUE ack's own key), not ``id``. A
        caller polling for a Move's own completion (e.g. ``planner.tour``)
        must poll on ``id`` itself, never this return value -- see
        ``MoveTransport``'s own docstring (``planner/tour.py``) for why
        that distinction is transparent to a tour.

        ``stop_time``/``stop_distance``/``stop_angle``/``timeout``/
        ``replace`` share the SAME contract as ``move_twist()``'s own --
        see that method's docstring. Raises ``ConnectionError`` if not
        connected; raises ``ValueError`` for a missing/ambiguous stop
        condition, a non-positive ``timeout``, or a lone
        ``v_left``/``v_right``.
        """
        move_id = id if id is not None else 0
        if v_left is not None or v_right is not None:
            if v_left is None or v_right is None:
                raise ValueError(
                    "move(): v_left and v_right must both be given for a "
                    "wheels Move (got only one)")
            return self.move_wheels(
                v_left, v_right, stop_time=stop_time, stop_distance=stop_distance,
                stop_angle=stop_angle, timeout=timeout, replace=replace, move_id=move_id)
        return self.move_twist(
            v_x, v_y, omega, stop_time=stop_time, stop_distance=stop_distance,
            stop_angle=stop_angle, timeout=timeout, replace=replace, move_id=move_id)

    def wheels(self, v_left: float, v_right: float, duration: float,  # [mm/s] [mm/s] [ms]
               *, move_id: int = 0) -> int:
        """Drive the two wheels at fixed velocities for ``duration`` ms —
        the dumb teleop primitive (``CommandEnvelope{wheels: Wheels{v_left,
        v_right, duration, id}}``, envelope.proto arm 22, wire verb
        ``WHEELS``; command-ingestion-ring-buffered-comms-subsystem-routing-
        two-stops.md §2).

        Routed firmware-side straight to ``Core::DifferentialDrive``, bypassing the
        planner entirely and superseding whatever it was doing. No profile,
        no shaping, no odometry stop condition — just a wheel pair held for
        a bounded window, then zero. This is the RIGHT call for teleop and
        for open-loop characterization; use ``move_wheels()`` instead when
        you want the planner's queue, ramps, and distance/angle stops.

        ``duration`` is REQUIRED and must be positive (validated host-side
        to save a wire round trip for a command the firmware would reject
        with ``ERR_BADARG``): a wheel command is always time-bounded. There
        is no separate ``timeout`` — the duration IS the backstop.

        **What that bound does and does not buy you (corrected 133-001).**
        This docstring used to add "so a dead host can never mean a
        runaway." That was false as shipped, and it was the reassurance
        that kept anyone from checking. MEASURED on ``vevov`` 2026-08-03,
        16/16 reproductions: a host that issued a stop ONCE and then went
        quiet got **936 mm of continued travel with no decay** — still
        going when the capture ended — and ``estop()`` failed 5 of 6
        attempts. The duration expires correctly inside ``Core::DifferentialDrive``;
        what failed was the expiry reaching the MOTOR. The Nezha brick
        physically latches its last commanded speed and does not reset on
        an nRF52 reset, so a single zero write that is lost on the bus is
        permanent, not transient.

        Sprint 133 ticket 001 closed both halves of that gap (a derived-idle
        safety arbitration step in ``Core::RobotLoop``, and arming the stop
        re-assertion window on the commanded nonzero→zero transition rather
        than on an encoder reading). Verified in sim and by construction as
        of that ticket; hardware re-verification on ``tovez`` is ticket 004.
        Until that lands, treat the time bound as a bound on what is
        COMMANDED, not as a guarantee about the wheels.

        ``move_id`` is echoed in this command's COMPLETION ack, which lands
        when the duration expires; the ENQUEUE ack echoes the returned
        corr_id, as always. Fire-and-poll, the same shape as every other
        command here: this call writes the bytes and returns immediately.

        Returns the corr_id assigned to this command. Raises
        ``ConnectionError`` if not connected; ``ValueError`` for a
        non-positive ``duration``.
        """
        if duration <= 0:
            raise ValueError(f"wheels(): duration must be > 0, got {duration!r}")
        envelope = envelope_pb2.CommandEnvelope(
            wheels=envelope_pb2.Wheels(v_left=v_left, v_right=v_right,
                                       duration=duration, id=move_id))
        return self._conn.send_envelope_fast(envelope)

    # ------------------------------------------------------------------
    # GO_TO / point-target navigation (135-004/135-007)
    # ------------------------------------------------------------------

    def go_to(self, x: float, y: float, *, frame: int,
              speed: float = 0.0,    # [mm/s] 0 = NavigatorLimits::speed config default
              arrive: float = 0.0,   # [mm] 0 = NavigatorLimits::defaultArrivalTolerance
              timeout: float,         # [ms]
              goto_id: int = 0) -> int:
        """Enqueue a bounded point-target GO_TO
        (``CommandEnvelope{go_to: GoTo{x, y, frame, speed, arrive, timeout,
        id}}``, envelope.proto arm 26, wire verb ``GO_TO``) -- hands
        ``Motion::Navigator`` (135-002/003) a world- or robot-frame target
        to drive to completion on its own, re-solving a tangent arc against
        live OTOS pose every internal cycle and issuing internal,
        replaceable Moves. Supersedes the host's own arc-solving/
        replace-throttling loop that used to live in
        ``pathplan.solver.solveArcToPoint()``/``pathplan.planner.
        ReplaceThreshold`` (both deleted 135-007) -- see
        ``pathplan.planner.gotoWorld()``/``gotoRobot()``/``followPath()``
        for the thin senders built on this method.

        Unrelated to the pre-104 ``go_to`` arm this file's own module
        docstring lists among 104-002's pruned pre-P4 methods (line ~45
        above) -- that was a different, long-retired wire arm from the
        ASCII-command era; this is a fresh method for the protocol-v5
        ``GO_TO`` verb (135-004), reusing the name because it is the right
        name, not a resurrection.

        x/y: [mm] target position. Interpreted per ``frame``:
            ``GOTO_FRAME_WORLD`` (0) is the world/OTOS/SEED frame;
            ``GOTO_FRAME_ROBOT`` (1) is the robot's own current body frame
            (+x forward, +y left) at the moment this command is ACCEPTED --
            resolved to world coordinates ONCE, firmware-side
            (``Core::RobotLoop::handleGoto()``), so the target does not
            chase the robot as it turns.
        speed: [mm/s] cruise-speed override; ``0.0`` (the default) falls
            open to the robot's own configured ``NavigatorLimits::speed``
            (configuration-discipline: every value the robot uses comes
            from this call or from the one robot config file, never a host
            constant).
        arrive: [mm] arrival-tolerance override; ``0.0`` falls open to
            ``NavigatorLimits::defaultArrivalTolerance``.
        timeout: [ms] REQUIRED whole-goto safety backstop (envelope.proto:
            "<=0 -> ERR_BADARG") -- fires (a fault-flagged completion ack)
            if the target is never reached, e.g. an OTOS disconnect
            outlasting ``Motion::Navigator``'s own bounded dead-reckoning
            window (SUC-005). Validated host-side (``ValueError`` for a
            non-positive value) to avoid a wasted wire round trip for a
            command the firmware would reject anyway.
        goto_id: echoed in the ONE completion ack this goto ends with (Done
            or Aborted) -- distinct from the ENQUEUE ack, which echoes this
            envelope's own ``corr_id`` as usual, exactly like
            ``move_twist()``'s ``move_id`` vs. its own ``corr_id``. The
            default ``0`` is fine for a caller that does not need to match
            the completion ack.

        Fire-and-poll, the same shape as ``move_twist()``/``move_wheels()``
        (103-009 Decision 2's "telemetry-only return path"): this call
        writes the bytes and returns immediately. Returns the corr_id
        assigned to this command -- pass it to ``wait_for_ack()`` to
        confirm the firmware accepted it, or watch the ack ring directly
        for a later entry keyed on ``goto_id``. Raises ``ConnectionError``
        if not connected; raises ``ValueError`` for a non-positive
        ``timeout``.

        Unlike ``move_twist()``/``move_wheels()``, there is no id-keyed
        acceptance/dedup ring on the firmware side for GO_TO
        (``RobotLoop::handleGoto()`` has none -- verified directly against
        ``src/firm/core/robot_loop.cpp``, 135-007): every GO_TO, retried or
        genuinely new, calls ``Motion::Navigator::start()`` and (re)starts
        navigation toward whatever target it carries. A RETRY of a lost
        enqueue ack is still safe to resend with the SAME ``goto_id`` (it
        carries the identical x/y/frame, so a redundant restart is
        harmless), but a caller streaming a SEQUENCE of genuinely different
        targets must allocate a FRESH ``goto_id`` for each one -- see
        ``pathplan.planner.MoveIdAllocator`` for the shared monotonic
        source that already guarantees this for every wire sender in that
        module.
        """
        if timeout <= 0:
            raise ValueError(f"go_to(): timeout must be > 0, got {timeout!r}")
        envelope = envelope_pb2.CommandEnvelope(
            go_to=envelope_pb2.GoTo(x=x, y=y, frame=frame, speed=speed,
                                    arrive=arrive, timeout=timeout, id=goto_id))
        return self._conn.send_envelope_fast(envelope)

    def estop(self) -> int:
        """Halt everything NOW (``CommandEnvelope{estop: Estop{}}``, wire
        verb ``ESTOP``) — a zero-field oneof arm that "cannot be malformed."

        Zeroes ``Core::DifferentialDrive``'s targets AND clears ``Motion::Planner``'s
        active + pending queue in the same cycle. The discarded queue
        entries get NO completion acks: you asked for a halt, not for a
        report that the things you cancelled finished.

        **This is what ``stop()`` used to be.** ``stop()`` still exists but
        now means a PLANNED stop that enters the planner's queue and
        executes in sequence (command-ingestion-...-two-stops.md §2) — any
        caller that meant "halt now" belongs here.

        Fire-and-poll: writes the bytes and returns immediately; the
        outcome rides the ack ring (``wait_for_ack()``). Returns the
        corr_id assigned to this command. Raises ``ConnectionError`` if not
        connected.
        """
        envelope = envelope_pb2.CommandEnvelope(estop=envelope_pb2.Estop())
        return self._conn.send_envelope_fast(envelope)

    def calibrate_imu(self, samples: int = 0) -> int:
        """Re-run the OTOS gyro bias calibration, robot PARKED
        (``CommandEnvelope{calibrate: Calibrate{samples}}``, wire verb
        ``CALIBRATE``).

        The firmware refuses with ``ERR_BUSY`` unless both wheels are
        encoder-still and nothing is commanding velocity that cycle, and
        with ``ERR_NOT_CONFIGURED`` if no OTOS is present -- park (estop)
        and confirm stillness before calling. Tracking and the seeded pose
        SURVIVE: this recalibrates bias only, unlike a reboot.

        Exists because the chip otherwise calibrates exactly once, at
        boot, and a robot that boots while being handled drives the whole
        session with a poisoned heading (measured on tovez 2026-08-08:
        +1.44 deg/s standstill drift after a mid-battery-swap boot; a
        still recalibration restored -0.006 deg/s).

        ``samples``: gyro samples to average, 1..255; 0 = firmware default
        (255, ~612ms of required stillness).

        Fire-and-poll like ``estop()``: returns the corr_id; the outcome
        (``err`` 0 on success) rides the ack ring (``wait_for_ack()``).
        On the radio relay, inbound commands are DROPPED outright --
        resend until the ack arrives.
        """
        if not 0 <= samples <= 255:
            raise ValueError(f"samples must be 0..255, got {samples}")
        envelope = envelope_pb2.CommandEnvelope(
            calibrate=envelope_pb2.Calibrate(samples=samples))
        return self._conn.send_envelope_fast(envelope)

    def stop(self, *, move_id: int = 0) -> int:
        """Enqueue a PLANNED stop (``CommandEnvelope{stop: Stop{id}}``, wire
        verb ``STOP``): "come to a stop when you reach THIS point in the
        queued sequence."

        **MEANING CHANGED** (command-ingestion-ring-buffered-comms-
        subsystem-routing-two-stops.md §2). This used to be the panic stop.
        It is now an ordinary planner queue entry: it waits behind whatever
        is already queued, then ramps the wheels down at the decel ceiling
        and completes once the robot is actually at rest. For "halt
        everything now" call ``estop()`` instead.

        Two acks, two keys, exactly like ``move_twist()``: the ENQUEUE ack
        echoes the returned corr_id (or ``ERR_FULL`` if the planner's 5-deep
        queue is full), and the COMPLETION ack — the one that means the
        robot has actually stopped — echoes ``move_id``. Pass a distinct
        ``move_id`` when you want to wait for the stop to happen; the
        default ``0`` is fine when you only care that it was queued.

        Fire-and-poll: writes the bytes and returns immediately. Returns the
        corr_id assigned to this command. Raises ``ConnectionError`` if not
        connected.
        """
        envelope = envelope_pb2.CommandEnvelope(
            stop=envelope_pb2.Stop(id=move_id))
        return self._conn.send_envelope_fast(envelope)

    # ------------------------------------------------------------------
    # Config: config()/otos_config()/estimator_config() -- DELETED, 132-014
    # (patch-surface retirement, host migration). Each built and sent
    # exactly ONE ConfigDelta{*ConfigPatch} envelope -- config.proto's
    # entire message family was deleted wholesale by 132-013. Every caller
    # is retargeted onto set_config_group()/set_config_field()/get_config()
    # above:
    #   - config()'s flat drivetrain/motor keys -> set_config()/
    #     set_config_field() via _SET_KEY_TARGETS.
    #   - otos_config(linear_scale=...)/otos_config(angular_scale=...) ->
    #     set_config_field(robot_config_pb2.OTOS, "linear_scale"/
    #     "angular_scale", value) -- see binary_bridge.py's
    #     _handle_otos_patch() for the OL/OA retarget.
    #   - otos_config(init=True)/OI -- NO successor: robot_config.proto's
    #     Otos message carries offset_x/offset_y/offset_yaw/linear_scale/
    #     angular_scale only, no fire-and-forget "reinitialize the chip"
    #     trigger field (configurator.h's own OTOS re-appliability-table
    #     row: "its 6th field, init, was a fire-and-forget trigger with no
    #     Config::Robot-shaped successor and was never persisted either").
    #     A genuine, flagged capability gap this ticket does not invent a
    #     firmware fix for -- see binary_bridge.py's own _handle_otos_patch()
    #     doc comment for where OI lands on the wire-verb surface now.
    #   - estimator_config()'s weight_heading_otos/weight_omega_otos/
    #     staleness_ms -> set_config_field(robot_config_pb2.ESTIMATOR, ...)
    #     -- ESTIMATOR decodes but install() permanently returns
    #     ERR_UNIMPLEMENTED (no live consumer, configurator.h), so a push
    #     here is honestly rejected rather than landing nowhere silently
    #     (the old EstimatorConfigPatch's own "acks 0, lands nowhere" trap,
    #     closed).
    #   - estimator_config()'s a_max/a_decel/alpha_max/alpha_decel/j_max/
    #     yaw_jerk_max -> folded into robot_config.proto's Planner message
    #     (fields 2/3/5/6/7/8) at first, alongside the group's boot-only
    #     rest -- a real, temporary capability regression (132-002 through
    #     132-013): PLANNER as a WHOLE was boot-only (ERR_NOT_LIVE), so a
    #     live push of these six genuinely-live-tunable-before-132 fields
    #     got rejected. FIXED, 132-017 (JSON reshape ticket,
    #     stakeholder-sanctioned mid-sprint scope addition): split into
    #     their own PlannerShaper message/PLANNER_SHAPER ConfigGroupTarget,
    #     which IS live (Motion::Planner::applyShaperLimits() was already
    #     one of the-configuration-object.md's eight safely re-appliable
    #     setters) -- set_config_field(robot_config_pb2.PLANNER_SHAPER,
    #     ...) now lands the same way it did before sprint 132.
    # ------------------------------------------------------------------

    def wait_for_ack(self, corr_id: int, timeout: int = 500) -> "AckEntry | None":  # [ms]
        """Poll incoming ``Telemetry`` pushes' bounded ack ring for an entry
        matching ``corr_id``, for up to ``timeout`` ms. Returns the matched
        ``AckEntry``, or ``None`` if the deadline passes with no match —
        this wait is always bounded, never infinite.

        The ack-ring matcher (120, bench-single-ack-slot-observability-
        collapses-at-40ms.md — replaces the pre-120 single-scalar-slot
        matcher this method used): ``move_twist()``/``move_wheels()``/
        ``stop()``/``config()`` get no synchronous ``ReplyEnvelope`` of
        their own — their outcome rides ``Telemetry.acks`` (a depth-4
        ring, each entry a real, once-pushed ``Core::Telemetry::ack()``
        call) inside a subsequent ``Telemetry`` push. The pre-120 single
        slot (``ack_corr``/``ack_err``, valid iff ``flags`` bit 5) lost a
        command's ack the instant a LATER command's ack landed within the
        same primary period, before the host's next read — bench-measured
        as 12/43 lost transient acks at the real 40ms cycle / ~15Hz host
        read rate. The ring survives up to ``kAckRingDepth`` (12) OTHER
        acks landing before this one is read; only a burst of MORE than
        that many unread acks for corr_ids other than this one would still
        time this out (unchanged bounded-wait contract; retry covers even
        that rare case).

        DEPTH 4 -> 12 (command-ingestion-ring-buffered-comms-subsystem-
        routing-two-stops.md §1): the firmware now buffers inbound commands
        in a ring (``Core::kCmdRingDepth``) and drains the WHOLE ring in one
        control cycle, so a burst of N commands produces N acks inside a
        single frame. The ack ring was raised to match the command ring for
        exactly that reason -- at the old depth 4 a 5-command burst would
        have executed correctly but lost an ack, hanging a host that chains
        on completion acks.

        104-003: the actual poll/match/timeout loop is no longer inline
        here — it lives in ``SerialConnection.wait_for_ack()`` (see that
        method's own docstring for the full ring-scan algorithm) so every
        future caller reading telemetry directly off ``SerialConnection``
        — not just ``NezhaProtocol`` — gets the identical matching
        guarantee without a second copy of the algorithm. This method is a
        thin adapter: delegate to the shared implementation, then wrap the
        matched raw packed ``int`` ring entry (124-008: plain ``int``, not
        ``telemetry_pb2.AckEntry`` — that wire message is deleted, issue
        §B4) in this module's own ``AckEntry`` dataclass
        (``AckEntry.from_ring_entry()``).
        """
        matched_entry = self._conn.wait_for_ack(corr_id, timeout=timeout)
        if matched_entry is None:
            return None
        return AckEntry.from_ring_entry(matched_entry)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def read_binary_tlm_frames(self, duration: int) -> "list[TLMFrame]":  # [ms]
        """Block for up to ``duration`` ms, returning every binary telemetry
        frame received during that window as ``TLMFrame`` objects (097-003).

        Telemetry is always-on in the P4 design (no arming step) — reads
        ``SerialConnection.read_binary_tlm()`` (``_binary_tlm_queue``) and
        adapts each raw ``pb2.ReplyEnvelope`` via ``TLMFrame.from_pb2()``.
        """
        return [TLMFrame.from_pb2(reply.tlm)
                for reply in self._conn.read_binary_tlm(duration)]

    def read_pending_binary_tlm_frames(self) -> "list[TLMFrame]":
        """Non-blocking drain of every currently-queued binary telemetry
        frame as ``TLMFrame`` objects (097-003) -- the binary-plane
        counterpart of ``read_pending_lines()``.

        127-004: stamps each frame's ``recvTime`` (host ``time.monotonic()``,
        not wall clock) at the point it is drained here -- the ONE call site
        that populates it (see ``TLMFrame.recvTime``'s own docstring).
        """
        frames = []
        for reply in self._conn.drain_binary_tlm():
            frame = TLMFrame.from_pb2(reply.tlm)
            frame.recvTime = time.monotonic()
            frames.append(frame)
        return frames

    # ------------------------------------------------------------------
    # Telemetry mode control (125-003, telemetry-emit-policy-rebuild-spec.md
    # Part 4). Three thin fire-and-forget wrappers over ``send_fast()`` --
    # the SAME "no corr_id, no blocking read" plumbing every existing
    # cleartext-verb caller already uses for HELLO/PING/etc. (see
    # ``src/tests/bench/radio_bench_gate.py``'s own ``send_fast(verb)``
    # calls). No new retry/backoff behavior: a mode change's reply is the
    # `STATUS` line (readable via ``read_lines()``/``read_pending_lines()``,
    # like any other cleartext reply), and `TLM:NOW`'s reply is one binary
    # telemetry frame (readable via ``read_pending_binary_tlm_frames()``) --
    # neither is read synchronously here, mirroring the existing bare-`TLM`
    # request path this ticket extends (comms.h's own
    # ``takeTelemetryRequest()`` doc comment, pre-125-003).
    # ------------------------------------------------------------------

    def tlmOn(self) -> None:
        """Send `TLM:ON` -- switch the robot to streaming-always mode
        (issue Part 3's ``kOn``): unsolicited telemetry at cadence, moving
        or parked."""
        self.send_fast("TLM:ON")

    def tlmOff(self) -> None:
        """Send `TLM:OFF` -- suppress unsolicited telemetry (issue Part 3's
        ``kOff``). Command replies (acks) and bare/`TLM:NOW` requests still
        work; only the unsolicited stream stops."""
        self.send_fast("TLM:OFF")

    def tlmNow(self) -> None:
        """Send `TLM:NOW` -- request exactly one telemetry frame right now,
        without changing the current mode. An explicit-argument alias of a
        bare `TLM` line (comms.cpp's own ``dispatchLine()`` doc comment)."""
        self.send_fast("TLM:NOW")

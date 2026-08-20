// diffdrive_adapter.cpp — Protocol::DiffDriveAdapter implementation. See
// diffdrive_adapter.h for the class contract and the documented scope
// decisions (which config fields are wire-reachable, how telemetry is
// projected, why the flags word is a local layout).
#include "diffdrive_adapter.h"

#include <cmath>
#include <cstring>

namespace Protocol {

namespace {

using Config = DiffDrive::DifferentialDrive::Config;
using Status = DiffDrive::DifferentialDrive::Status;

// float -> int32_t with round-to-nearest (ties away from zero), not
// truncation — a wire consumer summing per-frame deltas should not see a
// systematic bias toward zero. std::lround is fine here: this file links
// into a host test shim's shared library AND is meant to build for the
// firmware target, and lroundf is available in newlib-nano.
int32_t roundToInt32(float v) { return static_cast<int32_t>(std::lround(v)); }

// The 15 `wheel_control.*` fields spec §7.3 names, mapped by wire name
// onto DifferentialDrive::Config's plain-float members. Declaration order
// matches the spec table so fieldName()'s bare-GET dump reads in the same
// order a human reading §7.3 would expect.
struct FieldEntry {
  const char* name;         // wire key -- excluded from the units rule
  float Config::*member;
};

constexpr FieldEntry kFields[] = {
    {"wheel_control.v_min", &Config::vMin},
    {"wheel_control.bias_max", &Config::biasMax},
    {"wheel_control.tau_adapt", &Config::tauAdapt},
    {"wheel_control.a_steady", &Config::aSteady},
    {"wheel_control.deficit_threshold", &Config::deficitThreshold},
    {"wheel_control.deficit_window", &Config::deficitWindow},
    {"wheel_control.pid_kp", &Config::kp},
    {"wheel_control.pid_ki", &Config::ki},
    {"wheel_control.pid_i_max", &Config::iMax},
    {"wheel_control.pid_kaff", &Config::kaff},
    {"wheel_control.pid_max", &Config::pidMax},
    {"wheel_control.pos_err_max", &Config::posErrMax},
    {"wheel_control.stall_speed", &Config::stallSpeed},
    {"wheel_control.stall_demand", &Config::stallDemand},
    {"wheel_control.stall_window", &Config::stallWindow},
};
constexpr size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);

const FieldEntry* findField(const char* name) {
  for (const auto& entry : kFields) {
    if (std::strcmp(name, entry.name) == 0) return &entry;
  }
  return nullptr;
}

// checkCommandable()'s refusals (differential_drive.cpp) collapse onto
// Result::kNotReady -- "not ready to accept a motion command" is what
// kRefusedUnconfigured/kRefusedNotBegun/kRefusedEstopped all mean from
// the wire's point of view, and adapter.h's Result enum has no dedicated
// estopped/unconfigured/not-begun codes of its own (matching
// protocol.md §3's "minimal set: enough to exercise a wheel kernel").
// kRefusedNonFinite maps to kBadArg for the same reason it can never
// actually fire from onWheels() below: the wire's own int parse
// (protocol_handler.cpp's parseInt32) already rejects anything that
// would produce a non-finite float, so this arm is defensive, not
// reachable in practice. No `default:` -- a future Status enumerator
// should trip -Wswitch here, the same posture protocol_handler.cpp's
// own resultCode() takes.
Result statusToResult(Status status) {
  switch (status) {
    case Status::kOk: return Result::kOk;
    case Status::kRefusedUnconfigured: return Result::kNotReady;
    case Status::kRefusedNotBegun: return Result::kNotReady;
    case Status::kRefusedEstopped: return Result::kNotReady;
    case Status::kRefusedNonFinite: return Result::kBadArg;
    case Status::kCadencePreserved: return Result::kOk;
  }
  return Result::kUnknown;  // unreachable with every enumerator handled
}

const char* tlmModeWireName(TlmMode mode) {
  switch (mode) {
    case TlmMode::kOff: return "off";
    case TlmMode::kPose: return "pose";
    case TlmMode::kFull: return "full";
    case TlmMode::kAuto: return "auto";
    case TlmMode::kBuffer: return "buffer";
    // kNow is never STORED into mode_ (see onTlm()) -- kept here only so
    // the switch stays exhaustive against a future TlmMode enumerator.
    case TlmMode::kNow: return "pose";
  }
  return "off";
}

// LOCAL flags layout -- deliberately NOT spec §6.5's bit numbers. See
// diffdrive_adapter.h's file header for why reusing those numbers would
// misrepresent what they mean for a library with no OTOS/line/colour/
// planner.
constexpr uint32_t kFlagReady = 1u << 0;
constexpr uint32_t kFlagEstopped = 1u << 1;
constexpr uint32_t kFlagLeaseExpired = 1u << 2;
constexpr uint32_t kFlagStallHalted = 1u << 3;
constexpr uint32_t kFlagConnectedLeft = 1u << 4;
constexpr uint32_t kFlagConnectedRight = 1u << 5;
constexpr uint32_t kFlagWedgeLeft = 1u << 6;
constexpr uint32_t kFlagWedgeRight = 1u << 7;

uint32_t computeFlags(const DiffDrive::DifferentialDrive::Output& out) {
  uint32_t flags = 0;
  if (out.ready) flags |= kFlagReady;
  if (out.estopped) flags |= kFlagEstopped;
  if (out.leaseExpired) flags |= kFlagLeaseExpired;
  if (out.stallHalted) flags |= kFlagStallHalted;
  if (out.connectedLeft) flags |= kFlagConnectedLeft;
  if (out.connectedRight) flags |= kFlagConnectedRight;
  if (out.wedgeLeft) flags |= kFlagWedgeLeft;
  if (out.wedgeRight) flags |= kFlagWedgeRight;
  return flags;
}

}  // namespace

DiffDriveAdapter::DiffDriveAdapter(DiffDrive::DifferentialDrive& drive,
                                   float countsPerLength,
                                   const Identity& identity)
    : drive_(drive),
      countsPerLength_(countsPerLength > 0.0f ? countsPerLength : 1.0f),
      identity_(identity) {
  snapshot_ = Snapshot{columns_, 0};
  // Hard-code the kernel's bring-up parameters (see kMaxDuty's doc comment
  // in the header) so constructing this adapter alone is enough for the
  // wrapped kernel's begin() to succeed -- no external caller has to arm
  // maxDuty/fullDutyVelocity/cyclePeriod out-of-band first. This runs
  // before the caller's own begin() call, and DifferentialDrive's setters
  // are live pre-begin, so ordering relative to any other config the
  // caller pushes (e.g. via onSet(), or its own direct kernel access)
  // does not matter.
  drive_.setMaxDuty(kMaxDuty)
      .setFullDutyVelocity(kFullDutyVelocity)
      .setCyclePeriod(kCyclePeriod);
}

void DiffDriveAdapter::setCountsPerLength(float countsPerLength) {
  if (countsPerLength > 0.0f) countsPerLength_ = countsPerLength;
}

void DiffDriveAdapter::identity(Identity& out) const { out = identity_; }

uint32_t DiffDriveAdapter::now() const { return drive_.output().now; }

void DiffDriveAdapter::status(StatusFields& out) const {
  const DiffDrive::DifferentialDrive::Output snapshot = drive_.output();
  out.ready = snapshot.ready;
  // "active" here means "a motion command is currently in effect" -- the
  // closest reading of spec §6.5 bit 2 ("a move is running") this
  // library's WHEELS-only, planner-free command surface can produce.
  out.active = snapshot.ready && !snapshot.estopped &&
               !snapshot.leaseExpired && !snapshot.stallHalted &&
               (snapshot.velocity != 0.0f || snapshot.twist != 0.0f);
  out.connLeft = snapshot.connectedLeft;
  out.connRight = snapshot.connectedRight;
  out.otos = false;  // no OTOS in this library
  out.wedge = snapshot.wedgeLeft || snapshot.wedgeRight;
  out.flags = computeFlags(snapshot);
  out.tlm = tlmModeWireName(mode_);
}

Result DiffDriveAdapter::onWheels(float left, float right, uint32_t duration,
                                  uint32_t /*id*/) {
  // The wire's own ceiling (spec §5.2), enforced here per
  // diffdrive_adapter.h's file header -- the handler holds no bounds
  // table to do this itself.
  if (duration > kWheelsDurationCeiling) return Result::kRange;

  // docs/design/protocol.md §4: scale [mm/s] by countsPerLength
  // [counts/mm] to [counts/s], then the half-sum/half-difference split
  // DiffDrive::drive() expects. twist is CCW-positive by construction:
  // a faster RIGHT wheel (right > left) makes (right - left) > 0, which
  // is a positive twist -- CCW, per diffdrive.md §3's own convention.
  // Swap left and right here and every twist sign inverts; that is
  // exactly the bug this project shipped once (see the twist-sign test
  // in tests/adapter/).
  const float countsLeft = left * countsPerLength_;    // [counts/s]
  const float countsRight = right * countsPerLength_;  // [counts/s]
  const float velocity = (countsLeft + countsRight) * 0.5f;  // [counts/s]
  const float twist = (countsRight - countsLeft) * 0.5f;     // [counts/s]

  const Status status = drive_.drive(velocity, twist, duration);
  return statusToResult(status);
}

Result DiffDriveAdapter::onStop(uint32_t /*id*/) {
  // STOP -> neutral(): a commanded stop through the full stop path
  // (docs/design/diffdrive.md §3.2). neutral() has no refusal path of
  // its own (it unconditionally replaces the mailbox with a neutral
  // command, even pre-begin/pre-configured/estopped), so this always
  // acks kOk -- matching the kernel's own unconditional acceptance.
  drive_.neutral();
  return Result::kOk;
}

void DiffDriveAdapter::onEstop() {
  // ESTOP -> estop(): latched zero, no ack ever (spec §8.2; handled by
  // ProtocolHandler::handleEstop(), which never calls this method's
  // return value at all -- onEstop() returns void for exactly that
  // reason, per adapter.h).
  drive_.estop();
}

bool DiffDriveAdapter::onGet(const char* name, float& out) const {
  const FieldEntry* entry = findField(name);
  if (entry == nullptr) return false;
  const Config cfg = drive_.config();
  out = cfg.*(entry->member);
  return true;
}

Result DiffDriveAdapter::onSet(const char* name, float value, uint32_t id) {
  (void)id;
  const FieldEntry* entry = findField(name);
  if (entry == nullptr) return Result::kUnknown;
  Config cfg = drive_.config();
  cfg.*(entry->member) = value;
  // setConfig() replaces the whole block; cyclePeriod is copied back
  // verbatim from the current config above, so the cadence-preservation
  // path (differential_drive.cpp's own kCadencePreserved) never
  // triggers from here -- a single-field SET never touches cadence.
  const Status status = drive_.setConfig(cfg);
  return statusToResult(status);
}

size_t DiffDriveAdapter::fieldCount() const { return kFieldCount; }

const char* DiffDriveAdapter::fieldName(size_t index) const {
  return index < kFieldCount ? kFields[index].name : "";
}

Result DiffDriveAdapter::onTlm(TlmMode mode) {
  // TLM:NOW is a one-shot request in the CURRENT subscription's shape,
  // not a new subscription (spec §6.1: "does not change mode") -- so it
  // is deliberately never stored into mode_. Everything else (including
  // AUTO/BUFFER, per this file's header comment) becomes the persisted
  // mode.
  if (mode != TlmMode::kNow) mode_ = mode;
  return Result::kOk;
}

const Snapshot& DiffDriveAdapter::buildSnapshot() {
  const DiffDrive::DifferentialDrive::Output out = drive_.output();
  seq_ = (seq_ + 1) & 0x7Fu;  // wraps at 128, spec §6.2

  size_t n = 0;
  columns_[n++] = Column{"seq", static_cast<int32_t>(seq_), false};
  columns_[n++] = Column{"now", static_cast<int32_t>(out.now), false};
  columns_[n++] = Column{"flags", static_cast<int32_t>(computeFlags(out)),
                         true};
  // [mm] -- counts -> mm through the one geometry factor this adapter
  // owns (docs/design/protocol.md §4.1: "convert counts to the wire's
  // units").
  columns_[n++] =
      Column{"posl", roundToInt32(out.positionLeft / countsPerLength_),
             false};
  columns_[n++] =
      Column{"posr", roundToInt32(out.positionRight / countsPerLength_),
             false};
  // [mm/s x10] -- same x10 integer-quantum convention spec §6.4 uses
  // for elv/erv, so a captured CSV reads the same way.
  columns_[n++] = Column{
      "vell", roundToInt32(out.velocityLeft / countsPerLength_ * 10.0f),
      false};
  columns_[n++] = Column{
      "velr", roundToInt32(out.velocityRight / countsPerLength_ * 10.0f),
      false};

  if (mode_ == TlmMode::kFull) {
    columns_[n++] =
        Column{"lambda", roundToInt32(out.lambda * 1000.0f), false};
    columns_[n++] = Column{"biasl", roundToInt32(out.biasLeft), false};
    columns_[n++] = Column{"biasr", roundToInt32(out.biasRight), false};
    columns_[n++] =
        Column{"cyc", static_cast<int32_t>(out.cycleCount), false};
  }

  snapshot_ = Snapshot{columns_, n};
  return snapshot_;
}

}  // namespace Protocol

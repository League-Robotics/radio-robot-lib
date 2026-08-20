// configurator.cpp -- Core::Configurator implementation. See configurator.h
// for the module's boundary.
#include "core/configurator.h"

#include <cmath>
#include <cstring>

#include "core/boot_calibration.h"
#include "config/boot_config.h"
#include "config/wire_v6_config_fields.h"
#include "messages/wire.h"

namespace Core {

namespace {

// isLiveConfigurable() -- the per-ConfigGroupTarget re-appliability gate,
// consulted BEFORE decoding anything, so a boot-only push leaves config_
// untouched rather than silently no-op'ing after a successful decode.
// See configurator.h's own class-level doc comment for the full table.
bool isLiveConfigurable(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE:
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
    case msg::ConfigGroupTarget::MOTORS:
    case msg::ConfigGroupTarget::OTOS:
    case msg::ConfigGroupTarget::ESTIMATOR:
      return true;
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
    case msg::ConfigGroupTarget::NAVIGATOR:
      // PLANNER_SHAPER/NAVIGATOR: read-back only now -- Motion::Planner/
      // Motion::Navigator, their only consumers, are deleted (the
      // exploratory-kernel rewrite). Same boot-only shape as GEOMETRY/
      // PLANNER.
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      return false;
  }
  return false;
}

// --- protocol v6 GET/SET support (spec section 7) -- see configurator.h's
// own class-level doc comment for the addressing-scheme rationale. ---

// findConfigField() -- linear scan over Config::WireV6::kConfigFieldTable
// (80 rows; sprint 137 ticket 001) for an exact, case-sensitive match.
// Names are lowercase "<group>.<field>" (spec section 7.3) -- no
// normalization here, a mismatched case is simply ERR_UNKNOWN.
const Config::WireV6::ConfigFieldEntry* findConfigField(const char* name, size_t nameLen) {
  for (uint16_t i = 0; i < Config::WireV6::kConfigFieldCount; ++i) {
    const Config::WireV6::ConfigFieldEntry& entry = Config::WireV6::kConfigFieldTable[i];
    const size_t entryLen = std::strlen(entry.name);
    if (entryLen == nameLen && std::memcmp(entry.name, name, nameLen) == 0) return &entry;
  }
  return nullptr;
}

// readConfigFieldRaw()/writeConfigFieldRaw() -- the table's own offset +
// FieldType, applied directly against Config::Robot's bytes (the SAME
// offsetof()-computed offsets Config::Robot's own generated members live
// at -- see wire_v6_config_fields.h's own header comment: "the SAME
// field-descriptor walk that already generates Config::Robot's own C++
// members"). GET's wire format is always decimal (spec section 7.2)
// regardless of the field's underlying storage type, so an integer field
// widens to float on read and rounds on write.
float readConfigFieldRaw(const Config::Robot& config, const Config::WireV6::ConfigFieldEntry& entry) {
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&config) + entry.offset;
  switch (entry.type) {
    case Config::WireV6::FieldType::kFloat:
      return *reinterpret_cast<const float*>(base);
    case Config::WireV6::FieldType::kI32:
      return static_cast<float>(*reinterpret_cast<const int32_t*>(base));
    case Config::WireV6::FieldType::kU32:
      return static_cast<float>(*reinterpret_cast<const uint32_t*>(base));
  }
  return 0.0f;
}

void writeConfigFieldRaw(Config::Robot& config, const Config::WireV6::ConfigFieldEntry& entry, float value) {
  uint8_t* base = reinterpret_cast<uint8_t*>(&config) + entry.offset;
  switch (entry.type) {
    case Config::WireV6::FieldType::kFloat:
      *reinterpret_cast<float*>(base) = value;
      return;
    case Config::WireV6::FieldType::kI32:
      *reinterpret_cast<int32_t*>(base) = static_cast<int32_t>(std::lround(value));
      return;
    case Config::WireV6::FieldType::kU32:
      *reinterpret_cast<uint32_t*>(base) = static_cast<uint32_t>(std::lround(value));
      return;
  }
}

// groupTargetForFieldName() -- the group prefix (everything before the
// first '.') mapped onto the SAME ConfigGroupTarget applyGroup()/
// applyField() address, so setFieldByName() can reuse isLiveConfigurable()/
// install(target) verbatim rather than a second, possibly-drifting notion
// of liveness. The 9 prefixes below are exactly
// wire_v6_config_fields.h's own §7.3 group table -- both sides are
// generated from protos/robot_config.proto's 9 wire-addressable groups
// (ticket 001), so this mapping cannot silently go stale field-by-field,
// only whole-group, which a new ConfigGroupTarget value would already
// need a matching case added everywhere else in this file.
msg::ConfigGroupTarget groupTargetForFieldName(const char* name, size_t nameLen) {
  size_t dot = 0;
  while (dot < nameLen && name[dot] != '.') ++dot;

  auto is = [&](const char* group) {
    const size_t groupLen = std::strlen(group);
    return groupLen == dot && std::memcmp(name, group, groupLen) == 0;
  };

  if (is("geometry")) return msg::ConfigGroupTarget::GEOMETRY;
  if (is("motors")) return msg::ConfigGroupTarget::MOTORS;
  if (is("drive")) return msg::ConfigGroupTarget::DRIVE;
  if (is("wheel_control")) return msg::ConfigGroupTarget::WHEEL_CONTROL;
  if (is("planner_shaper")) return msg::ConfigGroupTarget::PLANNER_SHAPER;
  if (is("planner")) return msg::ConfigGroupTarget::PLANNER;
  if (is("navigator")) return msg::ConfigGroupTarget::NAVIGATOR;
  if (is("otos")) return msg::ConfigGroupTarget::OTOS;
  if (is("estimator")) return msg::ConfigGroupTarget::ESTIMATOR;
  return msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED;
}

}  // namespace

void Configurator::stampSource(msg::ConfigGroupTarget target, msg::ConfigSource source) {
  const auto slot = static_cast<size_t>(target);
  if (slot == 0 || slot >= kGroupSourceSlots) return;
  groupSource_[slot] = source;
}

msg::ConfigSource Configurator::configSource(msg::ConfigGroupTarget target) const {
  const auto slot = static_cast<size_t>(target);
  if (slot == 0 || slot >= kGroupSourceSlots) return msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED;
  return groupSource_[slot];
}

Configurator::Configurator(Control::DifferentialDrive& drive, Hal::Motor& motorL,
                           Hal::Motor& motorR, Hal::Otos& otos,
                           Config::TuningStore* tuningStore)
    : drive_(drive),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      tuningStore_(tuningStore) {}

void Configurator::persistIfEligible(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      persistedTuning_.wheelControlTuned = true;
      persistedTuning_.wheelControlPidKp = config_.wheelControl.pid_kp;
      persistedTuning_.wheelControlPidKi = config_.wheelControl.pid_ki;
      persistedTuning_.wheelControlPidIMax = config_.wheelControl.pid_i_max;
      persistedTuning_.wheelControlPidKaff = config_.wheelControl.pid_kaff;
      persistedTuning_.wheelControlPidMax = config_.wheelControl.pid_max;
      break;

    case msg::ConfigGroupTarget::MOTORS:
      persistedTuning_.motorsTravelCalibTuned = true;
      persistedTuning_.motorsTravelCalibLeft = config_.motors.travel_calib_left;
      persistedTuning_.motorsTravelCalibRight = config_.motors.travel_calib_right;
      break;

    case msg::ConfigGroupTarget::OTOS:
      persistedTuning_.otosTuned = true;
      persistedTuning_.otosOffsetX = config_.otos.offset_x;
      persistedTuning_.otosOffsetY = config_.otos.offset_y;
      persistedTuning_.otosOffsetYaw = config_.otos.offset_yaw;
      persistedTuning_.otosLinearScale = config_.otos.linear_scale;
      persistedTuning_.otosAngularScale = config_.otos.angular_scale;
      break;

    case msg::ConfigGroupTarget::DRIVE:
    case msg::ConfigGroupTarget::ESTIMATOR:
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
    case msg::ConfigGroupTarget::NAVIGATOR:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // Not in the persisted-tuning precedent set -- configurator.h's own
      // re-appliability table's PERSISTENCE SCOPE note.
      return;
  }

  persistTuningIfChanged();
}

// Change-detection debounce: only write flash when the serialized snapshot
// actually differs from the last one written.
void Configurator::persistTuningIfChanged() {
  if (tuningStore_ == nullptr) return;

  Config::Blob blob = Config::serializeSnapshot(persistedTuning_);
  if (blob == lastPersistedBlob_) return;

  tuningStore_->save(Config::kConfigSchemaVersion, blob);
  lastPersistedBlob_ = blob;
}

void Configurator::reapplyPersistedTuning(const Config::TuningSnapshot& snapshot) {
  if (snapshot.wheelControlTuned) {
    config_.wheelControl.pid_kp = snapshot.wheelControlPidKp;
    config_.wheelControl.pid_ki = snapshot.wheelControlPidKi;
    config_.wheelControl.pid_i_max = snapshot.wheelControlPidIMax;
    config_.wheelControl.pid_kaff = snapshot.wheelControlPidKaff;
    config_.wheelControl.pid_max = snapshot.wheelControlPidMax;
    stampSource(msg::ConfigGroupTarget::WHEEL_CONTROL,
                msg::ConfigSource::CONFIG_SOURCE_PERSISTED);
    install(msg::ConfigGroupTarget::WHEEL_CONTROL);
  }

  if (snapshot.motorsTravelCalibTuned) {
    config_.motors.travel_calib_left = snapshot.motorsTravelCalibLeft;
    config_.motors.travel_calib_right = snapshot.motorsTravelCalibRight;
    stampSource(msg::ConfigGroupTarget::MOTORS, msg::ConfigSource::CONFIG_SOURCE_PERSISTED);
    install(msg::ConfigGroupTarget::MOTORS);
  }

  if (snapshot.otosTuned) {
    config_.otos.offset_x = snapshot.otosOffsetX;
    config_.otos.offset_y = snapshot.otosOffsetY;
    config_.otos.offset_yaw = snapshot.otosOffsetYaw;
    config_.otos.linear_scale = snapshot.otosLinearScale;
    config_.otos.angular_scale = snapshot.otosAngularScale;
    stampSource(msg::ConfigGroupTarget::OTOS, msg::ConfigSource::CONFIG_SOURCE_PERSISTED);
    install(msg::ConfigGroupTarget::OTOS);
  }

  persistedTuning_ = snapshot;
  lastPersistedBlob_ = Config::serializeSnapshot(persistedTuning_);
}

void Configurator::loadBaked(const Config::WheelCorrection* wheelCorrectionOverride) {
  config_.geometry = Config::defaultGeometryGroup();
  config_.motors = Config::defaultMotorsGroup();
  config_.drive = Config::defaultDriveGroup();
  config_.wheelControl = Config::defaultWheelControlGroup();
  config_.planner = Config::defaultPlannerGroup();
  config_.plannerShaper = Config::defaultPlannerShaperGroup();
  config_.otos = Config::defaultOtosGroup();
  config_.estimator = Config::defaultEstimatorGroup();
  config_.navigator = Config::defaultNavigatorGroup();

  if (wheelCorrectionOverride != nullptr) {
    config_.drive.wheel_gain_left_accel = wheelCorrectionOverride->gainLeftAccel;
    config_.drive.wheel_intercept_left_accel = wheelCorrectionOverride->interceptLeftAccel;
    config_.drive.wheel_gain_left_decel = wheelCorrectionOverride->gainLeftDecel;
    config_.drive.wheel_intercept_left_decel = wheelCorrectionOverride->interceptLeftDecel;
    config_.drive.wheel_gain_right_accel = wheelCorrectionOverride->gainRightAccel;
    config_.drive.wheel_intercept_right_accel = wheelCorrectionOverride->interceptRightAccel;
    config_.drive.wheel_gain_right_decel = wheelCorrectionOverride->gainRightDecel;
    config_.drive.wheel_intercept_right_decel = wheelCorrectionOverride->interceptRightDecel;
  }

  // Everything this function established is BAKED. Slot 0
  // (CONFIG_GROUP_UNSPECIFIED) is not a group and is deliberately skipped.
  for (size_t slot = 1; slot < kGroupSourceSlots; ++slot) {
    groupSource_[slot] = msg::ConfigSource::CONFIG_SOURCE_BAKED;
  }
}

// installDriveKernelConfig() -- see configurator.h's own doc comment: the
// one call DRIVE/WHEEL_CONTROL/MOTORS all share.
void Configurator::installDriveKernelConfig() {
  drive_.setConfig(Core::buildDriveKernelConfig(config_));
}

void Configurator::install() {
  installDriveKernelConfig();
  // PLANNER/PLANNER_SHAPER/NAVIGATOR: no install fan-out any more --
  // Motion::Planner/Motion::Navigator are deleted. config_.planner/
  // plannerShaper/navigator stay populated (loadBaked() above) for
  // read-back only.
}

msg::ErrCode Configurator::applyGroup(msg::ConfigGroupTarget target, const uint8_t* wire,
                                      size_t len) {
  if (!isLiveConfigurable(target)) return msg::ErrCode::ERR_NOT_LIVE;

  const auto wireLen = static_cast<uint16_t>(len);
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE: {
      msg::Drive decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.drive = decoded;
      break;
    }
    case msg::ConfigGroupTarget::WHEEL_CONTROL: {
      msg::WheelControl decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.wheelControl = decoded;
      break;
    }
    case msg::ConfigGroupTarget::MOTORS: {
      msg::Motors decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.motors = decoded;
      break;
    }
    case msg::ConfigGroupTarget::OTOS: {
      msg::Otos decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.otos = decoded;
      break;
    }
    case msg::ConfigGroupTarget::ESTIMATOR: {
      msg::Estimator decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.estimator = decoded;
      break;
    }
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
    case msg::ConfigGroupTarget::NAVIGATOR:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // Unreachable: isLiveConfigurable() already filtered these out
      // above. Kept as an explicit case so a future ConfigGroupTarget
      // value added without a matching arm fails to compile.
      return msg::ErrCode::ERR_NOT_LIVE;
  }

  stampSource(target, msg::ConfigSource::CONFIG_SOURCE_LIVE);

  const msg::ErrCode result = install(target);
  if (result == msg::ErrCode::ERR_NONE) persistIfEligible(target);
  return result;
}

msg::ErrCode Configurator::applyField(msg::ConfigGroupTarget target, uint16_t fieldNumber,
                                      float value) {
  if (!isLiveConfigurable(target)) return msg::ErrCode::ERR_NOT_LIVE;
  if (!std::isfinite(value)) return msg::ErrCode::ERR_BADARG;

  msg::wire::Result r{false, fieldNumber, msg::ErrCode::ERR_BADARG};
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE:
      r = msg::wire::setField(config_.drive, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      r = msg::wire::setField(config_.wheelControl, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::MOTORS:
      r = msg::wire::setField(config_.motors, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::OTOS:
      r = msg::wire::setField(config_.otos, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::ESTIMATOR:
      r = msg::wire::setField(config_.estimator, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
    case msg::ConfigGroupTarget::NAVIGATOR:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      return msg::ErrCode::ERR_NOT_LIVE;
  }
  if (!r.ok) return r.code;

  stampSource(target, msg::ConfigSource::CONFIG_SOURCE_LIVE);

  const msg::ErrCode result = install(target);
  if (result == msg::ErrCode::ERR_NONE) persistIfEligible(target);
  return result;
}

msg::ErrCode Configurator::install(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE:
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
    case msg::ConfigGroupTarget::MOTORS:
      // All three now rebuild and push the WHOLE kernel config -- see
      // configurator.h's own re-appliability table. Neither re-clamps
      // state that is already running out of newly-lowered bounds (the
      // kernel's own running integrator/adaptation bias) -- a live push
      // can leave that state stale until the next steady-gate
      // accumulation.
      installDriveKernelConfig();
      return msg::ErrCode::ERR_NONE;

    case msg::ConfigGroupTarget::OTOS:
      Core::configureOtos(otos_, config_);
      return msg::ErrCode::ERR_NONE;

    case msg::ConfigGroupTarget::ESTIMATOR:
      // PERMANENT, not a gap -- Core::StateEstimator was deleted long
      // before this rewrite and no successor exists. config_.estimator is
      // still decoded and read-back-correct; there is nothing here to fan
      // it out TO.
      return msg::ErrCode::ERR_UNIMPLEMENTED;

    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
    case msg::ConfigGroupTarget::NAVIGATOR:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // GEOMETRY/PLANNER were always boot-only. PLANNER_SHAPER/NAVIGATOR
      // join them here in this rewrite: their only consumers (Motion::
      // Planner::applyShaperLimits(), Core::configureNavigator()) are
      // deleted along with src/firm/motion/.
      return msg::ErrCode::ERR_NOT_LIVE;
  }
  return msg::ErrCode::ERR_NOT_LIVE;
}

msg::ErrCode Configurator::encodeSnapshot(msg::ConfigGroupTarget target,
                                          msg::ConfigSnapshot& out) const {
  out.target = target;
  out.source = configSource(target);
  uint16_t len = 0;
  switch (target) {
    case msg::ConfigGroupTarget::GEOMETRY:
      len = msg::wire::encode(config_.geometry, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::MOTORS:
      len = msg::wire::encode(config_.motors, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::DRIVE:
      len = msg::wire::encode(config_.drive, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      len = msg::wire::encode(config_.wheelControl, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::PLANNER:
      len = msg::wire::encode(config_.planner, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
      len = msg::wire::encode(config_.plannerShaper, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::OTOS:
      len = msg::wire::encode(config_.otos, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::ESTIMATOR:
      len = msg::wire::encode(config_.estimator, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::NAVIGATOR:
      len = msg::wire::encode(config_.navigator, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
    default:
      out.source = msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED;
      return msg::ErrCode::ERR_BADARG;
  }
  out.body_count = static_cast<uint8_t>(len);
  return msg::ErrCode::ERR_NONE;
}

// --- protocol v6 GET/SET (spec section 7) -- see configurator.h's own
// class-level doc comment. ---

uint16_t Configurator::fieldCount() { return Config::WireV6::kConfigFieldCount; }

const char* Configurator::fieldName(uint16_t index) {
  return Config::WireV6::kConfigFieldTable[index].name;
}

float Configurator::fieldValueAt(uint16_t index) const {
  return readConfigFieldRaw(config_, Config::WireV6::kConfigFieldTable[index]);
}

bool Configurator::getFieldByName(const char* name, size_t nameLen, float* out) const {
  const Config::WireV6::ConfigFieldEntry* entry = findConfigField(name, nameLen);
  if (entry == nullptr) return false;
  *out = readConfigFieldRaw(config_, *entry);
  return true;
}

msg::ErrCode Configurator::setFieldByName(const char* name, size_t nameLen, float value) {
  const Config::WireV6::ConfigFieldEntry* entry = findConfigField(name, nameLen);
  if (entry == nullptr) return msg::ErrCode::ERR_UNKNOWN;

  // NaN-before-range, per spec section 7.1's own explicit ordering: NaN
  // compares false against both `<` and `>`, so an unchecked NaN would
  // pass any bound check below.
  if (!std::isfinite(value)) return msg::ErrCode::ERR_BADARG;

  if (value < entry->min || value > entry->max) return msg::ErrCode::ERR_RANGE;

  writeConfigFieldRaw(config_, *entry, value);

  // "SET applies immediately where the field is live, and is stored
  // otherwise" (spec section 7.1) -- config_ is already written above
  // unconditionally; a live target additionally gets the SAME
  // stampSource()/install(target)/persistIfEligible() fan-out
  // applyGroup()/applyField() use. A boot-only or no-live-consumer target
  // (isLiveConfigurable() false, or install() itself returning something
  // other than ERR_NONE, e.g. ESTIMATOR's permanent ERR_UNIMPLEMENTED) is
  // NOT surfaced as a SET failure -- v6 has no ERR_NOT_LIVE/
  // ERR_UNIMPLEMENTED in its own SET error set (spec section 7.1 lists
  // only ERR_BUSY/ERR_RANGE/ERR_BADARG), unlike v5's applyGroup()/
  // applyField().
  const msg::ConfigGroupTarget target = groupTargetForFieldName(name, nameLen);
  if (target != msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED && isLiveConfigurable(target)) {
    stampSource(target, msg::ConfigSource::CONFIG_SOURCE_LIVE);
    if (install(target) == msg::ErrCode::ERR_NONE) persistIfEligible(target);
  }

  return msg::ErrCode::ERR_NONE;
}

}  // namespace Core

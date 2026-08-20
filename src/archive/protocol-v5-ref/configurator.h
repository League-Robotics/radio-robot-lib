// configurator.h -- Core::Configurator: the CONFIG command's whole
// lifecycle. One of the routing destinations Core::RobotLoop::
// routeCommand() dispatches to.
//
// Owns the one Config::Robot instance -- `config_` below. `loadBaked()`
// populates it from the generated, robot-JSON-baked Config::default*Group()
// functions (config/boot_config.h); `config()` returns it for read-back;
// `install()` fans its re-appliable groups out to the subsystems that own
// them.
//
// `applyGroup(target, wire, len)` decodes a whole group straight into
// `config_` (no patch, no presence flags, no merge) and `install(target)`
// fans out just that one group. `applyField(target, fieldNumber, value)`
// is the single-field counterpart.
//
// EXPLORATORY-KERNEL REWRITE (2026-08-15, differentialdrive-one-class-one-
// fiber-exploratory-worktree.md): PLANNER/PLANNER_SHAPER/NAVIGATOR no
// longer have a live consumer -- Motion::Planner and Motion::Navigator are
// DELETED along with the whole src/firm/motion/ tree, so this class no
// longer holds a Motion::Planner& or a Motion::NavigatorLimits&. All
// three groups are now boot-only for read-back purposes (config_.planner/
// plannerShaper/navigator still decode/encode correctly -- GET_CONFIG
// keeps working -- there is simply no install(target) fan-out to reach any
// more, same shape as GEOMETRY). DRIVE/WHEEL_CONTROL/MOTORS all now
// install through the SAME call, `Control::DifferentialDrive::setConfig(
// Core::buildDriveKernelConfig(config_))` (boot_calibration.h) -- MOTORS
// joins this group because MOTORS.travel_calib_left/right is now an INPUT
// to that conversion (mm<->counts), not a value pushed onto the motor leaf
// (Hal::Motor::applyTravelCalib() is deleted; the leaf is counts-native).
// MOTORS therefore loses its old ERR_BUSY "refuses while moving" guard --
// a rebuild-and-setConfig() push is exactly as live-safe as any
// DRIVE/WHEEL_CONTROL push already was (the kernel's own fiber snapshots
// config at each cycle start, config.h's own "config is live" doc
// comment), so there is no motion-safety reason left to refuse it.
//
// Every `ConfigGroupTarget` declares whether it is safely re-appliable at
// runtime or boot-only -- config that acks OK and silently does nothing is
// worse than config that is rejected. `applyGroup()`/`install(target)`
// consult this table (`isLiveConfigurable()`, configurator.cpp) before
// touching anything:
//
//   | target        | live? | install(target) reaches                        | notes |
//   |---------------|-------|--------------------------------------------------|-------|
//   | GEOMETRY      | NO    | -- (ERR_NOT_LIVE)                                 | trackWidth has no post-construction setter anywhere; rotation calibration installs once, at boot, via RobotLoop::configure() called directly from boot_wiring.cpp |
//   | PLANNER       | NO    | -- (ERR_NOT_LIVE)                                 | Motion::Planner is deleted; this group is read-back only now, same shape as GEOMETRY |
//   | PLANNER_SHAPER| NO    | -- (ERR_NOT_LIVE)                                 | Motion::Planner::applyShaperLimits() is deleted along with Motion::Planner; read-back only |
//   | DRIVE         | yes   | `Control::DifferentialDrive::setConfig(buildDriveKernelConfig(config_))` | Stage A per-wheel gain/intercept + crawl pulse, converted mm->counts. NOT persisted -- no old curated live-tuning wire message ever existed for these fields |
//   | WHEEL_CONTROL | yes   | SAME call as DRIVE                                | Stage B/C gains/bounds, converted mm->counts. PERSISTED -- these 5 fields (pid_kp/pid_ki/pid_i_max/pid_kaff/pid_max) are the direct successor of the old curated Motor live-tuning message's kp/ki/i_max/kff/kaw |
//   | MOTORS        | yes   | SAME call as DRIVE                                | travel_calib_left/right feed buildDriveKernelConfig()'s mm<->counts conversion now (the leaf itself has no config left to push -- fwd_sign/port/output_deadband/reversal_dwell were never live-appliable either). PARTIALLY PERSISTED -- travel_calib_left/right only, matching the old precedent |
//   | OTOS          | yes   | `Core::configureOtos()`                           | unchanged. PERSISTED IN FULL |
//   | ESTIMATOR     | yes   | -- (`ERR_UNIMPLEMENTED`, PERMANENT)               | unchanged -- Core::StateEstimator was deleted long before this rewrite; no successor exists |
//   | NAVIGATOR     | NO    | -- (ERR_NOT_LIVE)                                 | Motion::Navigator is deleted; this group is read-back only now, same shape as GEOMETRY |
//
// PERSISTENCE SCOPE unchanged: WHEEL_CONTROL (in full) + MOTORS.
// travel_calib_left/right + OTOS (in full).
//
// Named `Configurator`, not `Config`, because `Config::` is already a
// namespace in this tree.
//
// Boundary: inside -- what a decoded SetConfigGroup/SetConfigField MEANS
// and where its values land; outside -- decoding it (Core::Comms), routing
// it (RobotLoop), and acking it (RobotLoop, from this class's returned
// error code).
#pragma once

#include <cstddef>
#include <cstdint>

#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "config/robot.h"
#include "control/differential_drive.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"

namespace Core {

class Configurator {
 public:
  // All references are already-constructed modules; the composition root
  // owns construction and wiring order. tuningStore may be null (sim/test
  // roots): persistence disabled, everything else unchanged.
  Configurator(Control::DifferentialDrive& drive, Hal::Motor& motorL, Hal::Motor& motorR,
               Hal::Otos& otos, Config::TuningStore* tuningStore = nullptr);

  // reapplyPersistedTuning() -- main.cpp's own post-boot step
  // (RobotGraph::loadPersistedTuning(), boot_wiring.cpp): writes a loaded
  // TuningSnapshot's persisted fields into config_ (only for a group whose
  // own per-group "tuned" flag is set), then fans each touched group out
  // via the SAME install(target) a live wire push uses.
  void reapplyPersistedTuning(const Config::TuningSnapshot& snapshot);

  // config() -- read-back: one call, whole truth. Reflects whatever
  // loadBaked()/applyGroup() last wrote.
  const Config::Robot& config() const { return config_; }

  // mutableConfig() -- HOST-BUILD SIM PLUMBING ONLY. sim_ctypes.cpp's
  // sim_configure_drive() writes pushed duty_per_speed values into the
  // live config so SimHarness::syncPlantToConfig() (which derives the
  // synthetic plant's own gain from it) and the kernel never describe two
  // different robots. Production code changes config exclusively through
  // applyGroup()/applyField()/loadBaked(); nothing on the ARM target may
  // call this.
#ifdef HOST_BUILD
  Config::Robot& mutableConfig() { return config_; }
#endif

  // loadBaked() -- populates config_ from the generated, robot-JSON-baked
  // Config::default*Group() functions -- one assignment per
  // msg::ConfigGroupTarget, no derivation of its own. Idempotent.
  //
  // wheelCorrectionOverride (nullptr = no override, the hardware default):
  // when non-null, config_.drive's eight wheel_gain_*/wheel_intercept_*
  // fields are replaced by it AFTER the bake lands, so the whole
  // downstream fan-out sees the override rather than the baked robot-JSON
  // value. Only Core::composeRobot() passes one, for a sim-plant fixture
  // (see BootOverrides::wheelCorrection, boot_wiring.h).
  void loadBaked(const Config::WheelCorrection* wheelCorrectionOverride = nullptr);

  // install() -- the boot-time fan-out: pushes config_'s re-appliable
  // groups into the subsystems that own them. This is the BOOT-TIME
  // fan-out (every group, once, at construction) -- for a LIVE per-target
  // push see install(ConfigGroupTarget) below.
  void install();

  // applyGroup() -- the live wire push path. Decodes `wire`/`len` (one
  // group's own encoded body) straight into config_'s matching member --
  // no patch, no presence flags, no merge -- then fans it out via
  // install(target).
  //
  // Boot-only targets (GEOMETRY/PLANNER/PLANNER_SHAPER/NAVIGATOR) are
  // refused with ERR_NOT_LIVE BEFORE any decode is attempted -- config_ is
  // left untouched by a rejected push.
  msg::ErrCode applyGroup(msg::ConfigGroupTarget target, const uint8_t* wire, size_t len);

  // applyField() -- the single-field counterpart of applyGroup() above --
  // writes exactly ONE field inside ONE already-live group, addressed by
  // (target, protobuf field number), rather than replacing the whole
  // group.
  msg::ErrCode applyField(msg::ConfigGroupTarget target, uint16_t fieldNumber, float value);

  // install(target) -- fans ONE already-decoded group out to the
  // subsystem(s) that own it. Returns ERR_NOT_LIVE for a boot-only target,
  // ERR_UNIMPLEMENTED if the target has no live consumer yet (ESTIMATOR),
  // ERR_NONE on success.
  msg::ErrCode install(msg::ConfigGroupTarget target);

  // configSource() -- `target`'s PROVENANCE, i.e. where the values
  // config() currently reports for it came from (msg::ConfigSource --
  // BAKED / LIVE / PERSISTED).
  msg::ConfigSource configSource(msg::ConfigGroupTarget target) const;

  // encodeSnapshot() -- read-back's own encode step. Fills `out` with
  // `target`'s CURRENT value from config_. NOT gated by
  // isLiveConfigurable() -- read-back is honest for every
  // ConfigGroupTarget, including the boot-only ones.
  msg::ErrCode encodeSnapshot(msg::ConfigGroupTarget target, msg::ConfigSnapshot& out) const;

  // --- protocol v6 GET/SET (spec section 7) ---------------------------
  //
  // Addressed against Config::WireV6::kConfigFieldTable (sprint 137
  // ticket 001) by NAME ("<group>.<field>"), not by the
  // ConfigGroupTarget+protobuf-field-number pair applyGroup()/
  // applyField() above use -- a different, flatter addressing scheme for
  // a different wire generation, sharing config_ as the one underlying
  // store. Every field is settable (spec section 7.1: "Every field is
  // settable; only the apply is gated") -- setFieldByName() always writes
  // config_ once name/NaN/range validation passes, then additionally
  // fans the write out live via the SAME isLiveConfigurable()/
  // install(target) applyGroup()/applyField() already use, but ONLY when
  // that target is live. Unlike applyField(), a boot-only or
  // no-live-consumer target (GEOMETRY/PLANNER/PLANNER_SHAPER/NAVIGATOR/
  // ESTIMATOR) is NOT an error here -- v6 collapses the old
  // ERR_NOT_LIVE/ERR_UNIMPLEMENTED split into "applied now" vs. "stored
  // for next boot", both ERR_NONE, matching spec section 7.1's own
  // collapse of v5's boot-only/live split.

  // fieldCount()/fieldName()/fieldValueAt() -- bare GET's own iteration
  // surface (spec section 7.1: "A bare GET dumps every field"). `index`
  // is a plain cursor into Config::WireV6::kConfigFieldTable, 0-based, not
  // the wire's own id space.
  static uint16_t fieldCount();
  static const char* fieldName(uint16_t index);
  float fieldValueAt(uint16_t index) const;

  // getFieldByName() -- GET:<name>'s single-field surface. Returns false
  // (no such field) without touching *out.
  bool getFieldByName(const char* name, size_t nameLen, float* out) const;

  // setFieldByName() -- SET's single-field surface. Order of checks
  // matches spec section 7.1 exactly: unknown name (ERR_UNKNOWN) if
  // `name` doesn't match a table row, THEN non-finite (ERR_BADARG) BEFORE
  // the bounds check (ERR_RANGE) -- NaN/Inf compare false against both
  // `<` and `>`, so an unchecked NaN would pass any bound.
  msg::ErrCode setFieldByName(const char* name, size_t nameLen, float value);

 private:
  // stampSource() -- the ONE writer of groupSource_. Called from exactly
  // the three functions that mutate config_ -- loadBaked() (every group
  // BAKED), reapplyPersistedTuning() (each restored group PERSISTED), and
  // applyGroup()/applyField() (the pushed group LIVE) -- and from nowhere
  // else.
  void stampSource(msg::ConfigGroupTarget target, msg::ConfigSource source);

  // persistIfEligible() -- called from applyGroup()/applyField() after
  // install(target) succeeds. Snapshots config_'s CURRENT values for
  // `target`'s persisted subset into persistedTuning_, marks that group's
  // own "tuned" flag, and calls persistTuningIfChanged(). A no-op for any
  // other target.
  void persistIfEligible(msg::ConfigGroupTarget target);

  // installDriveKernelConfig() -- the ONE call DRIVE/WHEEL_CONTROL/MOTORS
  // all now share: rebuild the whole Control::DifferentialDrive::Config
  // from the CURRENT config_ (boot_calibration.h's buildDriveKernelConfig())
  // and push it. Factored out because three install(target) arms and the
  // no-arg boot install() all make this exact call.
  void installDriveKernelConfig();

  // Flash write policy: save only when the serialized snapshot changed.
  void persistTuningIfChanged();

  Control::DifferentialDrive& drive_;
  Hal::Motor& motorL_;
  Hal::Motor& motorR_;
  Hal::Otos& otos_;

  // Persisted live-tuning: per-group snapshot of config_'s own current
  // values for the persisted subset (config/persisted_tuning.h), plus the
  // last blob actually written (change-detection baseline).
  Config::TuningStore* tuningStore_ = nullptr;
  Config::TuningSnapshot persistedTuning_ = {};
  Config::Blob lastPersistedBlob_ = {};

  // The one owned configuration object.
  Config::Robot config_ = {};

  // Per-group provenance, indexed by ConfigGroupTarget's own integer
  // value. Slot 0 (CONFIG_GROUP_UNSPECIFIED) is never a real group and
  // stays CONFIG_SOURCE_UNSPECIFIED forever.
  static constexpr size_t kGroupSourceSlots =
      static_cast<size_t>(msg::ConfigGroupTarget::NAVIGATOR) + 1;
  msg::ConfigSource groupSource_[kGroupSourceSlots] = {};
};

}  // namespace Core

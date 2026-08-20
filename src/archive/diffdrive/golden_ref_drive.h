// golden_ref_drive.h -- FROZEN COPY of src/firm/control/differential_drive.h
// as of commit ab43963c: the control law EXACTLY as it stood immediately
// before the kernel rework. Namespace Control -> GoldenRef, Types:: ->
// GoldenRef::, and configure(Config::Robot&) dropped (pure field copying;
// the harness sets the same fields directly). NO MATH TOUCHED.
//
// This is the reference half of the golden-trace fidelity gate. The
// rework's claim is "the pipeline is a unit rebake, zero math changes" --
// this file is what that claim is checked against. See
// golden_trace_harness.cpp.
//
// DO NOT modernise this file. Its value is that it is the OLD code.
#pragma once

#include <cstdint>

#include "golden_ref_state.h"

namespace GoldenRef {

// Motor -- the reference law's OWN two-method motor seam, replacing the
// firmware hal/motor.h include so this frozen copy stays compilable with
// zero firmware headers. The reference only ever calls these two; the
// substitution is interface-narrowing only, no behaviour change (same
// licence as the namespace rename in this file's banner).
class Motor {
 public:
  virtual ~Motor() = default;
  virtual void setDuty(float duty) = 0;   // [-1, 1]
  virtual float velocity() const = 0;     // [mm/s] signed
};

class DifferentialDrive {
 public:
  DifferentialDrive(GoldenRef::Motor& left, GoldenRef::Motor& right, float trackWidth);

  static constexpr float kDutyPerSpeed = 0.001182f;  // [duty/(mm/s)]

  void setDutyPerSpeed(float left, float right) {  // [duty/(mm/s)] x2
    dutyPerSpeedLeft_ = left;
    dutyPerSpeedRight_ = right;
    calibrated_ = left != 0.0f && right != 0.0f;
  }

  float dutyPerSpeedLeft() const { return dutyPerSpeedLeft_; }    // [duty/(mm/s)]
  float dutyPerSpeedRight() const { return dutyPerSpeedRight_; }  // [duty/(mm/s)]

  void setWheelCorrection(float gainLeftAccel, float interceptLeftAccel,
                          float gainLeftDecel, float interceptLeftDecel,
                          float gainRightAccel, float interceptRightAccel,
                          float gainRightDecel, float interceptRightDecel);

  void setCrawlPulse(float crawlPulse) { crawlPulse_ = crawlPulse; }

  struct ControlGains {
    float kp = 0.0f;      // [1] dimensionless: mm/s of PID output per mm/s of error
    float ki = 0.0f;      // [1/s]
    float iMax = 0.0f;    // [mm/s] I-term output clamp; 0 disables the I term
    float kaff = 0.0f;    // [s] accel feedforward ~= the plant time constant
    float pidMax = 0.0f;  // [mm/s]
  };
  void setControlGains(const ControlGains& gains) { gains_ = gains; }
  const ControlGains& controlGains() const { return gains_; }

  struct AdaptationBounds {
    float vMin = 0.0f;              // [mm/s] speed floor (Open Question 2)
    float biasMax = 0.0f;           // [mm/s] Stage C trim authority clamp
    float tauAdapt = 0.0f;          // [s] Stage C adaptation time constant; <=0 disables
    float aSteady = 0.0f;           // [mm/s^2] |a_cmd| below this counts as steady
    float posErrMax = 0.0f;         // [mm] Stage B position-error clamp; 0 = unclamped
    float deficitThreshold = 0.0f;  // [mm/s] sustained error magnitude that flags a deficit
    float deficitWindow = 0.0f;     // [ms] how long the deficit condition must sustain
    float stallSpeed = 0.0f;   // [mm/s] measured speed at or below this is not turning
    float stallDemand = 0.0f;  // [mm/s] commanded speed above this is asking for motion
    float stallWindow = 0.0f;  // [ms] sustain time before a stall latches; 0 = off
  };
  void setAdaptationBounds(const AdaptationBounds& bounds) { bounds_ = bounds; }
  const AdaptationBounds& adaptationBounds() const { return bounds_; }


  float biasLeft() const { return biasLeft_; }      // [mm/s] Stage C's adapted parameter
  float biasRight() const { return biasRight_; }    // [mm/s]
  float pidLeft() const { return lastPidLeft_; }    // [mm/s] last-computed Stage B output
  float pidRight() const { return lastPidRight_; }  // [mm/s]
  bool deficitLeft() const { return deficitLeft_; }
  bool deficitRight() const { return deficitRight_; }

  // A stall is the drivetrain being ASKED to move and not moving -- the robot
  // is jammed against something. Unlike deficit() (the wheel turns, just too
  // slowly) this is a HALT condition: Core::RobotLoop stops the robot on it.
  // See robot_config.proto's WheelControl for the three-way distinction
  // against deficit and wheelFrozen.
  bool stallLeft() const { return stallLeft_; }
  bool stallRight() const { return stallRight_; }

  bool calibrated() const { return calibrated_; }

  void command(float vLeft, float vRight, float duration, uint32_t moveId,
               uint32_t now);  // [mm/s] [mm/s] [ms] -- now [ms]

  void takeover();

  void estop();

  bool owns() const { return commandActive_; }

  bool takeCompletion(uint32_t* moveId);

  void tick(const GoldenRef::RobotState& state);

  void setPositionErrorMax(float posErrMax) {  // [mm]
    bounds_.posErrMax = (posErrMax > 0.0f) ? posErrMax : 0.0f;
  }

  void setSpeedFloor(float vMin) {  // [mm/s]
    bounds_.vMin = (vMin > 0.0f) ? vMin : 0.0f;
  }

  void setASteady(float aSteady) {  // [mm/s^2]
    bounds_.aSteady = (aSteady > 0.0f) ? aSteady : 0.0f;
  }

  void update(GoldenRef::RobotState& state, uint32_t now);  // [ms]

  float targetLeft() const { return targetLeft_; }    // [mm/s] signed
  float targetRight() const { return targetRight_; }  // [mm/s] signed

  float trackWidth() const { return trackWidth_; }  // [mm]

 private:
  float correctedCommand(float desired, float previous, bool leftWheel,
                         float bias) const;

  float corrGain_[2][2] = {{1.0f, 1.0f}, {1.0f, 1.0f}};
  float corrIntercept_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  float lastSpeedLeft_ = 0.0f;   // [mm/s]
  float lastSpeedRight_ = 0.0f;  // [mm/s]

  static constexpr float kAccelSmoothing = 0.35f;  // [1] first-order weight, per cycle
  float previousTargetLeft_ = 0.0f;   // [mm/s] last cycle's published target
  float previousTargetRight_ = 0.0f;  // [mm/s]
  float cmdAccelLeft_ = 0.0f;         // [mm/s^2] smoothed
  float cmdAccelRight_ = 0.0f;        // [mm/s^2]

  float crawlDuty(float duty, float& carry) const;

  struct PositionRef {
    float reference = 0.0f;  // [mm] integral of commanded speed since the anchor
    float origin = 0.0f;     // [mm] Wheel::position when anchored
    uint8_t epoch = 0;       // Wheel::positionEpoch when anchored
    bool armed = false;
  };

  float fastPid(float posError, float err, float aCmd) const;  // [mm] [mm/s] [mm/s^2]

  float positionError(float speed, const GoldenRef::RobotState::Wheel& wheel,
                      PositionRef& ref, float dt) const;  // [mm/s] [s] -> [mm]

  void adaptBias(float& bias, float err, float aCmd, float vCmdMagnitude,
                bool fresh, float dt) const;

  void applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                       float& speedRight) const;

  void updateStall(bool conditionNow, uint32_t now, uint32_t& since,
                   bool& latched) const;
  void updateDeficit(bool conditionNow, uint32_t now, uint32_t& since,
                     bool& latched) const;

  uint32_t sampleAge(uint32_t now, uint32_t sampleTime) const;

  ControlGains gains_;
  AdaptationBounds bounds_;

  mutable PositionRef posRefLeft_;
  mutable PositionRef posRefRight_;
  float lastPidLeft_ = 0.0f;       // [mm/s] observability: last-computed Stage B output
  float lastPidRight_ = 0.0f;      // [mm/s]

  float biasLeft_ = 0.0f;   // [mm/s] Stage C's ONE adapted parameter, per wheel
  float biasRight_ = 0.0f;  // [mm/s]

  uint32_t deficitSinceLeft_ = 0;   // [ms]
  uint32_t deficitSinceRight_ = 0;  // [ms]
  bool deficitLeft_ = false;
  bool deficitRight_ = false;
  uint32_t stallSinceLeft_ = 0;   // [ms] when the stall condition first held
  uint32_t stallSinceRight_ = 0;  // [ms]
  bool stallLeft_ = false;
  bool stallRight_ = false;

  static constexpr uint32_t kMaxSampleAge = 200;  // [ms]

  GoldenRef::Motor& left_;
  GoldenRef::Motor& right_;
  float trackWidth_;  // [mm]

  float targetLeft_ = 0.0f;   // [mm/s]
  float targetRight_ = 0.0f;  // [mm/s]

  bool commandActive_ = false;
  uint32_t commandDeadline_ = 0;  // [ms]
  uint32_t commandMoveId_ = 0;
  bool completionPending_ = false;
  uint32_t completedMoveId_ = 0;

  float dutyPerSpeedLeft_ = 0.0f;   // [duty/(mm/s)]
  float dutyPerSpeedRight_ = 0.0f;  // [duty/(mm/s)]
  bool calibrated_ = false;

  float crawlPulse_ = 0.0f;  // [-1, 1] pulse amplitude; 0 = off
  float crawlCarryLeft_ = 0.0f;   // Bresenham accumulators
  float crawlCarryRight_ = 0.0f;

  float writtenLeft_ = 0.0f;   // [-1, 1]
  float writtenRight_ = 0.0f;  // [-1, 1]

  uint8_t stopEnforceCountdown_ = 0;

  static constexpr uint8_t kStopEnforceTicks = 30;

  static constexpr float kRestVelocity = 8.0f;  // [mm/s]
};

}  // namespace GoldenRef

// differential_drive.cpp — DiffDrive::DifferentialDrive implementation.
// EXTRACTED from src/firm/control/differential_drive.cpp with only the
// namespace (Control -> DiffDrive) and include changed; the control law
// is byte-identical, and src/tests/diffdrive/ holds the two to the same
// behaviour. Fix bugs THERE first or HERE first, but always in both —
// until the firmware is cut over to consume this package directly.
#include "differential_drive.h"

#include <algorithm>
#include <cmath>

namespace DiffDrive {

namespace {

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// fastPid() already fails closed on non-finite input; these push that same
// posture out to the BOUNDARY so a NaN never enters the config or the
// mailbox in the first place, rather than being caught mid-pipeline.
bool isFinite(float v) { return std::isfinite(v); }

bool allFinite(const DiffDrive::DifferentialDrive::Config& c) {
  const float scalars[] = {
      c.maxDuty, c.fullDutyVelocity, c.kp, c.ki, c.iMax, c.kaff, c.pidMax,
      c.twistHoldGain, c.vMin, c.posErrMax, c.biasMax, c.tauAdapt, c.aSteady,
      c.deficitThreshold, c.deficitWindow, c.stallSpeed, c.stallDemand,
      c.stallWindow, c.crawlPulse,
  };
  for (float v : scalars) {
    if (!std::isfinite(v)) return false;
  }
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      if (!std::isfinite(c.wheelGain[i][j])) return false;
      if (!std::isfinite(c.wheelIntercept[i][j])) return false;
    }
  }
  return true;
}

}  // namespace

DifferentialDrive::DifferentialDrive(Motor& left, Motor& right,
                                     const Clock& clock,
                                     Sleeper& sleeper,
                                     FiberLauncher& launcher)
    : left_(left), right_(right), clock_(clock), sleeper_(sleeper),
      launcher_(launcher) {}

// ---------------------------------------------------------------------------
// Config surface 1: chainable setters (main-fiber writers of staged_; the
// kernel fiber snapshots at cycle start, so these are live mid-run).
// ---------------------------------------------------------------------------

DifferentialDrive& DifferentialDrive::setMaxDuty(float maxDuty) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(maxDuty)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.maxDuty = maxDuty;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setFullDutyVelocity(float velocity) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(velocity)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.fullDutyVelocity = velocity;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setKp(float kp) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(kp)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.kp = kp;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setKi(float ki) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(ki)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.ki = ki;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setIMax(float iMax) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(iMax)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.iMax = iMax;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setKaff(float kaff) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(kaff)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.kaff = kaff;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setPidMax(float pidMax) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(pidMax)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.pidMax = pidMax;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setTwistHoldGain(float gain) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(gain)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.twistHoldGain = gain;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setWheelCorrection(
    float gainLeftAccel, float interceptLeftAccel, float gainLeftDecel,
    float interceptLeftDecel, float gainRightAccel, float interceptRightAccel,
    float gainRightDecel, float interceptRightDecel) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(gainLeftAccel) || !isFinite(interceptLeftAccel) ||
      !isFinite(gainLeftDecel) || !isFinite(interceptLeftDecel) ||
      !isFinite(gainRightAccel) || !isFinite(interceptRightAccel) ||
      !isFinite(gainRightDecel) || !isFinite(interceptRightDecel)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.wheelGain[0][0] = gainLeftAccel;
  staged_.wheelIntercept[0][0] = interceptLeftAccel;
  staged_.wheelGain[0][1] = gainLeftDecel;
  staged_.wheelIntercept[0][1] = interceptLeftDecel;
  staged_.wheelGain[1][0] = gainRightAccel;
  staged_.wheelIntercept[1][0] = interceptRightAccel;
  staged_.wheelGain[1][1] = gainRightDecel;
  staged_.wheelIntercept[1][1] = interceptRightDecel;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setSpeedFloor(float vMin) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(vMin)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.vMin = (vMin > 0.0f) ? vMin : 0.0f;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setPositionErrorMax(float posErrMax) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(posErrMax)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.posErrMax = (posErrMax > 0.0f) ? posErrMax : 0.0f;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setAdaptation(float biasMax,
                                                    float tauAdapt,
                                                    float aSteady) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(biasMax) || !isFinite(tauAdapt) || !isFinite(aSteady)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.biasMax = biasMax;
  staged_.tauAdapt = tauAdapt;
  staged_.aSteady = (aSteady > 0.0f) ? aSteady : 0.0f;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setDeficit(float threshold,
                                                 float window) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(threshold) || !isFinite(window)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.deficitThreshold = threshold;
  staged_.deficitWindow = window;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setStall(float speed, float demand,
                                               float window) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(speed) || !isFinite(demand) || !isFinite(window)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.stallSpeed = speed;
  staged_.stallDemand = demand;
  staged_.stallWindow = window;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setLambdaEnabled(bool enabled) {
  // No finite check: a bool cannot be non-finite.
  staged_.lambdaEnabled = enabled;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setCrawlPulse(float crawlPulse) {
  // Non-finite in = refused, and RECORDED: this setter must return
  // *this to chain, so lastError() is the only way a caller can see it.
  if (!isFinite(crawlPulse)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.crawlPulse = crawlPulse;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setCyclePeriod(uint32_t period) {
  // REFUSED after begin(): the leaf's write throttle was derived from the
  // cadence at that moment (lesson 6), and any leaf window counted in
  // ticks changes wall-clock meaning with it. Live-mutating it would move
  // a constant out from under the leaf mid-cycle.
  if (begun_) {
    noteRefusal(Status::kCadencePreserved);
    return *this;
  }
  staged_.cyclePeriod = period;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive::Status DifferentialDrive::setConfig(const Config& config) {
  if (!allFinite(config)) {
    noteRefusal(Status::kRefusedNonFinite);
    return Status::kRefusedNonFinite;
  }
  // Cadence is FROZEN at begin(): the leaf's write throttle was derived
  // from it at that moment, and re-deriving it live would mutate a
  // constant out from under the leaf mid-cycle. Apply everything else and
  // say what happened, rather than refusing the whole block (persisted
  // tuning legitimately carries a full Config, cadence included).
  const bool cadenceDiffers =
      begun_ && config.cyclePeriod != staged_.cyclePeriod;
  const uint32_t frozen = staged_.cyclePeriod;
  staged_ = config;
  if (cadenceDiffers) {
    staged_.cyclePeriod = frozen;
    ++cfgSeq_;
    noteRefusal(Status::kCadencePreserved);
    return Status::kCadencePreserved;
  }
  ++cfgSeq_;
  return Status::kOk;
}

DifferentialDrive::Config DifferentialDrive::config() const {
  return staged_;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

DifferentialDrive::Status DifferentialDrive::begin() {
  // Primes both encoders (the 0x46 register sits frozen at 0 until its
  // first atomic read — the leaf's hardReset() burst) and arms the boot
  // zero-write: the first cycles run the stop path, and the leaf's
  // first-write slew exemption sends the zero immediately. The brick
  // latches its last commanded speed across nRF resets, so boot ALWAYS
  // re-asserts stop.
  //
  // The boot zero-write runs even when the config has no authority: an
  // unconfigured robot must still assert stop, which is the whole point
  // of the fail-closed posture. The refusal is REPORTED, not acted on by
  // skipping the safety work.
  left_.begin();
  right_.begin();
  stopEnforceCountdown_ = kStopEnforceTicks;
  begun_ = true;
  // Cadence freezes HERE — see setConfig()'s own comment.
  if (staged_.maxDuty <= 0.0f) {
    noteRefusal(Status::kRefusedUnconfigured);
    return Status::kRefusedUnconfigured;
  }
  return Status::kOk;
}

DifferentialDrive::Status DifferentialDrive::start() {
  if (!begun_) {
    noteRefusal(Status::kRefusedNotBegun);
    return Status::kRefusedNotBegun;
  }
  if (running_) return Status::kOk;  // idempotent
  running_ = true;
  launcher_.launch(&DifferentialDrive::fiberEntry, this);
  return Status::kOk;
}

void DifferentialDrive::fiberEntry(void* self) {
  static_cast<DifferentialDrive*>(self)->run();
}

void DifferentialDrive::run() {
  while (true) {
    const uint64_t cycleStartUs = clock_.nowMicros();
    step();
    // Absolute-deadline pacing (the 131-005 lesson): sleep to
    // cycleStart + period, never a gap relative to "now" — gap-relative
    // sleeps compound their own rounding into structural drift.
    const uint64_t deadlineUs =
        cycleStartUs + static_cast<uint64_t>(active_.cyclePeriod) * 1000ull;
    const uint64_t nowUs = clock_.nowMicros();
    if (nowUs < deadlineUs) {
      const uint32_t shortfall =
          static_cast<uint32_t>((deadlineUs - nowUs + 999) / 1000);  // [ms]
      sleeper_.sleepMillis(shortfall);
    } else {
      // Overrun: the cycle missed its absolute deadline. Counted (lesson
      // 17's observability half — a soak run whose overrun count climbs
      // is the signal that the cadence is not actually being met), and we
      // still yield once so the main fiber (comms, sensors) is never
      // starved by a hot kernel loop.
      ++cycleOverrunCount_;
      sleeper_.yield();
    }
  }
}

// ---------------------------------------------------------------------------
// Commands (main-fiber writers of the mailbox)
// ---------------------------------------------------------------------------

// Shared refusal gate for both command arms. A refused command leaves the
// mailbox UNTOUCHED — the previously-commanded motion continues under its
// own lease rather than being silently replaced by a rejected one.
DifferentialDrive::Status DifferentialDrive::checkCommandable(
    bool needsVelocityCalibration) const {
  if (!begun_) return Status::kRefusedNotBegun;
  if (estopLatch_) return Status::kRefusedEstopped;
  if (staged_.maxDuty <= 0.0f) return Status::kRefusedUnconfigured;
  if (needsVelocityCalibration && staged_.fullDutyVelocity <= 0.0f) {
    return Status::kRefusedUnconfigured;
  }
  return Status::kOk;
}

DifferentialDrive::Status DifferentialDrive::drive(float velocity, float twist,
                                                   uint32_t lease) {
  if (!isFinite(velocity) || !isFinite(twist)) {
    noteRefusal(Status::kRefusedNonFinite);
    return Status::kRefusedNonFinite;
  }
  const Status gate = checkCommandable(/*needsVelocityCalibration=*/true);
  if (gate != Status::kOk) {
    noteRefusal(gate);
    return gate;
  }
  Command c;
  c.mode = kModeVelocity;
  c.velocity = velocity;
  c.twist = twist;
  c.validUntil = static_cast<uint32_t>(clock_.nowMicros() / 1000) +
                 (lease > kLeaseMax ? kLeaseMax : lease);
  command_ = c;
  ++cmdSeq_;
  return Status::kOk;
}

DifferentialDrive::Status DifferentialDrive::driveDuty(float dutyLeft,
                                                       float dutyRight,
                                                       uint32_t lease) {
  if (!isFinite(dutyLeft) || !isFinite(dutyRight)) {
    noteRefusal(Status::kRefusedNonFinite);
    return Status::kRefusedNonFinite;
  }
  // Duty mode needs no fullDutyVelocity: it bypasses the velocity->duty
  // map entirely. That is what makes it usable for plant-ID runs on an
  // uncalibrated robot (the stall detector is disarmed there in exchange).
  const Status gate = checkCommandable(/*needsVelocityCalibration=*/false);
  if (gate != Status::kOk) {
    noteRefusal(gate);
    return gate;
  }
  Command c;
  c.mode = kModeRawDuty;
  c.dutyLeft = dutyLeft;
  c.dutyRight = dutyRight;
  c.validUntil = static_cast<uint32_t>(clock_.nowMicros() / 1000) +
                 (lease > kLeaseMax ? kLeaseMax : lease);
  command_ = c;
  ++cmdSeq_;
  return Status::kOk;
}

void DifferentialDrive::neutral() {
  Command c;  // mode defaults to kModeNeutral; no lease needed to be stopped
  command_ = c;
  ++cmdSeq_;
}

void DifferentialDrive::estop() {
  // Latch OUTSIDE the seq handshake — a single aligned store, effective
  // even if the mailbox discipline is wedged. The kernel forces the stop
  // path on its next cycle (worst case one cycle period away) and the
  // stop-enforce machinery re-asserts from there.
  estopLatch_ = true;
}

void DifferentialDrive::estopClear() {
  estopLatch_ = false;
}

void DifferentialDrive::emergencyStopMotors() {
  // Latch estop FIRST, so that if the kernel fiber is merely wedged
  // rather than dead and later wakes up, it resumes into a stopped state
  // instead of re-applying the pre-emergency command.
  estopLatch_ = 1;
  left_.emergencyStop();
  right_.emergencyStop();
}

void DifferentialDrive::clearStallLatch() {
  ++clearStallReq_;
}

void DifferentialDrive::rebasePosition() {
  ++rebaseReq_;
}

// ---------------------------------------------------------------------------
// Output (seq-consistent copy)
// ---------------------------------------------------------------------------

DifferentialDrive::Output DifferentialDrive::output() const {
  Output copy;
  uint32_t s1, s2;
  do {
    s1 = outSeq_;
    copy = out_;
    s2 = outSeq_;
  } while (s1 != s2 || (s1 & 1u));
  return copy;
}

// ---------------------------------------------------------------------------
// The kernel cycle
// ---------------------------------------------------------------------------

void DifferentialDrive::snapshotConfig() {
  const uint32_t seq = cfgSeq_;
  if (seq == activeCfgSeq_) return;
  active_ = staged_;
  activeCfgSeq_ = seq;
}

DifferentialDrive::Command DifferentialDrive::snapshotCommand() const {
  Command copy;
  uint32_t s1, s2;
  do {
    s1 = cmdSeq_;
    copy = command_;
    s2 = cmdSeq_;
  } while (s1 != s2);
  return copy;
}

void DifferentialDrive::step() {
  const uint64_t cycleStartUs = clock_.nowMicros();
  const uint32_t nowMs = static_cast<uint32_t>(cycleStartUs / 1000);

  // Measured period — feeds every dt term below (never the baked
  // constant: the baked-vs-delivered mismatch alone flipped the old
  // heading hold marginally unstable).
  const uint32_t measuredPeriodUs =
      everCycled_ ? static_cast<uint32_t>(cycleStartUs - previousCycleStartUs_)
                  : 0u;
  previousCycleStartUs_ = cycleStartUs;
  everCycled_ = true;
  const float dt = static_cast<float>(measuredPeriodUs) * 1e-6f;  // [s]
  ++cycleCount_;

  snapshotConfig();

  // One-shot requests (counter handshake — never lost to a concurrent
  // command write, unlike flags inside the mailbox would be).
  if (clearStallReq_ != seenClearStallReq_) {
    seenClearStallReq_ = clearStallReq_;
    stallHalted_ = false;
    stallLatched_ = false;
    stallSince_ = 0;
  }
  if (rebaseReq_ != seenRebaseReq_) {
    seenRebaseReq_ = rebaseReq_;
    left_.rebaseline();
    right_.rebaseline();
    ++epoch_;  // every integrator re-anchors; sample caches restart
    // Published, host-facing epochs: both wheels rebaseline together
    // today, but they are counted separately so a future single-wheel
    // rebaseline is not a contract change for the host.
    ++positionEpochLeft_;
    ++positionEpochRight_;
    sampleLeft_ = WheelSample{};
    sampleRight_ = WheelSample{};
  }

  const Command cmd = snapshotCommand();

  // ---- safety gates → effective mode ----
  const bool needsLease = cmd.mode != kModeNeutral;
  const bool leaseLive =
      needsLease && static_cast<int32_t>(cmd.validUntil - nowMs) > 0;
  const bool leaseExpired = needsLease && !leaseLive;
  if (leaseWasLive_ && leaseExpired) ++leaseExpiryCount_;
  leaseWasLive_ = leaseLive;

  uint8_t effective = cmd.mode;
  if (leaseExpired) effective = kModeNeutral;
  if (stallHalted_) effective = kModeNeutral;
  if (estopLatch_) effective = kModeNeutral;
  // Uncalibrated refuses VELOCITY visibly (ready=false in the output) —
  // and still runs the stop path, unlike the old silent no-write return.
  if (effective == kModeVelocity && active_.fullDutyVelocity <= 0.0f) {
    effective = kModeNeutral;
  }

  // estop/stall-halt entry resets ALL adaptive state (the old estop()'s
  // semantics — bias, position refs, latch timers — while an ordinary
  // commanded stop deliberately does not, matching takeover()).
  const bool halted = estopLatch_ || stallHalted_;
  if (halted && !wasForcedStop_) resetAdaptiveState();
  wasForcedStop_ = halted;

  // ---- control step: stage this cycle's duties from LAST cycle's
  // samples (the same one-cycle actuation latency the old loop had) ----
  controlStep(cmd, effective, dt, nowMs);

  // ---- encoder split-phase (the kernel is the ONLY 0x10 client; the
  // settle sleeps yield to the main fiber, which is where the comms pump
  // lives — the mandatory wait is spent usefully by construction) ----
  left_.requestSample();
  sleeper_.sleepMillis(kSettle);
  left_.tick(clock_.nowMicros());

  right_.requestSample();
  sleeper_.sleepMillis(kSettle);
  right_.tick(clock_.nowMicros());

  // i2cFaultCount, derived from sample-stamp NON-ADVANCE. The leaf's
  // requestSample()/tick() both return void, so there is no direct "that
  // collect failed" signal; sampleTime() failing to move across a cycle
  // in which we DID request a sample is the observable equivalent (the
  // 131-002 rule: a failed collect HOLDS the stamp rather than restamping
  // a stale reading). Counted once per cycle in which either wheel's
  // stamp stood still, not once per wheel — the failure is a bus event.
  const uint64_t stampLeftBefore = sampleLeft_.sampleTime;
  const uint64_t stampRightBefore = sampleRight_.sampleTime;
  refreshSample(left_, sampleLeft_);
  refreshSample(right_, sampleRight_);
  if (sampleLeft_.sampleTime == stampLeftBefore ||
      sampleRight_.sampleTime == stampRightBefore) {
    ++i2cFaultCount_;
  }

  const uint64_t busyEndUs = clock_.nowMicros();
  publishOutput(nowMs, cycleStartUs, busyEndUs, measuredPeriodUs,
                leaseExpired);
}

void DifferentialDrive::controlStep(const Command& cmd, uint8_t effectiveMode,
                                    float dt, uint32_t nowMs) {
  const float rail = active_.maxDuty * 0.01f;  // [-1,1] duty fraction

  if (effectiveMode == kModeNeutral) {
    dutyDemandLeft_ = 0.0f;
    dutyDemandRight_ = 0.0f;
    satLeft_ = false;
    satRight_ = false;
    lambda_ = 1.0f;
    twistRef_.armed = false;
    lastSpeedLeft_ = 0.0f;
    lastSpeedRight_ = 0.0f;
    previousTargetLeft_ = 0.0f;
    previousTargetRight_ = 0.0f;
    cmdAccelLeft_ = 0.0f;
    cmdAccelRight_ = 0.0f;
    lastPidLeft_ = 0.0f;
    lastPidRight_ = 0.0f;
    // A neutral command is not "demanding": the stall condition clears.
    updateLatch(false, active_.stallWindow, nowMs, stallSince_, stallLatched_);
    updateLatch(false, active_.deficitWindow, nowMs, deficitSinceLeft_,
                deficitLeft_);
    updateLatch(false, active_.deficitWindow, nowMs, deficitSinceRight_,
                deficitRight_);
    // positionError's own re-anchor fires next velocity cycle (speed==0
    // disarms the refs through the normal path).
    posRefLeft_.armed = false;
    posRefRight_.armed = false;
    stageStop();
    return;
  }

  if (effectiveMode == kModeRawDuty) {
    // Armor-mediated raw duty for calibration/bench: clamps to the rail,
    // records the raw demand for observability, bypasses λ and the
    // velocity pipeline entirely. The lease is still enforced upstream.
    const float demandL = cmd.dutyLeft * 0.01f;
    const float demandR = cmd.dutyRight * 0.01f;
    dutyDemandLeft_ = demandL;
    dutyDemandRight_ = demandR;
    satLeft_ = std::fabs(demandL) > rail;
    satRight_ = std::fabs(demandR) > rail;
    lambda_ = 1.0f;
    twistRef_.armed = false;
    stageDuty(clampf(demandL, -rail, rail), clampf(demandR, -rail, rail));
    return;
  }

  // ---- kModeVelocity ----
  const float dutyPerSpeed = 1.0f / active_.fullDutyVelocity;  // [1/(counts/s)]

  // Raw pre-floor, pre-λ targets. The stall "demanding" gate reads THESE
  // (what the application asked for), never the floored/scaled values.
  const float rawLeft = cmd.velocity - cmd.twist;
  const float rawRight = cmd.velocity + cmd.twist;

  // ---- λ authority from LAST cycle's pre-clamp duty demands ----
  const float demandMagLeft = std::fabs(dutyDemandLeft_);
  const float demandMagRight = std::fabs(dutyDemandRight_);
  satLeft_ = demandMagLeft > rail;
  satRight_ = demandMagRight > rail;
  // GATED (Config::lambdaEnabled, ships OFF). With λ disabled the kernel
  // is the pure port of the old pipeline — which is what the first bench
  // pass and the golden-trace fidelity gate must measure. Turning
  // authority-headroom scaling on from the start would mean every early
  // number carries a stage the old law never had, and any discrepancy
  // would be unattributable.
  //
  // λ is pinned at exactly 1.0 when off, so every downstream λ-scaled
  // term (targets, the twist-hold reference, the anti-windup position
  // reference) degenerates to the unscaled value rather than needing its
  // own gate.
  if (!active_.lambdaEnabled) {
    lambda_ = 1.0f;
  } else {
    float lambdaInstant = 1.0f;
    if (satLeft_) lambdaInstant = std::min(lambdaInstant, rail / demandMagLeft);
    if (satRight_) lambdaInstant = std::min(lambdaInstant, rail / demandMagRight);
    if (lambdaInstant < lambda_) {
      lambda_ = lambdaInstant;  // fast attack: shed authority immediately
    } else if (dt > 0.0f) {
      // Slow release: creep back toward full authority so the boundary
      // cannot limit-cycle.
      lambda_ += (lambdaInstant - lambda_) *
                 std::min(1.0f, dt / kLambdaReleaseTau);
    }
    lambda_ = clampf(lambda_, 0.0f, 1.0f);
  }

  // Scaling BOTH targets by one λ preserves the commanded ratio exactly —
  // the healthy wheel slows to match the saturated one (authority
  // headroom), curvature preserved.
  float scaledLeft = lambda_ * rawLeft;
  float scaledRight = lambda_ * rawRight;
  const float scaledTwist = lambda_ * cmd.twist;

  // ---- twist-integral hold (encoder-only ratio maintenance) ----
  float trim = 0.0f;
  if (active_.twistHoldGain > 0.0f && sampleLeft_.connected &&
      sampleRight_.connected) {
    if (!twistRef_.armed || twistRef_.epoch != epoch_) {
      twistRef_.reference = 0.0f;
      twistRef_.originLeft = sampleLeft_.position;
      twistRef_.originRight = sampleRight_.position;
      twistRef_.epoch = epoch_;
      twistRef_.armed = true;
    }
    if (dt > 0.0f) twistRef_.reference += scaledTwist * dt;
    const float measuredTwistPosition =
        0.5f * ((sampleRight_.position - twistRef_.originRight) -
                (sampleLeft_.position - twistRef_.originLeft));  // [counts]
    const float twistError = twistRef_.reference - measuredTwistPosition;
    // Headroom clamp: the trim may never push a wheel past the (λ-scaled
    // remaining) authority — if it would, next cycle's λ shrinks both and
    // the shrunken headroom shrinks the trim; the two mechanisms compose.
    const float authority = rail * active_.fullDutyVelocity;  // [counts/s]
    const float headroom = std::max(
        0.0f, authority - std::max(std::fabs(scaledLeft),
                                   std::fabs(scaledRight)));
    trim = clampf(active_.twistHoldGain * twistError, -headroom, headroom);
  } else {
    twistRef_.armed = false;
  }

  float targetLeft = scaledLeft - trim;
  float targetRight = scaledRight + trim;

  // ---- ported pipeline: floor → Stage A → posError → PID → duty ----
  float speedLeft, speedRight;
  applySpeedFloor(targetLeft, targetRight, speedLeft, speedRight);

  const float correctedLeft =
      correctedCommand(speedLeft, lastSpeedLeft_, true, biasLeft_);
  const float correctedRight =
      correctedCommand(speedRight, lastSpeedRight_, false, biasRight_);
  lastSpeedLeft_ = speedLeft;
  lastSpeedRight_ = speedRight;

  // Commanded-accel EMA (feeds kaff feedforward + the aSteady adaptation
  // gate — the old update()-side computation, relocated).
  if (dt > 0.0f) {
    const float rawAccelLeft = (speedLeft - previousTargetLeft_) / dt;
    const float rawAccelRight = (speedRight - previousTargetRight_) / dt;
    cmdAccelLeft_ += kAccelSmoothing * (rawAccelLeft - cmdAccelLeft_);
    cmdAccelRight_ += kAccelSmoothing * (rawAccelRight - cmdAccelRight_);
  }
  previousTargetLeft_ = speedLeft;
  previousTargetRight_ = speedRight;

  const uint64_t nowUs = previousCycleStartUs_;  // this cycle's start stamp
  const bool freshLeft =
      sampleLeft_.connected && !left_.wedgeSuspect() &&
      static_cast<float>(nowUs - sampleLeft_.sampleTime) <= kMaxSampleAge;
  const bool freshRight =
      sampleRight_.connected && !right_.wedgeSuspect() &&
      static_cast<float>(nowUs - sampleRight_.sampleTime) <= kMaxSampleAge;

  const float errLeft = speedLeft - sampleLeft_.velocity;
  const float errRight = speedRight - sampleRight_.velocity;

  const float posErrorLeft =
      positionError(speedLeft, sampleLeft_, posRefLeft_, dt);
  const float posErrorRight =
      positionError(speedRight, sampleRight_, posRefRight_, dt);
  const float pidLeft =
      (speedLeft == 0.0f) ? 0.0f
                          : fastPid(posErrorLeft, errLeft, cmdAccelLeft_);
  const float pidRight =
      (speedRight == 0.0f) ? 0.0f
                           : fastPid(posErrorRight, errRight, cmdAccelRight_);
  lastPidLeft_ = pidLeft;
  lastPidRight_ = pidRight;

  // Duty demand, UNCLAMPED — recorded before the rail so next cycle's λ
  // sees how far past the rail the plant was asked to go.
  const float demandLeft = (correctedLeft + pidLeft) * dutyPerSpeed;
  const float demandRight = (correctedRight + pidRight) * dutyPerSpeed;
  dutyDemandLeft_ = demandLeft;
  dutyDemandRight_ = demandRight;

  const float dutyLeft =
      crawlDuty(clampf(demandLeft, -rail, rail), crawlCarryLeft_);
  const float dutyRight =
      crawlDuty(clampf(demandRight, -rail, rail), crawlCarryRight_);

  // Stage C adaptation — λ-gated on top of the existing fresh-and-steady
  // gates: adapting the bias while authority-limited learns garbage.
  const bool adaptAllowed = lambda_ >= kLambdaAdaptFloor;
  adaptBias(biasLeft_, errLeft, cmdAccelLeft_, std::fabs(speedLeft),
            freshLeft && adaptAllowed, dt);
  adaptBias(biasRight_, errRight, cmdAccelRight_, std::fabs(speedRight),
            freshRight && adaptAllowed, dt);

  // Deficit latches (observability only).
  const bool biasSaturatedLeft =
      active_.biasMax > 0.0f && std::fabs(biasLeft_) >= active_.biasMax;
  const bool biasSaturatedRight =
      active_.biasMax > 0.0f && std::fabs(biasRight_) >= active_.biasMax;
  const bool pidSaturatedLeft =
      active_.pidMax > 0.0f && std::fabs(pidLeft) >= active_.pidMax;
  const bool pidSaturatedRight =
      active_.pidMax > 0.0f && std::fabs(pidRight) >= active_.pidMax;
  const bool deficitCondLeft = active_.deficitThreshold > 0.0f &&
                               std::fabs(errLeft) > active_.deficitThreshold &&
                               biasSaturatedLeft && pidSaturatedLeft;
  const bool deficitCondRight =
      active_.deficitThreshold > 0.0f &&
      std::fabs(errRight) > active_.deficitThreshold && biasSaturatedRight &&
      pidSaturatedRight;
  updateLatch(deficitCondLeft, active_.deficitWindow, nowMs, deficitSinceLeft_,
              deficitLeft_);
  updateLatch(deficitCondRight, active_.deficitWindow, nowMs,
              deficitSinceRight_, deficitRight_);

  // ---- stall (encoder/duty-based ONLY in the kernel — cannot catch a
  // slipping-wheel jam; the OTOS body-motion check is the application
  // observer's job, which stops the robot through this interface) ----
  //
  // "demanding" reads the RAW commanded targets (pre-floor, pre-λ): the
  // floor boosts sub-vMin commands and λ shrinks saturated ones — testing
  // either processed value misreads what the application actually asked.
  // The encoder-still test deliberately does NOT suppress on wedge
  // suspicion: "position unchanged while duty applied" IS a stall, and
  // suppressing exactly then hid real jams (measured 2026-08-08).
  const bool demanding =
      active_.stallDemand > 0.0f &&
      (std::fabs(rawLeft) > active_.stallDemand ||
       std::fabs(rawRight) > active_.stallDemand);
  const bool encoderStill =
      std::fabs(sampleLeft_.velocity) <= active_.stallSpeed &&
      std::fabs(sampleRight_.velocity) <= active_.stallSpeed &&
      sampleLeft_.connected && sampleRight_.connected;
  updateLatch(demanding && encoderStill, active_.stallWindow, nowMs,
              stallSince_, stallLatched_);
  if (stallLatched_ && !stallHalted_) {
    // One physical condition — the ROBOT is stuck — so the kernel halts
    // itself (the old RobotLoop::haltOnStall(), relocated in-class). The
    // halt takes effect via the forced-stop gate next cycle; the latch
    // holds until the application calls clearStallLatch().
    stallHalted_ = true;
  }

  stageDuty(dutyLeft, dutyRight);
}

void DifferentialDrive::stageStop() { stageDuty(0.0f, 0.0f); }

void DifferentialDrive::stageDuty(float dutyLeft, float dutyRight) {
  // Stop-enforce write gate (ported intact): a commanded stop is
  // re-written for kStopEnforceTicks after the transition AND
  // unconditionally while the encoders still read motion — the leaf's own
  // stopNotTaken is the second, independent layer below this one.
  const bool wheelsMoving = std::fabs(left_.velocity()) > kRestVelocity ||
                            std::fabs(right_.velocity()) > kRestVelocity;
  const bool enforceStop = stopEnforceCountdown_ > 0 || wheelsMoving;
  if (stopEnforceCountdown_ > 0) --stopEnforceCountdown_;

  const bool commandedStop = dutyLeft == 0.0f && dutyRight == 0.0f;
  const bool alreadyQuiet =
      commandedStop && writtenLeft_ == 0.0f && writtenRight_ == 0.0f;
  if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;

  if (alreadyQuiet && !enforceStop) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

void DifferentialDrive::refreshSample(Motor& motor, WheelSample& sample) {
  sample.connected = motor.connected();
  const uint64_t sampleTime = motor.sampleTime();
  const float position = motor.position();
  if (!sample.everSampled) {
    if (sampleTime != 0) {
      sample.everSampled = true;
      sample.sampleTime = sampleTime;
      sample.position = position;
    }
    return;  // velocity stays 0 until a second genuine sample exists
  }
  if (sampleTime != sample.sampleTime) {
    // Velocity over the GENUINE inter-sample interval — a failed collect
    // does not advance sampleTime (the 131-002 rule), so a dead bus HOLDS
    // the last velocity while the sample age grows, instead of decaying
    // toward a fake zero.
    const float interval =
        static_cast<float>(sampleTime - sample.sampleTime) * 1e-6f;  // [s]
    if (interval > 0.0f) {
      sample.velocity = (position - sample.position) / interval;
    }
    sample.sampleTime = sampleTime;
    sample.position = position;
  }
}

void DifferentialDrive::resetAdaptiveState() {
  posRefLeft_ = PositionRef{};
  posRefRight_ = PositionRef{};
  twistRef_ = TwistRef{};
  biasLeft_ = 0.0f;
  biasRight_ = 0.0f;
  deficitSinceLeft_ = 0;
  deficitSinceRight_ = 0;
  deficitLeft_ = false;
  deficitRight_ = false;
  stallSince_ = 0;
  stallLatched_ = false;
  crawlCarryLeft_ = 0.0f;
  crawlCarryRight_ = 0.0f;
  lastPidLeft_ = 0.0f;
  lastPidRight_ = 0.0f;
  stopEnforceCountdown_ = kStopEnforceTicks;
}

void DifferentialDrive::publishOutput(uint32_t nowMs, uint64_t cycleStartUs,
                                      uint64_t busyEndUs,
                                      uint32_t measuredPeriod,
                                      bool leaseExpired) {
  ++outSeq_;  // odd: write in progress
  out_.cyclePeriodMeasured = measuredPeriod;
  out_.leaseExpired = leaseExpired;
  out_.now = nowMs;
  out_.nowFine = static_cast<uint32_t>(busyEndUs);
  out_.cycleCount = cycleCount_;
  out_.cycleOverrunCount = cycleOverrunCount_;
  out_.cycleBusy = static_cast<uint32_t>(busyEndUs - cycleStartUs);
  out_.sampleTimeLeft = static_cast<uint32_t>(sampleLeft_.sampleTime);
  out_.sampleTimeRight = static_cast<uint32_t>(sampleRight_.sampleTime);
  out_.positionLeft = sampleLeft_.position;
  out_.positionRight = sampleRight_.position;
  out_.velocityLeft = sampleLeft_.velocity;
  out_.velocityRight = sampleRight_.velocity;
  out_.velocity = 0.5f * (sampleLeft_.velocity + sampleRight_.velocity);
  out_.twist = 0.5f * (sampleRight_.velocity - sampleLeft_.velocity);
  out_.appliedDutyLeft = left_.appliedDuty() * 100.0f;
  out_.appliedDutyRight = right_.appliedDuty() * 100.0f;
  out_.lambda = lambda_;
  out_.biasLeft = biasLeft_;
  out_.biasRight = biasRight_;
  out_.ready = begun_ && active_.fullDutyVelocity > 0.0f;
  out_.estopped = estopLatch_;
  out_.stallHalted = stallHalted_;
  out_.satLeft = satLeft_;
  out_.satRight = satRight_;
  out_.stallLeft = stallLatched_;
  out_.stallRight = stallLatched_;
  out_.wedgeLeft = left_.wedged();
  out_.wedgeRight = right_.wedged();
  out_.wedgeSuspectLeft = left_.wedgeSuspect();
  out_.wedgeSuspectRight = right_.wedgeSuspect();
  out_.deficitLeft = deficitLeft_;
  out_.deficitRight = deficitRight_;
  out_.connectedLeft = sampleLeft_.connected;
  out_.connectedRight = sampleRight_.connected;
  out_.leaseExpiryCount = leaseExpiryCount_;
  out_.i2cFaultCount = i2cFaultCount_;
  out_.positionEpochLeft = positionEpochLeft_;
  out_.positionEpochRight = positionEpochRight_;
  ++outSeq_;  // even: committed
}

// ---------------------------------------------------------------------------
// Ported pipeline stages (unit-parametric — identical math to the
// pre-kernel differential_drive.cpp, in counts; semantics per the
// hard-lessons ledger)
// ---------------------------------------------------------------------------

float DifferentialDrive::correctedCommand(float desired, float previous,
                                          bool leftWheel, float bias) const {
  if (desired == 0.0f) return 0.0f;  // stop is stop; never offset it
  const int w = leftWheel ? 0 : 1;
  const int d = (std::fabs(desired) > std::fabs(previous)) ? 0 : 1;
  const float magnitude =
      (std::fabs(desired) - active_.wheelIntercept[w][d]) /
      active_.wheelGain[w][d];
  if (magnitude <= 0.0f) return 0.0f;  // below the intercept: unreachable
  const float correctedMagnitude = magnitude + bias;
  if (correctedMagnitude <= 0.0f) return 0.0f;  // never flip direction
  return std::copysign(correctedMagnitude, desired);
}

float DifferentialDrive::fastPid(float posError, float err, float aCmd) const {
  const float proportional = active_.kp * err;
  const float feed = active_.kaff * aCmd;

  float integral = 0.0f;  // [counts/s]
  if (active_.iMax > 0.0f) {
    integral = active_.ki * posError;
    if (integral > active_.iMax) integral = active_.iMax;
    if (integral < -active_.iMax) integral = -active_.iMax;
  }

  float pid = proportional + feed + integral;
  if (active_.pidMax > 0.0f) {
    if (pid > active_.pidMax) pid = active_.pidMax;
    if (pid < -active_.pidMax) pid = -active_.pidMax;
  }
  if (!std::isfinite(pid)) return 0.0f;  // fail closed, never inject NaN
  return pid;
}

float DifferentialDrive::positionError(float speed, const WheelSample& wheel,
                                       PositionRef& ref, float dt) {
  if (speed == 0.0f || dt <= 0.0f || !wheel.connected ||
      ref.epoch != epoch_ || !ref.armed) {
    ref.armed = (speed != 0.0f) && wheel.connected;
    ref.epoch = epoch_;
    ref.origin = wheel.position;
    ref.reference = 0.0f;
    return 0.0f;
  }
  ref.reference += speed * dt;                                     // [counts]
  float error = ref.reference - (wheel.position - ref.origin);     // [counts]
  if (active_.posErrMax > 0.0f) {
    if (error > active_.posErrMax) error = active_.posErrMax;
    if (error < -active_.posErrMax) error = -active_.posErrMax;
  }
  return error;
}

void DifferentialDrive::adaptBias(float& bias, float err, float aCmd,
                                  float vCmdMagnitude, bool fresh,
                                  float dt) const {
  if (active_.tauAdapt <= 0.0f || dt <= 0.0f || !fresh) return;
  if (std::fabs(aCmd) >= active_.aSteady) return;  // ramping, not steady
  if (vCmdMagnitude < active_.vMin) return;        // below the speed floor
  bias += err * dt / active_.tauAdapt;
  if (active_.biasMax > 0.0f) {
    if (bias > active_.biasMax) bias = active_.biasMax;
    if (bias < -active_.biasMax) bias = -active_.biasMax;
  } else {
    bias = 0.0f;
  }
}

float DifferentialDrive::crawlDuty(float duty, float& carry) const {
  const float magnitude = std::fabs(duty);
  if (active_.crawlPulse == 0.0f || magnitude >= active_.crawlPulse) {
    return duty;
  }
  if (magnitude == 0.0f) {
    carry = 0.0f;
    return 0.0f;
  }
  carry += magnitude / active_.crawlPulse;
  if (carry < 1.0f) return 0.0f;
  carry -= 1.0f;
  return std::copysign(active_.crawlPulse, duty);
}

void DifferentialDrive::applySpeedFloor(float rawLeft, float rawRight,
                                        float& speedLeft,
                                        float& speedRight) const {
  speedLeft = rawLeft;
  speedRight = rawRight;
  // Unconditional when configured (the old owns()-gated teleop-only floor
  // is gone with the planner split: the kernel IS the one command path
  // now). Ratio-preserving: one common scale, both wheels.
  if (active_.vMin <= 0.0f) return;
  const float dominantMag =
      std::max(std::fabs(rawLeft), std::fabs(rawRight));
  if (dominantMag <= 0.0f || dominantMag >= active_.vMin) return;
  const float scale = active_.vMin / dominantMag;
  speedLeft = rawLeft * scale;
  speedRight = rawRight * scale;
}

void DifferentialDrive::updateLatch(bool conditionNow, float window,
                                    uint32_t now, uint32_t& since,
                                    bool& latched) const {
  if (window <= 0.0f || !conditionNow) {
    since = 0;
    latched = false;
    return;
  }
  if (since == 0) since = now;
  latched = (now - since) >= static_cast<uint32_t>(window);
}

}  // namespace DiffDrive

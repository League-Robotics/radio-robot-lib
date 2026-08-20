// differential_drive.h — DiffDrive::DifferentialDrive: a self-contained
// differential-drive wheel kernel. ONE class, TWO files (this header +
// differential_drive.cpp), and NOTHING else: the only include is
// <cstdint>, and the four small interfaces below are the package's OWN
// ports — the complete surface a host platform implements to run it.
//
// This is deliberately NOT derived from any firmware HAL. The firmware
// that grew this kernel connects it back through one-line forwarding
// adapters; a MakeCode/PXT package or a MicroPython C module implements
// the same four ports against its own platform instead. Same functions,
// no inheritance coupling, adapters only where integration wants them.
//
// PORTS (implement these):
//   Motor        — one wheel: staged duty writes, split-phase encoder
//                  sampling, an immediate emergency stop.
//   Clock        — monotonic microseconds.
//   Sleeper      — settle/pace sleeps + a cooperative yield.
//   FiberLauncher— start the kernel loop on its own thread of execution.
//                  OPTIONAL: a host that owns its own loop never calls
//                  start() and drives step() directly instead.
//
// Everything below the ports is the kernel itself, unchanged from the
// firmware tree (src/firm/control/) it is extracted from — the fidelity
// suite in src/tests/diffdrive/ holds the two byte-for-byte to the same
// control law.
#pragma once

#include <cstdint>

namespace DiffDrive {

// ---- Port: Motor ----------------------------------------------------
// One wheel. The device model is STAGE-THEN-EXECUTE: setDuty() only
// records the request; tick() performs the bus transaction that both
// executes the staged write AND collects the encoder. requestSample()
// begins a split-phase encoder read whose settle the kernel spends in
// Sleeper::sleepMillis(); position()/velocity() report the device's own
// native counts — the kernel is counts-native end to end and no
// millimetre exists at or below this interface.
class Motor {
 public:
  virtual ~Motor() = default;

  virtual void begin() = 0;
  virtual void requestSample() = 0;
  virtual void setDuty(float duty) = 0;   // [-1, 1] staged raw duty
  // Write zero to the device NOW, unstaged — the one call that must not
  // depend on a healthy tick(), because it exists for exactly the case
  // where the thing that calls tick() has died. Zero is never shaped.
  virtual void emergencyStop() = 0;
  virtual void tick(uint64_t nowUs) = 0;  // [us] execute staged + collect

  virtual float position() const = 0;     // [counts] accumulated
  virtual float velocity() const = 0;     // [counts/s] signed
  virtual float appliedDuty() const = 0;  // [-1, 1] last landed write
  virtual bool connected() const = 0;
  virtual uint64_t sampleTime() const = 0;  // [us] last SUCCESSFUL collect
  virtual void rebaseline() = 0;  // software re-anchor; no bus traffic

  // Wedge diagnostics — a device whose readback latched (see the Nezha
  // anti-latch contract). A platform without the failure mode returns
  // false from both.
  virtual bool wedged() const = 0;
  virtual bool wedgeSuspect() const = 0;
};

// ---- Port: Clock ----------------------------------------------------
class Clock {
 public:
  virtual ~Clock() = default;
  virtual uint64_t nowMicros() const = 0;  // [us] monotonic
};

// ---- Port: Sleeper --------------------------------------------------
class Sleeper {
 public:
  virtual ~Sleeper() = default;
  virtual void sleepMillis(uint32_t duration) = 0;  // [ms]
  virtual void yield() = 0;  // hand the processor to another task/fiber
};

// ---- Port: FiberLauncher --------------------------------------------
class FiberLauncher {
 public:
  virtual ~FiberLauncher() = default;
  // Launch a task running entry(context); entry never returns. A host
  // that drives step() itself should implement this to FAIL LOUDLY —
  // start() being called at all then signals a miswired composition.
  virtual void launch(void (*entry)(void*), void* context) = 0;
};

class DifferentialDrive {
 public:
  // Command modes (Command::mode). Not an enum class: the value crosses
  // the mailbox as a uint8_t and the names read better unscoped here.
  static constexpr uint8_t kModeNeutral = 0;
  static constexpr uint8_t kModeVelocity = 1;
  static constexpr uint8_t kModeRawDuty = 2;

  // Status — every refusal visible AT THE CALLSITE, not only as a bit in
  // the next published Output. The old silent no-write return is the
  // failure mode this exists to remove: a command that does nothing and
  // says nothing is indistinguishable from one that worked.
  enum class Status : uint8_t {
    kOk = 0,
    kRefusedUnconfigured,  // maxDuty == 0; or VELOCITY with fullDutyVelocity == 0
    kRefusedNotBegun,      // command before begin(). NOT before start(): the
                           //   host harness commands and step()s WITHOUT ever
                           //   launching the fiber, so readiness is begin()'s
                           //   to grant, not start()'s
    kRefusedEstopped,
    kRefusedNonFinite,
    kCadencePreserved,     // post-begin setConfig with a differing cyclePeriod:
                           //   block applied, frozen cadence kept
  };

  // Lease ceiling. Far below INT32_MAX ms so the wrap-safe signed compare
  // against Command::validUntil is never ambiguous.
  static constexpr uint32_t kLeaseMax = 3600000u;  // [ms]

  // Config — value type; fetch/replace by copy. All speeds counts/s.
  //
  // EVERY DEFAULT IS FAIL-CLOSED. A default-constructed Config refuses
  // BOTH modes: maxDuty == 0 means no authority at all, and
  // fullDutyVelocity == 0 additionally refuses VELOCITY. Nothing moves
  // until something configures it. This is deliberate and load-bearing —
  // an unconfigured robot that silently drives is the failure this whole
  // class of default exists to prevent.
  struct Config {
    // Authority / plant gain.
    float maxDuty = 0.0f;            // [%] authority rail (lambda scales to
                                     //   this); 0 = ALL modes refused
    float fullDutyVelocity = 0.0f;   // [counts/s] wheel rate at 100% duty;
                                     //   0 = uncalibrated → VELOCITY refused
    // Velocity PID (Stage B). tovez ships pure-I: kp=0, ki on the clamped
    // position error, kaff=0.
    float kp = 0.0f;                 // [1]
    float ki = 0.0f;                 // [1/s] on clamped position error
    float iMax = 0.0f;               // [counts/s] I-term clamp; 0 disables I
    float kaff = 0.0f;               // [s] accel feedforward
    float pidMax = 0.0f;             // [counts/s] whole-PID output clamp
    // Cross-wheel coupling.
    float twistHoldGain = 0.0f;      // [1/s] twist-integral ratio hold; 0 = off
    // Stage A wheel correction: [wheel 0=left,1=right][0=accel,1=decel].
    float wheelGain[2][2] = {{1.0f, 1.0f}, {1.0f, 1.0f}};       // [1]
    float wheelIntercept[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};  // [counts/s]
    // Stage C adaptation + Stage B bounds.
    float vMin = 0.0f;               // [counts/s] speed floor; 0 = off
    float posErrMax = 0.0f;          // [counts] position-error clamp; 0 = unclamped
    float biasMax = 0.0f;            // [counts/s] Stage C trim clamp; 0 disables
    float tauAdapt = 0.0f;           // [s] Stage C time constant; <=0 disables
    float aSteady = 0.0f;            // [counts/s^2] |aCmd| below this is steady
    // Observability latches.
    float deficitThreshold = 0.0f;   // [counts/s] 0 = detector off
    float deficitWindow = 0.0f;      // [ms]
    float stallSpeed = 0.0f;         // [counts/s]
    float stallDemand = 0.0f;        // [counts/s] 0 = detector off
    float stallWindow = 0.0f;        // [ms]
    // Authority-headroom scaling. Ships OFF so the first bench pass and
    // the golden-trace fidelity gate both run the PURE port of the old
    // pipeline -- λ is a stage the old control law never had, and leaving
    // it on from the start would make any discrepancy unattributable.
    bool lambdaEnabled = false;
    // Output shaping.
    float crawlPulse = 0.0f;         // [-1, 1] sub-breakaway pulse amplitude; 0 = off
    // Kernel cadence.
    uint32_t cyclePeriod = 24;       // [ms] fiber cadence (>= 2*kSettle + margin)
  };

  // Output — value type; output() returns a seq-consistent copy of the
  // block the kernel fiber publishes every cycle.
  struct Output {
    uint32_t now = 0;               // [ms] kernel clock at publish
    uint32_t nowFine = 0;           // [us] same instant — age-math base.
                                    //   No unit in the name (the [us] tag
                                    //   rules); named apart from now [ms].
    uint32_t cycleCount = 0;        // heartbeat — the RobotLoop sentinel
                                    //   watches THIS for advance
    uint32_t cyclePeriodMeasured = 0;  // [us] measured (feeds all dt terms).
                                    //   Named apart from Config::cyclePeriod
                                    //   [ms] so the two units cannot be
                                    //   confused at a call site.
    uint32_t cycleBusy = 0;         // [us]
    uint32_t cycleOverrunCount = 0;  // cycles that missed their absolute
                                    //   deadline — the observability half of
                                    //   lesson 17
    // Measurement timestamps: stamped at collect SUCCESS only (the
    // 131-002 rule) — a failed collect HOLDS the stamp, so age grows
    // honestly. Right is deterministically ~one settle window younger
    // than left (sequential split-phase). Age = (int32)(nowFine − t).
    uint32_t sampleTimeLeft = 0;    // [us]
    uint32_t sampleTimeRight = 0;   // [us]
    // Per-wheel software-rebaseline epoch. Bumped when THAT wheel's
    // accumulated position is rebaselined — never a device reset. Feeds
    // the telemetry encoder reading's own position_epoch so a host can
    // tell "the robot moved backwards" from "the origin moved".
    uint32_t positionEpochLeft = 0;
    uint32_t positionEpochRight = 0;
    float positionLeft = 0.0f;      // [counts] accumulated, never device-reset
    float positionRight = 0.0f;     // [counts]
    // Per-wheel velocity over the GENUINE inter-sample interval (computed
    // across successful collects only — a dead bus reads STALE here, not
    // zero; see the staleness flags below).
    float velocityLeft = 0.0f;      // [counts/s]
    float velocityRight = 0.0f;     // [counts/s]
    float velocity = 0.0f;          // [counts/s] measured mean
    float twist = 0.0f;             // [counts/s] measured half-differential, CCW+
    float appliedDutyLeft = 0.0f;   // [%]
    float appliedDutyRight = 0.0f;  // [%]
    float lambda = 1.0f;            // [1] authority scale currently applied
    // Stage C's adapted trim, published for the same reason lambda is: it is
    // the pipeline's one slow-moving LEARNED parameter, so "did adaptation
    // converge, and did it survive the last command?" is otherwise
    // unanswerable from outside. Bench-visible by design.
    float biasLeft = 0.0f;          // [counts/s]
    float biasRight = 0.0f;         // [counts/s]
    bool ready = false;             // begun + calibrated (velocity mode usable)
    bool estopped = false;
    bool leaseExpired = false;
    bool stallHalted = false;       // kernel self-halted on the stall latch
    bool satLeft = false, satRight = false;      // duty demand beyond the rail
    bool stallLeft = false, stallRight = false;
    bool wedgeLeft = false, wedgeRight = false;
    bool wedgeSuspectLeft = false, wedgeSuspectRight = false;
    bool deficitLeft = false, deficitRight = false;
    bool connectedLeft = false, connectedRight = false;
    uint32_t leaseExpiryCount = 0;  // sticky diagnostics
    // Failed-collect cycles, derived from sample-stamp NON-ADVANCE: the
    // leaf's requestSample()/tick() return void, so sampleTime() not
    // moving across a cycle is the only observable "that collect did not
    // land". Sticky, never reset — a climbing count under load is the
    // bus-health signal.
    uint32_t i2cFaultCount = 0;
  };

  // The launcher is injected HERE, at construction, alongside the other
  // seams -- not handed to start(). Every collaborator this class needs
  // arrives the same way, and "who can start a fiber" becomes a property
  // of how the object was composed rather than of who happens to call
  // start().
  DifferentialDrive(Motor& left, Motor& right,
                    const Clock& clock, Sleeper& sleeper,
                    FiberLauncher& launcher);

  // ---- config surface 1: chainable single-field setters -------------
  // Construct empty, chain setters. Live: the fiber snapshots the staged
  // config at each cycle start, so these work mid-run (bench tuning).
  DifferentialDrive& setMaxDuty(float maxDuty);            // [%]
  DifferentialDrive& setFullDutyVelocity(float velocity);  // [counts/s]
  DifferentialDrive& setKp(float kp);                      // [1]
  DifferentialDrive& setKi(float ki);                      // [1/s]
  DifferentialDrive& setIMax(float iMax);                  // [counts/s]
  DifferentialDrive& setKaff(float kaff);                  // [s]
  DifferentialDrive& setPidMax(float pidMax);              // [counts/s]
  DifferentialDrive& setTwistHoldGain(float gain);         // [1/s]
  DifferentialDrive& setWheelCorrection(
      float gainLeftAccel, float interceptLeftAccel,
      float gainLeftDecel, float interceptLeftDecel,
      float gainRightAccel, float interceptRightAccel,
      float gainRightDecel, float interceptRightDecel);    // [1]/[counts/s] x4
  DifferentialDrive& setSpeedFloor(float vMin);            // [counts/s]
  DifferentialDrive& setPositionErrorMax(float posErrMax); // [counts]
  DifferentialDrive& setAdaptation(float biasMax, float tauAdapt,
                                   float aSteady);  // [counts/s] [s] [counts/s^2]
  DifferentialDrive& setDeficit(float threshold, float window);  // [counts/s] [ms]
  DifferentialDrive& setStall(float speed, float demand,
                              float window);  // [counts/s] [counts/s] [ms]
  DifferentialDrive& setLambdaEnabled(bool enabled);
  DifferentialDrive& setCrawlPulse(float crawlPulse);      // [-1, 1]
  DifferentialDrive& setCyclePeriod(uint32_t period);      // [ms]

  // ---- config surfaces 2 + 3: whole-block replace / fetch (copies) ---
  // setConfig(): post-begin, a block carrying a DIFFERENT cyclePeriod
  // applies every other field and PRESERVES the frozen cadence, returning
  // kCadencePreserved to say so.
  Status setConfig(const Config& config);
  Config config() const;

  // ---- how a REFUSED SETTER is observed ------------------------------
  // The chainable setters above must return DifferentialDrive& to chain,
  // so they cannot return Status. Without this pair, a rejected
  // non-finite value or a post-begin() setCyclePeriod() would be refused
  // SILENTLY — exactly the behaviour Status exists to remove, reintroduced
  // across the whole config surface. Sticky: holds the FIRST refusal since
  // the last clear, so a caller can run a long chain and check once at the
  // end rather than after every call.
  Status lastError() const { return lastError_; }
  void clearLastError() { lastError_ = Status::kOk; }

  // ---- lifecycle: start the object, then start the fiber -------------
  // begin(): hardware init — primes both encoders (the 0x46 register sits
  // frozen at 0 until its first atomic read) and arms the boot zero-write
  // (the first cycles assert commanded zero — the Nezha brick latches its
  // last speed across nRF resets, so boot ALWAYS re-asserts stop). Also
  // FREEZES cyclePeriod. Returns kRefusedUnconfigured if the config still
  // has no authority (maxDuty == 0).
  Status begin();
  // start(): launch the kernel fiber, via the launcher injected at
  // construction (idempotent). A host-test harness never calls this at
  // all -- it drives step() directly -- and the host launcher fails hard
  // if it somehow is called.
  //
  // start() does NOT gate command acceptance: readiness is begin()'s to
  // grant. Gating on start() would make the golden-trace host harness
  // impossible to write, since it steps the kernel without ever launching
  // a fiber.
  Status start();
  bool running() const { return running_; }

  // ---- commands; lease is a DURATION [ms] from now — expiry stops ----
  // Lease is clamped to kLeaseMax. A refused command leaves current state
  // untouched AND returns the reason.
  Status drive(float velocity, float twist,
               uint32_t lease);       // [counts/s] [counts/s] [ms]
  Status driveDuty(float dutyLeft, float dutyRight,
                   uint32_t lease);   // [%] [%] [ms]
  void neutral();        // commanded stop through the full stop path
  void estop();          // latch: zero NOW; holds until estopClear()
  void estopClear();
  // emergencyStopMotors() -- write zero to BOTH motors NOW, from the
  // CALLER's fiber, bypassing the kernel entirely.
  //
  // This exists for exactly one caller: RobotLoop's heartbeat sentinel,
  // for the case where this kernel's own fiber has died and can therefore
  // never execute a staged stop. estop() alone is not enough there -- it
  // only sets a latch that the kernel fiber is supposed to act on.
  //
  // It lives HERE rather than in RobotLoop so that the motors stay owned
  // by one class and the "kernel fiber is the only 0x10 client" rule has
  // its single sanctioned exception in the same file that states it.
  // Emergency only: it can land between a 0x46 select and its read and
  // destroy that pending encoder sample.
  void emergencyStopMotors();
  // Clear the kernel's stall self-halt latch (the application decided the
  // jam is resolved). A fresh drive()/driveDuty() is still required.
  void clearStallLatch();
  // Software-rebaseline both encoders to ~0 at the next cycle (never a
  // device reset — the leaf's positionEpoch discipline stands).
  void rebasePosition();

  // ---- output: seq-consistent COPY of the published block ------------
  Output output() const;

  // ---- host-harness entry -------------------------------------------
  // One full kernel cycle, inline in the caller's context: snapshot →
  // safety gates → control step → encoder split-phase (sleeps via the
  // injected Sleeper) → publish. The fiber body is a loop over this plus
  // absolute-deadline pacing. Public FOR THE HOST TEST HARNESS (which has
  // no fiber); production code never calls it — start() is the one
  // production entry. Never called while the fiber is running.
  void step();

 private:
  // ---- mailbox / published blocks (see file header for the
  //      concurrency contract) ----------------------------------------
  struct Command {
    uint8_t mode = kModeNeutral;
    float velocity = 0.0f;     // [counts/s]
    float twist = 0.0f;        // [counts/s]
    float dutyLeft = 0.0f;     // [%]
    float dutyRight = 0.0f;    // [%]
    uint32_t validUntil = 0;   // [ms] absolute kernel clock; computed in drive()
  };

  // Cached per-wheel sample state (the kernel's view of each motor,
  // refreshed after each collect; feeds NEXT cycle's control step — the
  // same one-cycle actuation latency the old loop had, by design).
  struct WheelSample {
    float position = 0.0f;       // [counts]
    float velocity = 0.0f;       // [counts/s] successful-collect quotient
    uint64_t sampleTime = 0;     // [us] last SUCCESSFUL collect
    bool connected = false;
    bool everSampled = false;
  };

  // Stage B position reference — the integral-of-command integrator.
  struct PositionRef {
    float reference = 0.0f;  // [counts] integral of commanded speed since anchor
    float origin = 0.0f;     // [counts] wheel position when anchored
    uint8_t epoch = 0;       // kernel epoch when anchored (bumped on rebase)
    bool armed = false;
  };

  // Twist-integral hold reference (ratio maintenance).
  struct TwistRef {
    float reference = 0.0f;   // [counts] integral of commanded twist since anchor
    float originLeft = 0.0f;  // [counts]
    float originRight = 0.0f; // [counts]
    uint8_t epoch = 0;
    bool armed = false;
  };

  // ---- fiber ---------------------------------------------------------
  static void fiberEntry(void* self);
  void run();  // the kernel fiber body: step() + absolute-deadline pace

  // ---- cycle internals ----------------------------------------------
  // Shared refusal gate for drive()/driveDuty(). A refused command leaves
  // the mailbox UNTOUCHED, so whatever was already commanded keeps running
  // under its own lease rather than being replaced by a rejected command.
  Status checkCommandable(bool needsVelocityCalibration) const;

  void snapshotConfig();
  Command snapshotCommand() const;
  void controlStep(const Command& cmd, uint8_t effectiveMode, float dt,
                   uint32_t nowMs);  // [s] [ms]
  void stageStop();
  void stageDuty(float dutyLeft, float dutyRight);  // [-1,1] x2, write-gated
  void refreshSample(Motor& motor, WheelSample& sample);
  void resetAdaptiveState();
  void publishOutput(uint32_t nowMs, uint64_t cycleStartUs, uint64_t busyEndUs,
                     uint32_t measuredPeriod, bool leaseExpired);  // [us] x2 [us]

  // ---- ported pipeline (unit-parametric; semantics per the hard-lessons
  //      ledger — stop is stop, never flip direction, fail closed) ------
  float correctedCommand(float desired, float previous, bool leftWheel,
                         float bias) const;
  float fastPid(float posError, float err, float aCmd) const;  // [counts] [counts/s] [counts/s^2]
  float positionError(float speed, const WheelSample& wheel, PositionRef& ref,
                      float dt);  // [counts/s] [s] -> [counts]
  void adaptBias(float& bias, float err, float aCmd, float vCmdMagnitude,
                 bool fresh, float dt) const;
  float crawlDuty(float duty, float& carry) const;
  void applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                       float& speedRight) const;
  void updateLatch(bool conditionNow, float window, uint32_t now,
                   uint32_t& since, bool& latched) const;  // [ms]

  // ---- wiring --------------------------------------------------------
  Motor& left_;
  Motor& right_;
  const Clock& clock_;
  Sleeper& sleeper_;
  FiberLauncher& launcher_;

  // ---- config: staged (main-fiber writer) + active (kernel copy) -----
  Config staged_;
  Config active_;
  volatile uint32_t cfgSeq_ = 0;
  uint32_t activeCfgSeq_ = 0;

  // ---- command mailbox (main-fiber writer) ---------------------------
  Command command_;
  volatile uint32_t cmdSeq_ = 0;
  uint32_t seenCmdSeq_ = 0;

  // ---- latches OUTSIDE the seq handshake (single aligned stores) -----
  volatile bool estopLatch_ = false;

  // One-shot request counters (main-fiber writers; the kernel consumes by
  // tracking the last-seen count). Counters, not mailbox flags: a flag
  // inside Command could be lost to a concurrent drive() overwrite.
  volatile uint32_t clearStallReq_ = 0;
  volatile uint32_t rebaseReq_ = 0;
  uint32_t seenClearStallReq_ = 0;
  uint32_t seenRebaseReq_ = 0;

  // Sticky first-refusal, for the chainable setters that cannot return a
  // Status. Written from the caller's fiber only (the kernel fiber never
  // touches it), so it needs no seq protection.
  Status lastError_ = Status::kOk;
  void noteRefusal(Status status) {
    if (lastError_ == Status::kOk) lastError_ = status;
  }

  // ---- kernel-fiber state --------------------------------------------
  bool begun_ = false;
  volatile bool running_ = false;
  uint8_t epoch_ = 0;              // bumped on rebasePosition
  bool stallHalted_ = false;
  bool wasForcedStop_ = false;     // edge detector for adaptive reset
  bool leaseWasLive_ = false;      // edge detector for leaseExpiryCount

  WheelSample sampleLeft_;
  WheelSample sampleRight_;

  PositionRef posRefLeft_;
  PositionRef posRefRight_;
  TwistRef twistRef_;

  // Per-wheel rebaseline epochs published on Output. Separate from
  // epoch_ (which is the INTEGRATOR re-anchor generation, bumped on the
  // same event but consumed internally): the published epochs are a
  // host-facing contract, and keeping them apart means a future
  // single-wheel rebaseline does not have to re-anchor both integrators.
  uint32_t positionEpochLeft_ = 0;
  uint32_t positionEpochRight_ = 0;
  uint32_t i2cFaultCount_ = 0;     // failed-collect cycles, sticky
  uint32_t cycleOverrunCount_ = 0;  // missed absolute deadlines, sticky

  float biasLeft_ = 0.0f;          // [counts/s] Stage C's adapted parameter
  float biasRight_ = 0.0f;         // [counts/s]
  float lastSpeedLeft_ = 0.0f;     // [counts/s] Stage A direction-of-change memory
  float lastSpeedRight_ = 0.0f;    // [counts/s]
  float lastPidLeft_ = 0.0f;       // [counts/s]
  float lastPidRight_ = 0.0f;      // [counts/s]
  float crawlCarryLeft_ = 0.0f;    // Bresenham accumulators
  float crawlCarryRight_ = 0.0f;
  // Commanded-accel EMA (kaff feedforward + the aSteady adaptation gate).
  float previousTargetLeft_ = 0.0f;   // [counts/s]
  float previousTargetRight_ = 0.0f;  // [counts/s]
  float cmdAccelLeft_ = 0.0f;         // [counts/s^2] smoothed
  float cmdAccelRight_ = 0.0f;        // [counts/s^2]
  bool satLeft_ = false;              // duty demand beyond the rail
  bool satRight_ = false;
  uint32_t leaseExpiryCount_ = 0;     // sticky diagnostics

  // Authority feedback: last cycle's PRE-CLAMP duty demands feed this
  // cycle's lambda (1-tick lag, same philosophy Stage C already uses).
  float dutyDemandLeft_ = 0.0f;    // [-1,1] fraction, unclamped magnitude kept
  float dutyDemandRight_ = 0.0f;   // [-1,1]
  float lambda_ = 1.0f;            // [1] filtered authority scale

  uint32_t deficitSinceLeft_ = 0;  // [ms]
  uint32_t deficitSinceRight_ = 0; // [ms]
  bool deficitLeft_ = false;
  bool deficitRight_ = false;
  uint32_t stallSince_ = 0;        // [ms] one condition, both wheels latch
  bool stallLatched_ = false;

  // Stop-enforce machinery (ported): a commanded stop is re-written for
  // kStopEnforceTicks after the transition, and unconditionally while the
  // encoders still read motion.
  float writtenLeft_ = 0.0f;       // [-1, 1]
  float writtenRight_ = 0.0f;      // [-1, 1]
  uint8_t stopEnforceCountdown_ = 0;

  // ---- published output ---------------------------------------------
  Output out_;
  volatile uint32_t outSeq_ = 0;

  // ---- cycle timing --------------------------------------------------
  uint64_t previousCycleStartUs_ = 0;  // [us]
  bool everCycled_ = false;
  uint32_t cycleCount_ = 0;

  // ---- constants -----------------------------------------------------
  // The brick's mandatory encoder select→read settle, one window per
  // motor. NOT optional: I2CBus's clearance timers enforce the same
  // wait from requestSample()'s postClear — sleeping it here spends the
  // wait as a fiber yield (the main fiber's comms pump runs in it).
  static constexpr uint32_t kSettle = 4;          // [ms]
  static constexpr uint8_t kStopEnforceTicks = 30;
  // Rest threshold for the stop-enforce gate. COUNTS REBAKE: was 8 mm/s
  // (~102 counts/s at tovez's 0.7837 mm/deg); rounded to 100.
  static constexpr float kRestVelocity = 100.0f;  // [counts/s]
  static constexpr float kMaxSampleAge = 200000.0f;  // [us] freshness gate
  static constexpr float kAccelSmoothing = 0.35f;    // [1] cmdAccel EMA weight
  // Lambda filter: fast attack (immediate min), slow release.
  static constexpr float kLambdaReleaseTau = 0.3f;   // [s]
  // adaptBias is gated off while authority-limited (learning under a
  // saturated rail adapts garbage).
  static constexpr float kLambdaAdaptFloor = 0.95f;  // [1]
};

}  // namespace DiffDrive

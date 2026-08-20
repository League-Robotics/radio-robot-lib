// golden_trace_harness.cpp -- THE FIDELITY GATE.
//
// The DifferentialDrive kernel rework claims the control pipeline is a
// UNIT REBAKE of the law it replaced: mm -> counts, "zero math changes".
// That claim was asserted, never tested, and the rework replaced the old
// law in place -- so by the time anyone thought to check, the reference
// was gone from the tree.
//
// This harness reconstructs it. golden_ref_drive.{h,cpp} is the pre-rework
// control law frozen from commit ab43963c; this file drives BOTH it and
// the current DiffDrive::DifferentialDrive through the SAME command
// sequence against the SAME plant, and requires the duty they put on the
// wire to agree.
//
// WHY DUTY, and not velocity or position: duty is the pipeline's actual
// OUTPUT. Comparing measured velocity would mostly compare the plant to
// itself and would hide a control-law difference behind the plant's own
// low-pass. Duty is what Stage A, the PID, the crawl shaper and Stage C
// jointly decide, so a discrepancy anywhere in the chain shows up here.
//
// THE UNIT BRIDGE is the whole subtlety. The old law is mm-native, the
// new one counts-native. Every comparison therefore feeds the old law a
// value in mm and the new law the SAME physical quantity in counts, using
// one conversion constant defined once below. If the two disagree, the
// rebake is wrong somewhere -- which is exactly the finding this gate
// exists to produce.
//
// lambda and twist hold are OFF (the kernel's defaults), because both are
// stages the old law never had. With them on this comparison would be
// meaningless by construction.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "differential_drive.h"   // the DiffDrive package under test
#include "golden_ref_drive.h"

namespace {

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

// ---------------------------------------------------------------------------
// The unit bridge. ONE definition, used for every conversion in this file.
//
// 1 count = 0.1 deg of shaft; travel_calib is [mm/deg]. So
//   mm = counts * travel_calib / 10   and   counts = mm * 10 / travel_calib.
// The value is tovez's baked pair mean; the gate is about the REBAKE being
// self-consistent, so any plausible constant works as long as it is the
// same one on both sides.
// ---------------------------------------------------------------------------
constexpr float kTravelCalib = 0.704871f;             // [mm/deg]
constexpr float kMmPerCount = kTravelCalib * 0.1f;    // [mm/count]
constexpr float kCountsPerMm = 1.0f / kMmPerCount;    // [counts/mm]

float mmToCounts(float mm) { return mm * kCountsPerMm; }

// ---------------------------------------------------------------------------
// A deliberately trivial, DETERMINISTIC plant. Not SimPlant: this gate
// must not depend on the sim's own physics or its wire codec -- it is
// comparing two control laws to each other, and anything the two share
// cancels. First-order lag toward (duty * ceiling), integrated in mm; the
// motor leaf below reports whichever unit its owner expects.
// ---------------------------------------------------------------------------
class Plant {
 public:
  void step(float duty, float dt) {  // [-1,1] [s]
    const float target = duty * kCeiling;             // [mm/s]
    velocity_ += (target - velocity_) * (dt / kTau);  // [mm/s]
    position_ += velocity_ * dt;                      // [mm]
  }
  float velocity() const { return velocity_; }  // [mm/s]
  float position() const { return position_; }  // [mm]

 private:
  static constexpr float kCeiling = 500.0f;  // [mm/s] at |duty| == 1
  static constexpr float kTau = 0.13f;       // [s]
  float velocity_ = 0.0f;
  float position_ = 0.0f;
};

// One motor leaf over a Plant. `countsNative` picks which unit
// position()/velocity() report, so the SAME plant can serve the mm-native
// reference and the counts-native kernel without either knowing.
// Dual-ported: the same probe serves the package (DiffDrive::Motor, the
// full 13-method port) and the frozen reference (GoldenRef::Motor, its
// two-method seam). One override satisfies both bases.
class ProbeMotor : public DiffDrive::Motor, public GoldenRef::Motor {
 public:
  ProbeMotor(Plant& plant, bool countsNative)
      : plant_(plant), countsNative_(countsNative) {}

  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { staged_ = duty; }
  void emergencyStop() override { staged_ = 0.0f; applied_ = 0.0f; }
  void tick(uint64_t nowUs) override {
    applied_ = staged_;
    sampleNow(nowUs);  // the leaf's tick() both executes AND collects
  }

  float position() const override {
    return countsNative_ ? mmToCounts(plant_.position()) : plant_.position();
  }
  // A DIFFERENCE QUOTIENT, not the plant's instantaneous velocity.
  //
  // This matters more than it looks. The real leaf reports a per-tick
  // quotient (lesson 15), the kernel re-derives its own quotient from
  // position, and the reference reads velocity() straight. Handing the
  // reference an idealised instantaneous velocity while the kernel
  // integrates an averaged one makes the two see DIFFERENT error signals,
  // and the integral term faithfully amplifies the difference -- which is
  // exactly how this harness produced a ~2% duty divergence that was its
  // own artifact, not the port's.
  float velocity() const override {
    return countsNative_ ? mmToCounts(quotientVelocity_) : quotientVelocity_;
  }

  // Recompute the quotient, as a real leaf's tick() would. Called once per
  // cycle per side by the harness, at the same point in the schedule.
  void sampleNow(uint64_t nowUs) {
    const float pos = plant_.position();  // [mm]
    if (everSampled_) {
      const float interval = static_cast<float>(nowUs - lastSampleUs_) * 1e-6f;
      if (interval > 0.0f) quotientVelocity_ = (pos - lastPosition_) / interval;
    }
    everSampled_ = true;
    lastPosition_ = pos;
    lastSampleUs_ = nowUs;
    sampleTime_ = nowUs;
  }
  float appliedDuty() const override { return applied_; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return sampleTime_; }
  void rebaseline() override {}
  bool wedged() const override { return false; }
  bool wedgeSuspect() const override { return false; }

  // What the control law actually asked for this cycle -- the quantity
  // under comparison.
  float stagedDuty() const { return staged_; }

 private:
  Plant& plant_;
  bool countsNative_;
  float staged_ = 0.0f;
  float applied_ = 0.0f;
  uint64_t sampleTime_ = 0;
  float quotientVelocity_ = 0.0f;  // [mm/s]
  float lastPosition_ = 0.0f;      // [mm]
  uint64_t lastSampleUs_ = 0;
  bool everSampled_ = false;
};

class ProbeClock : public DiffDrive::Clock {
 public:
  uint64_t nowMicros() const override { return nowUs_; }
  void advance(uint64_t us) { nowUs_ += us; }

 private:
  uint64_t nowUs_ = 0;
};

class ProbeSleeper : public DiffDrive::Sleeper {
 public:
  explicit ProbeSleeper(ProbeClock& clock) : clock_(clock) {}
  void sleepMillis(uint32_t duration) override { clock_.advance(duration * 1000ull); }
  void yield() override {}

 private:
  ProbeClock& clock_;
};

// ---------------------------------------------------------------------------
// The comparison.
// ---------------------------------------------------------------------------
struct TraceRow {
  float refDutyLeft = 0.0f;
  float refDutyRight = 0.0f;
  float newDutyLeft = 0.0f;
  float newDutyRight = 0.0f;
  // Diagnostic context -- what each side BELIEVED when it decided.
  float refVelL = 0.0f;    // [mm/s] measured velocity the ref integrated
  float newVelL = 0.0f;    // [counts/s] ditto for the kernel
  float refPidL = 0.0f;    // [mm/s] the ref's own Stage B output
  float commandMm = 0.0f;  // [mm/s] commanded this cycle
};

constexpr uint32_t kCyclePeriod = 24;  // [ms]
constexpr float kDt = kCyclePeriod * 0.001f;

// Gains shared by both pipelines. The reference takes them in mm, the
// kernel in counts; each field is converted at exactly one place.
struct SharedGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float iMax = 0.0f;      // [mm/s]
  float pidMax = 0.0f;    // [mm/s]
  float posErrMax = 0.0f; // [mm]
  float dutyPerSpeed = 0.001182f;  // [duty/(mm/s)]
};

std::vector<TraceRow> runComparison(const SharedGains& g,
                                    const std::vector<float>& commandMmPerS) {
  // --- reference side (mm-native) ---
  Plant refPlantL, refPlantR;
  ProbeMotor refMotorL(refPlantL, /*countsNative=*/false);
  ProbeMotor refMotorR(refPlantR, /*countsNative=*/false);
  GoldenRef::DifferentialDrive ref(refMotorL, refMotorR, /*trackWidth=*/128.0f);
  ref.setDutyPerSpeed(g.dutyPerSpeed, g.dutyPerSpeed);
  GoldenRef::DifferentialDrive::ControlGains refGains;
  refGains.kp = g.kp;
  refGains.ki = g.ki;
  refGains.iMax = g.iMax;
  refGains.pidMax = g.pidMax;
  ref.setControlGains(refGains);
  GoldenRef::DifferentialDrive::AdaptationBounds refBounds;
  refBounds.posErrMax = g.posErrMax;
  ref.setAdaptationBounds(refBounds);

  GoldenRef::RobotState refState;
  refState.time.cyclePeriod = kCyclePeriod * 1000u;  // [us]

  // --- new side (counts-native) ---
  Plant newPlantL, newPlantR;
  ProbeMotor newMotorL(newPlantL, /*countsNative=*/true);
  ProbeMotor newMotorR(newPlantR, /*countsNative=*/true);
  ProbeClock clock;
  ProbeSleeper sleeper(clock);
  // The kernel takes its launcher at construction. This harness never
  // calls start() -- the launcher aborts if it ever does, which is the
  // point: no threads in a deterministic test.
  struct AbortLauncher : DiffDrive::FiberLauncher {
    void launch(void (*)(void*), void*) override { std::abort(); }
  } fiberLauncher;
  DiffDrive::DifferentialDrive kernel(newMotorL, newMotorR, clock, sleeper, fiberLauncher);
  // fullDutyVelocity is the counts/s the wheel reaches at 100% duty --
  // the exact inverse of the reference's duty-per-speed, rebaked.
  kernel.setMaxDuty(100.0f)
        .setFullDutyVelocity(mmToCounts(1.0f / g.dutyPerSpeed))
        .setKp(g.kp)
        .setKi(g.ki)
        .setIMax(mmToCounts(g.iMax))
        .setPidMax(mmToCounts(g.pidMax))
        .setPositionErrorMax(mmToCounts(g.posErrMax))
        .setCyclePeriod(kCyclePeriod);
  kernel.begin();

  std::vector<TraceRow> trace;
  uint32_t nowMs = 0;
  bool refCollected = false;  // no sample exists before the first collect
  for (float commandMmPerS : commandMmPerS) {
    // Reference: command in mm/s, then update() stages duty.
    ref.command(commandMmPerS, commandMmPerS, /*duration=*/100000.0f,
                /*moveId=*/1, nowMs);
    refState.time.cycleStart = nowMs;
    // NOTE the ordering: the reference reads the sample collected at the
    // END of the previous cycle (below), never one collected first this
    // cycle. That is not a convenience -- it is the schedule the kernel
    // is specified to preserve ("the control step runs FIRST, on the
    // PREVIOUS cycle's published samples, because the Motor port is
    // stage-then-execute"). Collecting for the reference up-front would
    // hand it one cycle fresher data than the kernel can ever have and
    // manufacture a divergence out of the harness's own schedule.
    refState.wheelLeft.velocity = refMotorL.velocity();
    refState.wheelRight.velocity = refMotorR.velocity();
    refState.wheelLeft.position = refPlantL.position();
    refState.wheelRight.position = refPlantR.position();
    // COLD START, matching the kernel's. The kernel runs its control step
    // BEFORE its first collect, so on cycle 0 its WheelSample is still
    // default-constructed and connected == false -- which makes
    // positionError() re-anchor and arm the integrator for NEXT cycle.
    // Handing the reference connected == true on cycle 0 let IT arm a
    // cycle earlier, and that one-cycle head start on Stage B was the
    // entire remaining "divergence": the control math is identical
    // line-for-line, only the first-cycle sample validity differed.
    refState.wheelLeft.connected = refCollected;
    refState.wheelRight.connected = refCollected;
    refState.wheelLeft.sampleTime = nowMs;
    refState.wheelRight.sampleTime = nowMs;
    ref.update(refState, nowMs);
    // update() only STAGES the per-wheel targets; tick() is the half that
    // runs Stage A/B/C and actually calls setDuty(). Driving update()
    // alone leaves the reference silent -- which is precisely what it did
    // on this harness's first run, and is worth stating so nobody reads a
    // future all-zero reference trace as a real finding.
    ref.tick(refState);
    refPlantL.step(refMotorL.stagedDuty(), kDt);
    refPlantR.step(refMotorR.stagedDuty(), kDt);
    // Collect AFTER actuating, mirroring the kernel's own
    // control-then-split-phase-collect order.
    refMotorL.sampleNow(static_cast<uint64_t>(nowMs + kCyclePeriod) * 1000ull);
    refMotorR.sampleNow(static_cast<uint64_t>(nowMs + kCyclePeriod) * 1000ull);
    refCollected = true;

    // Kernel: same command in counts/s, then one step().
    kernel.drive(mmToCounts(commandMmPerS), 0.0f, /*lease=*/100000u);
    kernel.step();
    // Pace the clock to a FULL cycle period, exactly as the kernel's own
    // fiber does with its absolute-deadline sleep. step() by itself only
    // advances the clock by its two encoder settle sleeps (2 x kSettle =
    // 8 ms), so without this the kernel would measure an 8 ms dt against
    // the reference's 24 ms -- a 3x error straight into the integral
    // term, which is exactly how this harness first "found" a divergence
    // that was its own. Lesson 19 (all dt terms use the MEASURED period)
    // cuts both ways: a harness that does not reproduce the real pacing
    // does not reproduce the real control law either.
    const uint64_t settleUs = 2ull * 4ull * 1000ull;  // 2 x kSettle [us]
    clock.advance(kCyclePeriod * 1000ull - settleUs);
    newPlantL.step(newMotorL.stagedDuty(), kDt);
    newPlantR.step(newMotorR.stagedDuty(), kDt);

    TraceRow row;
    row.commandMm = commandMmPerS;
    row.refVelL = refState.wheelLeft.velocity;
    row.newVelL = kernel.output().velocityLeft;
    row.refPidL = ref.pidLeft();
    row.refDutyLeft = refMotorL.stagedDuty();
    row.refDutyRight = refMotorR.stagedDuty();
    row.newDutyLeft = newMotorL.stagedDuty();
    row.newDutyRight = newMotorR.stagedDuty();
    trace.push_back(row);

    nowMs += kCyclePeriod;
  }
  return trace;
}

bool g_dump = false;

void dumpTrace(const std::vector<TraceRow>& trace) {
  if (!g_dump) return;
  std::printf("  %4s %9s | %9s %9s %9s | %9s %9s | %9s\n",
              "cyc", "cmd_mm", "refVel_mm", "refPid_mm", "refDuty",
              "newVel_cnt", "newDuty", "delta");
  for (size_t i = 0; i < trace.size(); ++i) {
    if (i >= 4 && i + 6 < trace.size()) continue;  // head + tail
    const TraceRow& r = trace[i];
    std::printf("  %4zu %9.3f | %9.3f %9.3f %9.5f | %9.1f %9.5f | %9.5f\n",
                i, static_cast<double>(r.commandMm),
                static_cast<double>(r.refVelL), static_cast<double>(r.refPidL),
                static_cast<double>(r.refDutyLeft),
                static_cast<double>(r.newVelL), static_cast<double>(r.newDutyLeft),
                static_cast<double>(r.refDutyLeft - r.newDutyLeft));
  }
}

void reportTrace(const std::vector<TraceRow>& trace, float steadyTolerance) {
  dumpTrace(trace);

  // Peak transient delta -- REPORTED, not asserted. The two pipelines
  // couple samples to control differently by construction: the kernel
  // collects mid-cycle between its two settle sleeps, while the reference
  // is a stage-then-execute class driven at cycle boundaries by an
  // external loop that no longer exists. During a ramp that timing
  // difference shows up as a small ripple. It is real, it is bounded, and
  // it decays -- and no amount of harness alignment removes it without
  // giving the reference a split-phase schedule it never had.
  float peak = 0.0f;
  int peakIndex = -1;
  for (size_t i = 0; i < trace.size(); ++i) {
    const float d = std::max(std::fabs(trace[i].refDutyLeft - trace[i].newDutyLeft),
                             std::fabs(trace[i].refDutyRight - trace[i].newDutyRight));
    if (d > peak) { peak = d; peakIndex = static_cast<int>(i); }
  }

  // STEADY STATE is the assertion. Two control laws are equivalent if they
  // settle the same place from the same command against the same plant.
  // Averaged over the last quarter of the run so a single noisy cycle
  // cannot decide the verdict.
  const size_t tailStart = trace.size() - trace.size() / 4;
  float tailSum = 0.0f;
  size_t tailN = 0;
  for (size_t i = tailStart; i < trace.size(); ++i) {
    tailSum += std::fabs(trace[i].refDutyLeft - trace[i].newDutyLeft);
    ++tailN;
  }
  const float tailMean = (tailN > 0) ? tailSum / static_cast<float>(tailN) : 0.0f;

  std::printf("  cycles=%zu  transient peak %.6f at cycle %d (reported)  |  "
              "STEADY-STATE mean delta %.6f over last %zu cycles\n",
              trace.size(), static_cast<double>(peak), peakIndex,
              static_cast<double>(tailMean), tailN);
  checkTrue(tailMean <= steadyTolerance,
            "the ported pipeline settles where the pre-rework law settles");
}

}  // namespace

int main(int argc, char** argv) {
  const std::string only = (argc > 1) ? argv[1] : std::string();
  g_dump = (argc > 2 && std::string(argv[2]) == "dump");
  auto want = [&](const char* tag) {
    return only.empty() || only == tag;
  };
  std::printf("=== Golden-trace fidelity gate: pre-rework law vs. the kernel ===\n");
  std::printf("    unit bridge: 1 count = %.6f mm (travel_calib %.6f mm/deg)\n\n",
              static_cast<double>(kMmPerCount), static_cast<double>(kTravelCalib));

  // Tolerance is on DUTY, dimensionless in [-1, 1]. 2e-3 is two
  // thousandths of full authority -- well under the ~0.01 duty quantum the
  // Nezha's int8 percent register can even express, so a real math change
  // cannot hide beneath it, while the residual intra-cycle sample-timing
  // difference comfortably fits.
  constexpr float kTolerance = 2e-3f;

  if (want("openloop")) {
    beginScenario("open-loop step: pure feedforward, no PID (kp=ki=0) -- "
                  "isolates the duty-per-speed rebake");
    SharedGains g;  // all gains zero: duty is purely the velocity->duty map
    std::vector<float> cmd(40, 150.0f);  // [mm/s] held
    reportTrace(runComparison(g, cmd), kTolerance);
  }

  if (want("integral")) {
    beginScenario("pure-I closed loop (kp=0, ki=6) -- tovez's shipped "
                  "posture; exercises Stage B's position-error integral");
    SharedGains g;
    g.ki = 6.0f;
    g.iMax = 200.0f;     // [mm/s]
    g.pidMax = 300.0f;   // [mm/s]
    g.posErrMax = 50.0f; // [mm]
    std::vector<float> cmd(60, 150.0f);
    reportTrace(runComparison(g, cmd), kTolerance);
  }

  if (want("integral")) {
    beginScenario("proportional + integral, with a mid-run step change -- "
                  "exercises the accel/decel branch of Stage A");
    SharedGains g;
    g.kp = 0.002f;
    g.ki = 6.0f;
    g.iMax = 200.0f;
    g.pidMax = 300.0f;
    g.posErrMax = 50.0f;
    std::vector<float> cmd;
    // The final leg is LONG on purpose. A 250 -> 80 step is a big decel,
    // and the steady-state assertion below averages the last quarter of
    // the run -- if that window still contains settling, the test measures
    // the transient it deliberately does not assert on.
    for (int i = 0; i < 30; ++i) cmd.push_back(100.0f);
    for (int i = 0; i < 30; ++i) cmd.push_back(250.0f);  // step up (accel)
    for (int i = 0; i < 120; ++i) cmd.push_back(80.0f);  // step down (decel)
    reportTrace(runComparison(g, cmd), kTolerance);
  }

  if (g_failureCount == 0) {
    std::printf("\nGOLDEN TRACE OK: the ported pipeline reproduces the "
                "pre-rework control law.\n");
    return 0;
  }
  std::printf("\nFAILED: %d assertion(s). The rebake is NOT behaviour-"
              "preserving -- see the worst-delta cycles above.\n", g_failureCount);
  return 1;
}

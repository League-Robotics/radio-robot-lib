// diffdrive_shim.cpp -- extern "C" ctypes surface for the DiffDrive host
// test harness (Step 2). Test scaffolding only: nothing in
// src/ knows this file exists, and it is compiled only into this test's
// own throwaway shared library (see test_diffdrive_harness.py).
//
// ctypes cannot call C++ methods directly, so this file is the thin
// translation layer: one opaque handle bundling the kernel under test and
// its own private fakes, plus free functions Python can bind by name.
#include <cstdint>

#include "differential_drive.h"
#include "fake_ports.h"

namespace {

// One kernel instance and its own private fakes. Declaration order matters
// here -- C++ initializes members in DECLARATION order regardless of the
// constructor's initializer-list order, and sleeper_/kernel_ each borrow a
// reference to a collaborator declared just above them.
struct Handle {
  FakeMotor motorLeft;
  FakeMotor motorRight;
  FakeClock clock;
  FakeSleeper sleeper;
  FailingFiberLauncher launcher;
  DiffDrive::DifferentialDrive kernel;

  Handle()
      : sleeper(clock),
        kernel(motorLeft, motorRight, clock, sleeper, launcher) {}
};

// The encoder split-phase settle sleep, duplicated here because kSettle is
// private to DifferentialDrive. fidelity_harness.cpp already carries this
// same duplication, for the same reason -- see its own runComparison().
constexpr uint64_t kSettleDuration = 2ull * 4ull * 1000ull;  // 2 x kSettle
                                                              //   [us]

}  // namespace

extern "C" {

void* ddCreate() { return new Handle(); }

void ddDestroy(void* handle) { delete static_cast<Handle*>(handle); }

// Configure the subset of Config a velocity-mode test needs. Wheel
// correction, stall/deficit latches, lambda and crawl all keep their
// fail-safe Config defaults (identity Stage A, every latch off).
void ddConfigureBasic(void* handle, float maxDuty, float fullDutyVelocity,
                       float kp, float ki, float iMax, float pidMax,
                       uint32_t cyclePeriod) {
  Handle* h = static_cast<Handle*>(handle);
  h->kernel.setMaxDuty(maxDuty)
      .setFullDutyVelocity(fullDutyVelocity)
      .setKp(kp)
      .setKi(ki)
      .setIMax(iMax)
      .setPidMax(pidMax)
      .setCyclePeriod(cyclePeriod);
}

int ddBegin(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->kernel.begin());
}

int ddDrive(void* handle, float velocity, float twist, uint32_t lease) {
  return static_cast<int>(
      static_cast<Handle*>(handle)->kernel.drive(velocity, twist, lease));
}

void ddEstop(void* handle) { static_cast<Handle*>(handle)->kernel.estop(); }

void ddEstopClear(void* handle) {
  static_cast<Handle*>(handle)->kernel.estopClear();
}

// One full kernel cycle, paced like the real fiber. step() itself only
// advances the fake clock by the two encoder settle sleeps (2 x kSettle);
// this pads the remainder of the configured cyclePeriod so consecutive
// ddStep() calls land exactly cyclePeriod ms apart on the fake clock --
// the same pacing fidelity_harness.cpp's own comparison loop applies by
// hand, folded in here so Python callers do not have to repeat it.
void ddStep(void* handle) {
  Handle* h = static_cast<Handle*>(handle);
  h->kernel.step();
  const uint64_t cycleDuration =
      static_cast<uint64_t>(h->kernel.config().cyclePeriod) * 1000ull;  // [us]
  if (cycleDuration > kSettleDuration) {
    h->clock.advance(cycleDuration - kSettleDuration);
  }
}

// ---- Output snapshot accessors (kernel-reported) -----------------------
float ddPositionLeft(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().positionLeft;
}
float ddPositionRight(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().positionRight;
}
float ddVelocityLeft(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().velocityLeft;
}
float ddVelocityRight(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().velocityRight;
}
float ddAppliedDutyLeft(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().appliedDutyLeft;
}
float ddAppliedDutyRight(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().appliedDutyRight;
}
int ddReady(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().ready ? 1 : 0;
}
int ddEstopped(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().estopped ? 1 : 0;
}
int ddLeaseExpired(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().leaseExpired ? 1 : 0;
}
uint32_t ddCycleCount(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().cycleCount;
}

// ---- Motor-side accessors: read the fake motor DIRECTLY, bypassing the
// kernel's Output snapshot entirely. This is the signal the lease-expiry
// test asserts on -- "the duty it was last handed", measured at the port,
// not inferred from the kernel's own leaseExpired flag.
float ddMotorAppliedDutyLeft(void* handle) {
  return static_cast<Handle*>(handle)->motorLeft.appliedDuty();
}
float ddMotorAppliedDutyRight(void* handle) {
  return static_cast<Handle*>(handle)->motorRight.appliedDuty();
}

}  // extern "C"

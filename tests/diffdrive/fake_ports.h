// fake_ports.h -- deterministic Motor/Clock/Sleeper/FiberLauncher fakes for
// the DiffDrive host test harness (docs/plan.md Step 2). Test scaffolding
// only: nothing in src/ knows this file exists, and it is never linked
// into anything but this test's own shared library.
//
// FiberLauncher is DECLINED on purpose (see differential_drive.h's own
// port doc comment): this harness drives the kernel by calling step()
// from the test's own loop -- single-threaded and repeatable -- and never
// calls DifferentialDrive::start(). FailingFiberLauncher::launch() fails
// LOUDLY (aborts) rather than silently no-op-ing, so a miswired test that
// somehow does call start() is caught immediately instead of hanging or
// racing a thread nothing else expects.
#pragma once

#include <cstdint>
#include <cstdlib>

#include "differential_drive.h"

// ---------------------------------------------------------------------------
// A deliberately trivial, deterministic plant: first-order lag toward
// (duty * ceiling), integrated. Same shape as fidelity_harness.cpp's own
// Plant class -- reused rather than reinvented, per docs/plan.md Step 2.
// Counts-native throughout: unlike the fidelity gate, this harness has no
// mm/counts bridge to maintain, because nothing outside this file ever
// sees millimetres.
// ---------------------------------------------------------------------------
class FakePlant {
 public:
  void step(float duty, float dt) {  // [-1,1] [s]
    const float target = duty * kCeiling;             // [counts/s]
    velocity_ += (target - velocity_) * (dt / kTau);  // [counts/s]
    position_ += velocity_ * dt;                       // [counts]
  }
  float velocity() const { return velocity_; }  // [counts/s]
  float position() const { return position_; }  // [counts]

 private:
  static constexpr float kCeiling = 1000.0f;  // [counts/s] at |duty| == 1
  static constexpr float kTau = 0.13f;        // [s]
  float velocity_ = 0.0f;
  float position_ = 0.0f;
};

// One wheel: DiffDrive::Motor over a FakePlant. Honors the two obligations
// docs/design/diffdrive.md S2.1 calls out explicitly:
//
//   - sampleTime() stamps on collect SUCCESS only. This fake's tick()
//     always succeeds (there is no simulated bus to fail), so the stamp
//     always advances -- but it is written in exactly ONE place, inside
//     tick()'s collect step, never anywhere else, so the contract holds
//     structurally rather than by accident.
//   - rebaseline() is a software re-anchor and issues no bus traffic: it
//     only moves this wheel's own origin_, never touches the plant.
class FakeMotor : public DiffDrive::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { staged_ = duty; }  // [-1,1]
  void emergencyStop() override {
    staged_ = 0.0f;
    applied_ = 0.0f;
  }
  // tick() is stage-then-execute: land the staged duty, advance the plant
  // by the time elapsed since THIS wheel's own last tick (never a caller-
  // supplied dt -- a fake with real physics has to track its own clock
  // the way a real leaf would), then collect.
  void tick(uint64_t now) override {  // [us]
    applied_ = staged_;
    const float dt = everTicked_
                          ? static_cast<float>(now - lastTickTime_) * 1e-6f
                          : 0.0f;  // [s]
    lastTickTime_ = now;
    everTicked_ = true;
    if (dt > 0.0f) plant_.step(applied_, dt);
    sampleTime_ = now;  // collect SUCCESS only -- see class comment
  }

  float position() const override {
    return plant_.position() - origin_;
  }  // [counts]
  float velocity() const override { return plant_.velocity(); }  // [counts/s]
  float appliedDuty() const override { return applied_; }  // [-1,1] last
                                                             //   landed write
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return sampleTime_; }  // [us]
  void rebaseline() override { origin_ = plant_.position(); }  // no bus
                                                                 //   traffic
  bool wedged() const override { return false; }
  bool wedgeSuspect() const override { return false; }

 private:
  FakePlant plant_;
  float staged_ = 0.0f;         // [-1,1]
  float applied_ = 0.0f;        // [-1,1] the duty this wheel was last handed
  float origin_ = 0.0f;         // [counts] software rebaseline anchor
  uint64_t sampleTime_ = 0;     // [us] last SUCCESSFUL collect
  uint64_t lastTickTime_ = 0;   // [us] this wheel's own previous tick()
  bool everTicked_ = false;
};

class FakeClock : public DiffDrive::Clock {
 public:
  uint64_t nowMicros() const override { return now_; }  // [us]
  void advance(uint64_t duration) { now_ += duration; }  // [us]

 private:
  uint64_t now_ = 0;  // [us]
};

class FakeSleeper : public DiffDrive::Sleeper {
 public:
  explicit FakeSleeper(FakeClock& clock) : clock_(clock) {}
  void sleepMillis(uint32_t duration) override {  // [ms]
    clock_.advance(static_cast<uint64_t>(duration) * 1000ull);
  }
  void yield() override {}

 private:
  FakeClock& clock_;
};

// DECLINED. A host harness drives step() from its own loop; start() should
// never be called here at all. If it somehow is, fail LOUDLY rather than
// silently doing nothing -- the same posture fidelity_harness.cpp's own
// AbortLauncher already takes.
class FailingFiberLauncher : public DiffDrive::FiberLauncher {
 public:
  void launch(void (*)(void*), void*) override { std::abort(); }
};

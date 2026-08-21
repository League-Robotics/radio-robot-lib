// diffdrive_protocol_shim.cpp -- extern "C" ctypes surface for the
// combined DiffDriveAdapter acceptance harness (Step 4).
// Test scaffolding only: nothing in src/ knows this file exists, and it
// is compiled only into this test's own throwaway shared library (see
// test_diffdrive_adapter.py).
//
// This is the one place all four pieces meet: the DiffDrive kernel over
// step 2's fakes (tests/diffdrive/fake_ports.h, reused verbatim), the
// ProtocolHandler over a recording sink (the same small Sink shape
// protocol_shim.cpp already uses), and the DiffDriveAdapter
// (src/adapter/diffdrive_adapter.{h,cpp}) wiring the two together --
// exactly the seam Step 4 exists to close.
#include <cstdint>
#include <cstring>
#include <string>

#include "diffdrive_adapter.h"
#include "differential_drive.h"
#include "fake_ports.h"
#include "protocol_handler.h"

namespace {

// Same shape as protocol_shim.cpp's own RecordingSink -- accumulates
// every Sink::write() so the whole test can inspect it. std::string is
// fine here: this file is host-only test scaffolding, not the
// firmware-targetable library it drives.
class RecordingSink : public Protocol::Sink {
 public:
  void write(const char* data, size_t length) override {
    buffer_.append(data, length);
  }
  const std::string& buffer() const { return buffer_; }
  void clear() { buffer_.clear(); }

 private:
  std::string buffer_;
};

// Identity strings need to outlive the adapter (adapter.h's borrowed-
// pointer contract) -- file-scope string literals have static storage
// duration, so this satisfies that trivially.
constexpr Protocol::Identity kIdentity{
    "diffdrive-adapter-test", "TESTSN0001", "differential", "step4",
    "v6-step4",
};

// One kernel instance, its own private fakes, the adapter bridging it to
// the handler, and the handler's own recording sink. Declaration order
// matters -- C++ initializes members in DECLARATION order regardless of
// the constructor's initializer-list order, and several members here
// borrow a reference to a collaborator declared just above them.
struct Handle {
  FakeMotor motorLeft;
  FakeMotor motorRight;
  FakeClock clock;
  FakeSleeper sleeper;
  FailingFiberLauncher launcher;
  DiffDrive::DifferentialDrive kernel;
  Protocol::DiffDriveAdapter adapter;
  RecordingSink sink;
  Protocol::ProtocolHandler handler;

  explicit Handle(float countsPerLength)
      : sleeper(clock),
        kernel(motorLeft, motorRight, clock, sleeper, launcher),
        adapter(kernel, countsPerLength, kIdentity),
        handler(adapter, sink) {}
};

// The encoder split-phase settle sleep, duplicated here for the same
// reason diffdrive_shim.cpp and fidelity_harness.cpp both already
// duplicate it: kSettle is private to DifferentialDrive.
constexpr uint64_t kSettleDuration = 2ull * 4ull * 1000ull;  // 2 x kSettle
                                                              //   [us]

}  // namespace

extern "C" {

void* paCreate(float countsPerLength) { return new Handle(countsPerLength); }

void paDestroy(void* handle) { delete static_cast<Handle*>(handle); }

// Configure the subset of Config a velocity-mode test needs, mirroring
// tests/diffdrive/diffdrive_shim.cpp's ddConfigureBasic() -- maxDuty,
// fullDutyVelocity and cyclePeriod are NOT reachable through
// GET/SET (see diffdrive_adapter.h), so a test harness arms them here,
// out-of-band, exactly as a real application would at boot.
void paConfigureBasic(void* handle, float maxDuty, float fullDutyVelocity,
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

int paBegin(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->kernel.begin());
}

// ---- protocol side --------------------------------------------------

void paFeed(void* handle, const char* data, int length) {
  static_cast<Handle*>(handle)->handler.feed(data,
                                              static_cast<size_t>(length));
}

uint32_t paMalformedCount(void* handle) {
  return static_cast<Handle*>(handle)->handler.malformedCount();
}

int paSinkLength(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->sink.buffer().size());
}

int paSinkRead(void* handle, char* out, int cap) {
  Handle* h = static_cast<Handle*>(handle);
  size_t n = h->sink.buffer().size();
  if (static_cast<int>(n) > cap) n = static_cast<size_t>(cap);
  std::memcpy(out, h->sink.buffer().data(), n);
  return static_cast<int>(n);
}

void paSinkClear(void* handle) { static_cast<Handle*>(handle)->sink.clear(); }

// ---- kernel stepping + telemetry -------------------------------------

// One full kernel cycle, paced like the real fiber -- same padding
// diffdrive_shim.cpp's own ddStep() applies, so consecutive paStep()
// calls land exactly cyclePeriod ms apart on the fake clock.
void paStep(void* handle) {
  Handle* h = static_cast<Handle*>(handle);
  h->kernel.step();
  const uint64_t cycleDuration =
      static_cast<uint64_t>(h->kernel.config().cyclePeriod) * 1000ull;  // [us]
  if (cycleDuration > kSettleDuration) {
    h->clock.advance(cycleDuration - kSettleDuration);
  }
}

// Builds a Snapshot off the CURRENT kernel Output and emits it through
// the handler -- but only if the adapter's subscription actually wants
// frames (TLM:OFF, the default, wants none; spec §6.1). Returns 1 if a
// frame was emitted, 0 otherwise, so a test can tell "TLM was off" apart
// from "emitTelemetry() wrote nothing for some other reason".
int paEmitTelemetryIfEnabled(void* handle) {
  Handle* h = static_cast<Handle*>(handle);
  if (!h->adapter.telemetryEnabled()) return 0;
  h->handler.emitTelemetry(h->adapter.buildSnapshot());
  return 1;
}

// ---- readback bypassing the kernel's own Output snapshot -------------
// This is the signal the lease-expiry test asserts on -- "the duty it
// was last handed", measured at the port, not inferred from the
// kernel's leaseExpired flag or the wire's own reply text.
float paMotorAppliedDutyLeft(void* handle) {
  return static_cast<Handle*>(handle)->motorLeft.appliedDuty();
}
float paMotorAppliedDutyRight(void* handle) {
  return static_cast<Handle*>(handle)->motorRight.appliedDuty();
}
float paMotorVelocityLeft(void* handle) {
  return static_cast<Handle*>(handle)->motorLeft.velocity();
}
float paMotorVelocityRight(void* handle) {
  return static_cast<Handle*>(handle)->motorRight.velocity();
}

// ---- kernel-reported Output accessors ---------------------------------
int paLeaseExpired(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().leaseExpired ? 1 : 0;
}
int paEstopped(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().estopped ? 1 : 0;
}
uint32_t paCycleCount(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().cycleCount;
}
float paPositionLeft(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().positionLeft;
}
float paPositionRight(void* handle) {
  return static_cast<Handle*>(handle)->kernel.output().positionRight;
}

}  // extern "C"

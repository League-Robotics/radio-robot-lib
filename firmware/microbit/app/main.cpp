// main.cpp -- the micro:bit CODAL cross-compile scaffold's entry point
// (sprint 005 ticket 001, Architecture Decision 1: "thin scaffold +
// copy-in, not an application port").
//
// This is NOT a robot application and does not attempt to be one -- this
// repo is a library, owns no HAL, no radio/serial wiring, and no robot
// config. Its only job is to construct and reference DifferentialDrive,
// ProtocolHandler, and DiffDriveAdapter (protocol/, diffdrive/, adapter/,
// copied in by the Dockerfile per Decision 3) so the linker cannot dead-
// strip them, proving the three modules actually compile and link for the
// real micro:bit ARM Cortex-M4 target -- not just that CODAL itself
// builds. A downstream firmware repo that wants a real robot composition
// (HAL device drivers, radio/serial transport, boot config) builds that
// on top of this library; it is explicitly out of scope here (sprint
// 005's Out of Scope: "Porting Elite's own application/robot logic").
//
// The four DiffDrive ports (Motor/Clock/Sleeper/FiberLauncher) below are
// deliberately inert -- return fixed/zero values, touch no CODAL hardware
// API -- because nothing here ever calls DifferentialDrive::begin() or
// start(). Exercising the kernel's actual runtime behavior against real
// hardware is a downstream firmware repo's job (with its own real Motor/
// Clock/Sleeper leaves, the same shape src/diffdrive/differential_drive.h's
// own port docs describe); this scaffold only needs every symbol to
// resolve at link time.
#include "MicroBit.h"

#include <cstddef>
#include <cstdint>

#include "adapter.h"
#include "diffdrive_adapter.h"
#include "differential_drive.h"
#include "protocol_handler.h"

namespace {

// ---- DiffDrive::Motor/Clock/Sleeper/FiberLauncher: inert stand-ins ----
// (see file header -- constructed only to satisfy DifferentialDrive's
// constructor, never driven).
class NullMotor : public DiffDrive::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { (void)duty; }
  void emergencyStop() override {}
  void tick(uint64_t nowUs) override { (void)nowUs; }
  float position() const override { return 0.0f; }
  float velocity() const override { return 0.0f; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return false; }
  uint64_t sampleTime() const override { return 0; }
  void rebaseline() override {}
  bool wedged() const override { return false; }
  bool wedgeSuspect() const override { return false; }
};

class NullClock : public DiffDrive::Clock {
 public:
  uint64_t nowMicros() const override { return 0; }
};

class NullSleeper : public DiffDrive::Sleeper {
 public:
  void sleepMillis(uint32_t duration) override { (void)duration; }
  void yield() override {}
};

class NullFiberLauncher : public DiffDrive::FiberLauncher {
 public:
  // Never called -- this scaffold never calls DifferentialDrive::start().
  void launch(void (*entry)(void*), void* context) override {
    (void)entry;
    (void)context;
  }
};

// ---- Protocol::Sink: inert stand-in ----
// A real firmware wires this to a transport (serial, radio); this
// scaffold has none, so replies are simply discarded.
class NullSink : public Protocol::Sink {
 public:
  void write(const char* data, size_t length) override {
    (void)data;
    (void)length;
  }
};

}  // namespace

static MicroBit uBit;

static NullMotor motorLeft;
static NullMotor motorRight;
static NullClock clock;
static NullSleeper sleeper;
static NullFiberLauncher launcher;
static DiffDrive::DifferentialDrive drive(motorLeft, motorRight, clock,
                                          sleeper, launcher);

static const Protocol::Identity kIdentity{
    "scaffold",   // name
    "0",          // serial
    "diffdrive",  // drivetrain
    "default",    // profile
    "0.0.0",      // version
};
static Protocol::DiffDriveAdapter adapter(drive, /*countsPerLength=*/1.0f,
                                          kIdentity);

static NullSink sink;
static Protocol::ProtocolHandler handler(adapter, sink);

int main() {
  uBit.init();

  // Touch the handler so the whole DifferentialDrive -> DiffDriveAdapter
  // -> ProtocolHandler chain is reachable from main(), not just
  // constructed as unused statics -- belt-and-braces against a linker
  // aggressive enough to still prune an unreferenced global's methods.
  handler.sendBanner();
  handler.sendReady();

  release_fiber();
}

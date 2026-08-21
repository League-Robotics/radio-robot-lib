// diffdrive_adapter.h — Protocol::DiffDriveAdapter: the concrete
// Protocol::Adapter (src/protocol/adapter.h) that closes the seam this
// whole repo exists for — WHEELS/STOP/ESTOP/GET/SET/TLM over a
// DiffDrive::DifferentialDrive (src/diffdrive/differential_drive.h).
// docs/design/protocol.md §5 is the spec this file implements literally.
//
// This is the ONLY place in either library that knows a millimetre.
// DiffDrive speaks counts; the wire speaks mm/mm-per-second; this file
// holds the one conversion factor (countsPerLength) that bridges them,
// per docs/design/protocol.md §4 point 2 and docs/design/diffdrive.md
// §1.1. It is a constructor argument, not a config field — it is NOT
// reachable through GET/SET, because it is a property of the robot's
// gearing/wheel, not a tunable control-law gain (docs/design/
// protocol.md §6: "the library stores none" — this adapter follows the
// same discipline for its own one piece of state).
//
// ---- What IS reachable through GET/SET ----
//
// The 15 `wheel_control.*` fields spec §7.3 lists, mapped 1:1 by name
// onto DifferentialDrive::Config's plain-float members (kFieldTable in
// the .cpp). `maxDuty`, `fullDutyVelocity` and `cyclePeriod` are NOT in
// that spec group and are NOT wire-reachable here — they are the
// kernel's authority/calibration/cadence, and per stakeholder decision
// (2026-08-20, docs/design/protocol.md §9.3) they are hard-coded build
// constants on THIS class (`kMaxDuty`/`kFullDutyVelocity`/
// `kCyclePeriod` below), applied to the kernel at construction. Nobody
// composing this adapter has to arm them separately. Wiring them onto
// the 15-row wheel_control group would be inventing a table this step's
// design doc does not define; if a future robot needs one of them to
// vary, that is new work — giving it a real wire home — not a bug in
// this adapter.
//
// ---- Telemetry: a REDUCED projection, not a literal spec §6.3/§6.4 ----
//
// The real wire's POSE/FULL columns are world-frame pose (x/y/h) fused
// from OTOS and encoder odometry — neither of which lives in this
// library (docs/design/protocol.md §5: MOVE/GOTO/SEED/CAL, and the
// odometry they need, are explicitly out of scope). What DiffDrive
// actually publishes is per-wheel counts and counts/s, so that is what
// this adapter projects: `posl`/`posr` [mm] and `vell`/`velr`
// [mm/s x10] (same x10 convention spec §6.4 uses for `elv`/`erv`),
// converted through the one geometry factor this file owns. `TLM:FULL`
// adds `lambda`/`biasl`/`biasr`/`cyc` — the kernel's own learned-state
// and heartbeat fields, useful for a bench, absent from the spec table
// because the spec table assumes subsystems this library doesn't have.
// `flags` is a LOCAL bit layout (documented at computeFlags() in the
// .cpp) — reusing spec §6.5's bit NUMBERS while meaning something else
// at half of them (this library has no OTOS/line/colour/planner) would
// be actively misleading to a future reader with the spec open.
//
// TLM:NOW / TLM:AUTO / TLM:BUFFER all read the persisted subscription
// as if it were POSE — this adapter has no independent scheduler, and
// mode-specific cadence (AUTO's "silent while parked", BUFFER's
// REPL-side accumulation) is the calling application's job per
// protocol_handler.h's own framing ("Unsolicited emissions the app
// drives, not the wire"), not this adapter's.
#pragma once

#include <cstddef>
#include <cstdint>

#include "adapter.h"
#include "differential_drive.h"

namespace Protocol {

class DiffDriveAdapter : public Adapter {
 public:
  // WHEELS's documented ceiling (spec §5.2): "duration [ms] required,
  // ceiling 5000 — a dead host cannot mean a runaway." protocol_handler.h
  // ambiguity note #3 leaves this unenforced in the handler on purpose
  // ("the handler holds no bounds table") and hands the job to whichever
  // adapter is wired in. This is that enforcement.
  static constexpr uint32_t kWheelsDurationCeiling = 5000;  // [ms]

  // ---- Hard-coded kernel bring-up parameters (stakeholder decision,
  // 2026-08-20) ----
  //
  // docs/design/protocol.md §9.3 used to record these three Config
  // fields as wire-unreachable gaps: DifferentialDrive::begin() needs
  // maxDuty > 0 to leave its fail-closed default, and a working
  // VELOCITY-mode drive() needs fullDutyVelocity too, but neither one
  // is in spec §7.3's 15-row wheel_control group, and cyclePeriod isn't
  // either. The stakeholder's call: "I don't see that max duty, full
  // duty velocity, and cycle period need to be configurable, so you can
  // just hard code them." So this adapter arms all three itself, at
  // construction (see the .cpp) — constructing a DiffDriveAdapter is
  // now sufficient for the wrapped kernel's begin() to succeed, with no
  // external caller (test shim, application boot path) required to
  // configure them out-of-band first. If a future robot genuinely needs
  // one of these to vary, that means giving it a real wire home (a
  // config system, or a new spec row) — not just moving the constant.
  //
  // Values match what every existing test already assumed before this
  // change: tests/adapter/diffdrive_protocol_shim.cpp's paConfigureBasic()
  // and tests/diffdrive/diffdrive_shim.cpp's ddConfigureBasic() are both
  // called, from their respective Python harnesses' default arguments
  // (test_diffdrive_adapter.py's _new_handle(), test_diffdrive_harness.py's
  // _new_kernel()), with these same three numbers — the two shims agree.
  static constexpr float kMaxDuty = 100.0f;  // [%] full authority rail
  static constexpr float kFullDutyVelocity =
      1000.0f;  // [counts/s] wheel rate at 100% duty
  static constexpr uint32_t kCyclePeriod = 24;  // [ms] fiber cadence

  // `drive` is borrowed and must outlive this adapter. `countsPerLength`
  // [counts/mm] is the one piece of robot geometry this whole path needs
  // (docs/design/protocol.md §4 point 2); a non-positive value is
  // rejected and the adapter falls back to 1.0 (mm == counts) rather
  // than dividing by zero later — firmware-targetable code has no
  // exceptions to throw here. `identity` is copied by value; its own
  // pointer fields are borrowed per adapter.h's Identity contract, so
  // the CALLER's identity strings must outlive this adapter.
  DiffDriveAdapter(DiffDrive::DifferentialDrive& drive,
                    float countsPerLength, const Identity& identity);

  // Re-point the geometry conversion (e.g. a bench script swapping
  // robots without reconstructing the adapter). Ignored, not clamped
  // to a floor, if <= 0 — the previous value stands.
  void setCountsPerLength(float countsPerLength);

  // ---- Adapter ----
  void identity(Identity& out) const override;
  uint32_t now() const override;  // [ms] — DifferentialDrive::Output::now
  void status(StatusFields& out) const override;

  Result onWheels(float left, float right,  // [mm/s] [mm/s]
                  uint32_t duration,         // [ms]
                  uint32_t id) override;
  Result onStop(uint32_t id) override;   // -> DifferentialDrive::neutral()
  void onEstop() override;               // -> DifferentialDrive::estop()

  bool onGet(const char* name, float& out) const override;
  Result onSet(const char* name, float value, uint32_t id) override;
  size_t fieldCount() const override;
  const char* fieldName(size_t index) const override;

  Result onTlm(TlmMode mode) override;

  // RUN: this library registers no callable functions at all -- there is
  // no wheel-kernel operation this adapter exposes by NAME the way
  // WHEELS/STOP/ESTOP already expose it structurally. adapter.h's own
  // onRun() doc frames a concrete Adapter's registration table as the
  // security allowlist; this adapter's allowlist is empty, so every RUN
  // is ERR_UNKNOWN, same as any other name a real registration table
  // does not recognize.
  Result onRun(const char* name, const char* const* argv, size_t argc,
              char* result, size_t resultCapacity,
              bool& hasResult) override;

  // ---- telemetry projection (NOT part of Protocol::Adapter — the app
  // driving the loop calls this once per frame it wants to emit, then
  // hands the result straight to ProtocolHandler::emitTelemetry()). The
  // returned Snapshot borrows this object's own storage and is only
  // valid until the next buildSnapshot() call, matching the contract
  // adapter.h's Snapshot doc already states for any caller-owned
  // Snapshot. ----
  const Snapshot& buildSnapshot();

  // Whether the current subscription wants ANY t: frames at all (spec
  // §6.1: TLM:OFF means none). The app checks this before bothering to
  // call buildSnapshot()/emitTelemetry() each cycle.
  bool telemetryEnabled() const { return mode_ != TlmMode::kOff; }

 private:
  static constexpr size_t kMaxColumns = 12;

  DiffDrive::DifferentialDrive& drive_;
  float countsPerLength_;  // [counts/mm]
  Identity identity_;
  TlmMode mode_ = TlmMode::kOff;

  uint32_t seq_ = 0;  // wraps at 128, spec §6.2 — incremented per built frame
  Column columns_[kMaxColumns] = {};
  Snapshot snapshot_;
};

}  // namespace Protocol

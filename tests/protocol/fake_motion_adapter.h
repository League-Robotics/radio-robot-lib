// fake_motion_adapter.h -- Protocol::FakeMotionAdapter: a deterministic,
// step()-driven test double implementing every motion verb
// ProtocolHandler can dispatch (WHEELS_X/WHEELS_V/MOVE_X/MOVE_V/
// GO_TO_R/GO_TO_W, plus STOP/ESTOP) -- built to exercise the
// reliability layer's completion channel (Adapter::lastDone()/
// lastDoneReason(), docs/design/protocol.md §8.8) for the first time
// with something that actually WRITES it. DiffDriveAdapter never does
// (it has no queue and no completion event of its own); this adapter
// exists so that field stops being wire-correct-but-inert and becomes
// genuinely live.
//
// Test scaffolding only: nothing under src/ knows this file exists, and
// it is never linked into anything but this test tree's own shared
// libraries/executables. Lives in tests/protocol/ (not tests/adapter/)
// because it exercises ProtocolHandler's own dispatch/reliability logic
// directly -- it needs no DiffDrive kernel at all, the same reason
// mock_adapter.h lives here rather than in tests/adapter/.
//
// NO TIMER, NO CLOCK: a command is accepted and immediately becomes a
// deterministic countdown (`stepsToComplete`, settable before the
// command is sent), and the harness calls step() explicitly to advance
// it -- one call, one tick, always. This mirrors motion-api.md §5.2's
// own "a posted move that nobody ticks does nothing" model, pushed down
// to the wire level, and it is what makes a test's progress
// reproducible instead of racing a real clock.
//
// This is a TEST DOUBLE standing in for a real planner, not a physics
// simulation -- it does not compute wheel kinematics, encoder counts,
// or arrival tolerances. What it DOES model faithfully is the part the
// reliability layer cares about: exactly one motion active at a time,
// a completion event that fires on some step() call, and a `DoneReason`
// a test controls directly (`completionReason`) so all four reasons in
// motion-api.md §5.3 -- stop / timeout / estop / aborted -- can be
// exercised on demand. STOP and ESTOP both complete the ACTIVE motion
// immediately (with reason kStop and kEstop respectively, matching
// motion-api.md §3.7); `forceAbort()` is a test-only hook (no wire verb
// exists for it) standing in for the host-side "the caller abandoned
// it" condition (§5.1's callback/generator-closed case).
#pragma once

#include <cstddef>
#include <cstdint>

#include "adapter.h"

namespace Protocol {

class FakeMotionAdapter : public Adapter {
 public:
  enum class Kind : uint8_t {
    kNone,
    kWheelsX,
    kWheelsV,
    kMoveX,
    kMoveV,
    kGoToR,
    kGoToW,
  };

  // ---- test-controlled knobs, set BEFORE issuing a motion command ----
  // How many step() calls the NEXT accepted motion command takes to
  // finish (0 is treated as 1 -- a motion always needs at least one
  // step() to complete, matching "no unticked move completes itself").
  uint32_t stepsToComplete = 3;
  // What DoneReason a motion reports when its own countdown reaches
  // zero (i.e. it finishes NORMALLY -- not via STOP/ESTOP/forceAbort(),
  // which always override this with their own fixed reason).
  DoneReason completionReason = DoneReason::kStop;
  // Canned acceptance for every onXxx() motion call -- kOk (default)
  // means "adapter accepts, dispatch()'s own ack/err follows the usual
  // rules"; anything else is a merits rejection (the sequence still
  // advances per docs/design/protocol.md §8.2 -- this is NOT a decode
  // failure).
  Result acceptResult = Result::kOk;

  // ---- identity / status, mirroring MockAdapter's own shape ----
  Identity identityToReturn;
  uint32_t nowToReturn = 0;

  // ---- observation ----
  Kind activeKind = Kind::kNone;
  uint32_t activeId = 0;
  bool active() const { return activeKind != Kind::kNone; }
  uint32_t stepsRemaining() const { return stepsRemaining_; }
  int stepCalls = 0;

  // Last-accepted command's own fields, recorded regardless of
  // `acceptResult` (so a test can assert what WOULD have been attempted
  // even on a merits rejection) -- kept flat and shared across kinds
  // rather than a union, matching this file's "simple test double, not
  // a production type" posture.
  float lastLeft = 0.0f, lastRight = 0.0f, lastCruise = 0.0f;
  float lastDistance = 0.0f, lastRotation = 0.0f;
  float lastVx = 0.0f, lastOmega = 0.0f;
  float lastX = 0.0f, lastY = 0.0f, lastSpeed = 0.0f, lastArrive = 0.0f;
  uint32_t lastDuration = 0, lastTimeout = 0;
  bool lastStopImmediate = false;
  int stopCalls = 0;
  int estopCalls = 0;

  // ---- Adapter: session ----
  void identity(Identity& out) const override { out = identityToReturn; }
  uint32_t now() const override { return nowToReturn; }
  void status(StatusFields& out) const override {
    out = StatusFields{};
    out.ready = true;
    out.active = active();
  }

  // ---- Adapter: motion ----
  Result onWheelsX(float left, float right, float cruise, uint32_t timeout,
                   uint32_t id) override {
    lastLeft = left;
    lastRight = right;
    lastCruise = cruise;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    beginMotion(Kind::kWheelsX, id);
    return Result::kOk;
  }
  Result onWheelsV(float left, float right, uint32_t duration,
                   uint32_t id) override {
    lastLeft = left;
    lastRight = right;
    lastDuration = duration;
    if (acceptResult != Result::kOk) return acceptResult;
    beginMotion(Kind::kWheelsV, id);
    return Result::kOk;
  }
  Result onMoveX(float distance, float rotation, float cruise,
                 uint32_t timeout, uint32_t id) override {
    lastDistance = distance;
    lastRotation = rotation;
    lastCruise = cruise;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    beginMotion(Kind::kMoveX, id);
    return Result::kOk;
  }
  Result onMoveV(float v_x, float omega, uint32_t duration,
                 uint32_t id) override {
    lastVx = v_x;
    lastOmega = omega;
    lastDuration = duration;
    if (acceptResult != Result::kOk) return acceptResult;
    beginMotion(Kind::kMoveV, id);
    return Result::kOk;
  }
  Result onGoToR(float x, float y, float speed, float arrive,
                uint32_t timeout, uint32_t id) override {
    lastX = x;
    lastY = y;
    lastSpeed = speed;
    lastArrive = arrive;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    beginMotion(Kind::kGoToR, id);
    return Result::kOk;
  }
  Result onGoToW(float x, float y, float speed, float arrive,
                uint32_t timeout, uint32_t id) override {
    lastX = x;
    lastY = y;
    lastSpeed = speed;
    lastArrive = arrive;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    beginMotion(Kind::kGoToW, id);
    return Result::kOk;
  }

  Result onStop(bool immediate, uint32_t /*id*/) override {
    ++stopCalls;
    lastStopImmediate = immediate;
    // STOP ends whatever is active, right now, with reason kStop --
    // "the normal case: you found the line, the program is done"
    // (motion-api.md §3.7). STOP on an idle adapter is a harmless no-op,
    // matching this library's own onStop()/neutral() (never refuses).
    if (active()) finish(DoneReason::kStop);
    return Result::kOk;
  }
  void onEstop() override {
    ++estopCalls;
    // The scenario the ticket calls out by name: "an ESTOP mid-move
    // must complete the in-flight move with reason estop."
    if (active()) finish(DoneReason::kEstop);
  }

  // Test-only hook: simulate the HOST-side "the caller abandoned it"
  // condition (motion-api.md §5.1's callback/generator-closed case) --
  // there is no wire verb for this at all, so a test calls it directly
  // rather than through feed().
  void forceAbort() {
    if (active()) finish(DoneReason::kAborted);
  }

  // ---- Adapter: configuration -- this test double stores none. Every
  // GET is unknown; every SET is a merits rejection (kUnknown), the
  // same as DiffDriveAdapter's own RUN answers for an unregistered
  // name -- there is no config table here to be wrong about.
  bool onGet(const char* /*name*/, float& /*out*/) const override {
    return false;
  }
  Result onSet(const char* /*name*/, float /*value*/,
              uint32_t /*id*/) override {
    return Result::kUnknown;
  }
  size_t fieldCount() const override { return 0; }
  const char* fieldName(size_t /*index*/) const override { return ""; }

  Result onTlm(TlmMode mode) override {
    mode_ = mode;
    return Result::kOk;
  }

  // ---- the reliability layer's completion channel -- the whole point
  // of this test double (docs/design/protocol.md §8.8: "the first time
  // that field is genuinely live"). ----
  uint32_t lastDone() const override { return lastDone_; }
  DoneReason lastDoneReason() const override { return lastDoneReason_; }

  // ---- RUN: no registration table, same posture as DiffDriveAdapter --
  Result onRun(const char* /*name*/, const char* const* /*argv*/,
              size_t /*argc*/, char* /*result*/, size_t /*resultCapacity*/,
              bool& hasResult) override {
    hasResult = false;
    return Result::kUnknown;
  }

  // ---- the step()-driven progress loop -- NO timer, NO clock. One
  // call, one tick, always; the harness decides the pace entirely. ----
  void step() {
    ++stepCalls;
    if (!active()) return;
    if (stepsRemaining_ > 0) --stepsRemaining_;
    if (stepsRemaining_ == 0) finish(completionReason);
  }

  // ---- telemetry projection (NOT part of Protocol::Adapter -- the
  // harness calls this once per frame it wants to emit, then hands the
  // Snapshot straight to ProtocolHandler::emitTelemetry(), the same
  // shape DiffDriveAdapter::buildSnapshot() already established). This
  // is what makes the ack/nack piggyback ride along for real during a
  // multi-step move, exactly as the ticket asks: "emit telemetry frames
  // as it progresses ... so the ack/nack piggyback rides along and is
  // exercised for real." ----
  const Snapshot& buildSnapshot() {
    size_t n = 0;
    columns_[n++] = Column{"active", active() ? 1 : 0, false};
    columns_[n++] = Column{"kind", static_cast<int32_t>(activeKind), false};
    columns_[n++] = Column{"id", static_cast<int32_t>(activeId), false};
    columns_[n++] =
        Column{"stepsleft", static_cast<int32_t>(stepsRemaining_), false};
    snapshot_ = Snapshot{columns_, n};
    return snapshot_;
  }

 private:
  void beginMotion(Kind kind, uint32_t id) {
    activeKind = kind;
    activeId = id;
    stepsRemaining_ = stepsToComplete == 0 ? 1 : stepsToComplete;
  }

  void finish(DoneReason reason) {
    lastDone_ = activeId;
    lastDoneReason_ = reason;
    activeKind = Kind::kNone;
    activeId = 0;
    stepsRemaining_ = 0;
  }

  uint32_t stepsRemaining_ = 0;
  uint32_t lastDone_ = 0;
  DoneReason lastDoneReason_ = DoneReason::kNone;
  TlmMode mode_ = TlmMode::kOff;

  static constexpr size_t kMaxColumns = 4;
  Column columns_[kMaxColumns] = {};
  Snapshot snapshot_;
};

}  // namespace Protocol

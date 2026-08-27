// fake_motion_adapter.h -- Protocol::FakeMotionAdapter: a deterministic,
// step()-driven test double implementing every motion verb
// ProtocolHandler can dispatch (WHEELS_X/WHEELS_V/MOVE_X/MOVE_V/
// GO_TO_R/GO_TO_W, plus STOP/ESTOP) -- built to exercise the
// reliability layer's completion channel (Adapter::lastDone()/
// lastDoneReason(), docs/design/protocol.md §8.8) for the first time
// with something that actually WRITES it. DiffDriveAdapter never does
// (it has no queue and no completion event of its own -- it drives
// DifferentialDrive directly, one command at a time); this adapter
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
// ---- 2026-08-22 correction: a REAL FIFO motion queue ----
//
// Stakeholder, verbatim, on an earlier report that framed this as an
// open protocol-design question: "Well, then put a fucking motion queue
// in it. If the test can't accurately test the code, then obviously you
// have to change your test. The system absolutely does give you ordered
// execution. The Reliability Layer is not there. The Reliability
// Layer's job is to get commands to the Motion Layer. The Motion Layer
// will execute them in order, because why would it mix them up? ... You
// have to explicitly replace things. You can't put them out of order."
//
// So: this class now owns a fixed-capacity FIFO of motions
// (`kMaxQueueDepth`, no dynamic allocation). A motion command that
// arrives while one is already active ENQUEUES behind it -- it does
// NOT supersede/clobber the running one. Each queued motion becomes
// active, runs its own step()-driven countdown to completion, and
// produces its OWN `lastDone()`/`lastDoneReason()` update in turn, so a
// caller watching that completion channel sees every id in the
// sequence complete, one at a time, in arrival order -- not just the
// last one to have been dispatched. `kMaxQueueDepth` is 5: big enough
// to pipeline a handful of legs/turns ahead of the active one (the
// flagship reliability test pipelines at most one command deep behind
// an active one at any moment) without unbounded growth; a 6th
// simultaneously-queued motion is refused with `Result::kFull`
// (`ERR_FULL`, wire code 4 -- the same code the wire's own error-code
// space already reserves for exactly this, §6.1), matching the depth
// this project's own firmware planner already uses elsewhere for the
// same reason (a bounded ring, not an unbounded backlog).
//
// STOP and ESTOP are NOT queue entries themselves -- they act on
// whatever is active RIGHT NOW, immediately, matching motion-api.md
// §3.7's "stop() takes effect on the current motion; it is not a queue
// entry that waits its turn". What differs between them is what happens
// to the QUEUED REMAINDER once the active motion ends:
//
//   - `STOP [now]` completes the active motion (reason `kStop`) and
//     DRAINS every motion still queued behind it -- they never run, and
//     produce no completion event of their own (they never started, so
//     there is nothing to report `done` about). This is the choice that
//     matches "you have to explicitly replace things, you can't put
//     them out of order": a STOP is the host's own explicit signal that
//     the whole planned sequence is over, not just its current leg, so
//     silently continuing on to whatever was queued next would be
//     exactly the kind of implicit reordering the stakeholder's
//     direction rules out. (Compare motion-api.md §6's own "a `wheels_*`
//     clears the planner" -- draining on an explicit stop/mode-change is
//     already this project's convention, not a new one invented here.)
//   - `ESTOP` completes the active motion (reason `kEstop`) and ALSO
//     drains the queue, for the same reason stated even more sharply: a
//     panic stop must not leave several more legs armed to run the
//     instant it clears. `onEstop()` has always run this "drain" step
//     even before this change (there was never more than the active
//     motion to drain); what changes is that there can now genuinely be
//     something queued behind it to drain FOR REAL.
//   - `forceAbort()` (the test-only "caller abandoned it" hook, no wire
//     verb) completes ONLY the active motion (reason `kAborted`) and
//     lets the queue continue -- it stands in for one abandoned motion
//     object, not a request to cancel the whole plan, so the next
//     queued motion is promoted and runs normally.
//
// NO TIMER, NO CLOCK: a command is accepted and immediately becomes a
// deterministic countdown (`stepsToComplete`, settable before the
// command is sent), and the harness calls step() explicitly to advance
// it -- one call, one tick, always. This mirrors motion-api.md §5.2's
// own "a posted move that nobody ticks does nothing" model, pushed down
// to the wire level, and it is what makes a test's progress
// reproducible instead of racing a real clock. `stepsToComplete` and
// `completionReason` are snapshotted onto EACH motion at the moment it
// is accepted (whether it starts immediately or is enqueued) -- not
// re-read live off the adapter's own fields when it later becomes
// active -- so changing either knob mid-test never retroactively
// recolors a command that was already accepted, queued or not.
//
// This is a TEST DOUBLE standing in for a real planner, not a physics
// simulation -- it does not compute wheel kinematics, encoder counts,
// or arrival tolerances. What it DOES model faithfully is the part the
// reliability layer cares about: motions run one at a time, IN ORDER,
// each producing its own completion event with a `DoneReason` a test
// controls directly (`completionReason`) so all four reasons in
// motion-api.md §5.3 -- stop / timeout / estop / aborted -- can be
// exercised on demand.
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

  // Fixed capacity of the motion queue -- how many motions may be
  // waiting BEHIND the currently active one. No dynamic allocation:
  // this bounds a plain array (`queue_` below), and a command that
  // would need a 6th slot is refused with `Result::kFull` rather than
  // grown into. See this file's own header comment for why 5.
  static constexpr size_t kMaxQueueDepth = 5;

  // ---- test-controlled knobs, set BEFORE issuing a motion command ----
  // How many step() calls the NEXT accepted motion command takes to
  // finish (0 is treated as 1 -- a motion always needs at least one
  // step() to complete, matching "no unticked move completes itself").
  // Snapshotted onto the command the instant it is accepted (started or
  // enqueued) -- see the file header's "NO TIMER, NO CLOCK" paragraph.
  uint32_t stepsToComplete = 3;
  // What DoneReason a motion reports when its own countdown reaches
  // zero (i.e. it finishes NORMALLY -- not via STOP/ESTOP/forceAbort(),
  // which always override this with their own fixed reason). Also
  // snapshotted at accept time, same as `stepsToComplete`.
  DoneReason completionReason = DoneReason::kStop;
  // Canned acceptance for every onXxx() motion call -- kOk (default)
  // means "adapter accepts, dispatch()'s own ack/err follows the usual
  // rules"; anything else is a merits rejection (the sequence still
  // advances per docs/design/protocol.md §8.2 -- this is NOT a decode
  // failure). A merits-rejected command is never started and never
  // queued.
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

  // How many motions are currently queued BEHIND the active one (0 when
  // nothing is active, by construction -- a motion is only ever queued
  // while another is active, and the queue is only ever consumed by
  // promoting its head to active).
  size_t queuedCount() const { return queueCount_; }
  // Peek at the id of the queued motion `offset` slots behind the head
  // of the queue (0 == next to run), without dequeuing it. Only valid
  // for `offset < queuedCount()`.
  uint32_t queuedIdAt(size_t offset) const {
    return queue_[(queueHead_ + offset) % kMaxQueueDepth].id;
  }

  // Last-accepted command's own fields, recorded regardless of
  // `acceptResult` (so a test can assert what WOULD have been attempted
  // even on a merits rejection) -- kept flat and shared across kinds
  // rather than a union, matching this file's "simple test double, not
  // a production type" posture. Reflects the MOST RECENTLY accepted
  // call, not necessarily the currently active motion, exactly as
  // before this file gained a queue.
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
    return acceptMotion(Kind::kWheelsX, id);
  }
  Result onWheelsV(float left, float right, uint32_t duration,
                   uint32_t id) override {
    lastLeft = left;
    lastRight = right;
    lastDuration = duration;
    if (acceptResult != Result::kOk) return acceptResult;
    return acceptMotion(Kind::kWheelsV, id);
  }
  Result onMoveX(float distance, float rotation, float cruise,
                 uint32_t timeout, uint32_t id) override {
    lastDistance = distance;
    lastRotation = rotation;
    lastCruise = cruise;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    return acceptMotion(Kind::kMoveX, id);
  }
  Result onMoveV(float v_x, float omega, uint32_t duration,
                 uint32_t id) override {
    lastVx = v_x;
    lastOmega = omega;
    lastDuration = duration;
    if (acceptResult != Result::kOk) return acceptResult;
    return acceptMotion(Kind::kMoveV, id);
  }
  Result onGoToR(float x, float y, float speed, float arrive,
                uint32_t timeout, uint32_t id) override {
    lastX = x;
    lastY = y;
    lastSpeed = speed;
    lastArrive = arrive;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    return acceptMotion(Kind::kGoToR, id);
  }
  Result onGoToW(float x, float y, float speed, float arrive,
                uint32_t timeout, uint32_t id) override {
    lastX = x;
    lastY = y;
    lastSpeed = speed;
    lastArrive = arrive;
    lastTimeout = timeout;
    if (acceptResult != Result::kOk) return acceptResult;
    return acceptMotion(Kind::kGoToW, id);
  }

  Result onStop(bool immediate, uint32_t /*id*/) override {
    ++stopCalls;
    lastStopImmediate = immediate;
    // STOP ends whatever is active, right now, with reason kStop --
    // "the normal case: you found the line, the program is done"
    // (motion-api.md §3.7) -- AND drains the queued remainder (this
    // file's own header comment explains why: an explicit stop is not
    // a queue entry, and it must not let the rest of the plan run
    // behind its back). STOP on an idle adapter is a harmless no-op,
    // matching this library's own onStop()/neutral() (never refuses).
    if (active()) finishActiveOnly(DoneReason::kStop);
    clearQueue();
    return Result::kOk;
  }
  void onEstop() override {
    ++estopCalls;
    // The scenario the ticket calls out by name: "an ESTOP mid-move
    // must complete the in-flight move with reason estop" -- AND clear
    // the queue: a panic stop must not leave several more legs armed
    // to run the instant it clears.
    if (active()) finishActiveOnly(DoneReason::kEstop);
    clearQueue();
  }

  // Test-only hook: simulate the HOST-side "the caller abandoned it"
  // condition (motion-api.md §5.1's callback/generator-closed case) --
  // there is no wire verb for this at all, so a test calls it directly
  // rather than through feed(). Only the ACTIVE motion is abandoned;
  // the queue (if any) continues normally, since this stands in for one
  // abandoned motion object, not a request to cancel the whole plan.
  void forceAbort() {
    if (active()) completeActiveAndAdvance(DoneReason::kAborted);
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
  // that field is genuinely live"). Monotonic contract preserved: since
  // motions run strictly one at a time, in arrival order, a later value
  // implies every earlier id has also completed -- true whether that
  // ordering comes from the queue draining naturally or from a single
  // motion completing on its own, exactly as adapter.h's own comment on
  // these two methods documents. ----
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
    if (stepsRemaining_ == 0) completeActiveAndAdvance(activeCompletionReason_);
  }

  // ---- telemetry projection (NOT part of Protocol::Adapter -- the
  // harness calls this once per frame it wants to emit, then hands the
  // Snapshot straight to ProtocolHandler::emitTelemetry(), the same
  // shape DiffDriveAdapter::buildSnapshot() already established).
  // 2026-08-26 (protocol.md §8.5): emitTelemetry() emits frames ONLY --
  // the ack/nack line that used to ride along is deleted; completion
  // state reaches a host on the ack of its next command (e.g. a STATUS
  // poll), which the tests now exercise instead. ----
  const Snapshot& buildSnapshot() {
    size_t n = 0;
    columns_[n++] = Column{"active", active() ? 1 : 0, false};
    columns_[n++] = Column{"kind", static_cast<int32_t>(activeKind), false};
    columns_[n++] = Column{"id", static_cast<int32_t>(activeId), false};
    columns_[n++] =
        Column{"stepsleft", static_cast<int32_t>(stepsRemaining_), false};
    columns_[n++] = Column{"queued", static_cast<int32_t>(queueCount_), false};
    snapshot_ = Snapshot{columns_, n};
    return snapshot_;
  }

 private:
  // One motion waiting in the queue -- its own steps/reason snapshot,
  // captured at ACCEPT time (see the file header), not re-read off the
  // adapter's live knobs when it is later promoted to active.
  struct QueuedMotion {
    Kind kind = Kind::kNone;
    uint32_t id = 0;
    uint32_t steps = 0;
    DoneReason reason = DoneReason::kStop;
  };

  // Accept a decoded, merits-approved motion command: start it
  // immediately if nothing is active, otherwise enqueue it behind
  // whatever is running. Returns kFull if the queue has no room left
  // (the wire's own ERR_FULL, §6.1) -- the command is neither started
  // nor queued in that case.
  Result acceptMotion(Kind kind, uint32_t id) {
    if (!active()) {
      startMotion(kind, id, stepsToComplete, completionReason);
      return Result::kOk;
    }
    if (queueCount_ >= kMaxQueueDepth) return Result::kFull;
    queue_[(queueHead_ + queueCount_) % kMaxQueueDepth] =
        QueuedMotion{kind, id, stepsToComplete, completionReason};
    ++queueCount_;
    return Result::kOk;
  }

  void startMotion(Kind kind, uint32_t id, uint32_t steps,
                    DoneReason reason) {
    activeKind = kind;
    activeId = id;
    stepsRemaining_ = steps == 0 ? 1 : steps;
    activeCompletionReason_ = reason;
  }

  // Complete the active motion (recording lastDone/lastDoneReason) but
  // do NOT touch the queue -- used by STOP/ESTOP, which drain the queue
  // themselves right afterwards instead of letting it advance.
  void finishActiveOnly(DoneReason reason) {
    lastDone_ = activeId;
    lastDoneReason_ = reason;
    activeKind = Kind::kNone;
    activeId = 0;
    stepsRemaining_ = 0;
  }

  // Complete the active motion AND promote the next queued one (if any)
  // to active -- the normal "this one finished, what's next" path used
  // by step()'s own countdown and by forceAbort().
  void completeActiveAndAdvance(DoneReason reason) {
    finishActiveOnly(reason);
    if (queueCount_ == 0) return;
    QueuedMotion next = queue_[queueHead_];
    queueHead_ = (queueHead_ + 1) % kMaxQueueDepth;
    --queueCount_;
    startMotion(next.kind, next.id, next.steps, next.reason);
  }

  void clearQueue() {
    queueHead_ = 0;
    queueCount_ = 0;
  }

  uint32_t stepsRemaining_ = 0;
  DoneReason activeCompletionReason_ = DoneReason::kStop;
  uint32_t lastDone_ = 0;
  DoneReason lastDoneReason_ = DoneReason::kNone;
  TlmMode mode_ = TlmMode::kOff;

  QueuedMotion queue_[kMaxQueueDepth] = {};
  size_t queueHead_ = 0;
  size_t queueCount_ = 0;

  static constexpr size_t kMaxColumns = 5;
  Column columns_[kMaxColumns] = {};
  Snapshot snapshot_;
};

}  // namespace Protocol

// adapter.h — Protocol::Adapter: the one class a robot implements to sit
// behind ProtocolHandler (protocol_handler.h), plus the small value types
// the two exchange. Mirrors the object model in
// docs/design/protocol.md §1-§9 — that document is the design authority;
// this header is its literal implementation, filling in the few shapes
// (Snapshot/Column, Identity, StatusFields) the design doc names but does
// not spell out field-by-field.
//
// The adapter never writes or parses a wire byte. It receives decoded,
// typed arguments and returns a typed Result; ProtocolHandler does every
// bit of wire formatting, exactly once per verb. Returning a Result
// rather than writing a reply means the adapter cannot emit a malformed
// reply, cannot forget to reply, and cannot invent a reply shape.
//
// ---- 2026-08-22 changes (docs/design/protocol.md §8/§9, the "six
// stakeholder-directed changes") ----
//
//   1. `Result::kDuplicateId` is DELETED. It was already unreachable
//      (the handler's own sequencing guarantees the adapter never sees a
//      repeated id) — this removes the dead enumerator entirely rather
//      than keeping it declared-but-unused.
//   2. `lastDone()`/`lastDoneReason()` move HERE, from a handler field
//      that nothing ever wrote. The handler now POLLS these two methods
//      every time it formats an `ack`/`nack` line, instead of owning the
//      value itself — see the two declarations below for the contract.
//   3. `onWheels()` is renamed `onWheelsV()` (WHEELS -> WHEELS_V, the
//      wire rename docs/design/motion-api.md §9.2 mandates), and five
//      new motion methods join it: `onWheelsX`, `onMoveX`, `onMoveV`,
//      `onGoToR`, `onGoToW` — one per wire verb in motion-api.md §9.1's
//      six-verb table. `onStop()` gains an `immediate` argument for
//      `STOP now` (motion-api.md §3.7/§9.1).
#pragma once

#include <cstddef>
#include <cstdint>

namespace Protocol {

// Maps 1:1 onto the wire outcome (docs/design/protocol.md §4; wire codes
// per spec §6.1). `kDuplicateId` is GONE (2026-08-22, stakeholder
// change 2): the handler's own strict sequencing already made it
// unreachable, so the dead enumerator is removed rather than kept for
// completeness.
enum class Result : uint8_t {
  kOk,             // -> the ack alone; no further reply
  kUnknown,        // -> err 1 #<id>   ERR_UNKNOWN
  kBadArg,         // -> err 2 #<id>   ERR_BADARG
  kRange,          // -> err 3 #<id>   ERR_RANGE
  kFull,           // -> err 4 #<id>   ERR_FULL
  kUnimplemented,  // -> err 6 #<id>   ERR_UNIMPLEMENTED
  kNotReady,       // -> err 8 #<id>   ERR_NOT_CONFIGURED
  kBusy,           // -> err 10 #<id>  ERR_BUSY
};

// TLM subscription modes (docs/design/protocol.md §10.1: `TLM <mode>` —
// telemetry is a subscription). The handler only decodes the wire
// token ("OFF"/"POSE"/"FULL"/"NOW"/"AUTO"/"BUFFER") into this enum and
// hands it to onTlm() — what each mode DOES (including whether "current
// mode" even changes for NOW) is entirely the adapter's business.
//
// `kHdr` (2026-08-23, docs/design/protocol.md §10.5) is the one
// exception to that rule: `TLM HDR` is a header-recovery request, not a
// subscription change, so ProtocolHandler::execTlm() handles it
// entirely itself (clearing its own remembered-header state) and NEVER
// forwards it to onTlm() — no Adapter ever observes this enumerator.
enum class TlmMode : uint8_t {
  kOff,
  kPose,
  kFull,
  kNow,
  kAuto,
  kBuffer,
  kHdr,
};

// The reliability layer's completion-reason vocabulary (docs/design/
// motion-api.md §5.3, docs/design/protocol.md §8.8): the four reasons a
// motion can finish, PLUS `kNone` for "nothing has completed yet" —
// the wire spelling `none` is what `lastDone() == 0` pairs with. A
// bare completed-id number loses the reason a caller needs to tell an
// ordinary `stop()` apart from a fault-path `estop()`, so the reason
// rides alongside the id on every `ack`/`nack` (protocol.md §8.8 — this
// library's own resolution of a conflict the reliability-layer brief did
// not settle).
enum class DoneReason : uint8_t {
  kNone,     // -> "none" -- lastDone() == 0, nothing has completed yet
  kStop,     // -> "stop" -- the stop condition was met, or stop() ended it
  kTimeout,  // -> "timeout" -- the backstop fired
  kEstop,    // -> "estop" -- a panic stop ended it (the fault path)
  kAborted,  // -> "aborted" -- the caller abandoned it
};

// Identity — everything HELLO/ID/VER read off (spec §4). Every pointer
// is borrowed: the adapter owns the storage (a string literal or a
// robot-config field) and must keep it alive at least until the call
// into ProtocolHandler that requested it returns.
struct Identity {
  const char* name = "";
  const char* serial = "";
  const char* drivetrain = "";
  const char* profile = "";
  const char* version = "";
};

// StatusFields — STATUS's `k=v` payload (spec §4). `tlm` is the CURRENT
// subscription mode's own lowercase wire name ("off"/"pose"/"full"/
// "auto"/"buffer") — the handler does not re-derive it from TlmMode, so
// an adapter tracking its own mode state machine never has to reconcile
// it against the handler's opinion of what "current" means.
struct StatusFields {
  bool ready = false;
  bool active = false;
  bool connLeft = false;
  bool connRight = false;
  bool otos = false;
  bool wedge = false;
  uint32_t flags = 0;
  const char* tlm = "off";
};

// One column of a telemetry frame — see Snapshot below. `value` is
// already fully scaled per docs/design/protocol.md §10.3 (This library's
// column sets; e.g. an `elv` column already carries mm/s x10);
// ProtocolHandler does not know or care what a column MEANS, only how to
// print it. `hex` selects docs/design/protocol.md §10.2's Value encoding
// paragraph's one exception: `flags` prints lowercase hex with no `0x`
// prefix, everything else prints signed base-10.
struct Column {
  const char* name = "";
  int32_t value = 0;
  bool hex = false;
};

// Snapshot — one telemetry frame, already fully projected by the caller
// (docs/design/protocol.md §4.1: "the adapter's telemetry job is a
// projection, not a computation... hand the handler an array").
// ProtocolHandler::emitTelemetry() formats ONLY: it holds no notion of
// what a column means, so a Snapshot from a differential-drive robot and
// one from a mecanum robot are equally valid input, as long as the
// column set (count, names, hex-ness, and order) stays stable across
// calls for as long as the subscription mode is unchanged — that
// stability is what the handler watches to decide whether a fresh
// `thdr:` is due (docs/design/protocol.md §10.2: a fresh `thdr:` is due
// whenever the column set changes, including the first frame this
// handler has ever emitted).
struct Snapshot {
  const Column* columns = nullptr;
  size_t count = 0;
};

class Adapter {
 public:
  virtual ~Adapter() = default;

  // ---- session ----
  virtual void identity(Identity& out) const = 0;
  virtual uint32_t now() const = 0;  // [ms] for pong
  virtual void status(StatusFields& out) const = 0;

  // ---- motion: WHEELS_V/WHEELS_X/MOVE_X/MOVE_V/GO_TO_R/GO_TO_W, plus
  // STOP/ESTOP (docs/design/motion-api.md §9.1's wire mapping). Angles
  // (rotation, omega) arrive already decoded from the wire's milliradian
  // integers into float milliradians — the API-level degrees<->wire
  // milliradians conversion is a LANGUAGE BINDING's job (motion-api.md
  // §9.1: "degrees at the API, milliradian integers on the wire ... the
  // conversion lives in the binding, in one place"), not this library's.
  //
  // `onWheelsV` is the 2026-08-22 rename of what was `onWheels()` — same
  // fields, same meaning (motion-api.md §9.2 confirms WHEELS *is*
  // wheels_v): left/right wheel velocity held for `duration`.
  virtual Result onWheelsV(float left, float right,  // [mm/s] [mm/s]
                           uint32_t duration,          // [ms]
                           uint32_t id) = 0;

  // `onWheelsX`: per-wheel commanded DISTANCE, not velocity — bounded by
  // encoder travel rather than time (motion-api.md §3.1). `cruise` is
  // the dominant wheel's own speed ceiling; `timeout` is the required
  // backstop if the commanded distance is never reached.
  virtual Result onWheelsX(float left, float right,  // [mm] [mm]
                           float cruise,               // [mm/s]
                           uint32_t timeout,            // [ms]
                           uint32_t id) = 0;

  // `onMoveX`: body displacement + heading change (motion-api.md §3.3).
  virtual Result onMoveX(float distance,   // [mm]
                         float rotation,    // [mrad]
                         float cruise,      // [mm/s]
                         uint32_t timeout,  // [ms]
                         uint32_t id) = 0;

  // `onMoveV`: body twist held for `duration` (motion-api.md §3.4).
  virtual Result onMoveV(float v_x,        // [mm/s]
                         float omega,       // [mrad/s]
                         uint32_t duration,  // [ms]
                         uint32_t id) = 0;

  // `onGoToR`/`onGoToW`: drive to a point in the robot's own frame, or
  // in world coordinates (motion-api.md §3.5/§3.6). `arrive` is the
  // arrival tolerance (0 = adapter's configured default); `timeout` is
  // the required backstop.
  virtual Result onGoToR(float x, float y,  // [mm] [mm]
                        float speed,         // [mm/s]
                        float arrive,        // [mm]
                        uint32_t timeout,    // [ms]
                        uint32_t id) = 0;
  virtual Result onGoToW(float x, float y,  // [mm] [mm]
                        float speed,         // [mm/s]
                        float arrive,        // [mm]
                        uint32_t timeout,    // [ms]
                        uint32_t id) = 0;

  // `immediate` is `STOP now`'s own flag (motion-api.md §3.7/§9.1: a
  // deceleration CHOICE, not a different verb) -- false is the ordinary,
  // jerk-limited "I meant to stop here" case; true is "I need the
  // distance more than I need the smoothness." An adapter with no ramp
  // machinery of its own (this library's own DiffDriveAdapter, whose
  // `neutral()` is already immediate either way — docs/design/
  // protocol.md §5.1) is free to treat both identically; the flag exists
  // for adapters that DO have a ramp to choose between.
  virtual Result onStop(bool immediate, uint32_t id) = 0;
  virtual void onEstop() = 0;  // never acked, never queued — spec §8.3

  // ---- configuration (no storage in the handler — pure delegation,
  // docs/design/protocol.md §7: which names are valid is entirely the
  // adapter's business) ----
  virtual bool onGet(const char* name, float& out) const = 0;
  virtual Result onSet(const char* name, float value, uint32_t id) = 0;
  virtual size_t fieldCount() const = 0;  // for a bare GET
  virtual const char* fieldName(size_t index) const = 0;

  // ---- telemetry ----
  virtual Result onTlm(TlmMode mode) = 0;

  // ---- the reliability layer's completion channel (2026-08-22,
  // docs/design/protocol.md §8.8) ----
  //
  // MOVED here from a ProtocolHandler field (`lastDone_`) that nothing
  // ever wrote, because it was handler-owned state with no handler-level
  // event that could ever set it — the handler has no clock and no
  // notion of "this motion, which I dispatched some cycles ago, just
  // finished." An ADAPTER that actually runs a motion to completion (a
  // planner, a fake test double driving a step() loop) is the only thing
  // that CAN know when that happens, so it is the thing that should own
  // the value. The handler now POLLS these two methods every time it
  // formats an `ack` or `nack` line (docs/design/protocol.md §8.1/§8.5)
  // — no callback, no clock, no handler-side state to keep in sync.
  //
  // Monotonic contract: a LATER value of lastDone() implies every
  // EARLIER id has also completed — this library's motion runs one
  // command at a time, in order, so a single scalar plus its reason is
  // a complete completion record, not just the most recent one. This is
  // what makes a dropped completion ack recoverable: the NEXT ack/nack
  // (piggybacked on the next reply, or the next telemetry frame) simply
  // re-states the same (or a newer) lastDone()/lastDoneReason() pair,
  // and the host learns everything through that id regardless of which
  // individual ack carried the news first.
  //
  // An adapter with no completion event of its own (this library's own
  // DiffDriveAdapter, whose WHEELS_V has no stop condition — docs/design/
  // protocol.md §8.8.1) returns 0 / kNone forever, which is wire-correct
  // (every ack/nack carries a well-defined value) even though it is
  // functionally inert on that adapter.
  virtual uint32_t lastDone() const = 0;
  virtual DoneReason lastDoneReason() const = 0;

  // ---- invocation by name (docs/design/protocol.md's RUN section) ----
  //
  // The HANDLER holds no function table -- it only parses
  // "RUN <name> [arg...] #id" into a name and the RAW, unconverted
  // argument tokens that followed it, and hands them here unchanged.
  // THIS method owns name resolution, per-argument type conversion,
  // invocation, and stringifying any return value. In a dynamic host
  // (MicroPython, JavaScript) that is close to free -- `globals()[name]`
  // plus reflecting the target's own parameter list. In C++, where
  // there is no lookup-by-name and no parameter-type reflection, a
  // concrete Adapter needs its own explicit name/arity/argument-type
  // registration table to implement this at all. RUN is therefore the
  // first verb where this C++ archetype does MATERIALLY MORE work than
  // a straight port to a dynamic language would need, not less -- a
  // porter should not conclude that registration machinery is itself
  // part of the wire contract.
  //
  // The registration table IS the security boundary: whatever a
  // concrete Adapter registers is invocable BY NAME from the wire by
  // anything that can talk to the robot, including any other host
  // overhearing a shared radio channel. Treat the table as an explicit
  // allowlist, not an implementation detail -- an Adapter with no
  // registration table at all (e.g. this library's own
  // DiffDriveAdapter, which owns no callable surface) has an empty
  // allowlist, and every RUN on it is correctly ERR_UNKNOWN.
  //
  // onRun() is called SYNCHRONOUSLY from ProtocolHandler::feed(), so a
  // slow registered function stalls line processing for as long as it
  // runs. Registered functions must return promptly; anything
  // long-running is the calling application's job to defer.
  //
  //   name          -- the function name token, borrowed, valid only
  //                     for the duration of this call.
  //   argv/argc     -- the RAW field tokens AFTER the function name,
  //                     unconverted; borrowed pointers into the
  //                     handler's own line buffer, valid only for the
  //                     duration of this call. An argument can never
  //                     itself contain a space (the wire grammar's own
  //                     field separator) or begin with '#' in the
  //                     line's last position (reserved for the id) --
  //                     both are genuine expressiveness limits on what
  //                     RUN can pass, not an omission in this seam.
  //   result        -- caller-owned buffer, resultCapacity bytes, this
  //                     method may fill with the stringified return
  //                     value. Left untouched if hasResult is false.
  //                     ProtocolHandler sanitizes and may further
  //                     truncate whatever is written here before it
  //                     reaches the wire (the same '\n'/'\r'-stripping
  //                     rule sendDebug()'s own text gets) -- this
  //                     method does not need to pre-sanitize its own
  //                     output.
  //   hasResult     -- set true ONLY if the target function returned a
  //                     value. A void-returning function leaves this
  //                     false, and the wire replies with the ack alone,
  //                     never `ret`.
  // Returns kUnknown for an unregistered name, kBadArg for wrong arity
  // or an argument that fails to convert to its target's declared
  // parameter type, kOk otherwise (whether or not hasResult ends up
  // true).
  virtual Result onRun(const char* name, const char* const* argv, size_t argc,
                       char* result, size_t resultCapacity,
                       bool& hasResult) = 0;
};

}  // namespace Protocol

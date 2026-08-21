// adapter.h — Protocol::Adapter: the one class a robot implements to sit
// behind ProtocolHandler (protocol_handler.h), plus the small value types
// the two exchange. Mirrors the object model in
// docs/design/protocol.md §1-§3 — that document is the design authority;
// this header is its literal implementation, filling in the few shapes
// (Snapshot/Column, Identity, StatusFields) the design doc names but does
// not spell out field-by-field.
//
// The adapter never writes or parses a wire byte. It receives decoded,
// typed arguments and returns a typed Result; ProtocolHandler does every
// bit of wire formatting, exactly once per verb. Returning a Result
// rather than writing a reply means the adapter cannot emit a malformed
// reply, cannot forget to reply, and cannot invent a reply shape.
#pragma once

#include <cstddef>
#include <cstdint>

namespace Protocol {

// Maps 1:1 onto the wire outcome (docs/design/protocol.md §3; wire codes
// per spec §8.3). The design doc's own Result sketch names six of these
// with two left uncommented/elided ("kFull, // -> err:<id>:..."); this is
// the completed set covering every §8.3 code this library's verbs can
// produce (5 ERR_DECODE, 7 ERR_OVERSIZE and 9 ERR_NOT_LIVE retire with
// the binary plane and the boot-only/live split respectively — spec
// §8.3's own note — so they have no enumerator here).
enum class Result : uint8_t {
  kOk,             // -> ok:<id>
  kUnknown,        // -> err:<id>:1   ERR_UNKNOWN     no such verb/field name
  kBadArg,         // -> err:<id>:2   ERR_BADARG      malformed/non-finite, wrong arity
  kRange,          // -> err:<id>:3   ERR_RANGE       declared bound violated
  kFull,           // -> err:<id>:4   ERR_FULL        queue full (4 pending)
  kUnimplemented,  // -> err:<id>:6   ERR_UNIMPLEMENTED recognized, not wired
  kNotReady,       // -> err:<id>:8   ERR_NOT_CONFIGURED refused pre-`ready`
  kBusy,           // -> err:<id>:10  ERR_BUSY        subsystem in motion
  kDuplicateId,    // -> err:<id>:11  ERR_DUPLICATE_ID  spec §8.2
};

// TLM subscription modes (spec §6.1). The handler only decodes the wire
// token ("OFF"/"POSE"/"FULL"/"NOW"/"AUTO"/"BUFFER") into this enum and
// hands it to onTlm() — what each mode DOES (including whether "current
// mode" even changes for NOW) is entirely the adapter's business.
enum class TlmMode : uint8_t {
  kOff,
  kPose,
  kFull,
  kNow,
  kAuto,
  kBuffer,
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
// already fully scaled per spec §6.3/§6.4 (e.g. an `elv` column already
// carries mm/s x10); ProtocolHandler does not know or care what a column
// MEANS, only how to print it. `hex` selects spec §6.5's one exception:
// `flags` prints lowercase hex with no `0x` prefix, everything else
// prints signed base-10.
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
// `thdr:` is due (spec §6.2: "whenever the subscription changes, and
// before the first frame after connect").
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

  // ---- motion (the minimal set: enough to exercise a wheel kernel) ----
  virtual Result onWheels(float left, float right,  // [mm/s] [mm/s]
                          uint32_t duration,         // [ms]
                          uint32_t id) = 0;
  virtual Result onStop(uint32_t id) = 0;
  virtual void onEstop() = 0;  // never acked, never queued — spec §8.2

  // ---- configuration (no storage in the handler — pure delegation,
  // docs/design/protocol.md §6: which names are valid is entirely the
  // adapter's business) ----
  virtual bool onGet(const char* name, float& out) const = 0;
  virtual Result onSet(const char* name, float value, uint32_t id) = 0;
  virtual size_t fieldCount() const = 0;  // for a bare GET
  virtual const char* fieldName(size_t index) const = 0;

  // ---- telemetry ----
  virtual Result onTlm(TlmMode mode) = 0;

  // ---- RUN: invocation by name (docs/design/protocol.md's RUN
  // section) ----
  //
  // The HANDLER holds no function table -- it only parses
  // "RUN <name> [arg...] [#id]" into a name and the RAW, unconverted
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
  //                     false, and the wire replies bare `ok`/`ok #id`,
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

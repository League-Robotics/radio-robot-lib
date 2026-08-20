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
};

}  // namespace Protocol

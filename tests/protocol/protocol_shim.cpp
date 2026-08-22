// protocol_shim.cpp -- extern "C" ctypes surface for the protocol host
// test harness (Step 3). Test scaffolding only: nothing in
// src/ knows this file exists, and it is compiled only into this
// test's own throwaway shared library (see test_protocol_harness.py).
//
// ctypes cannot call C++ methods directly, so this file is the thin
// translation layer: one opaque handle bundling the handler under test
// with its own private MockAdapter and RecordingSink, plus free
// functions Python can bind by name.
#include <cstdint>
#include <cstring>
#include <string>

#include "mock_adapter.h"
#include "protocol_handler.h"

namespace {

// Accumulates every Sink::write() into one buffer for the whole test to
// inspect -- callers slice it on '\n' from the Python side. std::string
// is fine here: this file is host-only test scaffolding, not the
// firmware-targetable library it drives (protocol_handler.cpp itself
// never allocates).
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

struct Handle {
  MockAdapter adapter;
  RecordingSink sink;
  Protocol::ProtocolHandler handler;
  Handle() : handler(adapter, sink) {}
};

}  // namespace

extern "C" {

void* phCreate() { return new Handle(); }
void phDestroy(void* handle) { delete static_cast<Handle*>(handle); }

void phFeed(void* handle, const char* data, int length) {
  static_cast<Handle*>(handle)->handler.feed(data,
                                              static_cast<size_t>(length));
}

void phSendBanner(void* handle) {
  static_cast<Handle*>(handle)->handler.sendBanner();
}
void phSendReady(void* handle) {
  static_cast<Handle*>(handle)->handler.sendReady();
}
// `text` may be nullptr -- ProtocolHandler::sendDebug() treats nullptr
// and "" identically (both emit the bare "debug\n" line); ctypes passes
// a Python `None` argument through as a NULL c_char_p unchanged.
void phSendDebug(void* handle, const char* text) {
  static_cast<Handle*>(handle)->handler.sendDebug(text);
}

uint32_t phMalformedCount(void* handle) {
  return static_cast<Handle*>(handle)->handler.malformedCount();
}

// ---- sink readback -------------------------------------------------------

int phSinkLength(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->sink.buffer().size());
}

// Copies up to `cap` bytes of the sink's accumulated output into `out`
// (NOT nul-terminated by this call -- the wire is ASCII text with no
// embedded NUL, so the Python side just slices `out[:phSinkLength()]`).
// Returns the number of bytes copied.
int phSinkRead(void* handle, char* out, int cap) {
  Handle* h = static_cast<Handle*>(handle);
  size_t n = h->sink.buffer().size();
  if (static_cast<int>(n) > cap) n = static_cast<size_t>(cap);
  std::memcpy(out, h->sink.buffer().data(), n);
  return static_cast<int>(n);
}

void phSinkClear(void* handle) { static_cast<Handle*>(handle)->sink.clear(); }

// ---- MockAdapter canned-response setup -----------------------------------
// NOTE: every const char* passed in must outlive its use -- the mock
// stores the pointer, not a copy (mirroring Protocol::Identity's own
// borrowed-pointer contract, adapter.h). Callers keep the Python bytes
// objects alive for the ctypes call's duration; the mock reads them
// again on every identity()/status() call after that, so the TEST must
// keep them alive for as long as the handle lives.
void phSetIdentity(void* handle, const char* name, const char* serial,
                    const char* drivetrain, const char* profile,
                    const char* version) {
  Protocol::Identity& id = static_cast<Handle*>(handle)->adapter.identityToReturn;
  id.name = name;
  id.serial = serial;
  id.drivetrain = drivetrain;
  id.profile = profile;
  id.version = version;
}

void phSetNow(void* handle, uint32_t now) {
  static_cast<Handle*>(handle)->adapter.nowToReturn = now;
}

// Points a single, arbitrary GET/bare-GET field name at a value --
// see mock_adapter.h's overrideName/overrideValue. `name` must outlive
// its use, same borrowed-pointer contract as phSetIdentity above.
void phSetGetOverride(void* handle, const char* name, float value) {
  MockAdapter& a = static_cast<Handle*>(handle)->adapter;
  a.overrideName = name;
  a.overrideValue = value;
}

void phSetStatus(void* handle, int ready, int active, int connL, int connR,
                  int otos, int wedge, uint32_t flags, const char* tlm) {
  Protocol::StatusFields& s =
      static_cast<Handle*>(handle)->adapter.statusToReturn;
  s.ready = ready != 0;
  s.active = active != 0;
  s.connLeft = connL != 0;
  s.connRight = connR != 0;
  s.otos = otos != 0;
  s.wedge = wedge != 0;
  s.flags = flags;
  s.tlm = tlm;
}

// `result` is Protocol::Result's DECLARATION-ORDER ordinal (adapter.h),
// not a wire error code -- see test_protocol_harness.py's RESULT_*
// constants, which mirror that same order.
void phSetWheelsResult(void* handle, int result) {  // WHEELS_V
  static_cast<Handle*>(handle)->adapter.wheelsResult =
      static_cast<Protocol::Result>(result);
}
void phSetWheelsXResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.wheelsXResult =
      static_cast<Protocol::Result>(result);
}
void phSetMoveXResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.moveXResult =
      static_cast<Protocol::Result>(result);
}
void phSetMoveVResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.moveVResult =
      static_cast<Protocol::Result>(result);
}
void phSetGoToRResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.goToRResult =
      static_cast<Protocol::Result>(result);
}
void phSetGoToWResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.goToWResult =
      static_cast<Protocol::Result>(result);
}
void phSetStopResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.stopResult =
      static_cast<Protocol::Result>(result);
}
// `reason` is Protocol::DoneReason's DECLARATION-ORDER ordinal
// (adapter.h) -- see test_protocol_harness.py's DONE_* constants.
void phSetLastDone(void* handle, uint32_t lastDone) {
  static_cast<Handle*>(handle)->adapter.lastDoneToReturn = lastDone;
}
void phSetLastDoneReason(void* handle, int reason) {
  static_cast<Handle*>(handle)->adapter.lastDoneReasonToReturn =
      static_cast<Protocol::DoneReason>(reason);
}
void phSetSetResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.setResult =
      static_cast<Protocol::Result>(result);
}
void phSetTlmResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.tlmResult =
      static_cast<Protocol::Result>(result);
}
void phSetRunResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.runResult =
      static_cast<Protocol::Result>(result);
}
void phSetRunHasResult(void* handle, int hasResult) {
  static_cast<Handle*>(handle)->adapter.runHasResult = hasResult != 0;
}
// `text` must outlive its use -- same borrowed-pointer contract as
// phSetGetOverride above (the mock stores the pointer, not a copy).
void phSetRunResultText(void* handle, const char* text) {
  static_cast<Handle*>(handle)->adapter.runResultText = text;
}

// ---- MockAdapter call-log readback ---------------------------------------

int phWheelsCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.wheelsCalls;
}
float phLastWheelsLeft(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsLeft;
}
float phLastWheelsRight(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsRight;
}
uint32_t phLastWheelsDuration(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsDuration;
}
uint32_t phLastWheelsId(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsId;
}

int phWheelsXCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.wheelsXCalls;
}
float phLastWheelsXLeft(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsXLeft;
}
float phLastWheelsXRight(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsXRight;
}
float phLastWheelsXCruise(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsXCruise;
}
uint32_t phLastWheelsXTimeout(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastWheelsXTimeout;
}

int phMoveXCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.moveXCalls;
}
float phLastMoveXDistance(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveXDistance;
}
float phLastMoveXRotation(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveXRotation;
}
float phLastMoveXCruise(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveXCruise;
}
uint32_t phLastMoveXTimeout(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveXTimeout;
}

int phMoveVCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.moveVCalls;
}
float phLastMoveVVx(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveVVx;
}
float phLastMoveVOmega(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveVOmega;
}
uint32_t phLastMoveVDuration(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastMoveVDuration;
}

int phGoToRCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.goToRCalls;
}
float phLastGoToRX(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToRX;
}
float phLastGoToRY(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToRY;
}
float phLastGoToRSpeed(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToRSpeed;
}
float phLastGoToRArrive(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToRArrive;
}
uint32_t phLastGoToRTimeout(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToRTimeout;
}

int phGoToWCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.goToWCalls;
}
float phLastGoToWX(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToWX;
}
float phLastGoToWY(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToWY;
}
float phLastGoToWSpeed(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToWSpeed;
}
float phLastGoToWArrive(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToWArrive;
}
uint32_t phLastGoToWTimeout(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastGoToWTimeout;
}

int phStopCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.stopCalls;
}
uint32_t phLastStopId(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastStopId;
}
int phLastStopImmediate(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastStopImmediate ? 1 : 0;
}

int phEstopCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.estopCalls;
}

int phGetCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.getCalls;
}

int phSetCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.setCalls;
}
float phLastSetValue(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastSetValue;
}
uint32_t phLastSetId(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastSetId;
}
int phLastSetNameMatches(void* handle, const char* name) {
  return std::strcmp(static_cast<Handle*>(handle)->adapter.lastSetName,
                      name) == 0
             ? 1
             : 0;
}

int phTlmCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.tlmCalls;
}
int phLastTlmMode(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->adapter.lastTlmMode);
}

int phRunCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.runCalls;
}
int phLastRunNameMatches(void* handle, const char* name) {
  return std::strcmp(static_cast<Handle*>(handle)->adapter.lastRunName,
                      name) == 0
             ? 1
             : 0;
}
int phLastRunArgc(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->adapter.lastRunArgc);
}
// Returns 1 if argv[index] from the last onRun() call equals `value`, 0
// if it does not match OR index is out of the recorded range -- so a
// test cannot mistake "out of range" for "matched an empty string".
int phLastRunArgMatches(void* handle, int index, const char* value) {
  MockAdapter& a = static_cast<Handle*>(handle)->adapter;
  if (index < 0 || static_cast<size_t>(index) >= a.lastRunArgc) return 0;
  if (static_cast<size_t>(index) >= MockAdapter::kMaxRecordedRunArgs) return 0;
  return std::strcmp(a.lastRunArgs[index], value) == 0 ? 1 : 0;
}

int phIdentityCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.identityCalls;
}
int phNowCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.nowCalls;
}
int phStatusCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.statusCalls;
}

// ---- telemetry emission ---------------------------------------------------
// General variable-arity emitTelemetry() driver: `names`/`values`/
// `hexFlags` are parallel C arrays of length `count`, letting Python
// reproduce spec S6.2's exact 9-column POSE example (or any other
// column set) without a second, mode-specific shim function.
void phEmitTelemetry(void* handle, int count, const char* const* names,
                      const int32_t* values, const int32_t* hexFlags) {
  static constexpr int kMaxColumns = 40;
  Protocol::Column cols[kMaxColumns];
  int n = count > kMaxColumns ? kMaxColumns : count;
  for (int i = 0; i < n; ++i) {
    cols[i].name = names[i];
    cols[i].value = values[i];
    cols[i].hex = hexFlags[i] != 0;
  }
  Protocol::Snapshot snapshot{cols, static_cast<size_t>(n)};
  static_cast<Handle*>(handle)->handler.emitTelemetry(snapshot);
}

}  // extern "C"

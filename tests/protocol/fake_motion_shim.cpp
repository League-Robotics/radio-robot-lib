// fake_motion_shim.cpp -- extern "C" ctypes surface for
// test_motion_reliability.py, mirroring protocol_shim.cpp's own shape
// but wiring Protocol::FakeMotionAdapter (fake_motion_adapter.h) instead
// of MockAdapter. Test scaffolding only: nothing under src/ knows this
// file exists, and it is compiled only into this test's own throwaway
// shared library.
#include <cstdint>
#include <cstring>
#include <string>

#include "fake_motion_adapter.h"
#include "protocol_handler.h"

namespace {

// Same shape as protocol_shim.cpp's own RecordingSink.
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
  Protocol::FakeMotionAdapter adapter;
  RecordingSink sink;
  Protocol::ProtocolHandler handler;
  Handle() : handler(adapter, sink) {}
};

}  // namespace

extern "C" {

void* fmCreate() { return new Handle(); }
void fmDestroy(void* handle) { delete static_cast<Handle*>(handle); }

void fmFeed(void* handle, const char* data, int length) {
  static_cast<Handle*>(handle)->handler.feed(data,
                                              static_cast<size_t>(length));
}

// ---- sink readback (identical shape to protocol_shim.cpp) --------------

int fmSinkLength(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->sink.buffer().size());
}
int fmSinkRead(void* handle, char* out, int cap) {
  Handle* h = static_cast<Handle*>(handle);
  size_t n = h->sink.buffer().size();
  if (static_cast<int>(n) > cap) n = static_cast<size_t>(cap);
  std::memcpy(out, h->sink.buffer().data(), n);
  return static_cast<int>(n);
}
void fmSinkClear(void* handle) { static_cast<Handle*>(handle)->sink.clear(); }

uint32_t fmMalformedCount(void* handle) {
  return static_cast<Handle*>(handle)->handler.malformedCount();
}

// ---- FakeMotionAdapter test knobs ---------------------------------------

void fmSetStepsToComplete(void* handle, uint32_t steps) {
  static_cast<Handle*>(handle)->adapter.stepsToComplete = steps;
}
// `reason` is Protocol::DoneReason's DECLARATION-ORDER ordinal
// (adapter.h) -- see test_motion_reliability.py's DONE_* constants.
void fmSetCompletionReason(void* handle, int reason) {
  static_cast<Handle*>(handle)->adapter.completionReason =
      static_cast<Protocol::DoneReason>(reason);
}
// `result` is Protocol::Result's DECLARATION-ORDER ordinal, mirroring
// protocol_shim.cpp's own phSetWheelsResult()-style convention.
void fmSetAcceptResult(void* handle, int result) {
  static_cast<Handle*>(handle)->adapter.acceptResult =
      static_cast<Protocol::Result>(result);
}

void fmStep(void* handle) { static_cast<Handle*>(handle)->adapter.step(); }
void fmForceAbort(void* handle) {
  static_cast<Handle*>(handle)->adapter.forceAbort();
}

int fmActive(void* handle) {
  return static_cast<Handle*>(handle)->adapter.active() ? 1 : 0;
}
uint32_t fmActiveId(void* handle) {
  return static_cast<Handle*>(handle)->adapter.activeId;
}
uint32_t fmStepsRemaining(void* handle) {
  return static_cast<Handle*>(handle)->adapter.stepsRemaining();
}
uint32_t fmLastDone(void* handle) {
  return static_cast<Handle*>(handle)->adapter.lastDone();
}
int fmLastDoneReason(void* handle) {
  return static_cast<int>(static_cast<Handle*>(handle)->adapter.lastDoneReason());
}
int fmStopCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.stopCalls;
}
int fmEstopCalls(void* handle) {
  return static_cast<Handle*>(handle)->adapter.estopCalls;
}

void fmEmitTelemetryIfActive(void* handle) {
  Handle* h = static_cast<Handle*>(handle);
  h->handler.emitTelemetry(h->adapter.buildSnapshot());
}

}  // extern "C"

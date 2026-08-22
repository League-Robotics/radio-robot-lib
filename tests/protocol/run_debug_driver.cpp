// run_debug_driver.cpp -- standalone executable, always compiled WITH
// -fsanitize=address,undefined (test_protocol_adversarial.py owns the
// exact flags), for two RUN/debug behaviors that default MockAdapter
// settings cannot exercise purely through feed() -- the same pattern
// nan_regression_driver.cpp already uses for an Adapter-seam-only
// finding:
//
//   1. sendDebug()'s own sanitization of hostile text: embedded '\n'/
//      '\r' bytes must never reach the sink (they could forge a second
//      line), nullptr and "" must both produce the documented bare
//      "debug\n", a text that is ENTIRELY '\n'/'\r' bytes must collapse
//      onto that same bare shape, and a too-long text must be
//      TRUNCATED to fit the 240-byte line cap, never overflow it.
//   2. RUN's ack-then-ret path with a REGISTERED return value under
//      ASan/UBSan -- this needs MockAdapter to actually have a return
//      value configured (runHasResult=true) for the check to mean
//      anything; the generic adversarial fuzz driver
//      (asan_fuzz_driver.cpp) never configures one. (Historical note:
//      this used to also demonstrate RUN's `#0`-suppresses-a-ret rule;
//      `#0` no longer suppresses anything as of the 2026-08-21
//      reliability layer -- docs/design/protocol.md §6.3 -- it is
//      simply always a stale retransmit that never invokes onRun() at
//      all, which test_protocol_harness.py's
//      test_run_id_zero_is_a_stale_retransmit_never_executes already
//      covers via the ctypes shim's own call-count introspection; this
//      driver has no such introspection, so it no longer tries to
//      demonstrate that case.)
//
// Nothing under src/ knows this file exists; test scaffolding only,
// like protocol_shim.cpp / mock_adapter.h / nan_regression_driver.cpp.
#include <cstdio>
#include <string>

#include "mock_adapter.h"
#include "protocol_handler.h"

namespace {

class StdoutSink : public Protocol::Sink {
 public:
  void write(const char* data, size_t length) override {
    std::fwrite(data, 1, length, stdout);
  }
};

void feedLine(Protocol::ProtocolHandler& handler, const std::string& line) {
  handler.feed(line.data(), line.size());
}

}  // namespace

int main() {
  MockAdapter adapter;
  StdoutSink sink;
  Protocol::ProtocolHandler handler(adapter, sink);

  // ---- 1. sendDebug() sanitization ----
  handler.sendDebug("hello\nworld\r\n");  // embedded '\n'/'\r' stripped
  handler.sendDebug("");                  // empty -> bare "debug\n"
  handler.sendDebug(nullptr);             // null -> the SAME case as ""
  handler.sendDebug("\n\r\n\r");          // entirely '\n'/'\r' -> bare too
  std::string longText(500, 'z');         // way over the 240-byte cap
  handler.sendDebug(longText.c_str());

  // ---- 2. RUN with a REGISTERED return value, under ASan/UBSan ----
  adapter.runResult = Protocol::Result::kOk;
  adapter.runHasResult = true;
  adapter.runResultText = "42";
  feedLine(handler, "RUN foo #1\n");  // must produce "ack 1 0\nret 42 #1\n"

  std::fflush(stdout);
  return 0;
}

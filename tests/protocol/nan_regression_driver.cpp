// nan_regression_driver.cpp -- standalone executable, always compiled
// WITH -fsanitize=address,undefined (test_protocol_adversarial.py owns
// the exact flags), for two specific ProtocolHandler regressions the
// adversarial-input sweep found that feed() ALONE cannot reach --
// both live on the GET reply-formatting path (handleGet() /
// formatConfigValue(), protocol_handler.cpp), driven by a value the
// ADAPTER hands back from onGet(), not by anything parsed off the
// wire:
//
//   1. formatConfigValue(NaN) used to reach
//      static_cast<uint32_t>(<NaN>) -- undefined behavior (a NaN can
//      never arrive VIA the wire; parseFloatField rejects it on input,
//      spec S2.2/S7.2's "no NaN, no inf" -- but nothing stopped the
//      adapter's own stored value from being one, e.g. a divide-by-zero
//      elsewhere in a real firmware's config math). Fixed by clamping
//      NaN to 0.0 before the cast; this driver is the regression test
//      for that fix, and would abort under UBSan without it.
//   2. The historical GET reply-buffer bug this file's own header
//      comment already documents (protocol_handler.h: the echoed field
//      name can legally reach ~235 bytes, wire-controlled by spec S2's
//      240-byte line cap, but the reply buffer used to be a fixed
//      char[96]) is re-run here under ASan for extra assurance, per
//      "assume there are more" -- this is a non-regression check, not
//      a new finding.
//
// Nothing under src/ knows this file exists; test scaffolding only,
// like protocol_shim.cpp and mock_adapter.h.
#include <cmath>
#include <cstdio>
#include <limits>
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

  // ---- 1. NaN / +Inf / -Inf through GET's reply-formatting path ----
  adapter.overrideName = "nan.field";
  adapter.overrideValue = std::nanf("");
  feedLine(handler, "GET:nan.field\n");

  adapter.overrideName = "posinf.field";
  adapter.overrideValue = std::numeric_limits<float>::infinity();
  feedLine(handler, "GET:posinf.field\n");

  adapter.overrideName = "neginf.field";
  adapter.overrideValue = -std::numeric_limits<float>::infinity();
  feedLine(handler, "GET:neginf.field\n");

  // ---- 2. A 235-byte GET field name (spec S2's 240-byte line cap is
  // the only bound on it) through the SAME reply buffer, non-regression
  // re-check under ASan.
  std::string longName(235, 'n');
  adapter.overrideName = longName.c_str();
  adapter.overrideValue = 1.5f;
  feedLine(handler, "GET:" + longName + "\n");

  std::fflush(stdout);
  return 0;
}

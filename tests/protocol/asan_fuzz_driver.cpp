// asan_fuzz_driver.cpp -- standalone executable, always compiled WITH
// -fsanitize=address,undefined (test_protocol_adversarial.py owns the
// exact flags), for the protocol handler's adversarial-input and
// recovery-invariant sweep. Nothing under src/ knows this file exists;
// like protocol_shim.cpp and mock_adapter.h it is test scaffolding
// only, and it is never linked into anything but this test's own
// throwaway executable.
//
// Unlike the ctypes shim (protocol_shim.cpp), this is a real process,
// not a shared library loaded into a running Python interpreter --
// AddressSanitizer's interceptors must be present from process start
// (see the sanitizer runtime's own "Interceptors are not working"
// diagnostic if you try to retrofit ASan onto a dlopen()'d library
// instead), so a small standalone `main()` compiled straight into an
// executable is the simplest way to get a real crash/UB report on
// stderr and a nonzero exit code out of a hostile feed().
//
// stdin protocol: zero or more records of
//   uint32_t length   (native byte order -- host-only tooling, no wire
//                       byte of this format ever crosses a real
//                       transport)
//   `length` raw bytes
// read until EOF (a length read that comes back short, or a payload
// read that comes back short, ends the stream early rather than
// looping forever). Each record is handed to ONE feed() call, in
// order, on the SAME handler instance -- so a Python-side test chooses
// how many records/bytes-per-record to send and thereby exercises
// feed()'s own cross-call buffering contract (docs/design/protocol.md
// S2.1), not just a single hostile blob.
//
// The sink's entire accumulated output is written to stdout once, at
// the end, so the Python side can assert on it (e.g. "a well-formed
// command after the garbage still produced its normal reply" -- the
// recovery invariant). Exit code 0 with sanitizers linked in is itself
// half the assertion: a heap/stack overflow, use-after-free, or
// undefined-behavior trap aborts the process with a sanitizer report
// on stderr and a nonzero exit before this ever reaches the final
// return.
#include <cstdint>
#include <cstdio>
#include <vector>

#include "mock_adapter.h"
#include "protocol_handler.h"

namespace {

class StdoutSink : public Protocol::Sink {
 public:
  void write(const char* data, size_t length) override {
    std::fwrite(data, 1, length, stdout);
  }
};

}  // namespace

int main() {
  MockAdapter adapter;
  StdoutSink sink;
  Protocol::ProtocolHandler handler(adapter, sink);

  std::vector<char> chunk;
  for (;;) {
    uint32_t length = 0;
    size_t got = std::fread(&length, sizeof(length), 1, stdin);
    if (got != 1) break;  // clean EOF between records

    chunk.assign(length, '\0');
    if (length > 0) {
      size_t payloadGot = std::fread(chunk.data(), 1, length, stdin);
      if (payloadGot != length) break;  // truncated record -- stop here
    }
    handler.feed(chunk.data(), chunk.size());
  }

  std::fflush(stdout);
  return 0;
}

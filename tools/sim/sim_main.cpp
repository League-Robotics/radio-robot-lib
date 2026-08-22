// tools/sim/sim_main.cpp -- a host-only development tool. Composes the
// real Protocol::ProtocolHandler (src/protocol/protocol_handler.h) with
// Protocol::FakeMotionAdapter (tests/protocol/fake_motion_adapter.h) and
// serves protocol-v6 lines over a stream -- TCP (--listen HOST:PORT) or
// stdio (--stdio). This is "the compiled host version of the firmware":
// it lets a v6 client (rogo, or this repo's own robot_v6 Python package)
// be built and tested against something that speaks the real wire
// grammar and the real reliability layer, with no robot and no serial
// port anywhere in the loop.
//
// ---- Why tools/, not src/ ----
//
// This file links against the unmodified src/protocol/protocol_handler.cpp
// and the test-only tests/protocol/fake_motion_adapter.h, but it is
// itself firmware-UNCONSTRAINED: it uses the full standard library
// (sockets, <chrono>, <string>, exceptions are still avoided by
// convention but not required). Putting it under src/ would blur the
// line src/protocol/README.md and docs/design/protocol.md both draw
// deliberately -- "src/ stays pure" (no dynamic allocation, no
// std::string, no exceptions in src/protocol|diffdrive) -- so it lives
// under tools/ instead, the same way a firmware repo keeps its flashing
// scripts and bench harnesses out of the tree that gets frozen and
// split into its own repository. It reuses fake_motion_adapter.h
// (normally test-only scaffolding) rather than duplicating it, exactly
// as the dispatch that created this file asked.
//
// ---- What it does ----
//
// One session = one Protocol::FakeMotionAdapter + one ProtocolHandler.
// sendBanner() fires the instant a peer is connected (TCP accept, or
// process start for --stdio). A single loop then either:
//   - reads whatever bytes are available from the peer and feed()s them
//     straight into the handler (replies come back over the same
//     stream, synchronously, from inside feed()), or
//   - on a --period ms cadence, calls FakeMotionAdapter::step() once
//     and hands its buildSnapshot() to emitTelemetry() -- this is what
//     makes a multi-step move actually progress, and what rides the
//     ack/nack reliability piggyback (docs/design/protocol.md S8.5)
//     out onto the wire on its own, exactly like a real robot's own
//     periodic tick would.
//
// TCP mode serves one client at a time (accept() again once a client
// disconnects); stdio mode serves exactly the one process it was
// launched as -- both are asked for in the dispatch that created this
// file, and tests should prefer --stdio (no port allocation, no bind
// races, no leaked listeners in CI).
//
// ---- Shutdown ----
//
// SIGINT/SIGTERM set a flag checked every loop iteration; EOF on the
// input stream (read() returning 0) ends that session's loop the same
// way. Either way, BEFORE the process exits, the session feeds a
// synthetic "ESTOP\n" through the very same handler it has been
// running -- the same code path a real ESTOP takes (docs/design/
// protocol.md S8.3), so it both stops whatever FakeMotionAdapter motion
// might still be "active" AND emits the real `estop` confirmation line
// to whatever is still listening. A sim that exits leaving a move
// "running" teaches the wrong habit (the dispatch's own words) -- this
// is the fix, reusing the real verb rather than inventing a side
// channel that bypasses the wire entirely.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>  // sigaction -- POSIX, not guaranteed by <csignal> alone
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>  // std::sig_atomic_t
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "fake_motion_adapter.h"
#include "protocol_handler.h"

namespace {

volatile std::sig_atomic_t gShutdownRequested = 0;

void onShutdownSignal(int /*signum*/) { gShutdownRequested = 1; }

void installSignalHandlers() {
  struct sigaction action {};
  action.sa_handler = onShutdownSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;  // deliberately NOT SA_RESTART: poll()/accept()
                         // must return EINTR promptly so the shutdown
                         // flag is observed without waiting out a full
                         // --period tick or a pending accept().
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  // A client that goes away mid-write must not kill this process.
  signal(SIGPIPE, SIG_IGN);
}

// Sink that writes formatted reply lines straight to a file descriptor
// (a TCP client socket, or stdout for --stdio). Best-effort: Sink's own
// interface is void, so a write() failure (peer gone) has nowhere to
// report to -- the caller finds out the same way any real transport
// would, via the next read() returning EOF/error.
class FdSink : public Protocol::Sink {
 public:
  explicit FdSink(int fd) : fd_(fd) {}

  void write(const char* data, size_t length) override {
    size_t written = 0;
    while (written < length) {
      ssize_t n = ::write(fd_, data + written, length - written);
      if (n < 0) {
        if (errno == EINTR) continue;
        return;  // peer gone; nothing further this interface can do
      }
      written += static_cast<size_t>(n);
    }
  }

 private:
  int fd_;
};

void printUsage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s (--stdio | --listen HOST:PORT) [--period MS]\n"
               "\n"
               "  --stdio           speak protocol-v6 on stdin/stdout "
               "(preferred for tests)\n"
               "  --listen H:PORT   speak protocol-v6 on a TCP socket, "
               "one client at a time\n"
               "  --period MS       telemetry/step cadence in "
               "milliseconds (default 24)\n",
               argv0);
}

// Runs one session (one peer, one FakeMotionAdapter, one
// ProtocolHandler) until EOF, a read error, or gShutdownRequested is
// set. Returns true if shutdown was requested (so the caller knows not
// to accept another TCP client afterward).
bool runSession(int fdIn, int fdOut, int period) {
  Protocol::FakeMotionAdapter adapter;
  adapter.identityToReturn =
      Protocol::Identity{"sim", "SIMHOST0001", "differential", "sim", "6.0.0"};

  FdSink sink(fdOut);
  Protocol::ProtocolHandler handler(adapter, sink);
  handler.sendBanner();

  using Clock = std::chrono::steady_clock;
  auto nextTick = Clock::now() + std::chrono::milliseconds(period);

  char buf[4096];
  bool peerGone = false;
  while (!gShutdownRequested && !peerGone) {
    auto now = Clock::now();
    auto untilNextTick =  // [ms]
        std::chrono::duration_cast<std::chrono::milliseconds>(nextTick - now)
            .count();
    int pollTimeout =  // [ms] -- poll()'s own third argument is
                        // unnamed-but-milliseconds by POSIX convention
        untilNextTick < 0 ? 0 : static_cast<int>(untilNextTick);

    struct pollfd pfd {};
    pfd.fd = fdIn;
    pfd.events = POLLIN;
    int rc = ::poll(&pfd, 1, pollTimeout);

    if (gShutdownRequested) break;

    if (rc < 0) {
      if (errno == EINTR) continue;
      std::perror("sim: poll");
      break;
    }

    if (rc > 0 && (pfd.revents & (POLLIN | POLLHUP | POLLERR))) {
      ssize_t n = ::read(fdIn, buf, sizeof(buf));
      if (n <= 0) {
        peerGone = true;  // EOF (n==0) or a read error (n<0): either
                           // way this session is over -- fall through
                           // to the shared shutdown-stop below rather
                           // than looping on a dead descriptor.
      } else {
        handler.feed(buf, static_cast<size_t>(n));
      }
    }

    now = Clock::now();
    if (!peerGone && now >= nextTick) {
      adapter.step();
      handler.emitTelemetry(adapter.buildSnapshot());
      nextTick += std::chrono::milliseconds(period);
      if (nextTick < now) nextTick = now + std::chrono::milliseconds(period);
    }
  }

  // Stop the motion on the way out, through the SAME wire verb a real
  // panic stop takes -- see this file's own header comment.
  static constexpr char kEstop[] = "ESTOP\n";
  handler.feed(kEstop, sizeof(kEstop) - 1);

  return gShutdownRequested != 0;
}

int makeListenSocket(const std::string& host, int port) {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    std::perror("sim: socket");
    return -1;
  }
  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    std::fprintf(stderr, "sim: not a valid IPv4 address: %s\n", host.c_str());
    ::close(fd);
    return -1;
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::perror("sim: bind");
    ::close(fd);
    return -1;
  }
  if (::listen(fd, 1) < 0) {
    std::perror("sim: listen");
    ::close(fd);
    return -1;
  }
  return fd;
}

}  // namespace

int main(int argc, char** argv) {
  installSignalHandlers();

  bool stdioMode = false;
  std::string listenSpec;
  int period = 24;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--stdio") {
      stdioMode = true;
    } else if (arg == "--listen" && i + 1 < argc) {
      listenSpec = argv[++i];
    } else if (arg == "--period" && i + 1 < argc) {
      period = std::atoi(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "sim: unrecognized argument: %s\n", arg.c_str());
      printUsage(argv[0]);
      return 2;
    }
  }

  if (stdioMode == !listenSpec.empty()) {
    std::fprintf(stderr, "sim: specify exactly one of --stdio or --listen\n");
    printUsage(argv[0]);
    return 2;
  }
  if (period <= 0) {
    std::fprintf(stderr, "sim: --period must be a positive integer\n");
    return 2;
  }

  if (stdioMode) {
    runSession(STDIN_FILENO, STDOUT_FILENO, period);
    return 0;
  }

  size_t colon = listenSpec.rfind(':');
  if (colon == std::string::npos) {
    std::fprintf(stderr, "sim: --listen wants HOST:PORT, got %s\n",
                 listenSpec.c_str());
    return 2;
  }
  std::string host = listenSpec.substr(0, colon);
  int port = std::atoi(listenSpec.substr(colon + 1).c_str());

  int listenFd = makeListenSocket(host, port);
  if (listenFd < 0) return 1;
  std::fprintf(stderr, "sim: listening on %s:%d (protocol-v6)\n", host.c_str(),
               port);

  while (!gShutdownRequested) {
    sockaddr_in clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);
    int clientFd = ::accept(
        listenFd, reinterpret_cast<sockaddr*>(&clientAddr), &clientLen);
    if (clientFd < 0) {
      if (errno == EINTR) continue;
      if (gShutdownRequested) break;
      std::perror("sim: accept");
      break;
    }
    std::fprintf(stderr, "sim: client connected\n");
    bool shuttingDown = runSession(clientFd, clientFd, period);
    ::close(clientFd);
    std::fprintf(stderr, "sim: client disconnected\n");
    if (shuttingDown) break;
    // "one client at a time is fine" -- go back and accept the next
    // one rather than exiting, so a test session can reconnect without
    // relaunching the whole sim process.
  }

  ::close(listenFd);
  return 0;
}

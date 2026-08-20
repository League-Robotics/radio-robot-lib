#pragma once

#include <cstddef>
#include "core/debug.h"
#include <cstdint>

#include "firm/types/robot_state.h"
#include "hal/transport.h"
#include "messages/commands.h"
#include "messages/envelope.h"
#include "messages/wire.h"

namespace Core {

class Telemetry;

constexpr uint8_t kCobsDelimiter = 0x0A;

constexpr uint16_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;  // == 192

constexpr uint16_t kMaxCrcPayloadBytes = kMaxEnvelopeBytes + 2;  // == 194

constexpr uint16_t kFramedMaxBytes = 200;
static_assert(kFramedMaxBytes >= kMaxCrcPayloadBytes + kMaxCrcPayloadBytes / 254 + 1,
              "kFramedMaxBytes must cover cobsEncodedMaxLength(kMaxCrcPayloadBytes)");

constexpr size_t maxVerbNameLength() {
  size_t maxLen = 0;
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    size_t len = 0;
    for (const char* p = msg::kVerbTable[i].name; *p != '\0'; ++p) ++len;
    if (len > maxLen) maxLen = len;
  }
  return maxLen;
}

constexpr uint16_t kMaxCommandPrefixBytes = static_cast<uint16_t>(maxVerbNameLength() + 1);  // name + ':'

constexpr uint16_t kMaxLineBytes = kFramedMaxBytes + kMaxCommandPrefixBytes;

enum class CmdStatus : uint8_t { kNone = 0, kDecoded = 1 };

struct Cmd {
  CmdStatus status = CmdStatus::kNone;
  msg::CommandEnvelope env;
};

constexpr uint8_t kCmdRingDepth = 12;

constexpr uint8_t kPumpMaxLines = 2 * kCmdRingDepth;

class Comms {
 public:
  struct Status {
    bool ready = false;             // boot() finished; Moves are accepted
    bool active = false;            // a Move is running
    bool wheelLeftConnected = false;
    bool wheelRightConnected = false;
    bool otosPresent = false;
    bool wedged = false;            // encoder stuck-position latch
    uint32_t flags = 0;             // the full telemetry flags word

    uint8_t tlmMode = 1;

    int32_t otosX = 0;        // [mm]
    int32_t otosY = 0;        // [mm]
    int32_t otosHeading = 0;  // [mrad]
    int32_t encX = 0;         // [mm]
    int32_t encY = 0;         // [mm]
    int32_t encHeading = 0;   // [mrad]
  };

  void setStatus(const Status& status) { status_ = status; }

  void updateStatus(const Types::RobotState& state, const Telemetry& tlm);

  enum class TlmAction : uint8_t { kNone, kFrame, kSetOff, kSetAuto, kSetOn, kUnrecognized };

  TlmAction takeTlmAction() {
    const TlmAction action = tlmAction_;
    tlmAction_ = TlmAction::kNone;
    return action;
  }

  // An external world fix waiting for the loop to install it. Staged here
  // because Comms cannot reach the OTOS or the odometry itself.
  struct SeedRequest {
    bool pending = false;
    float x = 0.0f;        // [mm]
    float y = 0.0f;        // [mm]
    float heading = 0.0f;  // [rad]
    Hal::Transport* reply = nullptr;
  };

  SeedRequest takeSeed() {
    const SeedRequest seed = seed_;
    seed_ = SeedRequest{};
    return seed;
  }

  // GET/SET staging (protocol v6 spec section 7) -- Comms parses/validates
  // the ASCII line ONLY; the actual field lookup, NaN/range checks, and
  // config_ read/write live in Core::Configurator, which Comms does not
  // reference directly. RobotLoop mediates -- the SAME shape SeedRequest
  // above already uses, for the same reason: Comms cannot reach
  // Configurator/Otos itself, only RobotLoop holds both.
  //
  // Single-slot, "last wins" -- mirrors SeedRequest/tlmAction_ above, not
  // a ring like DbgAction. A burst of more than one GET/SET line arriving
  // within the SAME cycle's pump() drain loses all but the last; accepted
  // for the same reason SeedRequest/TlmAction already accept it (one
  // interactive tuning request at a time is the expected usage, not a
  // command stream).
  enum class ConfigRequestKind : uint8_t { kNone, kGetOne, kGetAll, kSet };

  // Longest name in Config::WireV6::kConfigFieldTable is 35 bytes
  // ("navigator.default_arrival_tolerance") -- 48 leaves headroom without
  // chasing the exact figure via a constexpr scan the way
  // kMaxCommandPrefixBytes does for the binary verb table.
  static constexpr uint8_t kMaxConfigFieldNameBytes = 48;

  struct ConfigRequest {
    ConfigRequestKind kind = ConfigRequestKind::kNone;
    char name[kMaxConfigFieldNameBytes] = {};  // kGetOne/kSet; unused for kGetAll
    float value = 0.0f;                        // kSet only
    uint32_t id = 0;                           // kSet only; defaults to 0 when the wire omits it (spec section 7.1's own examples)
    Hal::Transport* reply = nullptr;
  };

  ConfigRequest takeConfigRequest() {
    const ConfigRequest request = configRequest_;
    configRequest_ = ConfigRequest{};
    return request;
  }

  enum class DbgActionKind : uint8_t { kNone, kMark, kPing, kWedge, kClear,
                                       kVmin, kGain, kASteady, kPos, kOtos,
                                       kUnrecognized };
  struct DbgAction {
    DbgActionKind kind = DbgActionKind::kNone;
    char text[64] = {};   // kMark: the full original data ("mark leg1a")
    uint8_t port = 0;     // kWedge: 1 = left, 2 = right, 3 = both
    uint32_t duration = 0;  // [ms] kWedge auto-clear; 0 = latched
    float value = 0.0f;
    float value2 = 0.0f;
  };

  DbgAction takeDbgAction() {
    if (dbgCount_ == 0) return DbgAction{};
    const DbgAction action = dbgRing_[dbgHead_];
    dbgHead_ = (dbgHead_ + 1) % kDbgRingDepth;
    --dbgCount_;
    return action;
  }

  void sendTlmReply(TlmAction action);

  // Comms registers a COLLECTION of transports, not two named slots. Every
  // registered transport is pumped for inbound lines and receives every
  // broadcast (banner, READY, telemetry); a reply goes back out the one
  // transport its command arrived on.
  //
  // Fixed-size, no heap, like the rest of this codebase: kMaxTransports is
  // the ceiling and registration past it is dropped (see addTransport()).
  // Registration happens once, at Robot-composition time, before boot.
  //
  // Ordering IS significant and is the caller's: pump() offers each
  // transport a line in registration order, so the first-registered
  // transport wins a tie when two have traffic in the same cycle. The
  // composition roots register serial first, then radio, which preserves
  // the exact precedence the previous two-slot pump() had.
  static constexpr uint8_t kMaxTransports = 4;

  // The two-transport convenience form every current composition root
  // uses. Equivalent to default-constructing and calling addTransport()
  // twice, in this order.
  Comms(Hal::Transport& serialLink, Hal::Transport& radioLink, const char* banner, const char* idLine = "ID:unknown");

  explicit Comms(const char* banner, const char* idLine = "ID:unknown");

  // Register one more transport. Returns false if kMaxTransports are
  // already registered -- a caller that ignores it silently loses a link,
  // so this is [[nodiscard]].
  [[nodiscard]] bool addTransport(Hal::Transport& transport);

  uint8_t transportCount() const { return transportCount_; }

  // --- On the WiFi transport this reorganization does NOT ship ---
  //
  // The structural block the reorganization proposal named is gone: Comms
  // held two named slots, and adding a third transport meant changing this
  // class. It no longer does -- a composition root registers what the robot
  // has, and Core::composeRobot() still takes the serial/radio pair every
  // robot in this fleet has while a caller adds extras through
  // addTransport() before boot().
  //
  // What is NOT here is Core::WifiTransport around the Ai-WB2-12F module.
  // It is not a missing wrapper -- it needs a whole layer that does not
  // exist yet: a Platform::Uart interface (every device in this tree today
  // is I2C), an AT-command driver in hardware/ for the module, and then
  // this class on top of them. The proposal itself says the UART platform
  // surface "should be shaped by the first Raspberry-Pi or WiFi-module port
  // that actually needs them, rather than speculatively designed now."
  //
  // There is also a measured hardware reason not to rush it. The bench AT
  // bridge (src/tests/bench/wifi/atbridge_main.cpp) records that CODAL's
  // UARTE RX drops ~5-10% of characters at a sustained 115200 on this board
  // and runs the module side at 57600 to widen the IRQ race window (bench
  // finding 2026-08-08). A Transport built on that link today would be
  // lossy by construction, which is a bench project with its own hardware
  // acceptance, not a mechanical addition to a reorganization.

  void pump(uint32_t now);  // [ms]

  bool takeCommand(Cmd& out);

  uint8_t pendingCommandCount() const { return cmdCount_; }

  void sendReply(const msg::ReplyEnvelope& reply);

  void sendBanner();

  void sendReady();

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)
  void sendDebug(const char* line);
#endif

 private:
  void sendStatus(Hal::Transport& t);
  void sendHelp(Hal::Transport& t);

 public:

  uint32_t malformedCount() const { return malformedCount_; }

  uint32_t commandsDroppedCount() const { return commandsDroppedCount_; }

 private:
  bool pumpTransport(Hal::Transport& t, uint32_t now);  // [ms]

  void pushCommand(const Cmd& cmd);

  void dispatchLine(Hal::Transport& t, const char* line, uint16_t lineLen, Cmd& out, uint32_t now);  // [ms]

  void dispatchCleartext(msg::Verb verb, Hal::Transport& t, uint32_t now);  // [ms]
  void stageSeed(const uint8_t* data, uint16_t len, Hal::Transport& t);
  void sendPose(Hal::Transport& t);

  // GET/SET wire-level parsing (protocol v6 spec section 7.1) --
  // dispatchLine() calls these directly, ahead of the v5 registry lookup
  // (same interception point as the relay control-plane carve-out), since
  // "GET"/"SET" are not v5 kVerbTable entries. hasName distinguishes bare
  // "GET" (dump every field) from "GET:<name>" (one field).
  void stageConfigGet(const uint8_t* data, uint16_t len, bool hasName, Hal::Transport& t);
  void stageConfigSet(const uint8_t* data, uint16_t len, Hal::Transport& t);

  void decodeBinaryFrame(const uint8_t* command, size_t commandLen, const uint8_t* data, uint16_t dataLen, Cmd& out);

  // Pointers, not references: a fixed-size array of references is not a
  // thing C++ has. Every entry below transportCount_ is non-null by
  // construction (addTransport() takes a reference).
  Hal::Transport* transports_[kMaxTransports] = {};
  uint8_t transportCount_ = 0;

  // broadcast helpers -- every registered transport, in registration order.
  void broadcast(const uint8_t* data, uint16_t len);
  void broadcastReliable(const char* line);
  const char* banner_;
  const char* idLine_;

  Status status_{};

  TlmAction tlmAction_ = TlmAction::kNone;
  SeedRequest seed_{};  // staged by dispatchLine(); drained by RobotLoop
  ConfigRequest configRequest_{};  // staged by dispatchLine(); drained by RobotLoop
  static constexpr uint8_t kDbgRingDepth = 4;
  DbgAction dbgRing_[kDbgRingDepth]{};  // staged by dispatchLine(); drained by RobotLoop
  uint8_t dbgHead_ = 0;
  uint8_t dbgCount_ = 0;
  void pushDbgAction(const DbgAction& action) {
    if (dbgCount_ >= kDbgRingDepth) return;  // drop-newest
    dbgRing_[(dbgHead_ + dbgCount_) % kDbgRingDepth] = action;
    ++dbgCount_;
  }
  Hal::Transport* tlmReplyTransport_ = nullptr;
  uint32_t malformedCount_ = 0;
  uint32_t commandsDroppedCount_ = 0;

  Cmd cmdRing_[kCmdRingDepth]{};
  uint8_t cmdHead_ = 0;
  uint8_t cmdCount_ = 0;
};

}  // namespace Core

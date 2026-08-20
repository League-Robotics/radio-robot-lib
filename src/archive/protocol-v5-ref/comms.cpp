#include "core/comms.h"
#include "core/debug.h"
#include <cstdlib>

#include <cstdio>
#include <cstring>

#include "core/telemetry.h"
#include "messages/wire_runtime.h"
#include "types/version_generated.h"

namespace Core {

namespace {

uint16_t crcOverScope(const uint8_t* command, size_t commandLen, const uint8_t* payload, size_t payloadLen) {
  uint16_t crc = WireRuntime::crcInit();
  if (commandLen > 0) {
    crc = WireRuntime::crcUpdate(crc, command, commandLen);
    static constexpr uint8_t kCommandSeparator = ':';
    crc = WireRuntime::crcUpdate(crc, &kCommandSeparator, 1);
  }
  return WireRuntime::crcUpdate(crc, payload, payloadLen);
}

bool isRelayControlPlaneLine(const char* line, uint16_t lineLen) {
  if (lineLen == 0) return false;
  const char first = line[0];
  return first == '#' || first == '!' || first == '?';
}

// matchesVerbLiteral() -- case-sensitive exact match against a literal
// verb string. GET/SET (protocol v6 spec section 7.1) are not v5
// kVerbTable entries (messages/commands.h is generated from
// protos/commands.proto, out of this ticket's scope), so they cannot go
// through findVerb() -- dispatchLine() checks this ahead of the registry
// lookup instead, the same interception shape isRelayControlPlaneLine()
// above already uses.
bool matchesVerbLiteral(const char* cmdPtr, uint16_t cmdLen, const char* literal) {
  const size_t literalLen = std::strlen(literal);
  return cmdLen == literalLen && std::memcmp(cmdPtr, literal, literalLen) == 0;
}

const msg::VerbEntry* findVerb(const char* name, uint16_t nameLen) {
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    const msg::VerbEntry& entry = msg::kVerbTable[i];
    size_t entryLen = 0;
    while (entry.name[entryLen] != '\0') ++entryLen;
    if (entryLen == nameLen && std::memcmp(entry.name, name, nameLen) == 0) return &entry;
  }
  return nullptr;
}

const char* verbName(msg::Verb verb) {
  for (uint8_t i = 0; i < msg::kVerbCount; ++i) {
    if (msg::kVerbTable[i].verb == verb) return msg::kVerbTable[i].name;
  }
  return nullptr;
}

Core::Comms::TlmAction classifyTlmArg(const uint8_t* data, uint16_t len) {
  if (len > 0 && data[len - 1] == '\r') --len;

  auto matches = [data, len](const char* token) {
    const size_t tokenLen = std::strlen(token);
    if (tokenLen != len) return false;
    for (size_t i = 0; i < tokenLen; ++i) {
      char a = static_cast<char>(data[i]);
      char b = token[i];
      if (a >= 'a' && a <= 'z') a = static_cast<char>(a - 'a' + 'A');
      if (b >= 'a' && b <= 'z') b = static_cast<char>(b - 'a' + 'A');
      if (a != b) return false;
    }
    return true;
  };

  if (matches("NOW")) return Core::Comms::TlmAction::kFrame;
  if (matches("ON")) return Core::Comms::TlmAction::kSetOn;
  if (matches("AUTO")) return Core::Comms::TlmAction::kSetAuto;
  if (matches("OFF")) return Core::Comms::TlmAction::kSetOff;
  return Core::Comms::TlmAction::kUnrecognized;
}

#ifdef ROBOT_DEBUG
Core::Comms::DbgAction classifyDbgArg(const uint8_t* data, uint16_t len) {
  Core::Comms::DbgAction action;
  action.kind = Core::Comms::DbgActionKind::kUnrecognized;
  if (data == nullptr || len == 0) return action;
  const size_t cap = sizeof(action.text) - 1;
  const size_t n = (len < cap) ? len : cap;
  std::memcpy(action.text, data, n);
  action.text[n] = '\0';

  char* saveptr = action.text;
  auto isSep = [](char c) {
    return c == ' ' || c == '\n' || c == '\r' || c == '\t';
  };
  auto nextToken = [&saveptr, &isSep]() -> char* {
    while (isSep(*saveptr)) ++saveptr;
    if (*saveptr == '\0') return nullptr;
    char* tok = saveptr;
    while (!isSep(*saveptr) && *saveptr != '\0') ++saveptr;
    if (*saveptr != '\0') { *saveptr = '\0'; ++saveptr; }
    return tok;
  };

  char verbatim[sizeof(action.text)];
  std::memcpy(verbatim, action.text, sizeof(verbatim));

  const char* sub = nextToken();
  if (sub == nullptr) return action;
  if (std::strcmp(sub, "mark") == 0) {
    action.kind = Core::Comms::DbgActionKind::kMark;
    std::memcpy(action.text, verbatim, sizeof(action.text));
    return action;
  }
  if (std::strcmp(sub, "ping") == 0) {
    action.kind = Core::Comms::DbgActionKind::kPing;
    return action;
  }
  if (std::strcmp(sub, "clear") == 0) {
    action.kind = Core::Comms::DbgActionKind::kClear;
    return action;
  }
  if (std::strcmp(sub, "otos") == 0) {
    action.kind = Core::Comms::DbgActionKind::kOtos;
    return action;
  }
  auto parseScalar = [&nextToken](float& out) -> bool {
    const char* token = nextToken();
    if (token == nullptr) return false;
    char* end = nullptr;
    const float parsed = std::strtof(token, &end);
    if (end == token || *end != '\0') return false;  // empty or trailing junk
    if (!(parsed >= 0.0f)) return false;             // negative, or NaN
    out = parsed;
    return true;
  };

  if (std::strcmp(sub, "vmin") == 0) {
    if (!parseScalar(action.value)) return action;
    action.kind = Core::Comms::DbgActionKind::kVmin;
    return action;
  }
  if (std::strcmp(sub, "asteady") == 0) {
    if (!parseScalar(action.value)) return action;
    action.kind = Core::Comms::DbgActionKind::kASteady;
    return action;
  }
  if (std::strcmp(sub, "pos") == 0) {
    if (!parseScalar(action.value)) return action;
    action.kind = Core::Comms::DbgActionKind::kPos;
    return action;
  }
  if (std::strcmp(sub, "gain") == 0) {
    if (!parseScalar(action.value)) return action;
    if (!parseScalar(action.value2)) return action;
    if (action.value <= 0.0f || action.value2 <= 0.0f) return action;
    action.kind = Core::Comms::DbgActionKind::kGain;
    return action;
  }
  if (std::strcmp(sub, "wedge") == 0) {
    const char* which = nextToken();
    if (which == nullptr) return action;
    if (std::strcmp(which, "left") == 0) action.port = 1;
    else if (std::strcmp(which, "right") == 0) action.port = 2;
    else if (std::strcmp(which, "both") == 0) action.port = 3;
    else return action;
    const char* ms = nextToken();
    if (ms != nullptr) action.duration = static_cast<uint32_t>(std::strtoul(ms, nullptr, 10));
    action.kind = Core::Comms::DbgActionKind::kWedge;
    return action;
  }
  return action;
}
#endif  // ROBOT_DEBUG

msg::Verb bodyKindToVerb(msg::ReplyEnvelope::BodyKind kind) {
  switch (kind) {
    case msg::ReplyEnvelope::BodyKind::TLM: return msg::Verb::TLM;
    case msg::ReplyEnvelope::BodyKind::OK: return msg::Verb::OK;
    case msg::ReplyEnvelope::BodyKind::ERR: return msg::Verb::ERR;
    case msg::ReplyEnvelope::BodyKind::CFG: return msg::Verb::CFG;
    case msg::ReplyEnvelope::BodyKind::NONE:
    default: return msg::Verb::VERB_UNSPECIFIED;
  }
}

}  // namespace

Comms::Comms(const char* banner, const char* idLine)
    : banner_(banner), idLine_(idLine) {}

Comms::Comms(Hal::Transport& serialLink, Hal::Transport& radioLink, const char* banner, const char* idLine)
    : banner_(banner), idLine_(idLine) {
  // Serial first, then radio -- pump() offers transports a line in
  // registration order, so this preserves the precedence the previous
  // two-named-slot pump() had exactly.
  (void)addTransport(serialLink);
  (void)addTransport(radioLink);
}

bool Comms::addTransport(Hal::Transport& transport) {
  if (transportCount_ >= kMaxTransports) return false;
  transports_[transportCount_++] = &transport;
  return true;
}

void Comms::broadcast(const uint8_t* data, uint16_t len) {
  for (uint8_t i = 0; i < transportCount_; ++i) transports_[i]->send(data, len);
}

void Comms::broadcastReliable(const char* line) {
  for (uint8_t i = 0; i < transportCount_; ++i) transports_[i]->sendReliable(line);
}

void Comms::pump(uint32_t now) {
  for (uint8_t consumed = 0; consumed < kPumpMaxLines; ++consumed) {
    bool any = false;
    for (uint8_t i = 0; i < transportCount_; ++i) {
      if (pumpTransport(*transports_[i], now)) {
        any = true;
        break;  // one line per outer iteration, first-registered wins a tie
      }
    }
    if (!any) return;  // every transport dry
  }
}

bool Comms::pumpTransport(Hal::Transport& t, uint32_t now) {
  char line[kMaxLineBytes];
  uint16_t lineLen = 0;
  if (!t.readLine(line, sizeof(line), &lineLen)) return false;

  Cmd decoded;
  dispatchLine(t, line, lineLen, decoded, now);
  if (decoded.status == CmdStatus::kDecoded) pushCommand(decoded);
  return true;
}

void Comms::pushCommand(const Cmd& cmd) {
  if (cmdCount_ >= kCmdRingDepth) {
    ++commandsDroppedCount_;
    return;
  }
  const uint8_t slot = static_cast<uint8_t>((cmdHead_ + cmdCount_) % kCmdRingDepth);
  cmdRing_[slot] = cmd;
  ++cmdCount_;
}

bool Comms::takeCommand(Cmd& out) {
  if (cmdCount_ == 0) return false;
  out = cmdRing_[cmdHead_];
  cmdHead_ = static_cast<uint8_t>((cmdHead_ + 1) % kCmdRingDepth);
  --cmdCount_;
  return true;
}

void Comms::dispatchLine(Hal::Transport& t, const char* line, uint16_t lineLen, Cmd& out, uint32_t now) {
  if (isRelayControlPlaneLine(line, lineLen)) return;

  uint16_t colonPos = lineLen;  // sentinel: no ':' found
  for (uint16_t i = 0; i < lineLen; ++i) {
    if (line[i] == ':') {
      colonPos = i;
      break;
    }
  }

  const char* cmdPtr = line;
  uint16_t cmdLen = colonPos;
  const uint8_t* dataPtr = nullptr;
  uint16_t dataLen = 0;
  if (colonPos < lineLen) {
    dataPtr = reinterpret_cast<const uint8_t*>(line + colonPos + 1);
    dataLen = static_cast<uint16_t>(lineLen - colonPos - 1);
  } else {
    if (cmdLen > 0 && cmdPtr[cmdLen - 1] == '\r') --cmdLen;
  }

  // GET/SET (protocol v6 spec section 7.1) -- checked ahead of the v5
  // registry lookup, same interception point as the relay control-plane
  // carve-out above. Not v5 kVerbTable entries (see matchesVerbLiteral()'s
  // own doc comment).
  if (matchesVerbLiteral(cmdPtr, cmdLen, "GET")) {
    stageConfigGet(dataPtr, dataLen, /*hasName=*/colonPos < lineLen, t);
    return;
  }
  if (matchesVerbLiteral(cmdPtr, cmdLen, "SET")) {
    stageConfigSet(dataPtr, dataLen, t);
    return;
  }

  const msg::VerbEntry* entry = findVerb(cmdPtr, cmdLen);
  if (entry == nullptr) {
    ++malformedCount_;
    return;
  }

  if (entry->verb == msg::Verb::TLM) {
    tlmReplyTransport_ = &t;
    tlmAction_ = (colonPos >= lineLen) ? TlmAction::kFrame : classifyTlmArg(dataPtr, dataLen);
    return;
  }

  if (entry->verb == msg::Verb::SEED) {
    stageSeed(dataPtr, dataLen, t);
    return;
  }

#ifdef ROBOT_DEBUG
  if (entry->verb == msg::Verb::DBG) {
    pushDbgAction(classifyDbgArg(dataPtr, dataLen));
    return;
  }
#endif

  if (entry->binary) {
    decodeBinaryFrame(reinterpret_cast<const uint8_t*>(cmdPtr), cmdLen, dataPtr, dataLen, out);
  } else {
    dispatchCleartext(entry->verb, t, now);
  }
}

void Comms::sendPose(Hal::Transport& t) {
  char line[96];
  std::snprintf(line, sizeof(line), "POSE:%ld:%ld:%ld:%ld:%ld:%ld:%d",
                static_cast<long>(status_.otosX), static_cast<long>(status_.otosY),
                static_cast<long>(status_.otosHeading),
                static_cast<long>(status_.encX), static_cast<long>(status_.encY),
                static_cast<long>(status_.encHeading),
                status_.otosPresent ? 1 : 0);
  t.sendReliable(line);
}

void Comms::stageSeed(const uint8_t* data, uint16_t len, Hal::Transport& t) {
  // "<x>,<y>,<heading>" -- commas or spaces, all three required and signed.
  char buf[64];
  if (data == nullptr || len == 0 || len >= sizeof(buf)) {
    ++malformedCount_;
    return;
  }
  std::memcpy(buf, data, len);
  buf[len] = '\0';

  float parsed[3] = {0.0f, 0.0f, 0.0f};
  const char* cursor = buf;
  for (int i = 0; i < 3; ++i) {
    while (*cursor == ',' || *cursor == ' ') ++cursor;
    char* end = nullptr;
    const float value = std::strtof(cursor, &end);
    if (end == cursor || value != value) {  // nothing consumed, or NaN
      ++malformedCount_;
      return;
    }
    parsed[i] = value;
    cursor = end;
  }

  seed_.x = parsed[0];
  seed_.y = parsed[1];
  seed_.heading = parsed[2];
  seed_.reply = &t;
  seed_.pending = true;
}

// stageConfigGet() -- "GET" (hasName == false, dump every field) or
// "GET:<name>" (hasName == true, one field). GET carries no id (protocol
// v6 spec section 3.1's own Fields column: "[name]", nothing else), so
// there is nothing to key a malformed-line err: reply off -- an
// oversized/empty name just increments malformedCount_ and stages
// nothing, matching spec section 2's "unparseable field ... otherwise no
// reply" rule.
void Comms::stageConfigGet(const uint8_t* data, uint16_t len, bool hasName, Hal::Transport& t) {
  if (!hasName) {
    configRequest_ = ConfigRequest{};
    configRequest_.kind = ConfigRequestKind::kGetAll;
    configRequest_.reply = &t;
    return;
  }

  if (len > 0 && data[len - 1] == '\r') --len;
  if (data == nullptr || len == 0 || len >= sizeof(configRequest_.name)) {
    ++malformedCount_;
    return;
  }

  configRequest_ = ConfigRequest{};
  configRequest_.kind = ConfigRequestKind::kGetOne;
  std::memcpy(configRequest_.name, data, len);
  configRequest_.name[len] = '\0';
  configRequest_.reply = &t;
}

// stageConfigSet() -- "SET:<name>:<value>[:<id>]" (protocol v6 spec
// section 7.1). Wire-level parsing only: token arity, name length, and
// whether <value>/<id> are well-formed numbers -- NOT whether <name>
// names a real field or <value> is finite/in range, which is
// Core::Configurator::setFieldByName()'s job once RobotLoop drains this
// request. A parse failure here increments malformedCount_ and stages
// nothing (no id could be recovered from an unparseable line to key an
// err: reply off, matching stageConfigGet()'s own reasoning above).
void Comms::stageConfigSet(const uint8_t* data, uint16_t len, Hal::Transport& t) {
  if (data == nullptr || len == 0) {
    ++malformedCount_;
    return;
  }
  if (len > 0 && data[len - 1] == '\r') --len;

  uint16_t nameEnd = len;  // sentinel: no ':' found -- missing the required <value>
  for (uint16_t i = 0; i < len; ++i) {
    if (data[i] == ':') {
      nameEnd = i;
      break;
    }
  }
  if (nameEnd == len || nameEnd == 0 || nameEnd >= sizeof(configRequest_.name)) {
    ++malformedCount_;
    return;
  }

  const uint8_t* rest = data + nameEnd + 1;
  const uint16_t restLen = static_cast<uint16_t>(len - nameEnd - 1);

  uint16_t valueEnd = restLen;  // sentinel: no ':' found -- id field omitted
  for (uint16_t i = 0; i < restLen; ++i) {
    if (rest[i] == ':') {
      valueEnd = i;
      break;
    }
  }

  char valueBuf[32];
  if (valueEnd == 0 || valueEnd >= sizeof(valueBuf)) {
    ++malformedCount_;
    return;
  }
  std::memcpy(valueBuf, rest, valueEnd);
  valueBuf[valueEnd] = '\0';

  char* valueParseEnd = nullptr;
  const float value = std::strtof(valueBuf, &valueParseEnd);
  if (valueParseEnd == valueBuf || *valueParseEnd != '\0') {
    ++malformedCount_;  // empty or trailing junk -- not "non-finite", see
                         // setFieldByName() for the NaN/inf path, which
                         // strtof() itself parses successfully ("nan"/"inf")
    return;
  }

  uint32_t id = 0;  // default when the optional id field is omitted (spec section 7.1's own examples)
  if (valueEnd < restLen) {
    const uint8_t* idPtr = rest + valueEnd + 1;
    const uint16_t idLen = static_cast<uint16_t>(restLen - valueEnd - 1);
    char idBuf[16];
    if (idLen == 0 || idLen >= sizeof(idBuf)) {
      ++malformedCount_;
      return;
    }
    std::memcpy(idBuf, idPtr, idLen);
    idBuf[idLen] = '\0';
    char* idParseEnd = nullptr;
    const unsigned long parsedId = std::strtoul(idBuf, &idParseEnd, 10);
    if (idParseEnd == idBuf || *idParseEnd != '\0') {
      ++malformedCount_;
      return;
    }
    id = static_cast<uint32_t>(parsedId);
  }

  configRequest_ = ConfigRequest{};
  configRequest_.kind = ConfigRequestKind::kSet;
  std::memcpy(configRequest_.name, data, nameEnd);
  configRequest_.name[nameEnd] = '\0';
  configRequest_.value = value;
  configRequest_.id = id;
  configRequest_.reply = &t;
}

void Comms::dispatchCleartext(msg::Verb verb, Hal::Transport& t, uint32_t now) {
  switch (verb) {
    case msg::Verb::HELLO:
      t.sendReliable(banner_);
      return;
    case msg::Verb::PING: {
      char pong[32];
      std::snprintf(pong, sizeof(pong), "PONG:t=%lu", static_cast<unsigned long>(now));
      t.sendReliable(pong);
      return;
    }
    case msg::Verb::ID:
      t.sendReliable(idLine_);
      return;
    case msg::Verb::VER:
      t.sendReliable("VER:" FIRMWARE_VERSION_STR);
      return;
    case msg::Verb::STATUS:
      sendStatus(t);
      return;
    case msg::Verb::HELP:
      sendHelp(t);
      return;
    case msg::Verb::POSE:
      sendPose(t);
      return;
    default:
      ++malformedCount_;
      return;
  }
}

void Comms::decodeBinaryFrame(const uint8_t* command, size_t commandLen, const uint8_t* data, uint16_t dataLen,
                               Cmd& out) {
  uint8_t combined[kMaxCrcPayloadBytes];
  size_t combinedLen = 0;
  if (!WireRuntime::cobsDecode(data, dataLen, combined, sizeof(combined), &combinedLen, kCobsDelimiter)) {
    ++malformedCount_;
    return;
  }
  if (combinedLen < 2) {
    ++malformedCount_;
    return;
  }

  const size_t payloadLen = combinedLen - 2;
  size_t crcPos = payloadLen;
  uint16_t receivedCrc = 0;
  if (!WireRuntime::decodeCrc16(combined, combinedLen, &crcPos, &receivedCrc)) {
    ++malformedCount_;
    return;
  }
  const uint16_t expectedCrc = crcOverScope(command, commandLen, combined, payloadLen);
  if (expectedCrc != receivedCrc) {
    ++malformedCount_;
    return;
  }

  msg::CommandEnvelope decoded;
  const msg::wire::Result r =
      msg::wire::decode(decoded, combined, static_cast<uint16_t>(payloadLen));
  if (!r.ok) {
    ++malformedCount_;
    return;
  }

  out.status = CmdStatus::kDecoded;
  out.env = decoded;
}

void Comms::sendReply(const msg::ReplyEnvelope& reply) {
  const msg::Verb verb = bodyKindToVerb(reply.body_kind);
  const char* name = verbName(verb);
  if (name == nullptr) return;  // BodyKind::NONE -- nothing to send
  const size_t nameLen = std::strlen(name);

  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    return;
  }

  uint8_t combined[kMaxCrcPayloadBytes];
  std::memcpy(combined, rawBuf, n);
  size_t combinedLen = n;
  const uint16_t crc = crcOverScope(reinterpret_cast<const uint8_t*>(name), nameLen, rawBuf, n);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) {
    return;  // unreachable in practice -- combined is sized for exactly this
  }

  uint8_t cobsOut[kFramedMaxBytes];
  size_t cobsLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, cobsOut, sizeof(cobsOut), &cobsLen, kCobsDelimiter)) {
    return;  // unreachable in practice -- cobsOut is sized to the worst case
  }

  uint8_t line[kMaxLineBytes];
  if (nameLen + 1 + cobsLen > sizeof(line)) {
    return;  // unreachable in practice -- kMaxLineBytes covers the worst case
  }
  std::memcpy(line, name, nameLen);
  line[nameLen] = ':';
  std::memcpy(line + nameLen + 1, cobsOut, cobsLen);
  const uint16_t lineLen = static_cast<uint16_t>(nameLen + 1 + cobsLen);

  broadcast(line, lineLen);
}

void Comms::sendBanner() {
  broadcastReliable(banner_);
}

void Comms::sendStatus(Hal::Transport& t) {
  const char* tlmStr = "auto";
  switch (status_.tlmMode) {
    case 0: tlmStr = "off"; break;
    case 1: tlmStr = "auto"; break;
    case 2: tlmStr = "on"; break;
    default: break;  // unreachable: only 0/1/2 are ever written
  }

  char line[128];
  std::snprintf(line, sizeof(line),
                "STATUS:ready=%d:active=%d:connL=%d:connR=%d:otos=%d"
                ":wedge=%d:flags=0x%lx:tlm=%s",
                status_.ready ? 1 : 0, status_.active ? 1 : 0,
                status_.wheelLeftConnected ? 1 : 0,
                status_.wheelRightConnected ? 1 : 0,
                status_.otosPresent ? 1 : 0, status_.wedged ? 1 : 0,
                static_cast<unsigned long>(status_.flags), tlmStr);
  t.sendReliable(line);
}

void Comms::sendHelp(Hal::Transport& t) {
  char line[192];
  std::size_t n = 0;
  n += static_cast<std::size_t>(std::snprintf(line, sizeof(line), "HELP:"));
  for (std::size_t i = 0; i < msg::kVerbCount && n + 1 < sizeof(line); ++i) {
    const msg::VerbEntry& e = msg::kVerbTable[i];
    if (e.binary && e.verb != msg::Verb::TLM) continue;
    const char* token = (e.verb == msg::Verb::TLM) ? "TLM[:NOW|ON|AUTO|OFF]" : e.name;
    const int written = std::snprintf(line + n, sizeof(line) - n, "%s%s",
                                      n > 5 ? " " : "", token);
    if (written <= 0) break;
    n += static_cast<std::size_t>(written);
  }
  t.sendReliable(line);
}

void Comms::sendTlmReply(TlmAction action) {
  if (tlmReplyTransport_ == nullptr) return;
  switch (action) {
    case TlmAction::kSetOff:
    case TlmAction::kSetAuto:
    case TlmAction::kSetOn:
      sendStatus(*tlmReplyTransport_);
      break;
    case TlmAction::kUnrecognized:
      sendHelp(*tlmReplyTransport_);
      break;
    case TlmAction::kNone:
    case TlmAction::kFrame:
    default:
      break;  // kFrame's reply is the forced telemetry frame emit() sends
  }
}

void Comms::sendReady() {
  broadcastReliable("READY");
}

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)
void Comms::sendDebug(const char* line) {
  char buf[210];
  std::snprintf(buf, sizeof(buf), "DBG:%s", line);
  broadcastReliable(buf);
}
#endif  // ROBOT_DEBUG || HOST_BUILD

void Comms::updateStatus(const Types::RobotState& state, const Telemetry& tlm) {
  Status status;
  status.ready = state.health.ready;
  status.active = state.command.moveActive;
  status.wheelLeftConnected = state.wheelLeft.connected;
  status.wheelRightConnected = state.wheelRight.connected;
  status.otosPresent = state.otos.present;
  status.wedged = state.health.wedgeLatch;
  status.flags = tlm.flags();
  status.tlmMode = static_cast<uint8_t>(tlm.mode());
  status.otosX = static_cast<int32_t>(state.otos.x);
  status.otosY = static_cast<int32_t>(state.otos.y);
  status.otosHeading = static_cast<int32_t>(state.otos.heading * 1000.0f);
  status.encX = static_cast<int32_t>(state.pose.x);
  status.encY = static_cast<int32_t>(state.pose.y);
  status.encHeading = static_cast<int32_t>(state.pose.heading * 1000.0f);
  status_ = status;
}

}  // namespace Core

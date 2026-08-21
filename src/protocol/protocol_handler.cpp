// protocol_handler.cpp — Protocol::ProtocolHandler implementation. See
// protocol_handler.h for the class contract and the numbered list of
// wire-spec ambiguities this file resolves.
#include "protocol_handler.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace Protocol {

namespace {

// ---- strict field decoders --------------------------------------------
// Every wire value is a base-10 ASCII integer, optionally signed, except
// config values (spec §2.2). "Strict" means the WHOLE field must be
// consumed by strtol/strtoul/strtof — a trailing '.', letter, or stray
// space makes the field unparseable, matching spec §2.2's "no
// exponents, no NaN, no inf" and docs/design/protocol.md §2.2's "Wrong
// arity is a rejection, not a best-effort parse" extended to field
// content.

// strtol/strtoul/strtof all skip LEADING whitespace before the first
// digit (a C-standard behavior, not a project choice), which would
// silently accept a field like " 100" or "\t100" as a valid "100" --
// contradicting this file's own "stray space makes the field
// unparseable" contract above (that sentence is only true of a
// TRAILING space; a leading one sails through). An adversarial-input
// sweep found this by construction (a stray '\r' that has drifted
// mid-field rather than sitting immediately before the terminator is
// exactly this case). Reject it explicitly, once, ahead of every
// numeric decode below.
bool isWireSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' ||
         c == '\r';
}

bool parseInt32(const char* field, int32_t& out) {
  if (field == nullptr || field[0] == '\0' || isWireSpace(field[0])) {
    return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  long value = std::strtol(field, &endPtr, 10);
  if (endPtr == field || *endPtr != '\0') return false;
  if (errno == ERANGE || value < INT32_MIN || value > INT32_MAX) return false;
  out = static_cast<int32_t>(value);
  return true;
}

bool parseUint32(const char* field, uint32_t& out) {
  // strtoul silently accepts a leading '-' and wraps around, which
  // would turn "-5" into a huge unsigned value instead of failing --
  // reject it up front.
  if (field == nullptr || field[0] == '\0' || field[0] == '-' ||
      isWireSpace(field[0])) {
    return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  unsigned long value = std::strtoul(field, &endPtr, 10);
  if (endPtr == field || *endPtr != '\0') return false;
  if (errno == ERANGE || value > UINT32_MAX) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

// Config values are the one place floats appear on the wire (spec
// §7.2). "No exponents, no NaN, no inf" is stated for the integer
// fields (§2.2) but this handler applies the same posture here too --
// nothing in this project ever needs a robot to accept "1e10" or "nan"
// as a gain.
bool parseFloatField(const char* field, float& out) {
  if (field == nullptr || field[0] == '\0' || isWireSpace(field[0])) {
    return false;
  }
  for (const char* p = field; *p != '\0'; ++p) {
    // 'e'/'E' bars decimal-exponent notation ("1e10", spec §2.2). 'x'/'X'
    // bars C99 HEX FLOAT notation ("0x1p3", "0X1.8P3") -- strtof accepts
    // this syntax unconditionally (it is not gated behind the 'e'/'E'
    // check at all, since a hex float's exponent letter is 'p', not
    // 'e'), so an adversarial-input sweep found that a wire value like
    // `SET:foo:0x1.8p3` silently decoded to 12.0 instead of being
    // rejected -- exactly the "no exponents" rule this function exists
    // to enforce, bypassed by a spelling the spec's authors never had in
    // mind. A MicroPython or JavaScript port would not reproduce this:
    // neither `float()` nor `Number()`/`parseFloat()` accepts hex-float
    // syntax, so this was a C++-only divergence from every other
    // implementation of this same fixture.
    if (*p == 'e' || *p == 'E' || *p == 'x' || *p == 'X') return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  float value = std::strtof(field, &endPtr);
  if (endPtr == field || *endPtr != '\0') return false;
  if (std::isnan(value) || std::isinf(value)) return false;
  out = value;
  return true;
}

bool parseTlmMode(const char* field, TlmMode& mode) {
  struct ModeEntry {
    const char* name;
    TlmMode mode;
  };
  static constexpr ModeEntry kModes[] = {
      {"OFF", TlmMode::kOff},   {"POSE", TlmMode::kPose},
      {"FULL", TlmMode::kFull}, {"NOW", TlmMode::kNow},
      {"AUTO", TlmMode::kAuto}, {"BUFFER", TlmMode::kBuffer},
  };
  for (const auto& entry : kModes) {
    if (std::strcmp(field, entry.name) == 0) {
      mode = entry.mode;
      return true;
    }
  }
  return false;
}

// Resolves an OPTIONAL trailing id field (SET, WHEELS) per the
// reconciliation documented in protocol_handler.h's ambiguity note #1:
// absent -> id 0, acked; explicit "0" -> id 0, NOT acked (spec §8.2);
// anything else -> that value, acked. Returns false only if `present`
// but the field fails to parse as a base-10 unsigned integer -- the
// caller should then treat the whole line as malformed with no reply,
// since the id cannot be trusted.
bool resolveOptionalId(const char* field, bool present, uint32_t& id,
                        bool& sendAck) {
  if (!present) {
    id = 0;
    sendAck = true;
    return true;
  }
  uint32_t parsed = 0;
  if (!parseUint32(field, parsed)) return false;
  id = parsed;
  sendAck = (parsed != 0);
  return true;
}

// formatConfigValue() -- spec §7.2's formatFixed(), reproduced here
// (not included from src/archive/protocol-v6/wire_v6_format.{h,cpp},
// which is reference-only): six fractional digits, always present, no
// exponent, using integer arithmetic because newlib-nano's printf has
// no %f. formatConfigValue(0.02f) -> "0.020000",
// formatConfigValue(-51.5f) -> "-51.500000" (spec's own examples).
//
// `value` is NOT wire-parsed here -- it is whatever the ADAPTER's own
// onGet() handed back (parseFloatField already rejects NaN/Inf on the
// way IN, spec §2.2/§7.2's "no NaN, no inf"), so this function cannot
// assume it is finite. +-Inf is already handled correctly below: it
// compares greater than kMaxScaled and gets clamped before the cast.
// NaN does not: EVERY comparison against a NaN is false, so
// `scaled > kMaxScaled` is false too and a NaN sails past the clamp
// intact into `static_cast<uint32_t>(scaled)` -- converting a NaN to an
// unsigned integer is undefined behavior (caught live by
// -fsanitize=undefined's float-cast-overflow check during the
// adversarial-input sweep). There is no wire spelling for NaN to
// preserve, so fail safe to 0.0 rather than invent one.
void formatConfigValue(float value, char* out, size_t cap) {
  if (std::isnan(value)) value = 0.0f;
  constexpr uint32_t kDivisor = 1000000u;  // 10^6 -- spec's fixed 6 digits
  const bool negative = value < 0.0f;
  const float magnitude = negative ? -value : value;
  constexpr float kMaxScaled = 4294967040.0f;  // largest float < UINT32_MAX
  float scaled = magnitude * static_cast<float>(kDivisor) + 0.5f;
  if (scaled > kMaxScaled) scaled = kMaxScaled;
  const uint32_t scaledInt = static_cast<uint32_t>(scaled);
  const uint32_t wholePart = scaledInt / kDivisor;
  const uint32_t fracPart = scaledInt % kDivisor;
  std::snprintf(out, cap, "%s%lu.%06lu", negative ? "-" : "",
                static_cast<unsigned long>(wholePart),
                static_cast<unsigned long>(fracPart));
}

}  // namespace

const ProtocolHandler::VerbEntry ProtocolHandler::kCommandTable[12] = {
    {"HELLO", &ProtocolHandler::handleHello},
    {"PING", &ProtocolHandler::handlePing},
    {"ID", &ProtocolHandler::handleId},
    {"VER", &ProtocolHandler::handleVer},
    {"STATUS", &ProtocolHandler::handleStatus},
    {"HELP", &ProtocolHandler::handleHelp},
    {"GET", &ProtocolHandler::handleGet},
    {"SET", &ProtocolHandler::handleSet},
    {"TLM", &ProtocolHandler::handleTlm},
    {"WHEELS", &ProtocolHandler::handleWheels},
    {"STOP", &ProtocolHandler::handleStop},
    {"ESTOP", &ProtocolHandler::handleEstop},
};

ProtocolHandler::ProtocolHandler(Adapter& adapter, Sink& sink)
    : adapter_(adapter), sink_(sink) {}

// ---- feed() / line reassembly ------------------------------------------

void ProtocolHandler::feed(const char* data, size_t length) {
  for (size_t i = 0; i < length; ++i) appendByte(data[i]);
}

void ProtocolHandler::appendByte(char c) {
  if (c == '\n') {
    onLineComplete();
    return;
  }
  if (overflowing_) return;  // discard content until the next '\n'
  if (lineLen_ >= kMaxLineBytes - 1) {
    // Storing this byte would make the line's content alone reach
    // kMaxLineBytes - 1, i.e. the line (content + '\n') would exceed
    // the wire's 240-byte cap. Discard to the next '\n' rather than
    // truncate: a truncated prefix that still parses as a legal verb
    // with legal arity would be a command the host never sent
    // (docs/design/protocol.md §2.1).
    overflowing_ = true;
    lineLen_ = 0;
    return;
  }
  lineBuf_[lineLen_++] = c;
}

void ProtocolHandler::onLineComplete() {
  if (overflowing_) {
    overflowing_ = false;
    lineLen_ = 0;
    ++malformedCount_;
    return;
  }
  // A lone '\r' immediately before '\n' is a terminal artifact and is
  // stripped; '\r' appears nowhere else on the wire (spec §2).
  if (lineLen_ > 0 && lineBuf_[lineLen_ - 1] == '\r') --lineLen_;
  lineBuf_[lineLen_] = '\0';

  char* verb = lineBuf_;
  char* colon = std::strchr(lineBuf_, ':');
  char* rest = nullptr;
  if (colon != nullptr) {
    *colon = '\0';
    rest = colon + 1;
  }
  dispatch(verb, rest);
  lineLen_ = 0;
}

// ---- dispatch ------------------------------------------------------------

void ProtocolHandler::dispatch(char* verb, char* rest) {
  // Case is direction (spec §2.1): commands are UPPERCASE, replies are
  // lowercase, and verb lookup is case-sensitive. A verb starting with
  // a lowercase letter can never be a command this table knows about --
  // it is another robot's reply, overheard on a shared channel, and it
  // is dropped SILENTLY, not counted malformed. This is the structural
  // fix for the DBG:-flood incident (hardware-bench-testing.md): a
  // reply can never parse as a command under v6.
  if (verb[0] >= 'a' && verb[0] <= 'z') return;

  for (const auto& entry : kCommandTable) {
    if (std::strcmp(verb, entry.name) == 0) {
      (this->*entry.handler)(rest);
      return;
    }
  }
  // Unknown verb: no arity is knowable, so no id can be trusted even if
  // the line happens to contain colon-separated fields -- no reply,
  // just the malformed count (spec §2).
  ++malformedCount_;
}

size_t ProtocolHandler::splitFields(char* rest, char** fields,
                                     size_t maxFields) {
  if (rest == nullptr) return 0;  // verb had no ':' at all -- zero fields
  size_t count = 0;
  char* p = rest;
  while (true) {
    if (count < maxFields) fields[count] = p;
    ++count;
    char* colon = std::strchr(p, ':');
    if (colon == nullptr) break;
    *colon = '\0';
    p = colon + 1;
  }
  return count;
}

void ProtocolHandler::replyOk(uint32_t id) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "ok:%lu\n", static_cast<unsigned long>(id));
  writeLine(buf);
}

void ProtocolHandler::replyErr(uint32_t id, uint8_t code) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "err:%lu:%u\n",
                static_cast<unsigned long>(id), static_cast<unsigned>(code));
  writeLine(buf);
}

void ProtocolHandler::writeLine(const char* text) {
  sink_.write(text, std::strlen(text));
}

uint8_t ProtocolHandler::resultCode(Result result) {
  switch (result) {
    case Result::kOk: return 0;  // never used as an error code
    case Result::kUnknown: return 1;
    case Result::kBadArg: return 2;
    case Result::kRange: return 3;
    case Result::kFull: return 4;
    case Result::kUnimplemented: return 6;
    case Result::kNotReady: return 8;
    case Result::kBusy: return 10;
    case Result::kDuplicateId: return 11;
  }
  return 1;  // unreachable with every enumerator handled above; kept so
             // a FUTURE enumerator trips -Wswitch instead of silently
             // falling through a default case
}

// ---- session verbs --------------------------------------------------------

void ProtocolHandler::handleHello(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  sendBanner();  // spec §4: HELLO's reply is byte-identical to the
                 // unsolicited boot banner
}

void ProtocolHandler::handlePing(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "pong:%lu\n",
                static_cast<unsigned long>(adapter_.now()));
  writeLine(buf);
}

void ProtocolHandler::handleVer(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  Identity identity;
  adapter_.identity(identity);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "ver:%s\n", identity.version);
  writeLine(buf);
}

void ProtocolHandler::handleId(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "id:%s:%s:%s\n", identity.drivetrain,
                identity.profile, identity.version);
  writeLine(buf);
}

void ProtocolHandler::handleStatus(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  StatusFields status;
  adapter_.status(status);
  char buf[160];
  std::snprintf(buf, sizeof(buf),
                "status:ready=%d:active=%d:connL=%d:connR=%d:otos=%d:"
                "wedge=%d:flags=%x:tlm=%s\n",
                status.ready ? 1 : 0, status.active ? 1 : 0,
                status.connLeft ? 1 : 0, status.connRight ? 1 : 0,
                status.otos ? 1 : 0, status.wedge ? 1 : 0,
                static_cast<unsigned int>(status.flags), status.tlm);
  writeLine(buf);
}

void ProtocolHandler::handleHelp(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  // "Generated by walking the verb table at runtime, so it cannot drift
  // from the dispatcher" (spec §4) -- kCommandTable is the SAME table
  // dispatch() looks verbs up in.
  char buf[160];
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < sizeof(buf) - 1) buf[pos++] = *text++;
  };
  append("help:");
  for (size_t i = 0; i < sizeof(kCommandTable) / sizeof(kCommandTable[0]);
       ++i) {
    if (i > 0) append(" ");
    append(kCommandTable[i].name);
  }
  append("\n");
  buf[pos] = '\0';
  writeLine(buf);
}

// ---- configuration: pure delegation, no storage here (spec §6) ----------

void ProtocolHandler::handleGet(char* rest) {
  char* fields[2] = {};
  size_t count = splitFields(rest, fields, 2);
  if (count > 1) { ++malformedCount_; return; }

  char buf[kMaxGetReplyBytes];
  char formatted[32];
  if (count == 0) {
    // Bare GET: dump every field the adapter declares, one line each
    // (spec §7.1: "one line per field, 80 lines" for the real 80-row
    // table -- this library carries none, so the line count here is
    // whatever THIS adapter's fieldCount() says).
    size_t total = adapter_.fieldCount();
    for (size_t i = 0; i < total; ++i) {
      const char* name = adapter_.fieldName(i);
      float value = 0.0f;
      if (!adapter_.onGet(name, value)) continue;
      formatConfigValue(value, formatted, sizeof(formatted));
      std::snprintf(buf, sizeof(buf), "get:%s:%s\n", name, formatted);
      writeLine(buf);
    }
    return;
  }

  const char* name = fields[0];
  float value = 0.0f;
  // Unknown name: GET never carries an id (`GET | [name]`, spec §3.1),
  // so there is no wire channel to reject it on -- silent, per this
  // file's header-comment ambiguity note #2.
  if (!adapter_.onGet(name, value)) return;
  formatConfigValue(value, formatted, sizeof(formatted));
  std::snprintf(buf, sizeof(buf), "get:%s:%s\n", name, formatted);
  writeLine(buf);
}

void ProtocolHandler::handleSet(char* rest) {
  char* fields[3] = {};
  size_t count = splitFields(rest, fields, 3);
  if (count != 2 && count != 3) { ++malformedCount_; return; }

  const char* name = fields[0];
  float value = 0.0f;
  if (!parseFloatField(fields[1], value)) {
    // The VALUE field itself is malformed -- this is a handler-level
    // decode failure (spec §7.2: SET's value is decoded by the
    // handler), never reaching onSet(). Still apply the same
    // present/absent id resolution as the success path, per this
    // file's ambiguity note #1, so a typo'd value on an otherwise
    // well-formed SET still gets an err reply when one is owed.
    uint32_t id = 0;
    bool sendAck = false;
    ++malformedCount_;
    if (resolveOptionalId(count == 3 ? fields[2] : nullptr, count == 3, id,
                           sendAck) &&
        sendAck) {
      replyErr(id, 2);  // ERR_BADARG
    }
    return;
  }

  uint32_t id = 0;
  bool sendAck = false;
  if (!resolveOptionalId(count == 3 ? fields[2] : nullptr, count == 3, id,
                          sendAck)) {
    ++malformedCount_;  // id field present but unparseable
    return;
  }

  Result result = adapter_.onSet(name, value, id);
  if (!sendAck) return;
  if (result == Result::kOk) replyOk(id);
  else replyErr(id, resultCode(result));
}

// ---- telemetry -------------------------------------------------------------

void ProtocolHandler::handleTlm(char* rest) {
  char* fields[2] = {};
  size_t count = splitFields(rest, fields, 2);
  if (count != 1) { ++malformedCount_; return; }
  TlmMode mode;
  if (!parseTlmMode(fields[0], mode)) { ++malformedCount_; return; }
  // TLM carries no id (spec §3.1) so there is no wire channel to ack or
  // reject it on -- the Result is the adapter's own business (e.g.
  // logging) and never surfaces on the wire.
  (void)adapter_.onTlm(mode);
}

// ---- motion ----------------------------------------------------------------

void ProtocolHandler::handleWheels(char* rest) {
  char* fields[4] = {};
  size_t count = splitFields(rest, fields, 4);
  if (count != 3 && count != 4) { ++malformedCount_; return; }

  int32_t left = 0, right = 0;
  uint32_t duration = 0;
  if (!parseInt32(fields[0], left) || !parseInt32(fields[1], right) ||
      !parseUint32(fields[2], duration)) {
    uint32_t id = 0;
    bool sendAck = false;
    ++malformedCount_;
    if (resolveOptionalId(count == 4 ? fields[3] : nullptr, count == 4, id,
                           sendAck) &&
        sendAck) {
      replyErr(id, 2);  // ERR_BADARG
    }
    return;
  }

  uint32_t id = 0;
  bool sendAck = false;
  if (!resolveOptionalId(count == 4 ? fields[3] : nullptr, count == 4, id,
                          sendAck)) {
    ++malformedCount_;
    return;
  }

  // duration's documented "ceiling 5000" (spec §5.2) is NOT enforced
  // here -- see protocol_handler.h's ambiguity note #3. It reaches the
  // adapter untouched.
  Result result = adapter_.onWheels(static_cast<float>(left),
                                     static_cast<float>(right), duration, id);
  if (!sendAck) return;
  if (result == Result::kOk) replyOk(id);
  else replyErr(id, resultCode(result));
}

void ProtocolHandler::handleStop(char* rest) {
  char* fields[2] = {};
  size_t count = splitFields(rest, fields, 2);
  if (count != 1) { ++malformedCount_; return; }
  uint32_t id = 0;
  if (!parseUint32(fields[0], id)) { ++malformedCount_; return; }
  // STOP's id is REQUIRED (`STOP:<id>`, spec §3.1 -- no brackets), so
  // unlike SET/WHEELS there is no "explicit 0 means no ack" carve-out:
  // it is always acked.
  Result result = adapter_.onStop(id);
  if (result == Result::kOk) replyOk(id);
  else replyErr(id, resultCode(result));
}

void ProtocolHandler::handleEstop(char* rest) {
  char* fields[1] = {};
  if (splitFields(rest, fields, 1) != 0) { ++malformedCount_; return; }
  adapter_.onEstop();
  // No reply, ever: spec §8.2 -- ESTOP never carries an id and is never
  // acked, so it can never queue behind anything, including an ack.
}

// ---- unsolicited emissions -------------------------------------------------

void ProtocolHandler::sendBanner() {
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "device:NEZHA2:robot:%s:%s\n",
                identity.name, identity.serial);
  writeLine(buf);
}

void ProtocolHandler::sendReady() { writeLine("ready\n"); }

// ---- telemetry emission -----------------------------------------------------

bool ProtocolHandler::headerChanged(const Snapshot& snapshot) const {
  if (!everEmittedHeader_) return true;
  if (snapshot.count != headerCount_) return true;
  size_t limit =
      snapshot.count < kMaxHeaderColumns ? snapshot.count : kMaxHeaderColumns;
  for (size_t i = 0; i < limit; ++i) {
    if (headerHex_[i] != snapshot.columns[i].hex) return true;
    if (std::strcmp(headerNames_[i], snapshot.columns[i].name) != 0) {
      return true;
    }
  }
  return false;
}

void ProtocolHandler::rememberHeader(const Snapshot& snapshot) {
  headerCount_ = snapshot.count;
  size_t limit =
      snapshot.count < kMaxHeaderColumns ? snapshot.count : kMaxHeaderColumns;
  for (size_t i = 0; i < limit; ++i) {
    std::snprintf(headerNames_[i], kMaxHeaderNameBytes, "%s",
                  snapshot.columns[i].name);
    headerHex_[i] = snapshot.columns[i].hex;
  }
  everEmittedHeader_ = true;
}

void ProtocolHandler::emitHeader(const Snapshot& snapshot) {
  char buf[kMaxTelemetryLineBytes];
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < sizeof(buf) - 1) buf[pos++] = *text++;
  };
  append("thdr");
  for (size_t i = 0; i < snapshot.count; ++i) {
    append(":");
    append(snapshot.columns[i].name);
  }
  append("\n");
  buf[pos] = '\0';
  writeLine(buf);
}

void ProtocolHandler::emitFrame(const Snapshot& snapshot) {
  char buf[kMaxTelemetryLineBytes];
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < sizeof(buf) - 1) buf[pos++] = *text++;
  };
  append("t");
  char valueText[16];
  for (size_t i = 0; i < snapshot.count; ++i) {
    append(":");
    if (snapshot.columns[i].hex) {
      // flags: lowercase hex, no "0x" prefix (spec §6.5).
      std::snprintf(valueText, sizeof(valueText), "%x",
                    static_cast<unsigned int>(
                        static_cast<uint32_t>(snapshot.columns[i].value)));
    } else {
      std::snprintf(valueText, sizeof(valueText), "%ld",
                    static_cast<long>(snapshot.columns[i].value));
    }
    append(valueText);
  }
  append("\n");
  buf[pos] = '\0';
  writeLine(buf);
}

void ProtocolHandler::emitTelemetry(const Snapshot& snapshot) {
  if (headerChanged(snapshot)) {
    emitHeader(snapshot);
    rememberHeader(snapshot);
  }
  emitFrame(snapshot);
}

}  // namespace Protocol

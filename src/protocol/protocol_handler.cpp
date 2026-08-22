// protocol_handler.cpp — Protocol::ProtocolHandler implementation. See
// protocol_handler.h for the class contract, the reliability-layer state
// machine summary, and the numbered list of wire-spec ambiguities this
// file resolves.
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
// consumed by strtol/strtoul/strtof -- a trailing letter or stray
// interior byte makes the field unparseable, matching spec §2.2's "no
// exponents, no NaN, no inf" and docs/design/protocol.md §2.2's "Wrong
// arity is a rejection, not a best-effort parse" extended to field
// content.

// strtol/strtoul/strtof all skip LEADING whitespace before the first
// digit (a C-standard behavior, not a project choice). Under the space
// grammar, a literal leading ' ' (0x20) can never reach a field decoder
// at all: tokenizeLine() below collapses every run of ' ' into one
// separator and trims leading/trailing line whitespace before a token
// pointer is ever handed to a field decoder. That part of the guard is
// dead code, kept only as cheap, harmless defense in depth.
//
// The guard is NOT fully dead, though -- it stays genuinely load-bearing
// for the OTHER C whitespace bytes. Spec §2's field grammar is
// `field ::= any bytes except ' ' and '\n'`, which means '\t', '\v',
// '\f' and '\r' are all LEGAL, ordinary field bytes -- nothing about
// tokenizing on ' ' stops a field from starting with one of them (e.g.
// "SET foo.bar<TAB>1.0 #9" tokenizes `<TAB>1.0` as the value field, tab
// included). strtof/strtol would silently skip that leading tab per the
// C standard and parse "1.0" anyway.
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

// The id's own numeric grammar (spec §2.2: `id ::= '#' [0-9]+`) is
// STRICTER than parseUint32() above: no sign at all, not even a leading
// '+', which C's strtoul() would otherwise accept as valid syntax and
// parseUint32() does not itself reject (it only rejects '-', spec
// §2.2's general integer-field rule being "optionally signed"). A
// pre-pass that requires every byte to be an ASCII digit before
// strtoul() ever runs means "#+5" is correctly NOT a well-formed id --
// it falls through to "no valid id at all" (docs/design/protocol.md
// §8.4), not "id 5".
bool parseIdDigits(const char* text, uint32_t& out) {
  if (text == nullptr || text[0] == '\0') return false;
  for (const char* p = text; *p != '\0'; ++p) {
    if (*p < '0' || *p > '9') return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  unsigned long value = std::strtoul(text, &endPtr, 10);
  if (endPtr == text || *endPtr != '\0') return false;
  if (errno == ERANGE || value > UINT32_MAX) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

// Resolves `token` (the line's raw last token, or nullptr if the line
// was just the verb) as a mandatory sequence id (docs/design/
// protocol.md §2.2/§8): must be present and match `#[0-9]+` exactly.
// There is no "#0 is special" branch here at all -- deleting that
// special case (§2.2) means every well-formed id, including 0, is
// handled identically by dispatch()'s own three-way sequence compare.
bool parseMandatoryId(const char* token, uint32_t& id) {
  if (token == nullptr || token[0] != '#') return false;
  return parseIdDigits(token + 1, id);
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
    // this syntax unconditionally, so an adversarial-input sweep found
    // that a wire value like `SET foo.bar 0x1.8p3 #9` silently decoded
    // to 12.0 instead of being rejected. A MicroPython or JavaScript
    // port would not reproduce this: neither `float()` nor
    // `Number()`/`parseFloat()` accepts hex-float syntax.
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

// The raw LAST token of `line` (verb included), independent of any
// fixed-size fields[] array's own storage cap -- see protocol_handler.h's
// VerbHandler comment for why this matters on an adversarial line with
// more junk fields than fields[] has room to store pointers for.
//
// MUST be called BEFORE tokenizeLine() mutates any of `line`'s
// separator spaces to '\0': it walks real ' ' bytes backward from the
// end of the string. Returns nullptr if `line` has no token besides the
// verb itself (nothing after it to resolve an id from).
const char* findLastFieldToken(const char* line) {
  const char* end = line + std::strlen(line);
  const char* p = end;
  while (p > line && *(p - 1) == ' ') --p;  // skip trailing spaces
  while (p > line && *(p - 1) != ' ') --p;  // scan back through the token
  return p == line ? nullptr : p;
}

// Copies `text` into `out` (a buffer of `outCap` bytes), STRIPPING every
// '\n'/'\r' byte rather than rejecting the call outright -- see
// sendDebug()'s and handleRun()'s own comments for why: both format
// caller- or adapter-supplied free text directly onto a single wire
// line, and an embedded terminator byte reaching the sink could forge a
// second line the far end would parse as a separate, unintended
// command/reply. `text == nullptr` is treated exactly like `text == ""`
// (both produce a zero-length result) so every caller of this function
// gets one behavior for "nothing to say," not two. Truncates, never
// overflows, once `out` is full -- always NUL-terminates within
// `outCap`. Returns the number of bytes written (excluding the
// terminator), so a caller can tell "produced nothing" apart from
// "produced text" without a second strlen().
size_t sanitizeLineText(const char* text, char* out, size_t outCap) {
  if (text == nullptr) text = "";
  size_t len = 0;
  for (const char* p = text; *p != '\0' && len + 1 < outCap; ++p) {
    if (*p == '\n' || *p == '\r') continue;  // stripped -- never reaches out
    out[len++] = *p;
  }
  out[len] = '\0';
  return len;
}

}  // namespace

const ProtocolHandler::VerbEntry ProtocolHandler::kCommandTable[13] = {
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
    {"RUN", &ProtocolHandler::handleRun},
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

  // A blank or all-whitespace line is ignored SILENTLY (spec §2) -- a
  // terminal artifact, not an error; it does NOT count malformed. Cheap
  // pre-check before the real tokenizer runs.
  bool anyNonSpace = false;
  for (size_t i = 0; i < lineLen_; ++i) {
    if (lineBuf_[i] != ' ') {
      anyNonSpace = true;
      break;
    }
  }
  if (!anyNonSpace) {
    lineLen_ = 0;
    return;
  }

  // The mandatory trailing id (docs/design/protocol.md §8) must be
  // located BEFORE tokenizeLine() below mutates any separator space to
  // '\0' -- see findLastFieldToken()'s own comment.
  const char* lastFieldToken = findLastFieldToken(lineBuf_);

  char* tokens[kMaxFieldTokens];
  size_t count = tokenizeLine(lineBuf_, tokens, kMaxFieldTokens);
  // count >= 1 here: anyNonSpace being true guarantees at least the
  // verb token was found.
  char* verb = tokens[0];
  dispatch(verb, tokens + 1, count - 1, lastFieldToken);
  lineLen_ = 0;
}

// ---- tokenizing (spec §2, §11.1) ----------------------------------------

size_t ProtocolHandler::tokenizeLine(char* line, char** tokens,
                                      size_t maxTokens) {
  size_t count = 0;
  char* p = line;
  while (true) {
    while (*p == ' ') ++p;  // skip a run of separator spaces (sp ::= ' '+)
    if (*p == '\0') break;  // end of line -- no more tokens
    if (count < maxTokens) tokens[count] = p;
    ++count;
    while (*p != '\0' && *p != ' ') ++p;  // scan to next separator or end
    if (*p == '\0') break;
    *p = '\0';  // terminate this token
    ++p;        // step past the separator byte just nulled
  }
  return count;
}

// ---- dispatch / the reliability layer -----------------------------------
// docs/design/protocol.md §8 in full; this is the state machine summary
// from protocol_handler.h's own file header, implemented.

void ProtocolHandler::dispatch(char* verb, char** fields, size_t fieldCount,
                                const char* lastFieldToken) {
  // Case is direction (spec §2.1): commands are UPPERCASE, replies are
  // lowercase, and verb lookup is case-sensitive. A verb starting with
  // a lowercase letter can never be a command this table knows about --
  // it is another robot's reply, overheard on a shared channel, and it
  // is dropped SILENTLY, not counted malformed.
  if (verb[0] >= 'a' && verb[0] <= 'z') return;

  // ---- the two verbs OUTSIDE the sequence (docs/design/protocol.md
  // §8.3) -- checked by verb identity, before any id is even looked at.
  if (std::strcmp(verb, "ESTOP") == 0) {
    // ESTOP is maximally forgiving: it executes and replies regardless
    // of fieldCount or content, so its own handler does not need
    // fieldCount/lastFieldToken at all. The id parameter is unused by
    // handleEstop() -- passed as 0 only to satisfy the shared
    // VerbHandler signature every kCommandTable entry shares.
    handleEstop(fields, fieldCount, 0);
    return;
  }
  if (std::strcmp(verb, "HELLO") == 0) {
    // HELLO's own arity is unchanged from before the reliability layer
    // (zero fields, id or otherwise) -- a HELLO with a trailing field
    // is wrong arity, same as any other extra field, and (since HELLO
    // is outside the sequence) has no ack to anchor an err against, so
    // it is silently malformed like any other unsequenced-verb failure
    // (docs/design/protocol.md §9.8).
    if (fieldCount != 0) {
      ++malformedCount_;
      return;
    }
    handleHello(fields, fieldCount, 0);
    return;
  }

  // ---- everything else is on the sequenced plane (docs/design/
  // protocol.md §8.1/§8.4): a mandatory, well-formed #<id> is REQUIRED
  // as the line's last token, independent of whether the verb itself is
  // even recognized.
  uint32_t id = 0;
  if (!parseMandatoryId(lastFieldToken, id)) {
    // No trailing field at all, or one that isn't a well-formed
    // '#'[0-9]+ -- the line cannot be sequence-classified. Nothing to
    // compare against expectedNext_, so there is no reply of any kind
    // (§8.4 items 1-2).
    ++malformedCount_;
    return;
  }

  // The id itself is always fields[fieldCount - 1] once well-formed
  // (findLastFieldToken() found it), so the verb's own DATA fields are
  // everything before it.
  const size_t dataFieldCount = fieldCount - 1;

  if (id < expectedNext_) {
    // A stale retransmit -- the host never saw our ack for something we
    // already accepted. Do NOT re-execute (a resent WHEELS must not
    // drive twice); just re-state what we already have.
    replyAck(expectedNext_ - 1);
    return;
  }
  if (id > expectedNext_) {
    // A gap: something between expectedNext_ and id never arrived (or
    // arrived out of order). Discard -- do NOT execute, and do not even
    // look up the verb -- and tell the host exactly what we need next.
    gapOutstanding_ = true;
    replyNack(expectedNext_);
    return;
  }

  // id == expectedNext_: in order. The sequence advances and the ack is
  // sent UNCONDITIONALLY, before the verb is even looked up -- "did the
  // bytes arrive, in order" is answered here regardless of what they
  // turn out to contain (docs/design/protocol.md §8.2).
  expectedNext_ = id + 1;
  gapOutstanding_ = false;
  replyAck(id);

  for (const auto& entry : kCommandTable) {
    if (std::strcmp(verb, entry.name) == 0) {
      (this->*entry.handler)(fields, dataFieldCount, id);
      return;
    }
  }
  // Unknown verb, but in order: the ack above already covered "arrived
  // in sequence"; this is the "content rejected" half (§8.2/§8.4 item 1).
  ++malformedCount_;
  replyErr(id, resultCode(Result::kUnknown));
}

void ProtocolHandler::replyAck(uint32_t ackedId) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "ack %lu %lu\n",
                static_cast<unsigned long>(ackedId),
                static_cast<unsigned long>(lastDone_));
  writeLine(buf);
}

void ProtocolHandler::replyNack(uint32_t nextId) {
  char buf[40];
  std::snprintf(buf, sizeof(buf), "nack %lu %lu\n",
                static_cast<unsigned long>(nextId),
                static_cast<unsigned long>(lastDone_));
  writeLine(buf);
}

void ProtocolHandler::replyErr(uint32_t id, uint8_t code) {
  // Field order: code THEN #id -- the id is always the LAST token of
  // ANY line under this grammar, replies included (docs/design/
  // protocol.md §8.6). This used to be `err #<id> <code>`, an
  // undocumented exception to that same rule; fixed 2026-08-21.
  char buf[32];
  std::snprintf(buf, sizeof(buf), "err %u #%lu\n", static_cast<unsigned>(code),
                static_cast<unsigned long>(id));
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
    case Result::kDuplicateId: return 11;  // unreachable as of 2026-08-21
                                            // (docs/design/protocol.md
                                            // §2.2/§9.8) -- kept for
                                            // completeness against the
                                            // Result enum, never actually
                                            // produced by this handler.
  }
  return 1;  // unreachable with every enumerator handled above; kept so
             // a FUTURE enumerator trips -Wswitch instead of silently
             // falling through a default case
}

// ---- session verbs --------------------------------------------------------
// HELLO is handled entirely in dispatch() (it is unsequenced and has its
// own arity check there). PING/ID/VER/STATUS/HELP all take zero DATA
// fields (spec §3.1, id already stripped by dispatch()) -- any
// remaining field at all is wrong arity.

void ProtocolHandler::handleHello(char** fields, size_t fieldCount,
                                   uint32_t id) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  // HELLO resets the reliability layer's entire state (docs/design/
  // protocol.md §8.3) -- the session-start resync a (re)connecting host
  // performs.
  expectedNext_ = 1;
  lastDone_ = 0;
  gapOutstanding_ = false;
  sendBanner();  // spec §4: HELLO's reply is byte-identical to the
                 // unsolicited boot banner
}

void ProtocolHandler::handlePing(char** fields, size_t fieldCount,
                                  uint32_t id) {
  (void)fields;
  if (fieldCount != 0) { ++malformedCount_; replyErr(id, 2); return; }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "pong %lu\n",
                static_cast<unsigned long>(adapter_.now()));
  writeLine(buf);
}

void ProtocolHandler::handleVer(char** fields, size_t fieldCount,
                                 uint32_t id) {
  (void)fields;
  if (fieldCount != 0) { ++malformedCount_; replyErr(id, 2); return; }
  Identity identity;
  adapter_.identity(identity);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "ver %s\n", identity.version);
  writeLine(buf);
}

void ProtocolHandler::handleId(char** fields, size_t fieldCount,
                                uint32_t id) {
  (void)fields;
  if (fieldCount != 0) { ++malformedCount_; replyErr(id, 2); return; }
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "id %s %s %s\n", identity.drivetrain,
                identity.profile, identity.version);
  writeLine(buf);
}

void ProtocolHandler::handleStatus(char** fields, size_t fieldCount,
                                    uint32_t id) {
  (void)fields;
  if (fieldCount != 0) { ++malformedCount_; replyErr(id, 2); return; }
  StatusFields status;
  adapter_.status(status);
  char buf[176];
  std::snprintf(buf, sizeof(buf),
                "status ready=%d active=%d connL=%d connR=%d otos=%d "
                "wedge=%d flags=%x tlm=%s next=%lu\n",
                status.ready ? 1 : 0, status.active ? 1 : 0,
                status.connLeft ? 1 : 0, status.connRight ? 1 : 0,
                status.otos ? 1 : 0, status.wedge ? 1 : 0,
                static_cast<unsigned int>(status.flags), status.tlm,
                static_cast<unsigned long>(expectedNext_));
  writeLine(buf);
}

void ProtocolHandler::handleHelp(char** fields, size_t fieldCount,
                                  uint32_t id) {
  (void)fields;
  if (fieldCount != 0) { ++malformedCount_; replyErr(id, 2); return; }
  // "Generated by walking the verb table at runtime, so it cannot drift
  // from the dispatcher" (spec S4) -- kCommandTable is the SAME table
  // dispatch() looks verbs up in.
  char buf[160];
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < sizeof(buf) - 1) buf[pos++] = *text++;
  };
  append("help");
  for (size_t i = 0; i < sizeof(kCommandTable) / sizeof(kCommandTable[0]);
       ++i) {
    append(" ");
    append(kCommandTable[i].name);
  }
  append("\n");
  buf[pos] = '\0';
  writeLine(buf);
}

// ---- configuration: pure delegation, no storage here (spec §6) ----------

void ProtocolHandler::handleGet(char** fields, size_t fieldCount,
                                 uint32_t id) {
  if (fieldCount > 1) { ++malformedCount_; replyErr(id, 2); return; }

  char buf[kMaxGetReplyBytes];
  char formatted[32];
  if (fieldCount == 0) {
    // Bare GET: dump every field the adapter declares, one line each
    // (spec §7.1: "one line per field, 80 lines" for the real 80-row
    // table -- this library carries none, so the line count here is
    // whatever THIS adapter's fieldCount() says). The GET command was
    // already acked once, as a whole, by dispatch() -- there is no
    // per-field ack.
    size_t total = adapter_.fieldCount();
    for (size_t i = 0; i < total; ++i) {
      const char* name = adapter_.fieldName(i);
      float value = 0.0f;
      if (!adapter_.onGet(name, value)) continue;
      formatConfigValue(value, formatted, sizeof(formatted));
      std::snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
      writeLine(buf);
    }
    return;
  }

  const char* name = fields[0];
  float value = 0.0f;
  // Unknown name: no `get` line, but the command is still acked (it
  // arrived fine and was answered with an empty result) -- not an
  // error, and not counted malformed (docs/design/protocol.md §6, §8.2).
  if (!adapter_.onGet(name, value)) return;
  formatConfigValue(value, formatted, sizeof(formatted));
  std::snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
  writeLine(buf);
}

void ProtocolHandler::handleSet(char** fields, size_t fieldCount,
                                 uint32_t id) {
  if (fieldCount != 2) { ++malformedCount_; replyErr(id, 2); return; }

  const char* name = fields[0];
  float value = 0.0f;
  if (!parseFloatField(fields[1], value)) {
    // The VALUE field itself is malformed -- a handler-level decode
    // failure (spec §7.2: SET's value is decoded by the handler), never
    // reaching onSet(). The command was already acked (it arrived, in
    // order); this err is the separate "content rejected" signal
    // (docs/design/protocol.md §8.2).
    ++malformedCount_;
    replyErr(id, 2);
    return;
  }

  Result result = adapter_.onSet(name, value, id);
  if (result != Result::kOk) replyErr(id, resultCode(result));
  // kOk: nothing further -- the ack dispatch() already sent IS the
  // acceptance (§8.2). `ok` no longer exists.
}

// ---- telemetry -------------------------------------------------------------

void ProtocolHandler::handleTlm(char** fields, size_t fieldCount,
                                 uint32_t id) {
  if (fieldCount != 1) { ++malformedCount_; replyErr(id, 2); return; }
  TlmMode mode;
  if (!parseTlmMode(fields[0], mode)) {
    ++malformedCount_;
    replyErr(id, 2);
    return;
  }
  // The Result is the adapter's own business (e.g. logging) and never
  // surfaces on the wire -- unchanged from before the reliability layer;
  // TLM is acked (it is sequenced now) but its own Result still is not
  // separately reported.
  (void)adapter_.onTlm(mode);
}

// ---- motion ----------------------------------------------------------------

void ProtocolHandler::handleWheels(char** fields, size_t fieldCount,
                                    uint32_t id) {
  if (fieldCount != 3) { ++malformedCount_; replyErr(id, 2); return; }

  int32_t left = 0, right = 0;
  uint32_t duration = 0;
  if (!parseInt32(fields[0], left) || !parseInt32(fields[1], right) ||
      !parseUint32(fields[2], duration)) {
    ++malformedCount_;
    replyErr(id, 2);
    return;
  }

  // duration's documented "ceiling 5000" (spec §5.2) is NOT enforced
  // here -- see protocol_handler.h's ambiguity note #1. It reaches the
  // adapter untouched.
  Result result = adapter_.onWheels(static_cast<float>(left),
                                     static_cast<float>(right), duration, id);
  if (result != Result::kOk) replyErr(id, resultCode(result));
}

void ProtocolHandler::handleStop(char** fields, size_t fieldCount,
                                  uint32_t id) {
  (void)fields;
  // STOP's id was ITS entire old field list; now that dispatch() strips
  // the mandatory id centrally, STOP has no DATA fields of its own at
  // all -- any remaining field is wrong arity.
  if (fieldCount != 0) { ++malformedCount_; replyErr(id, 2); return; }
  Result result = adapter_.onStop(id);
  if (result != Result::kOk) replyErr(id, resultCode(result));
}

void ProtocolHandler::handleEstop(char** fields, size_t fieldCount,
                                   uint32_t id) {
  // ESTOP is OUTSIDE the sequence entirely (docs/design/protocol.md
  // §8.3) -- dispatch() calls this directly, before any id is even
  // looked at, and passes fieldCount/lastFieldToken exactly as
  // tokenizeLine() produced them (fields may include what LOOKS like an
  // id-shaped token; it is never treated as one here).
  //
  // Maximally forgiving: ANY line whose verb is ESTOP executes the
  // stop, regardless of trailing junk or arity -- a panic stop must
  // never be refused over a syntax nit. `fields`/`fieldCount` are
  // therefore unused; `id` is always 0 here (dispatch() never resolves
  // a real one for this verb).
  (void)fields;
  (void)fieldCount;
  (void)id;

  // Execute BEFORE replying -- the stop must never wait on the sink
  // (docs/design/protocol.md §8.3, superseding the pre-2026-08-21 "never
  // reply at all" rule, whose own rationale -- "must not queue behind an
  // outbound reply" -- this ordering already satisfies).
  adapter_.onEstop();
  writeLine("estop\n");
}

// ---- RUN: parse-and-delegate only, per adapter.h's own onRun() doc --------
//
// This handler holds no function table, does no name resolution, and
// does no type conversion -- it extracts the function-name token and
// the raw argument tokens that follow it, and hands them to the
// adapter unchanged. Everything past that (resolving the name, per-
// argument conversion, invocation, stringifying a return value) is the
// adapter's job (adapter.h's onRun() doc, docs/design/protocol.md's RUN
// section).
//
// The mandatory id is already stripped by dispatch() before this is
// ever called (docs/design/protocol.md §6.3, 2026-08-21) -- unlike the
// pre-reliability-layer design, this handler no longer inspects the
// line's last field for a leading '#' itself; `fields`/`fieldCount` here
// are purely the function name plus its arguments.
void ProtocolHandler::handleRun(char** fields, size_t fieldCount,
                                 uint32_t id) {
  if (fieldCount == 0) {
    // "RUN #<id>" -- the id consumed the only field, leaving no function
    // name at all. Still acked (dispatch() already did that); this err
    // is the "content rejected" half.
    ++malformedCount_;
    replyErr(id, 2);
    return;
  }
  if (fieldCount > kMaxFieldTokens - 1) {
    // More real DATA tokens than this line's fixed-size field array can
    // safely hold pointers for (protocol_handler.h ambiguity note #3 /
    // tokenizeLine()'s own comment) -- reject before indexing fields[]
    // anywhere near that boundary. RUN is the only verb in this file
    // whose arity is not fixed, so it is the only one that needs this
    // check at all: every OTHER handler's own fixed-arity comparison is
    // already far inside kMaxFieldTokens by construction.
    ++malformedCount_;
    replyErr(id, 2);
    return;
  }

  const char* name = fields[0];
  const size_t argc = fieldCount - 1;  // fields[1 .. fieldCount-1]

  if (argc > kMaxRunArgs) {
    // A firmware resource limit (the fixed argv[] array below), not a
    // claim about any function's real arity.
    ++malformedCount_;
    replyErr(id, 2);
    return;
  }

  const char* argv[kMaxRunArgs];
  for (size_t i = 0; i < argc; ++i) argv[i] = fields[1 + i];

  char result[kMaxRunResultBytes] = {};
  bool hasResult = false;
  Result outcome =
      adapter_.onRun(name, argv, argc, result, sizeof(result), hasResult);

  if (outcome != Result::kOk) {
    replyErr(id, resultCode(outcome));
    return;
  }

  if (!hasResult) {
    // A void-returning function: nothing further -- the ack already
    // sent is the whole story, exactly like any other accepted command
    // with nothing to report (§8.2).
    return;
  }

  // Sanitize the ADAPTER's own returned text before it reaches the
  // sink -- the same '\n'/'\r'-stripping rule sendDebug()'s text gets,
  // and for the same reason: this string is about to be formatted
  // directly onto a single wire line, and this handler cannot assume a
  // concrete Adapter's own onRun() already did this. kMaxRunResultBytes
  // already guarantees the sanitized text plus "ret "/" #<id>"/'\n'
  // fits kMaxLineBytes, and sanitizing can only shrink it further, never
  // risk overflow.
  char sanitized[kMaxRunResultBytes];
  sanitizeLineText(result, sanitized, sizeof(sanitized));

  // +1: kMaxLineBytes already counts the WIRE content up to and
  // including '\n' (protocol_handler.h's own doc on the constant), but
  // snprintf() also needs room for its own NUL terminator -- a content
  // string that legitimately reaches the full 240 bytes needs a 241-byte
  // buffer, or snprintf silently truncates the last byte (here, the
  // trailing '\n' itself) to make room for the NUL it always writes.
  char buf[kMaxLineBytes + 1];
  std::snprintf(buf, sizeof(buf), "ret %s #%lu\n", sanitized,
                static_cast<unsigned long>(id));
  writeLine(buf);
}

// ---- unsolicited emissions -------------------------------------------------

void ProtocolHandler::sendBanner() {
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "device NEZHA2 robot %s %s\n",
                identity.name, identity.serial);
  writeLine(buf);
}

void ProtocolHandler::sendReady() { writeLine("ready\n"); }

void ProtocolHandler::sendDebug(const char* text) {
  // Sanitize into a temporary buffer FIRST, then decide the reply's
  // shape off the RESULT -- not off whether `text` itself was empty --
  // so "nullptr", "\"\"", and "text that is ENTIRELY '\n'/'\r' bytes"
  // (e.g. "\r\n") all collapse onto the exact same bare "debug\n" output
  // instead of the last one alone leaving a dangling separator space
  // ("debug \n") that no other field-less reply in this file ever
  // produces (protocol_handler.h's own sendDebug() doc: "an empty token
  // cannot exist between spaces"). Entirely unaffected by the
  // reliability layer -- debug never carries an id.
  char sanitized[kMaxDebugTextBytes];
  size_t len = sanitizeLineText(text, sanitized, sizeof(sanitized));

  // +1: kMaxLineBytes already counts the WIRE content up to and
  // including '\n' (protocol_handler.h's own doc on the constant), but
  // snprintf() also needs room for its own NUL terminator -- a content
  // string that legitimately reaches the full 240 bytes needs a 241-byte
  // buffer, or snprintf silently truncates the last byte (here, the
  // trailing '\n' itself) to make room for the NUL it always writes.
  char buf[kMaxLineBytes + 1];
  if (len == 0) {
    std::snprintf(buf, sizeof(buf), "debug\n");
  } else {
    std::snprintf(buf, sizeof(buf), "debug %s\n", sanitized);
  }
  writeLine(buf);
}

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
    append(" ");
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
    append(" ");
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

  // The reliability layer's own periodic emission (docs/design/
  // protocol.md §8.5) -- rides the caller's own telemetry cadence, no
  // timer of this class's own. A stalled stream (gapOutstanding_) keeps
  // re-nacking for free at this rate; otherwise this simply re-states
  // the highest id already accepted, so a host that goes quiet after
  // its last command still eventually learns it landed.
  if (gapOutstanding_) {
    replyNack(expectedNext_);
  } else {
    replyAck(expectedNext_ - 1);
  }
}

}  // namespace Protocol

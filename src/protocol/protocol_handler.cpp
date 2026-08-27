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
// config values (spec §7.2). "Strict" means the WHOLE field must be
// consumed by strtol/strtoul/strtof -- a trailing letter or stray
// interior byte makes the field unparseable ("wrong arity is a
// rejection, not a best-effort parse" extended to field content).

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
// tokenizing on ' ' stops a field from starting with one of them.
// strtof/strtol would silently skip that leading tab per the C standard
// and parse the rest anyway.
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

// The id's own numeric grammar (`id ::= '#' [0-9]+`) is STRICTER than
// parseUint32() above: no sign at all, not even a leading '+', which
// C's strtoul() would otherwise accept as valid syntax and parseUint32()
// does not itself reject (it only rejects '-'). A pre-pass that requires
// every byte to be an ASCII digit before strtoul() ever runs means "#+5"
// is correctly NOT a well-formed id -- it falls through to "no valid id
// at all" (docs/design/protocol.md §8.4), not "id 5".
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
// There is no "#0 is special" branch here at all -- every well-formed
// id, including 0, is handled identically by dispatch()'s own three-way
// sequence compare.
bool parseMandatoryId(const char* token, uint32_t& id) {
  if (token == nullptr || token[0] != '#') return false;
  return parseIdDigits(token + 1, id);
}

// Config values are the one place floats appear on the wire (spec
// §7.2). "No exponents, no NaN, no inf" is stated for the integer
// fields but this handler applies the same posture here too -- nothing
// in this project ever needs a robot to accept "1e10" or "nan" as a
// gain.
bool parseFloatField(const char* field, float& out) {
  if (field == nullptr || field[0] == '\0' || isWireSpace(field[0])) {
    return false;
  }
  for (const char* p = field; *p != '\0'; ++p) {
    // 'e'/'E' bars decimal-exponent notation ("1e10"). 'x'/'X' bars C99
    // HEX FLOAT notation ("0x1p3", "0X1.8P3") -- strtof accepts this
    // syntax unconditionally, so an adversarial-input sweep found that a
    // wire value like `SET foo.bar 0x1.8p3 #9` silently decoded to 12.0
    // instead of being rejected. A MicroPython or JavaScript port would
    // not reproduce this: neither `float()` nor `Number()`/`parseFloat()`
    // accepts hex-float syntax.
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
      {"HDR", TlmMode::kHdr},
  };
  for (const auto& entry : kModes) {
    if (std::strcmp(field, entry.name) == 0) {
      mode = entry.mode;
      return true;
    }
  }
  return false;
}

// formatConfigValue() -- spec §7.2's formatFixed(): six fractional
// digits, always present, no exponent, using integer arithmetic because
// newlib-nano's printf has no %f. formatConfigValue(0.02f) ->
// "0.020000", formatConfigValue(-51.5f) -> "-51.500000".
//
// `value` is NOT wire-parsed here -- it is whatever the ADAPTER's own
// onGet() handed back (parseFloatField already rejects NaN/Inf on the
// way IN), so this function cannot assume it is finite. +-Inf is already
// handled correctly below: it compares greater than kMaxScaled and gets
// clamped before the cast. NaN does not: EVERY comparison against a NaN
// is false, so `scaled > kMaxScaled` is false too and a NaN sails past
// the clamp intact into `static_cast<uint32_t>(scaled)` -- converting a
// NaN to an unsigned integer is undefined behavior. There is no wire
// spelling for NaN to preserve, so fail safe to 0.0 rather than invent
// one.
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
// fixed-size fields[] array's own storage cap.
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
// '\n'/'\r' byte rather than rejecting the call outright -- both
// sendDebug() and RUN's own returned-value formatting call this to keep
// an embedded terminator byte from forging a second wire line.
// `text == nullptr` is treated exactly like `text == ""`. Truncates,
// never overflows, once `out` is full -- always NUL-terminates within
// `outCap`. Returns the number of bytes written (excluding the
// terminator).
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

const ProtocolHandler::VerbEntry ProtocolHandler::kCommandTable[18] = {
    {"HELLO", &ProtocolHandler::decodeAlwaysTrue, &ProtocolHandler::execNoop},
    {"PING", &ProtocolHandler::decodeAlwaysTrue, &ProtocolHandler::execNoop},
    {"ID", &ProtocolHandler::decodeNoFields, &ProtocolHandler::execId},
    {"VER", &ProtocolHandler::decodeNoFields, &ProtocolHandler::execVer},
    {"STATUS", &ProtocolHandler::decodeNoFields, &ProtocolHandler::execStatus},
    {"HELP", &ProtocolHandler::decodeNoFields, &ProtocolHandler::execHelp},
    {"GET", &ProtocolHandler::decodeGet, &ProtocolHandler::execGet},
    {"SET", &ProtocolHandler::decodeSet, &ProtocolHandler::execSet},
    {"TLM", &ProtocolHandler::decodeTlm, &ProtocolHandler::execTlm},
    {"WHEELS_X", &ProtocolHandler::decodeWheelsX,
     &ProtocolHandler::execWheelsX},
    {"WHEELS_V", &ProtocolHandler::decodeWheelsV,
     &ProtocolHandler::execWheelsV},
    {"MOVE_X", &ProtocolHandler::decodeMoveX, &ProtocolHandler::execMoveX},
    {"MOVE_V", &ProtocolHandler::decodeMoveV, &ProtocolHandler::execMoveV},
    {"GO_TO_R", &ProtocolHandler::decodeGoToR, &ProtocolHandler::execGoToR},
    {"GO_TO_W", &ProtocolHandler::decodeGoToW, &ProtocolHandler::execGoToW},
    {"STOP", &ProtocolHandler::decodeStop, &ProtocolHandler::execStop},
    {"ESTOP", &ProtocolHandler::decodeAlwaysTrue, &ProtocolHandler::execNoop},
    {"RUN", &ProtocolHandler::decodeRun, &ProtocolHandler::execRun},
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
    // with legal arity would be a command the host never sent.
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

// ---- tokenizing (spec §2, §3.2) -----------------------------------------

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

  // ---- the three verbs OUTSIDE the sequence entirely (docs/design/
  // protocol.md §8.3) -- checked by verb identity, before any id is
  // even looked at.
  if (std::strcmp(verb, "ESTOP") == 0) {
    handleEstop();
    return;
  }
  if (std::strcmp(verb, "PING") == 0) {
    // PING joins ESTOP/HELLO in the unsequenced exemption set as of
    // 2026-08-22 (stakeholder direction: "ESTOP, ping, and HELLO
    // shouldn't require IDs ... it is liveness and must answer while
    // the stream is stalled in a gap"). Whether PING should then be
    // MAXIMALLY FORGIVING of trailing content (like ESTOP) or STRICT
    // zero-arity (like HELLO) is this file's own call -- see
    // protocol_handler.h's ambiguity note #4: resolved forgiving, so a
    // host still appending an old-style `#<id>` to PING out of habit
    // keeps working unchanged, and PING can never itself wedge on a
    // syntax nit the way its own purpose (liveness) argues it must not.
    handlePing();
    return;
  }
  if (std::strcmp(verb, "HELLO") == 0) {
    // HELLO's own arity is unchanged (zero fields, id or otherwise) --
    // a HELLO with a trailing field is wrong arity, same as any other
    // extra field, and (since HELLO is outside the sequence) has no ack
    // to anchor an err against, so it is silently malformed like any
    // other unsequenced-verb failure.
    if (fieldCount != 0) {
      ++malformedCount_;
      return;
    }
    handleHello(fields, fieldCount);
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
    // compare against expectedNext_, so there is no reply of any kind.
    ++malformedCount_;
    return;
  }

  // The id itself is always fields[fieldCount - 1] once well-formed
  // (findLastFieldToken() found it), so the verb's own DATA fields are
  // everything before it.
  const size_t dataFieldCount = fieldCount - 1;

  if (id < expectedNext_) {
    // A stale retransmit -- the host never saw our ack for something we
    // already accepted. Do NOT re-execute (a resent WHEELS_V must not
    // drive twice); just re-state what we already have.
    replyAck(expectedNext_ - 1);
    return;
  }
  if (id > expectedNext_) {
    // A numeric gap: something between expectedNext_ and id never
    // arrived (or arrived out of order). Discard -- do NOT execute, and
    // do not even look up the verb -- and tell the host exactly what we
    // need next. Every further inbound line re-triggers this same nack
    // until the missing id arrives (§8.1) -- that per-inbound-line
    // repeat is the whole retransmit story now (2026-08-26, §8.5).
    replyNack(expectedNext_);
    return;
  }

  // id == expectedNext_: find the verb and DECODE its own fields BEFORE
  // sending any reply at all (2026-08-22, docs/design/protocol.md §8.9
  // -- the "decode failure is a NAK" direction). This is the behavior
  // change from the pre-2026-08-22 design, which acked unconditionally
  // the instant the id was in order and only THEN checked the verb's
  // own content -- see the file header's own note for the full
  // rationale and the give-up-path hazard it flags.
  const VerbEntry* entry = nullptr;
  for (const auto& e : kCommandTable) {
    if (std::strcmp(verb, e.name) == 0) {
      entry = &e;
      break;
    }
  }
  if (entry == nullptr) {
    // Unrecognized verb: a decode failure exactly like a known verb's
    // own bad arity or unparseable field (docs/design/protocol.md
    // §8.9) -- the sequence does NOT advance.
    handleDecodeFailure(id, resultCode(Result::kUnknown));
    return;
  }
  if (!(this->*entry->decode)(fields, dataFieldCount)) {
    handleDecodeFailure(id, resultCode(Result::kBadArg));
    return;
  }

  // Decoded fine: the line arrived intact. The sequence advances and
  // the ack is sent UNCONDITIONALLY at this point -- "did the bytes
  // arrive, in order, and did they parse" is answered here regardless
  // of whether the ADAPTER goes on to refuse the content on its own
  // merits (docs/design/protocol.md §8.2).
  expectedNext_ = id + 1;
  replyAck(id);

  uint8_t errCode = 0;
  (this->*entry->execute)(fields, dataFieldCount, id, errCode);
  if (errCode != 0) replyErr(id, errCode);
}

void ProtocolHandler::handleDecodeFailure(uint32_t id, uint8_t code) {
  // The sequence does NOT advance: `id` is still expectedNext_ at this
  // point (that equality is what routed dispatch() into this function
  // at all), so nack-ing expectedNext_ unchanged tells the host to
  // resend EXACTLY this id -- "NAK and resend from that point on"
  // (docs/design/protocol.md §8.9, the stakeholder's own framing).
  // A stalled stream keeps re-nacking because every subsequent inbound
  // line re-triggers nack(expectedNext_) exactly like a numeric gap
  // would (§8.1), until a well-formed line finally arrives carrying
  // this same id -- there is no periodic re-nack (2026-08-26, §8.5).
  ++malformedCount_;
  replyNack(expectedNext_);
  replyErr(id, code);
}

void ProtocolHandler::replyAck(uint32_t ackedId) {
  char buf[56];
  std::snprintf(buf, sizeof(buf), "ack %lu %lu %s\n",
                static_cast<unsigned long>(ackedId),
                static_cast<unsigned long>(adapter_.lastDone()),
                doneReasonWireName(adapter_.lastDoneReason()));
  writeLine(buf);
}

void ProtocolHandler::replyNack(uint32_t nextId) {
  char buf[56];
  std::snprintf(buf, sizeof(buf), "nack %lu %lu %s\n",
                static_cast<unsigned long>(nextId),
                static_cast<unsigned long>(adapter_.lastDone()),
                doneReasonWireName(adapter_.lastDoneReason()));
  writeLine(buf);
}

void ProtocolHandler::replyErr(uint32_t id, uint8_t code) {
  // Field order: code THEN #id -- the id is always the LAST token of
  // ANY line under this grammar, replies included (docs/design/
  // protocol.md §8.6).
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
  }
  return 1;  // unreachable with every enumerator handled above; kept so
             // a FUTURE enumerator trips -Wswitch instead of silently
             // falling through a default case
}

const char* ProtocolHandler::doneReasonWireName(DoneReason reason) {
  switch (reason) {
    case DoneReason::kNone: return "none";
    case DoneReason::kStop: return "stop";
    case DoneReason::kTimeout: return "timeout";
    case DoneReason::kEstop: return "estop";
    case DoneReason::kAborted: return "aborted";
  }
  return "none";  // unreachable with every enumerator handled above
}

// ---- three verbs outside the sequence -----------------------------------

void ProtocolHandler::handleHello(char** fields, size_t fieldCount) {
  (void)fields;
  (void)fieldCount;
  // HELLO resets the reliability layer's own sequencing state
  // (docs/design/protocol.md §8.3) -- the session-start resync a
  // (re)connecting host performs. It does NOT touch the Adapter's own
  // lastDone()/lastDoneReason() (2026-08-22): that state moved OFF this
  // class entirely, and a handler-level reset has no business reaching
  // into the Adapter to clear something it does not own. An Adapter
  // that wants a HELLO to also clear ITS OWN notion of "last completed
  // motion" is free to do so from wherever it observes HELLO (e.g. its
  // own onRun()/identity() call, or simply never -- a completed motion
  // stays completed across a reconnect, which is arguably the more
  // useful default). Flagged here because the pre-2026-08-22 text
  // explicitly reset `lastDone_` as part of this same call.
  expectedNext_ = 1;
  sendBanner();  // spec §4: HELLO's reply is byte-identical to the
                 // unsolicited boot banner
}

void ProtocolHandler::handlePing() {
  // Unsequenced and maximally forgiving (see dispatch()'s own comment
  // at the PING branch): ANY line whose verb is PING replies `pong`,
  // regardless of what -- if anything -- follows it.
  char buf[32];
  std::snprintf(buf, sizeof(buf), "pong %lu\n",
                static_cast<unsigned long>(adapter_.now()));
  writeLine(buf);
}

void ProtocolHandler::handleEstop() {
  // ESTOP is OUTSIDE the sequence entirely (docs/design/protocol.md
  // §8.3) -- maximally forgiving: ANY line whose verb is ESTOP executes
  // the stop, regardless of trailing junk or arity. Execute BEFORE
  // replying -- the stop must never wait on the sink.
  adapter_.onEstop();
  writeLine("estop\n");
}

// ---- trivial stand-ins for HELLO/PING/ESTOP's table rows -----------------
// Never actually invoked through kCommandTable (all three are
// intercepted by verb identity in dispatch() before the table lookup
// ever runs) -- present purely so HELP's generated listing walks one
// table for every verb name.

bool ProtocolHandler::decodeAlwaysTrue(char** fields, size_t fieldCount) {
  (void)fields;
  (void)fieldCount;
  return true;
}

void ProtocolHandler::execNoop(char** fields, size_t fieldCount, uint32_t id,
                                uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
}

// ---- session verbs --------------------------------------------------------
// ID/VER/STATUS/HELP all take zero DATA fields (id already stripped by
// dispatch()) -- any remaining field at all is wrong arity, a decode
// failure.

bool ProtocolHandler::decodeNoFields(char** fields, size_t fieldCount) {
  (void)fields;
  return fieldCount == 0;
}

void ProtocolHandler::execVer(char** fields, size_t fieldCount, uint32_t id,
                               uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  Identity identity;
  adapter_.identity(identity);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "ver %s\n", identity.version);
  writeLine(buf);
}

void ProtocolHandler::execId(char** fields, size_t fieldCount, uint32_t id,
                              uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "id %s %s %s\n", identity.drivetrain,
                identity.profile, identity.version);
  writeLine(buf);
}

void ProtocolHandler::execStatus(char** fields, size_t fieldCount,
                                  uint32_t id, uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
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

void ProtocolHandler::execHelp(char** fields, size_t fieldCount, uint32_t id,
                                uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  // "Generated by walking the verb table at runtime, so it cannot drift
  // from the dispatcher" -- kCommandTable is the SAME table dispatch()
  // looks verbs up in.
  char buf[224];
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

// ---- configuration: pure delegation, no storage here (spec §7) ----------

bool ProtocolHandler::decodeGet(char** fields, size_t fieldCount) {
  (void)fields;
  return fieldCount <= 1;
}

void ProtocolHandler::execGet(char** fields, size_t fieldCount, uint32_t id,
                               uint8_t& errCode) {
  (void)id;
  errCode = 0;  // GET never produces an err -- an unknown name is just a
                // silent no-`get`-line answer (spec §7, §8.2).

  char buf[kMaxGetReplyBytes];
  char formatted[32];
  if (fieldCount == 0) {
    // Bare GET: dump every field the adapter declares, one line each.
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
  // error, and not counted malformed.
  if (!adapter_.onGet(name, value)) return;
  formatConfigValue(value, formatted, sizeof(formatted));
  std::snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
  writeLine(buf);
}

bool ProtocolHandler::decodeSet(char** fields, size_t fieldCount) {
  if (fieldCount != 2) return false;
  float discard = 0.0f;
  return parseFloatField(fields[1], discard);
}

void ProtocolHandler::execSet(char** fields, size_t fieldCount, uint32_t id,
                               uint8_t& errCode) {
  (void)fieldCount;
  float value = 0.0f;
  parseFloatField(fields[1], value);  // decodeSet() already proved this
                                       // succeeds
  Result result = adapter_.onSet(fields[0], value, id);
  errCode = resultCode(result);
}

// ---- telemetry -------------------------------------------------------------

bool ProtocolHandler::decodeTlm(char** fields, size_t fieldCount) {
  if (fieldCount != 1) return false;
  TlmMode discard;
  return parseTlmMode(fields[0], discard);
}

void ProtocolHandler::execTlm(char** fields, size_t fieldCount, uint32_t id,
                               uint8_t& errCode) {
  (void)fieldCount;
  (void)id;
  errCode = 0;  // the adapter's own Result never surfaces on the wire
                // for TLM (unchanged from before the reliability layer).
  TlmMode mode;
  parseTlmMode(fields[0], mode);  // decodeTlm() already proved this succeeds

  // TLM HDR (docs/design/protocol.md §10.5): a header-recovery request,
  // not a subscription change. Handled entirely here by clearing the
  // handler's own remembered-header state directly -- the same field
  // headerChanged() already checks -- so the very next emitTelemetry()
  // call re-emits `thdr` before its next `t` frame. This never reaches
  // Adapter::onTlm(): the handler already owns every bit of state that
  // needs clearing (headerCount_/headerNames_/headerHex_/
  // everEmittedHeader_), and forwarding it would wrongly let a
  // DiffDriveAdapter-shaped `if (mode != TlmMode::kNow) mode_ = mode;`
  // persist kHdr as the current subscription mode.
  if (mode == TlmMode::kHdr) {
    everEmittedHeader_ = false;
    return;
  }
  (void)adapter_.onTlm(mode);
}

// ---- motion: WHEELS_X / WHEELS_V / MOVE_X / MOVE_V / GO_TO_R / GO_TO_W ----
// docs/design/motion-api.md §9.1's wire mapping. Angles (rotation, omega)
// are milliradian integers on the wire (§9.1: "degrees at the API,
// milliradian integers on the wire ... the conversion lives in the
// binding, in one place" -- NOT this library's job), decoded here with
// the ordinary signed-integer field parser and handed to the Adapter as
// float milliradians, the same "wire integer -> float for arithmetic
// convenience" pattern WHEELS_V's own left/right fields already used
// before this change.

bool ProtocolHandler::decodeWheelsX(char** fields, size_t fieldCount) {
  if (fieldCount != 4) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseInt32(fields[2], discard32) && parseUint32(fields[3], discardU);
}

void ProtocolHandler::execWheelsX(char** fields, size_t fieldCount,
                                   uint32_t id, uint8_t& errCode) {
  (void)fieldCount;
  int32_t left = 0, right = 0, cruise = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], left);
  parseInt32(fields[1], right);
  parseInt32(fields[2], cruise);
  parseUint32(fields[3], timeout);
  Result result =
      adapter_.onWheelsX(static_cast<float>(left), static_cast<float>(right),
                         static_cast<float>(cruise), timeout, id);
  errCode = resultCode(result);
}

bool ProtocolHandler::decodeWheelsV(char** fields, size_t fieldCount) {
  if (fieldCount != 3) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseUint32(fields[2], discardU);
}

void ProtocolHandler::execWheelsV(char** fields, size_t fieldCount,
                                   uint32_t id, uint8_t& errCode) {
  (void)fieldCount;
  int32_t left = 0, right = 0;
  uint32_t duration = 0;
  parseInt32(fields[0], left);
  parseInt32(fields[1], right);
  parseUint32(fields[2], duration);
  Result result = adapter_.onWheelsV(static_cast<float>(left),
                                     static_cast<float>(right), duration, id);
  errCode = resultCode(result);
}

bool ProtocolHandler::decodeMoveX(char** fields, size_t fieldCount) {
  if (fieldCount != 4) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseInt32(fields[2], discard32) && parseUint32(fields[3], discardU);
}

void ProtocolHandler::execMoveX(char** fields, size_t fieldCount, uint32_t id,
                                 uint8_t& errCode) {
  (void)fieldCount;
  int32_t distance = 0, rotation = 0, cruise = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], distance);
  parseInt32(fields[1], rotation);
  parseInt32(fields[2], cruise);
  parseUint32(fields[3], timeout);
  Result result = adapter_.onMoveX(static_cast<float>(distance),
                                   static_cast<float>(rotation),
                                   static_cast<float>(cruise), timeout, id);
  errCode = resultCode(result);
}

bool ProtocolHandler::decodeMoveV(char** fields, size_t fieldCount) {
  if (fieldCount != 3) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseUint32(fields[2], discardU);
}

void ProtocolHandler::execMoveV(char** fields, size_t fieldCount, uint32_t id,
                                 uint8_t& errCode) {
  (void)fieldCount;
  int32_t v_x = 0, omega = 0;
  uint32_t duration = 0;
  parseInt32(fields[0], v_x);
  parseInt32(fields[1], omega);
  parseUint32(fields[2], duration);
  Result result = adapter_.onMoveV(static_cast<float>(v_x),
                                   static_cast<float>(omega), duration, id);
  errCode = resultCode(result);
}

bool ProtocolHandler::decodeGoToR(char** fields, size_t fieldCount) {
  if (fieldCount != 5) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseInt32(fields[2], discard32) && parseInt32(fields[3], discard32) &&
         parseUint32(fields[4], discardU);
}

void ProtocolHandler::execGoToR(char** fields, size_t fieldCount, uint32_t id,
                                 uint8_t& errCode) {
  (void)fieldCount;
  int32_t x = 0, y = 0, speed = 0, arrive = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], x);
  parseInt32(fields[1], y);
  parseInt32(fields[2], speed);
  parseInt32(fields[3], arrive);
  parseUint32(fields[4], timeout);
  Result result =
      adapter_.onGoToR(static_cast<float>(x), static_cast<float>(y),
                       static_cast<float>(speed), static_cast<float>(arrive),
                       timeout, id);
  errCode = resultCode(result);
}

bool ProtocolHandler::decodeGoToW(char** fields, size_t fieldCount) {
  return decodeGoToR(fields, fieldCount);  // identical field shape
}

void ProtocolHandler::execGoToW(char** fields, size_t fieldCount, uint32_t id,
                                 uint8_t& errCode) {
  (void)fieldCount;
  int32_t x = 0, y = 0, speed = 0, arrive = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], x);
  parseInt32(fields[1], y);
  parseInt32(fields[2], speed);
  parseInt32(fields[3], arrive);
  parseUint32(fields[4], timeout);
  Result result =
      adapter_.onGoToW(static_cast<float>(x), static_cast<float>(y),
                       static_cast<float>(speed), static_cast<float>(arrive),
                       timeout, id);
  errCode = resultCode(result);
}

// ---- STOP: `STOP [now] #<id>` (docs/design/motion-api.md §3.7/§9.1) -----

bool ProtocolHandler::decodeStop(char** fields, size_t fieldCount) {
  if (fieldCount == 0) return true;
  return fieldCount == 1 && std::strcmp(fields[0], "now") == 0;
}

void ProtocolHandler::execStop(char** fields, size_t fieldCount, uint32_t id,
                                uint8_t& errCode) {
  const bool immediate = fieldCount == 1;  // decodeStop() already proved
                                            // this is exactly "now"
  (void)fields;
  Result result = adapter_.onStop(immediate, id);
  errCode = resultCode(result);
}

// ---- RUN: parse-and-delegate only, per adapter.h's own onRun() doc --------
//
// This handler holds no function table, does no name resolution, and
// does no type conversion -- it extracts the function-name token and
// the raw argument tokens that follow it, and hands them to the
// adapter unchanged. Everything past that (resolving the name, per-
// argument conversion, invocation, stringifying a return value) is the
// adapter's job.
//
// decodeRun()'s own DECODE FAILURES are purely structural: no function
// name at all, or more raw tokens than this line's fixed-size arrays can
// safely hold pointers for. An UNKNOWN function name, or a wrong arity
// the ADAPTER itself detects, is NOT a decode failure -- RUN's own
// grammar was satisfied (a name plus some argument tokens), so those are
// merits rejections the adapter reports through its own Result, exactly
// like SET's own "unknown field name" (docs/design/protocol.md §8.9).

bool ProtocolHandler::decodeRun(char** fields, size_t fieldCount) {
  (void)fields;
  if (fieldCount == 0) return false;  // no function name at all
  if (fieldCount > kMaxFieldTokens - 1) return false;  // storage overflow
  const size_t argc = fieldCount - 1;
  return argc <= kMaxRunArgs;
}

void ProtocolHandler::execRun(char** fields, size_t fieldCount, uint32_t id,
                               uint8_t& errCode) {
  const char* name = fields[0];
  const size_t argc = fieldCount - 1;  // fields[1 .. fieldCount-1]

  const char* argv[kMaxRunArgs];
  for (size_t i = 0; i < argc; ++i) argv[i] = fields[1 + i];

  char result[kMaxRunResultBytes] = {};
  bool hasResult = false;
  Result outcome =
      adapter_.onRun(name, argv, argc, result, sizeof(result), hasResult);

  errCode = resultCode(outcome);
  if (outcome != Result::kOk) return;
  if (!hasResult) return;  // a void-returning function: the ack already
                            // sent is the whole story.

  // Sanitize the ADAPTER's own returned text before it reaches the
  // sink -- the same '\n'/'\r'-stripping rule sendDebug()'s text gets.
  // kMaxRunResultBytes already guarantees the sanitized text plus
  // "ret "/" #<id>"/'\n' fits kMaxLineBytes, and sanitizing can only
  // shrink it further, never risk overflow.
  char sanitized[kMaxRunResultBytes];
  sanitizeLineText(result, sanitized, sizeof(sanitized));

  // +1: kMaxLineBytes already counts the WIRE content up to and
  // including '\n', but snprintf() also needs room for its own NUL
  // terminator -- a content string that legitimately reaches the full
  // 240 bytes needs a 241-byte buffer, or snprintf silently truncates
  // the last byte (here, the trailing '\n' itself) to make room for the
  // NUL it always writes.
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
  // all collapse onto the exact same bare "debug\n" output.
  char sanitized[kMaxDebugTextBytes];
  size_t len = sanitizeLineText(text, sanitized, sizeof(sanitized));

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
      // flags: lowercase hex, no "0x" prefix (docs/design/protocol.md
      // §10.2's Value encoding paragraph).
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
  // No reliability line rides here any more (2026-08-26, docs/design/
  // protocol.md §8.5): an ack/nack is only ever a direct reply to an
  // inbound sequenced line -- never a beacon on the telemetry cadence.
}

}  // namespace Protocol

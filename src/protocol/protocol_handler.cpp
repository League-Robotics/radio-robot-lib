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
// consumed by strtol/strtoul/strtof -- a trailing letter or stray
// interior byte makes the field unparseable, matching spec §2.2's "no
// exponents, no NaN, no inf" and docs/design/protocol.md §2.2's "Wrong
// arity is a rejection, not a best-effort parse" extended to field
// content.

// strtol/strtoul/strtof all skip LEADING whitespace before the first
// digit (a C-standard behavior, not a project choice). Under the
// COLON-delimited grammar this library used before the 2026-08-20
// grammar switch (commit 5a5b6da), that was a live bug: a field like
// " 100" or "\t100" sat directly after a ':' separator and could
// legitimately start with a stray space, which strtol would silently
// digest before parsing the digits, contradicting this file's own
// "strict, whole field consumed" contract.
//
// Under the SPACE grammar, the exact hazard the original fix targeted
// -- a token beginning with a literal ' ' (0x20) -- is now structurally
// IMPOSSIBLE to reach through this check: tokenizeLine() below collapses
// every run of ' ' into one separator and trims leading/trailing space
// before a token pointer is ever handed to a field decoder, so
// field[0] == ' ' can never be true for a token this parser produced.
// That part of the guard is dead code, kept only as cheap, harmless
// defense in depth (see the file-header note below).
//
// The guard is NOT fully dead, though -- it stays genuinely load-bearing
// for the OTHER C whitespace bytes. Spec §2's field grammar is
// `field ::= any bytes except ' ' and '\n'`, which means '\t', '\v',
// '\f' and '\r' are all LEGAL, ordinary field bytes under the new
// grammar -- nothing about tokenizing on ' ' stops a field from starting
// with one of them (e.g. "SET foo.bar<TAB>1.0" tokenizes `<TAB>1.0` as
// the value field, tab included). strtof/strtol would silently skip
// that leading tab per the C standard and parse "1.0" anyway, exactly
// reproducing the original bug's shape for a byte the space grammar
// never made a separator. So this check survives the migration, just
// with a narrower live threat surface than it had under the colon
// grammar: reachable and load-bearing for '\t'/'\v'/'\f'/'\r', vestigial
// (but harmless to keep) for ' ' itself.
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

// The id's own numeric grammar (spec §8.2/§2: `id ::= '#' [0-9]+`) is
// STRICTER than parseUint32() above: no sign at all, not even a leading
// '+', which C's strtoul() would otherwise accept as valid syntax and
// parseUint32() does not itself reject (it only rejects '-', spec
// §2.2's general integer-field rule being "optionally signed"). A
// pre-pass that requires every byte to be an ASCII digit before
// strtoul() ever runs means "#+5" is correctly NOT a well-formed id --
// it falls through to being treated as an ordinary (malformed) field,
// not as id 5.
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
    // `SET foo.bar 0x1.8p3` silently decoded to 12.0 instead of being
    // rejected -- exactly the "no exponents" rule this function exists
    // to enforce, bypassed by a spelling the spec's authors never had in
    // mind. A MicroPython or JavaScript port would not reproduce this:
    // neither `float()` nor `Number()`/`parseFloat()` accepts hex-float
    // syntax, so this was a C++-only divergence from every other
    // implementation of this same fixture. Unaffected by the
    // colon-to-space grammar migration: this is a property of strtof()
    // parsing an already-extracted field's own CONTENT, independent of
    // how that field was delimited on the wire.
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

// Outcome of resolving a verb's own OPTIONAL trailing id (SET, WHEELS)
// against spec §8.2's now-unambiguous rule (protocol_handler.h's
// "Resolved-by-the-new-grammar" note): omitted, explicit `#0`, an
// explicit nonzero `#<n>`, or a trailing token that is present but is
// NOT a well-formed id at all (e.g. a bare "9" with no '#', or "#abc")
// -- which, since SET/WHEELS have no OTHER use for that positional
// slot, means the whole line is malformed, not just "id-less".
enum class IdOutcome : uint8_t { kOmitted, kZero, kNonzero, kMalformed };

// `token` must be non-null when called (callers only invoke this when a
// trailing token IS present -- an omitted id is handled by the caller
// without inspecting anything, per IdOutcome::kOmitted above).
IdOutcome resolveTrailingOptionalId(const char* token, uint32_t& id) {
  if (token[0] != '#') return IdOutcome::kMalformed;
  uint32_t parsed = 0;
  if (!parseIdDigits(token + 1, parsed)) return IdOutcome::kMalformed;
  id = parsed;
  return parsed == 0 ? IdOutcome::kZero : IdOutcome::kNonzero;
}

// Spec §2's generic malformed-line recovery: "If the line's last token
// is a well-formed nonzero #id, reply err #<id> <code>." `token` may be
// nullptr (a line that was just a verb, nothing after it) -- always
// false in that case, matching "otherwise no reply".
bool recoverTrailingId(const char* token, uint32_t& id) {
  if (token == nullptr || token[0] != '#') return false;
  uint32_t parsed = 0;
  if (!parseIdDigits(token + 1, parsed)) return false;
  if (parsed == 0) return false;  // id 0 never gets an err reply (§8.2)
  id = parsed;
  return true;
}

// formatConfigValue() -- spec §7.2's formatFixed(), reproduced here
// (not included from src/archive/protocol-v6/wire_v6_format.{h,cpp},
// which is reference-only): six fractional digits, always present, no
// exponent, using integer arithmetic because newlib-nano's printf has
// no %f. formatConfigValue(0.02f) -> "0.020000",
// formatConfigValue(-51.5f) -> "-51.500000" (spec's own examples).
// Unaffected by the grammar migration -- pure value formatting, no
// wire delimiter involved.
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
// verb itself (nothing after it to recover an id from).
const char* findLastFieldToken(const char* line) {
  const char* end = line + std::strlen(line);
  const char* p = end;
  while (p > line && *(p - 1) == ' ') --p;  // skip trailing spaces
  while (p > line && *(p - 1) != ' ') --p;  // scan back through the token
  return p == line ? nullptr : p;
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

  // The self-marking trailing id (spec §2/§8.2) must be located BEFORE
  // tokenizeLine() below mutates any separator space to '\0' -- see
  // findLastFieldToken()'s own comment.
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

// ---- dispatch ------------------------------------------------------------

void ProtocolHandler::dispatch(char* verb, char** fields, size_t fieldCount,
                                const char* lastFieldToken) {
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
      (this->*entry.handler)(fields, fieldCount, lastFieldToken);
      return;
    }
  }
  // Unknown verb: no arity is knowable, but the line's own last token
  // can still be a well-formed nonzero #id worth acking against (spec
  // §2's own "including unknown verbs" framing -- protocol_handler.h's
  // ambiguity note #2).
  rejectMalformed(lastFieldToken, resultCode(Result::kUnknown));
}

void ProtocolHandler::replyOk(uint32_t id) {
  char buf[24];
  std::snprintf(buf, sizeof(buf), "ok #%lu\n", static_cast<unsigned long>(id));
  writeLine(buf);
}

void ProtocolHandler::replyOkBare() { writeLine("ok\n"); }

void ProtocolHandler::replyErr(uint32_t id, uint8_t code) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "err #%lu %u\n",
                static_cast<unsigned long>(id), static_cast<unsigned>(code));
  writeLine(buf);
}

void ProtocolHandler::replyErrBare(uint8_t code) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "err %u\n", static_cast<unsigned>(code));
  writeLine(buf);
}

void ProtocolHandler::rejectMalformed(const char* lastFieldToken,
                                       uint8_t code) {
  ++malformedCount_;
  uint32_t id = 0;
  if (recoverTrailingId(lastFieldToken, id)) replyErr(id, code);
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
// HELLO/PING/ID/VER/STATUS/HELP all take zero fields (spec §3.1) -- any
// trailing token at all, id-shaped or not, is wrong arity.

void ProtocolHandler::handleHello(char** fields, size_t fieldCount,
                                   const char* lastFieldToken) {
  (void)fields;
  if (fieldCount != 0) { rejectMalformed(lastFieldToken, 2); return; }
  sendBanner();  // spec §4: HELLO's reply is byte-identical to the
                 // unsolicited boot banner
}

void ProtocolHandler::handlePing(char** fields, size_t fieldCount,
                                  const char* lastFieldToken) {
  (void)fields;
  if (fieldCount != 0) { rejectMalformed(lastFieldToken, 2); return; }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "pong %lu\n",
                static_cast<unsigned long>(adapter_.now()));
  writeLine(buf);
}

void ProtocolHandler::handleVer(char** fields, size_t fieldCount,
                                 const char* lastFieldToken) {
  (void)fields;
  if (fieldCount != 0) { rejectMalformed(lastFieldToken, 2); return; }
  Identity identity;
  adapter_.identity(identity);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "ver %s\n", identity.version);
  writeLine(buf);
}

void ProtocolHandler::handleId(char** fields, size_t fieldCount,
                                const char* lastFieldToken) {
  (void)fields;
  if (fieldCount != 0) { rejectMalformed(lastFieldToken, 2); return; }
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "id %s %s %s\n", identity.drivetrain,
                identity.profile, identity.version);
  writeLine(buf);
}

void ProtocolHandler::handleStatus(char** fields, size_t fieldCount,
                                    const char* lastFieldToken) {
  (void)fields;
  if (fieldCount != 0) { rejectMalformed(lastFieldToken, 2); return; }
  StatusFields status;
  adapter_.status(status);
  char buf[160];
  std::snprintf(buf, sizeof(buf),
                "status ready=%d active=%d connL=%d connR=%d otos=%d "
                "wedge=%d flags=%x tlm=%s\n",
                status.ready ? 1 : 0, status.active ? 1 : 0,
                status.connLeft ? 1 : 0, status.connRight ? 1 : 0,
                status.otos ? 1 : 0, status.wedge ? 1 : 0,
                static_cast<unsigned int>(status.flags), status.tlm);
  writeLine(buf);
}

void ProtocolHandler::handleHelp(char** fields, size_t fieldCount,
                                  const char* lastFieldToken) {
  (void)fields;
  if (fieldCount != 0) { rejectMalformed(lastFieldToken, 2); return; }
  // "Generated by walking the verb table at runtime, so it cannot drift
  // from the dispatcher" (spec §4) -- kCommandTable is the SAME table
  // dispatch() looks verbs up in. help's OWN reply is rest-of-line
  // (spec §2) but that only matters to a PARSER reading it back in --
  // this handler only ever emits it, so it is built by hand as a plain
  // space-joined list, same as before the grammar switch.
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
                                 const char* lastFieldToken) {
  if (fieldCount > 1) { rejectMalformed(lastFieldToken, 2); return; }

  char buf[kMaxGetReplyBytes];
  char formatted[32];
  if (fieldCount == 0) {
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
      std::snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
      writeLine(buf);
    }
    return;
  }

  const char* name = fields[0];
  float value = 0.0f;
  // Unknown name: GET never carries an id (`GET | [name]`, spec §3.1),
  // so there is no wire channel to reject it on -- silent (spec §7.1,
  // stated explicitly: "GET with an unknown name is silent -- no
  // reply, and not counted malformed").
  if (!adapter_.onGet(name, value)) return;
  formatConfigValue(value, formatted, sizeof(formatted));
  std::snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
  writeLine(buf);
}

void ProtocolHandler::handleSet(char** fields, size_t fieldCount,
                                 const char* lastFieldToken) {
  if (fieldCount != 2 && fieldCount != 3) {
    rejectMalformed(lastFieldToken, 2);
    return;
  }

  const bool idProvided = (fieldCount == 3);
  uint32_t id = 0;
  IdOutcome idOutcome = idProvided
      ? resolveTrailingOptionalId(fields[2], id)
      : IdOutcome::kOmitted;
  if (idOutcome == IdOutcome::kMalformed) {
    // The 3rd token is present but is not a well-formed `#id` -- SET
    // has no other use for a 3rd positional field, so this is a
    // malformed line, not "a SET with an id-less extra field".
    rejectMalformed(lastFieldToken, 2);
    return;
  }

  const char* name = fields[0];
  float value = 0.0f;
  if (!parseFloatField(fields[1], value)) {
    // The VALUE field itself is malformed -- a handler-level decode
    // failure (spec §7.2: SET's value is decoded by the handler), never
    // reaching onSet(). Still apply the same id-outcome-driven reply
    // shape as the success path, so a typo'd value on an otherwise
    // well-formed SET still gets the ack format its own id calls for.
    ++malformedCount_;
    switch (idOutcome) {
      case IdOutcome::kNonzero: replyErr(id, 2); break;
      case IdOutcome::kOmitted: replyErrBare(2); break;
      case IdOutcome::kZero: break;  // #0 -- no ack wanted, stays silent
      case IdOutcome::kMalformed: break;  // unreachable, handled above
    }
    return;
  }

  Result result = adapter_.onSet(name, value, id);
  switch (idOutcome) {
    case IdOutcome::kZero:
      return;  // executes silently, no ack at all (spec §8.2)
    case IdOutcome::kOmitted:
      if (result == Result::kOk) replyOkBare();
      else replyErrBare(resultCode(result));
      return;
    case IdOutcome::kNonzero:
      if (result == Result::kOk) replyOk(id);
      else replyErr(id, resultCode(result));
      return;
    case IdOutcome::kMalformed:
      return;  // unreachable, handled above
  }
}

// ---- telemetry -------------------------------------------------------------

void ProtocolHandler::handleTlm(char** fields, size_t fieldCount,
                                 const char* lastFieldToken) {
  if (fieldCount != 1) { rejectMalformed(lastFieldToken, 2); return; }
  TlmMode mode;
  if (!parseTlmMode(fields[0], mode)) {
    rejectMalformed(lastFieldToken, 2);
    return;
  }
  // TLM carries no id (spec §3.1) so there is no wire channel to ack or
  // reject it on -- the Result is the adapter's own business (e.g.
  // logging) and never surfaces on the wire.
  (void)adapter_.onTlm(mode);
}

// ---- motion ----------------------------------------------------------------

void ProtocolHandler::handleWheels(char** fields, size_t fieldCount,
                                    const char* lastFieldToken) {
  if (fieldCount != 3 && fieldCount != 4) {
    rejectMalformed(lastFieldToken, 2);
    return;
  }

  const bool idProvided = (fieldCount == 4);
  uint32_t id = 0;
  IdOutcome idOutcome = idProvided
      ? resolveTrailingOptionalId(fields[3], id)
      : IdOutcome::kOmitted;
  if (idOutcome == IdOutcome::kMalformed) {
    rejectMalformed(lastFieldToken, 2);
    return;
  }

  int32_t left = 0, right = 0;
  uint32_t duration = 0;
  if (!parseInt32(fields[0], left) || !parseInt32(fields[1], right) ||
      !parseUint32(fields[2], duration)) {
    ++malformedCount_;
    switch (idOutcome) {
      case IdOutcome::kNonzero: replyErr(id, 2); break;
      case IdOutcome::kOmitted: replyErrBare(2); break;
      case IdOutcome::kZero: break;
      case IdOutcome::kMalformed: break;  // unreachable, handled above
    }
    return;
  }

  // duration's documented "ceiling 5000" (spec §5.2) is NOT enforced
  // here -- see protocol_handler.h's ambiguity note #1. It reaches the
  // adapter untouched.
  Result result = adapter_.onWheels(static_cast<float>(left),
                                     static_cast<float>(right), duration, id);
  switch (idOutcome) {
    case IdOutcome::kZero:
      return;
    case IdOutcome::kOmitted:
      if (result == Result::kOk) replyOkBare();
      else replyErrBare(resultCode(result));
      return;
    case IdOutcome::kNonzero:
      if (result == Result::kOk) replyOk(id);
      else replyErr(id, resultCode(result));
      return;
    case IdOutcome::kMalformed:
      return;  // unreachable, handled above
  }
}

void ProtocolHandler::handleStop(char** fields, size_t fieldCount,
                                  const char* lastFieldToken) {
  // STOP's id is REQUIRED (`STOP #<id>`, spec §3.1 -- no brackets), and
  // it is the verb's ONLY field, so fieldCount must be exactly 1.
  if (fieldCount != 1) { rejectMalformed(lastFieldToken, 2); return; }
  uint32_t id = 0;
  // recoverTrailingId() requires a well-formed AND NONZERO id, which is
  // exactly STOP's own rule: spec §8.2 states "#0 is legal only where
  // the id is optional; on MOVE/GOTO/STOP it is malformed" -- so a
  // literal "STOP #0" is rejected here the same way "STOP notanid" is,
  // and (since the id is what would have been recovered) correctly
  // produces no reply either way.
  if (!recoverTrailingId(fields[0], id)) {
    rejectMalformed(lastFieldToken, 2);
    return;
  }
  Result result = adapter_.onStop(id);
  if (result == Result::kOk) replyOk(id);
  else replyErr(id, resultCode(result));
}

void ProtocolHandler::handleEstop(char** fields, size_t fieldCount,
                                   const char* lastFieldToken) {
  (void)fields;
  (void)lastFieldToken;
  if (fieldCount != 0) {
    // ESTOP is NEVER acked, not even on wrong arity -- spec §5.4/§8.2's
    // own emphatic, repeated "never carries an id and is never acked...
    // must not queue behind anything, including an ack" wins over §2's
    // generic malformed-line id-recovery rule (protocol_handler.h's
    // ambiguity note #2). Deliberately does NOT call rejectMalformed().
    ++malformedCount_;
    return;
  }
  adapter_.onEstop();
  // No reply, ever: spec §8.2 -- ESTOP never carries an id and is never
  // acked, so it can never queue behind anything, including an ack.
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
}

}  // namespace Protocol

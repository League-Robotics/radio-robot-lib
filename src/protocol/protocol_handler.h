// protocol_handler.h — Protocol::ProtocolHandler: the ASCII line-grammar
// codec (docs/design/protocol.md §2-§6) behind the Sink/Adapter seams
// that same document's §1/§3 defines, PLUS the reliability layer
// (docs/design/protocol.md §8, 2026-08-21) — mandatory sequence ids and
// cumulative ack/nack. This is the ONLY class in this library that ever
// touches a wire byte: feed() reassembles arbitrary byte blocks into
// '\n'-terminated lines, tokenizes each line in place on runs of ' '
// (§2/§3.2 — no allocation, no std::string, no exceptions), sequence-
// checks its mandatory trailing id (§8), dispatches to the Adapter, and
// formats the reply — once, per verb, so the Adapter can neither forget
// a reply nor invent a shape for one.
//
// No kernel, no motors, no config storage, no transport, and (deliberately
// — §8.1) NO CLOCK: bytes in via feed(), bytes out via Sink, with exactly
// three small pieces of state between calls (expectedNext_, lastDone_,
// gapOutstanding_) plus the pre-existing partial-line buffer and
// malformed counter.
//
// ---- The grammar (docs/design/protocol.md §2), in one line ----
//
//   line   ::= sp? verb ( sp field )* sp? '\n'
//   sp     ::= ' '+
//   verb   ::= [A-Za-z][A-Za-z0-9_]*
//   field  ::= any bytes except ' ' and '\n'
//   id     ::= '#' [0-9]+        (a field in trailing position, §8.2)
//
// A run of spaces is ONE separator; leading/trailing whitespace on the
// line is ignored; a blank or all-whitespace line is ignored SILENTLY
// (not malformed). This was a colon-delimited, positional grammar before
// the 2026-08-20 stakeholder decision (commit 5a5b6da); this file is the
// post-cutover rewrite. See docs/design/protocol.md §9.1/§9.6 for that
// resolution history.
//
// ---- The reliability layer (docs/design/protocol.md §8, 2026-08-21) ----
//
// Every sequenced verb (PING ID VER STATUS HELP GET SET TLM WHEELS STOP
// RUN) now carries a MANDATORY trailing id that doubles as a strictly
// incrementing sequence number, starting at 1. HELLO and ESTOP are the
// two exceptions (§8.3) — HELLO because it resets the sequence and so
// cannot itself be inside it, ESTOP because it is safety-critical and
// must execute even while the stream is stalled on a gap.
//
// dispatch() resolves the id FIRST, against the handler's own
// expectedNext_ counter, BEFORE the verb is even looked up:
//   - no id at all, or a malformed one -> the line cannot be classified;
//     malformedCount() increments, no reply (§8.4 items 1-2).
//   - id < expectedNext_  -> a stale retransmit. NOT re-executed. Replies
//     ack(expectedNext_ - 1, lastDone_) -- the already-accepted id, not
//     the resent one, so the host learns "I already have this."
//   - id > expectedNext_  -> a gap. NOT executed, and the verb is not
//     even looked up. Replies nack(expectedNext_, lastDone_), and sets
//     gapOutstanding_ so emitTelemetry() (§8.5) keeps re-nacking for free
//     until the missing id arrives.
//   - id == expectedNext_ -> in order. expectedNext_ advances past it,
//     ack(id, lastDone_) is sent UNCONDITIONALLY, and only THEN is the
//     verb looked up and its own fields validated -- an unrecognized verb
//     or a decode failure at this point still emits ack, PLUS a
//     following err (docs/design/protocol.md §8.2/§8.4 item 1: "arrived
//     fine, content rejected" is one uniform idea, whether the rejection
//     came from the handler's own parsing or the adapter's Result).
//
// `ok` and standalone `done` are GONE (§6.1, §8.2): an in-order ack IS
// the acceptance signal; every per-verb handler below, on success, emits
// nothing further at all. `err` always carries an id now (`err <code>
// #<id>`, id last -- §8.6's field-order fix) because it is only ever
// reached after an id has already been sequence-validated.
//
// `#0` is not special-cased anywhere in this file. Since expectedNext_
// starts at (and never goes below) 1, an inbound `#0` is unconditionally
// `< expectedNext_` and falls into the ordinary stale-retransmit bucket
// with zero extra code (§2.2, §9.8 item in the design doc).
//
// ---- Ambiguities/design calls this file DOES still have to make ----
//
// 1. WHEELS's documented "ceiling 5000" is stated in prose at the
//    verb-definition level, not in the Adapter interface
//    (docs/design/protocol.md §4) or anywhere this handler owns a
//    bounds table for. Per §7's "the handler holds no field table, no
//    bounds, no storage" — generalized here from config bounds to
//    motion bounds for consistency — this handler does NOT enforce the
//    ceiling itself; it passes `duration` through unchecked and leaves
//    the ceiling to whatever adapter a future step supplies (its
//    `onWheels` can return kRange). Flagged, not silently assumed.
//    Unaffected by the grammar migration or the reliability layer;
//    carried forward verbatim.
//
// 2. The id's own numeric grammar (`id ::= '#' [0-9]+`) is STRICTER
//    than the general "every wire value is a base-10 ASCII integer,
//    optionally signed" rule for ordinary integer fields (WHEELS's
//    `duration`, etc.): the id grammar allows ONLY decimal digits after
//    the `#`, no sign at all — not even a leading `+`, which C's
//    strtoul() would otherwise accept as valid syntax. This file parses
//    ids with a dedicated digit-only scan (parseIdDigits() in the .cpp)
//    rather than reusing the general unsigned-field parser,
//    specifically so `#+5` is rejected as not-an-id.
//
// 3. RUN (docs/design/protocol.md's RUN section) has NO fixed arity —
//    unlike every other verb in kCommandTable, the number of DATA fields
//    (after the mandatory id is stripped) is open-ended (however many
//    arguments the target function takes). Every OTHER handler's arity
//    check compares a data-field count against a small FIXED constant
//    that is always well inside kMaxFieldTokens, so fields[] is always
//    known to hold a real pointer for every index that check ever
//    touches. handleRun() alone has to check its own data-field count
//    against kMaxFieldTokens itself, BEFORE indexing fields[] at all, or
//    an adversarial line with more real tokens than fields[] has room to
//    store pointers for would read an uninitialized array slot. See
//    handleRun()'s own comment in the .cpp for the full resolution.
//
// 4. See docs/design/protocol.md §9.8 for eight further ambiguities the
//    reliability layer itself raised (whether a handler-level decode
//    failure still advances the sequence, whether an out-of-order line
//    counts as malformed, what lastDone_ tracks in a queueless library,
//    reply ordering, telemetry-line ordering, HELLO's own malformed-line
//    reply, and ERR_DUPLICATE_ID's new unreachability) — not repeated
//    here to avoid drifting out of sync with the doc's own numbering.
#pragma once

#include <cstddef>
#include <cstdint>

#include "adapter.h"

namespace Protocol {

// Sink — where finished reply lines go. Exactly one write() per
// formatted line, INCLUDING the trailing '\n'; the caller owns
// transport (serial, radio, UDP, or a test's recording buffer).
class Sink {
 public:
  virtual ~Sink() = default;
  virtual void write(const char* data, size_t length) = 0;
};

class ProtocolHandler {
 public:
  // Wire line ceiling, spec §2: "Max line: 240 bytes including the
  // terminator." The handler's buffer sizes off this constant so there
  // is exactly one place that number is spelled.
  static constexpr size_t kMaxLineBytes = 240;

  ProtocolHandler(Adapter& adapter, Sink& sink);

  // Feed an arbitrary block from the port — may contain zero, one, or
  // several complete lines, and may end mid-line. Partial lines are
  // buffered across calls; complete lines are parsed and dispatched
  // immediately, in the order they complete. Must survive (spec §2,
  // docs/design/protocol.md §2.1):
  //   - several complete lines in one block;
  //   - a block ending mid-line (the remainder is buffered);
  //   - a block that is only a line fragment;
  //   - a lone '\r' immediately before '\n' (stripped; '\r' never
  //     appears anywhere else);
  //   - a blank or all-whitespace line (ignored silently, spec §2 —
  //     NOT counted malformed);
  //   - a line longer than the 240-byte maximum: discarded to the next
  //     '\n' and counted malformed — NEVER truncated into a prefix that
  //     might still parse as a command the host never sent.
  void feed(const char* data, size_t length);

  // Unsolicited emissions the app drives, not the wire (spec §4).
  void sendBanner();  // device NEZHA2 robot <name> <serial>
  void sendReady();   // ready

  // debug: robot-to-host ONLY -- there is no inbound wire form of this
  // verb at all (an inbound lowercase-led line is another robot's reply
  // on a shared channel and is dropped silently, spec §2.1 -- the same
  // mechanism that structurally closed the v5 DBG:-flood incident this
  // verb is named after). Emits "debug <text>\n", a rest-of-line verb
  // exactly like `help`'s own reply shape. Entirely unaffected by the
  // reliability layer -- debug never carries an id and never will.
  //
  // Design calls made here, not left to the caller:
  //   - `text == nullptr` and `text == ""` are the SAME case: both emit
  //     the bare line "debug\n" (no trailing space before the
  //     terminator, matching the wire grammar's own "an empty token
  //     cannot exist between spaces" rule -- see golden_vectors.txt).
  //   - Every '\n'/'\r' byte in `text` is STRIPPED, not rejected. This
  //     method is void with no channel to report a rejection through
  //     (unlike SET/WHEELS/RUN, sendDebug's caller gets no Result back),
  //     so silently discarding the WHOLE message over one bad byte
  //     would lose strictly more information than delivering everything
  //     else in it. The alternative (drop the whole call) was
  //     considered and rejected for exactly that reason.
  //   - The whole line (verb + space + text + terminator) is truncated,
  //     never overflowed, to fit kMaxLineBytes -- the same posture
  //     feed() itself takes on an overlong INBOUND line (§3.1).
  void sendDebug(const char* text);

  // thdr: once, on the first call and again whenever the column set
  // changes (spec §6.2); t: every call; THEN the current reliability
  // line -- nack(expectedNext_, lastDone_) if a gap is outstanding,
  // ack(expectedNext_ - 1, lastDone_) otherwise (docs/design/protocol.md
  // §8.5). This is the ONLY periodic entry point in this class, and it
  // rides a cadence the CALLER drives -- the handler itself still owns
  // no timer and no clock.
  void emitTelemetry(const Snapshot& snapshot);

  // Lines dropped as unknown verb, wrong arity, or an unparseable field
  // (spec §2's malformed counter, flags bit 9), OR a sequenced verb
  // whose id could not be determined at all (docs/design/protocol.md
  // §8.4 items 1-2). A lowercase-led inbound verb — another robot's
  // reply on a shared channel, spec §2.1 — is dropped silently and does
  // NOT increment this. Neither does a blank/all-whitespace line, nor a
  // well-formed-but-OUT-OF-ORDER sequenced command (§8's own "not
  // malformed, just out of sequence" distinction, docs/design/
  // protocol.md §9.8).
  uint32_t malformedCount() const { return malformedCount_; }

 private:
  // A per-verb handler receives:
  //   fields          — pointers to the verb's own DATA field tokens,
  //                      with the mandatory trailing id already
  //                      resolved and stripped by dispatch() before the
  //                      handler is ever called (docs/design/
  //                      protocol.md §8.4) -- unlike the pre-2026-08-21
  //                      shape, no handler here inspects an id-shaped
  //                      trailing token itself any more.
  //   fieldCount       — the TRUE number of DATA field tokens (id
  //                      excluded) the line had after the verb (may
  //                      exceed the fields[] array's own storage cap —
  //                      see tokenizeLine() in the .cpp for why an
  //                      arity check on this is still correct even past
  //                      that cap).
  //   id               — the already-validated, already in-sequence id
  //                      (never 0 in a build where #0 always falls into
  //                      the stale-retransmit bucket before a handler is
  //                      ever reached -- see the file header above).
  using VerbHandler = void (ProtocolHandler::*)(char** fields,
                                                 size_t fieldCount,
                                                 uint32_t id);
  struct VerbEntry {
    const char* name;
    VerbHandler handler;
  };

  static const VerbEntry kCommandTable[13];

  // Field-token storage cap for one line, verb-exclusive (id excluded --
  // see dispatch()'s own comment for where the id is resolved). Every
  // FIXED-arity verb's largest declared arity is WHEELS's 3 data fields
  // (left, right, duration) -- comfortably inside this cap, so a line
  // with more real tokens than THEIR arity is always wrong-arity
  // regardless of storage, and capping storage here never turns a
  // truly-too-long line into a falsely-accepted one for any of them
  // (see tokenizeLine()'s own comment). RUN is the one exception: its
  // arity is open-ended (however many arguments its target function
  // takes), so this cap doubles as RUN's own hard ceiling on how many
  // raw DATA tokens it will trust fields[] to hold pointers for at all --
  // handleRun() checks its own fieldCount against this constant BEFORE
  // indexing fields[] (protocol_handler.h's own file-header ambiguity
  // note #3), and kMaxRunArgs below is deliberately smaller still,
  // leaving room for RUN's own function-name field inside this same
  // budget.
  static constexpr size_t kMaxFieldTokens = 20;

  // RUN's own ceiling on how many ARGUMENTS (excluding the function
  // name) it will forward to onRun() -- a firmware resource limit (the
  // fixed argv[] array handleRun() builds on the stack), not a claim
  // about any real function's arity. A line with more real arguments
  // than this is rejected (ack + err) before onRun() is ever called.
  // Deliberately well under kMaxFieldTokens - 1 (which reserves room for
  // the function-name field in the same fieldCount budget), so the two
  // constants can never disagree about how many pointers fields[]
  // actually holds.
  static constexpr size_t kMaxRunArgs = 16;

  // RUN's stringified return value (docs/design/protocol.md's RUN
  // section) -- an ARRAY SIZE (content bytes plus the NUL terminator),
  // sized so the WHOLE reply line -- "ret " + this text + " #<id>" at
  // id's maximum width (a 10-digit uint32_t) + '\n' -- can never exceed
  // kMaxLineBytes (which itself already counts the terminator, hence no
  // separate "-1" here: the NUL this array reserves and the '\n'
  // kMaxLineBytes reserves net out) even before handleRun()'s own
  // sanitize pass, which can only shrink the text further (stripping
  // '\n'/'\r', the same rule sendDebug()'s text gets), never grow it.
  // The id suffix is no longer optional (§8), but the budget already
  // reserved room for it unconditionally, so the constant is unchanged.
  static constexpr size_t kMaxRunResultBytes =
      kMaxLineBytes - 4 /* "ret " */ - 12 /* " #4294967295" */;

  // sendDebug()'s own text budget, same accounting as
  // kMaxRunResultBytes above: "debug " + this text + '\n' must fit
  // kMaxLineBytes. No id suffix to reserve room for -- debug is
  // robot-to-host only and never carries one -- so this is a larger
  // budget than kMaxRunResultBytes even though both share the same
  // "verb + one free-text field, capped at kMaxLineBytes" shape.
  static constexpr size_t kMaxDebugTextBytes = kMaxLineBytes - 6 /* "debug " */;

  static constexpr size_t kMaxHeaderColumns = 40;
  static constexpr size_t kMaxHeaderNameBytes = 16;
  static constexpr size_t kMaxTelemetryLineBytes = 256;
  // GET's echoed-back field name is WIRE-CONTROLLED (spec S2's own
  // 240-byte line cap is the only bound on it, unlike identity/status
  // strings which are adapter-owned and short by construction) -- the
  // reply buffer must be sized off kMaxLineBytes, not a small fixed
  // guess, or a long-but-legal GET name truncates its own reply and
  // drops the trailing '\n'.
  static constexpr size_t kMaxGetReplyBytes = kMaxLineBytes * 2;

  // ---- feed() / line reassembly ----
  void appendByte(char c);
  void onLineComplete();

  // ---- tokenizing (spec §2, §11.1) ----
  // Splits `line` (already NUL-terminated) into tokens in place on runs
  // of ' ', collapsing separators and ignoring leading/trailing space.
  // Returns the TRUE total token count (verb included), which may
  // exceed `maxTokens` — only the first `maxTokens` pointers are
  // stored, matching every per-verb arity check's own "count vs a
  // small legitimate maximum" comparison (see kMaxFieldTokens above).
  static size_t tokenizeLine(char* line, char** tokens, size_t maxTokens);

  // ---- dispatch / the reliability layer (docs/design/protocol.md §8) ----
  // Resolves the mandatory trailing id against expectedNext_ BEFORE the
  // verb is even looked up (ESTOP/HELLO excepted, handled first and
  // never sequenced at all -- see the file header). See the .cpp for
  // the full state machine.
  void dispatch(char* verb, char** fields, size_t fieldCount,
                const char* lastFieldToken);
  void replyAck(uint32_t ackedId);    // "ack <ackedId> <lastDone_>\n"
  void replyNack(uint32_t nextId);    // "nack <nextId> <lastDone_>\n"
  void replyErr(uint32_t id, uint8_t code);  // "err <code> #<id>\n"
  void writeLine(const char* text);  // one Sink::write() per line
  static uint8_t resultCode(Result result);

  // ---- per-verb handlers -- see VerbHandler's own comment above for
  // the (fields, fieldCount, id) contract every one shares. Every
  // handler here is called ONLY after dispatch() has already sent the
  // in-order ack (docs/design/protocol.md §8.2) -- on success, none of
  // these emit anything further; on failure, each calls replyErr(id,
  // code) exactly once.
  void handleHello(char** fields, size_t fieldCount, uint32_t id);
  void handlePing(char** fields, size_t fieldCount, uint32_t id);
  void handleVer(char** fields, size_t fieldCount, uint32_t id);
  void handleId(char** fields, size_t fieldCount, uint32_t id);
  void handleStatus(char** fields, size_t fieldCount, uint32_t id);
  void handleHelp(char** fields, size_t fieldCount, uint32_t id);
  void handleGet(char** fields, size_t fieldCount, uint32_t id);
  void handleSet(char** fields, size_t fieldCount, uint32_t id);
  void handleTlm(char** fields, size_t fieldCount, uint32_t id);
  void handleWheels(char** fields, size_t fieldCount, uint32_t id);
  void handleStop(char** fields, size_t fieldCount, uint32_t id);
  void handleEstop(char** fields, size_t fieldCount, uint32_t id);
  void handleRun(char** fields, size_t fieldCount, uint32_t id);

  // ---- telemetry header change detection ----
  bool headerChanged(const Snapshot& snapshot) const;
  void rememberHeader(const Snapshot& snapshot);
  void emitHeader(const Snapshot& snapshot);
  void emitFrame(const Snapshot& snapshot);

  Adapter& adapter_;
  Sink& sink_;

  char lineBuf_[kMaxLineBytes] = {};
  size_t lineLen_ = 0;
  bool overflowing_ = false;
  uint32_t malformedCount_ = 0;

  // ---- the reliability layer's own state (docs/design/protocol.md §8.1)
  // -- exactly these three fields, and deliberately NO clock/timer. ----
  uint32_t expectedNext_ = 1;       // next sequence id expected from the host
  uint32_t lastDone_ = 0;           // most recent completed motion id (§8.5.1
                                     // -- always 0 in this library; see there)
  bool gapOutstanding_ = false;     // a nack is currently owed (§8.5)

  // Last-emitted thdr: column set (copied, not borrowed — see
  // rememberHeader() in the .cpp for why a copy is worth the fixed
  // memory: it removes any assumption about how long a caller's own
  // Snapshot storage stays valid between calls).
  char headerNames_[kMaxHeaderColumns][kMaxHeaderNameBytes] = {};
  bool headerHex_[kMaxHeaderColumns] = {};
  size_t headerCount_ = 0;
  bool everEmittedHeader_ = false;
};

}  // namespace Protocol

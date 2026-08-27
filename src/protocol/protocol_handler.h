// protocol_handler.h — Protocol::ProtocolHandler: the ASCII line-grammar
// codec (docs/design/protocol.md §2-§7) behind the Sink/Adapter seams
// that same document's §1/§3 defines, PLUS the reliability layer
// (docs/design/protocol.md §8, 2026-08-21/22) — mandatory sequence ids,
// cumulative ack/nack, and the six stakeholder-directed changes of
// 2026-08-22 (§8.8). This is the ONLY class in this library that ever
// touches a wire byte: feed() reassembles arbitrary byte blocks into
// '\n'-terminated lines, tokenizes each line in place on runs of ' '
// (§2/§3.2 — no allocation, no std::string, no exceptions), sequence-
// checks its mandatory trailing id (§8), dispatches to the Adapter, and
// formats the reply — once, per verb, so the Adapter can neither forget
// a reply nor invent a shape for one.
//
// No kernel, no motors, no config storage, no transport, and (deliberately
// — §8.1) NO CLOCK: bytes in via feed(), bytes out via Sink, with exactly
// one small piece of state between calls (expectedNext_ — gapOutstanding_
// is GONE, 2026-08-26, §8.5: its only reader was the deleted telemetry
// ack piggyback) plus the pre-existing partial-line buffer and malformed
// counter.
// `lastDone_` is GONE from this class (2026-08-22) — see adapter.h's own
// `lastDone()`/`lastDoneReason()` doc comment for where it moved and why.
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
// (not malformed).
//
// ---- The reliability layer (docs/design/protocol.md §8) ----
//
// Every sequenced verb (ID VER STATUS HELP GET SET TLM WHEELS_X WHEELS_V
// MOVE_X MOVE_V GO_TO_R GO_TO_W STOP RUN) carries a MANDATORY trailing id
// that doubles as a strictly incrementing sequence number, starting at
// 1. HELLO, ESTOP, and (as of 2026-08-22) PING are the three exceptions
// (§8.3) — HELLO because it resets the sequence and so cannot itself be
// inside it, ESTOP because it is safety-critical and must execute even
// while the stream is stalled on a gap, and PING because it is the
// liveness probe and must answer even while the stream is stalled on a
// gap (the same reasoning as ESTOP, extended 2026-08-22 per stakeholder
// direction).
//
// dispatch() resolves the id FIRST, against the handler's own
// expectedNext_ counter, BEFORE the verb's own fields are ever decoded:
//   - no id at all, or a malformed one -> the line cannot be classified;
//     malformedCount() increments, no reply (§8.4 items 1-2).
//   - id < expectedNext_  -> a stale retransmit. NOT re-executed. Replies
//     ack(expectedNext_ - 1) -- the already-accepted id, not the resent
//     one, so the host learns "I already have this."
//   - id > expectedNext_  -> a gap. NOT executed, and the verb is not
//     even looked up. Replies nack(expectedNext_); every further inbound
//     line re-triggers the same nack until the missing id arrives (§8.1)
//     -- there is no periodic re-nack (2026-08-26, §8.5).
//   - id == expectedNext_ -> the verb is looked up and its OWN fields
//     are decoded (arity + per-field parseability) BEFORE any reply is
//     sent at all (2026-08-22, docs/design/protocol.md §8.9 -- this is
//     the behavior change from before, see the note below):
//       - unrecognized verb, wrong arity, or an unparseable field ("a
//         DECODE FAILURE" -- the line did not arrive intact) -> the
//         sequence does NOT advance; replies nack(expectedNext_)
//         (still the SAME id, since it was never accepted) AND
//         err(id, code); malformedCount() increments; every further
//         inbound line keeps re-nacking until the SAME id arrives
//         well-formed.
//       - decoded fine (arity and every field parse), but the ADAPTER
//         refuses it on merit (e.g. an out-of-range speed) -> the
//         sequence ADVANCES (it arrived intact); replies ack(id) AND
//         err(id, code).
//       - decoded fine and the adapter accepts -> the sequence
//         advances; replies ack(id) alone.
//
// **2026-08-22 behavior change (the "decode failure is a NAK"
// direction, docs/design/protocol.md §8.9):** before this change, EVERY
// in-order id advanced the sequence and got an ack, with an unknown verb
// or bad arity/field only adding a following err on top -- so a garbled
// line and a merits-rejected line were wire-indistinguishable except by
// error code. Now they are distinguished by transport reply: a decode
// failure nacks and holds the sequence in place (the host must resend
// THAT SAME line); a merits rejection acks and moves on (resending would
// just be refused again, identically). See docs/design/protocol.md §8.9
// for the stakeholder's own rationale (a dropped/garbled turn in a
// square-tour sequence must not silently let the rest of the tour run
// out of order) and its stated hazard (a host that genuinely
// CONSTRUCTS a malformed line is nacked forever -- the host needs its
// own give-up/reconnect path; this handler does not supply one).
//
// `ok` and standalone `done` remain GONE (§8.2): an in-order ack IS the
// acceptance signal; every per-verb handler below, on success, emits
// nothing further at all. `err` always carries an id now (`err <code>
// #<id>`, id last -- §8.6's field-order fix).
//
// `#0` is not special-cased anywhere in this file. Since expectedNext_
// starts at (and never goes below) 1, an inbound `#0` is unconditionally
// `< expectedNext_` and falls into the ordinary stale-retransmit bucket
// with zero extra code (§2.2).
//
// ---- Ambiguities/design calls this file DOES still have to make ----
//
// 1. WHEELS_X/WHEELS_V's documented duration/timeout ceilings are stated
//    in prose at the verb-definition level, not in the Adapter interface
//    (docs/design/protocol.md §4) or anywhere this handler owns a
//    bounds table for. Per §7's "the handler holds no field table, no
//    bounds, no storage" this handler does NOT enforce any such ceiling
//    itself; it passes the decoded fields through unchecked and leaves
//    bounds enforcement to whichever adapter is wired in (its onWheelsV/
//    onWheelsX/etc. can return kRange). Unaffected by the 2026-08-22
//    changes; carried forward verbatim from the WHEELS-era design.
//
// 2. The id's own numeric grammar (`id ::= '#' [0-9]+`) is STRICTER
//    than the general "every wire value is a base-10 ASCII integer,
//    optionally signed" rule for ordinary integer fields: the id
//    grammar allows ONLY decimal digits after the `#`, no sign at all —
//    not even a leading `+`, which C's strtoul() would otherwise accept
//    as valid syntax. This file parses ids with a dedicated digit-only
//    scan (parseIdDigits() in the .cpp) rather than reusing the general
//    unsigned-field parser, specifically so `#+5` is rejected as
//    not-an-id.
//
// 3. RUN (docs/design/protocol.md's RUN section) has NO fixed arity —
//    unlike every other verb in kCommandTable, the number of DATA fields
//    (after the mandatory id is stripped) is open-ended (however many
//    arguments the target function takes). handleRun()'s own decode
//    step checks its own data-field count against kMaxFieldTokens
//    itself, BEFORE indexing fields[] at all -- see decodeRun()'s own
//    comment in the .cpp for the full resolution.
//
// 4. PING joining the unsequenced exemption set (2026-08-22, stakeholder
//    direction: "ESTOP, ping, and HELLO shouldn't require IDs") is
//    THIS FILE'S OWN CALL on one point the direction did not spell out:
//    whether PING should be MAXIMALLY FORGIVING of trailing content
//    (like ESTOP) or STRICT zero-arity (like HELLO). Resolved: maximally
//    forgiving, matching ESTOP -- see handlePing()'s own comment in the
//    .cpp for the reasoning (liveness must work even for a host that
//    still appends `#<id>` to PING out of habit from before this
//    change).
//
// 5. See docs/design/protocol.md §8.9/§9 for further ambiguities the
//    2026-08-22 changes raised (whether HELLO's reset still touches
//    lastDone_ now that it lives on the Adapter, the `ack`/`nack`
//    reason-piggyback shape, DiffDriveAdapter's kUnknown-vs-
//    kUnimplemented choice for the five unimplemented motion verbs) —
//    not repeated here to avoid drifting out of sync with the doc's own
//    numbering.
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
  // on a shared channel and is dropped silently, spec §2.1). Emits
  // "debug <text>\n", a rest-of-line verb exactly like `help`'s own
  // reply shape. Entirely unaffected by the reliability layer -- debug
  // never carries an id and never will.
  //
  // Design calls made here, not left to the caller:
  //   - `text == nullptr` and `text == ""` are the SAME case: both emit
  //     the bare line "debug\n" (no trailing space before the
  //     terminator).
  //   - Every '\n'/'\r' byte in `text` is STRIPPED, not rejected (this
  //     method is void with no channel to report a rejection through).
  //   - The whole line is truncated, never overflowed, to fit
  //     kMaxLineBytes.
  void sendDebug(const char* text);

  // thdr: once, on the first call and again whenever the column set
  // changes (docs/design/protocol.md §10.2: `thdr` / `t` — the frame is
  // self-describing); t: every call. NOTHING ELSE (2026-08-26,
  // docs/design/protocol.md §8.5): the reliability line that used to
  // ride every call is deleted -- an ack/nack is only ever a direct
  // reply to an inbound sequenced line, never a beacon. This rides a
  // cadence the CALLER drives -- the handler itself still owns no timer
  // and no clock.
  void emitTelemetry(const Snapshot& snapshot);

  // Lines dropped as unknown verb, wrong arity, or an unparseable field
  // (spec §2's malformed counter), OR a sequenced verb whose id could
  // not be determined at all (docs/design/protocol.md §8.4 items 1-2).
  // A lowercase-led inbound verb — another robot's reply on a shared
  // channel, spec §2.1 — is dropped silently and does NOT increment
  // this. Neither does a blank/all-whitespace line, nor a well-formed-
  // but-OUT-OF-ORDER sequenced command (a numeric gap, as opposed to a
  // decode failure on an IN-ORDER id, which DOES still increment this —
  // see the file header's 2026-08-22 behavior-change note).
  uint32_t malformedCount() const { return malformedCount_; }

 private:
  // Every DECODE function is pure: no adapter call, no sink write, no
  // mutation of handler state. It answers exactly one question -- "does
  // this line's own content parse?" -- so dispatch() can decide
  // ack-vs-nack BEFORE anything with a wire or Adapter side effect runs
  // (docs/design/protocol.md §8.9). Returns false (a DECODE FAILURE) for
  // wrong arity or an unparseable field; true otherwise. Re-parses
  // fields the corresponding EXECUTE function will parse again a moment
  // later -- a deliberate, cheap duplication (this is not a hot path)
  // that keeps "what counts as decodable" defined in exactly one place
  // per verb without threading decoded values across two calls.
  using DecodeFn = bool (ProtocolHandler::*)(char** fields, size_t fieldCount);

  // Every EXECUTE function runs ONLY after dispatch() has already
  // decided the line decodes AND has already sent the `ack` for it — so
  // an execute function is free to write informational reply lines
  // (get/id/ver/help/status/ret) directly to the sink; nothing it does
  // can race the ack that must precede those lines on the wire. It
  // reports any ADAPTER-level (merits) rejection through `errCode`
  // (0 == kOk == no err line; nonzero == the wire code dispatch() will
  // emit as `err <errCode> #<id>` right after whatever this function
  // itself already wrote).
  using ExecuteFn = void (ProtocolHandler::*)(char** fields, size_t fieldCount,
                                               uint32_t id, uint8_t& errCode);

  struct VerbEntry {
    const char* name;
    DecodeFn decode;
    ExecuteFn execute;
  };

  static const VerbEntry kCommandTable[18];

  // Field-token storage cap for one line, verb-exclusive (id excluded --
  // see dispatch()'s own comment for where the id is resolved). Every
  // FIXED-arity verb's largest declared arity is GO_TO_R/GO_TO_W's 5
  // data fields (x, y, speed, arrive, timeout) -- comfortably inside
  // this cap. RUN is the one exception: its arity is open-ended, so this
  // cap doubles as RUN's own hard ceiling on how many raw DATA tokens it
  // will trust fields[] to hold pointers for at all -- decodeRun() checks
  // its own fieldCount against this constant BEFORE indexing fields[]
  // (ambiguity note #3 above), and kMaxRunArgs below is deliberately
  // smaller still, leaving room for RUN's own function-name field inside
  // this same budget.
  static constexpr size_t kMaxFieldTokens = 20;

  // RUN's own ceiling on how many ARGUMENTS (excluding the function
  // name) it will forward to onRun() -- a firmware resource limit (the
  // fixed argv[] array handleRun() builds on the stack), not a claim
  // about any real function's arity. A line with more real arguments
  // than this is rejected before onRun() is ever called.
  static constexpr size_t kMaxRunArgs = 16;

  // RUN's stringified return value (docs/design/protocol.md's RUN
  // section) -- an ARRAY SIZE (content bytes plus the NUL terminator),
  // sized so the WHOLE reply line -- "ret " + this text + " #<id>" at
  // id's maximum width (a 10-digit uint32_t) + '\n' -- can never exceed
  // kMaxLineBytes even before handleRun()'s own sanitize pass, which can
  // only shrink the text further, never grow it.
  static constexpr size_t kMaxRunResultBytes =
      kMaxLineBytes - 4 /* "ret " */ - 12 /* " #4294967295" */;

  // sendDebug()'s own text budget, same accounting as
  // kMaxRunResultBytes above: "debug " + this text + '\n' must fit
  // kMaxLineBytes. No id suffix to reserve room for -- debug is
  // robot-to-host only and never carries one.
  static constexpr size_t kMaxDebugTextBytes = kMaxLineBytes - 6 /* "debug " */;

  static constexpr size_t kMaxHeaderColumns = 40;
  static constexpr size_t kMaxHeaderNameBytes = 16;
  static constexpr size_t kMaxTelemetryLineBytes = 256;
  // GET's echoed-back field name is WIRE-CONTROLLED (spec §2's own
  // 240-byte line cap is the only bound on it, unlike identity/status
  // strings which are adapter-owned and short by construction) -- the
  // reply buffer must be sized off kMaxLineBytes, not a small fixed
  // guess, or a long-but-legal GET name truncates its own reply and
  // drops the trailing '\n'.
  static constexpr size_t kMaxGetReplyBytes = kMaxLineBytes * 2;

  // ---- feed() / line reassembly ----
  void appendByte(char c);
  void onLineComplete();

  // ---- tokenizing (spec §2, §3.2) ----
  // Splits `line` (already NUL-terminated) into tokens in place on runs
  // of ' ', collapsing separators and ignoring leading/trailing space.
  // Returns the TRUE total token count (verb included), which may
  // exceed `maxTokens` — only the first `maxTokens` pointers are
  // stored, matching every per-verb arity check's own "count vs a
  // small legitimate maximum" comparison (see kMaxFieldTokens above).
  static size_t tokenizeLine(char* line, char** tokens, size_t maxTokens);

  // ---- dispatch / the reliability layer (docs/design/protocol.md §8) ----
  // Resolves the mandatory trailing id against expectedNext_, decodes
  // the verb's own fields BEFORE sending any reply, and only then
  // decides ack-vs-nack (ESTOP/HELLO/PING excepted, handled first and
  // never sequenced at all -- see the file header). See the .cpp for
  // the full state machine.
  void dispatch(char* verb, char** fields, size_t fieldCount,
                const char* lastFieldToken);
  // A DECODE FAILURE on an in-order id: the sequence does NOT advance
  // (`code` and `id` cite the SAME id, unchanged from expectedNext_).
  void handleDecodeFailure(uint32_t id, uint8_t code);
  void replyAck(uint32_t ackedId);    // "ack <ackedId> <lastDone> <reason>\n"
  void replyNack(uint32_t nextId);    // "nack <nextId> <lastDone> <reason>\n"
  void replyErr(uint32_t id, uint8_t code);  // "err <code> #<id>\n"
  void writeLine(const char* text);  // one Sink::write() per line
  static uint8_t resultCode(Result result);
  static const char* doneReasonWireName(DoneReason reason);

  // ---- three verbs OUTSIDE the sequence entirely (docs/design/
  // protocol.md §8.3): no id, never nacked. Called DIRECTLY by
  // dispatch() before any id is even looked at; also present in
  // kCommandTable (with trivial decode/execute stand-ins that are never
  // actually invoked through the table) purely so HELP's generated
  // listing includes their names.
  void handleHello(char** fields, size_t fieldCount);
  void handlePing();
  void handleEstop();

  // ---- per-verb decode/execute pairs -- see DecodeFn/ExecuteFn's own
  // comments above for the shared contract. ----
  bool decodeNoFields(char** fields, size_t fieldCount);
  void execVer(char** fields, size_t fieldCount, uint32_t id, uint8_t& errCode);
  void execId(char** fields, size_t fieldCount, uint32_t id, uint8_t& errCode);
  void execStatus(char** fields, size_t fieldCount, uint32_t id,
                  uint8_t& errCode);
  void execHelp(char** fields, size_t fieldCount, uint32_t id,
               uint8_t& errCode);

  bool decodeGet(char** fields, size_t fieldCount);
  void execGet(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  bool decodeSet(char** fields, size_t fieldCount);
  void execSet(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  bool decodeTlm(char** fields, size_t fieldCount);
  void execTlm(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  bool decodeWheelsX(char** fields, size_t fieldCount);
  void execWheelsX(char** fields, size_t fieldCount, uint32_t id,
                   uint8_t& errCode);
  bool decodeWheelsV(char** fields, size_t fieldCount);
  void execWheelsV(char** fields, size_t fieldCount, uint32_t id,
                   uint8_t& errCode);
  bool decodeMoveX(char** fields, size_t fieldCount);
  void execMoveX(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);
  bool decodeMoveV(char** fields, size_t fieldCount);
  void execMoveV(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);
  bool decodeGoToR(char** fields, size_t fieldCount);
  void execGoToR(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);
  bool decodeGoToW(char** fields, size_t fieldCount);
  void execGoToW(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);

  bool decodeStop(char** fields, size_t fieldCount);
  void execStop(char** fields, size_t fieldCount, uint32_t id,
               uint8_t& errCode);

  bool decodeRun(char** fields, size_t fieldCount);
  void execRun(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  // Trivial stand-ins for HELLO/PING/ESTOP's own kCommandTable rows
  // (never actually invoked -- those three are intercepted by verb
  // identity before the table is ever consulted for dispatch, only for
  // HELP's listing).
  bool decodeAlwaysTrue(char** fields, size_t fieldCount);
  void execNoop(char** fields, size_t fieldCount, uint32_t id,
               uint8_t& errCode);

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
  // -- exactly this one field now (2026-08-22: lastDone_ moved to the
  // Adapter, adapter.h; 2026-08-26: gapOutstanding_ deleted with the
  // telemetry ack piggyback, §8.5), and deliberately NO clock/timer. ----
  uint32_t expectedNext_ = 1;       // next sequence id expected from the host

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

// protocol_handler.h — Protocol::ProtocolHandler: the ASCII line-grammar
// codec (docs/design/protocol.md §2-§6) behind the Sink/Adapter seams
// that same document's §1/§3 defines. This is the ONLY class in this
// library that ever touches a wire byte: feed() reassembles arbitrary
// byte blocks into '\n'-terminated lines, tokenizes each line in place
// on runs of ' ' (§2/§3.2 — no allocation, no std::string, no
// exceptions), dispatches to the Adapter, and formats the reply — once,
// per verb, so the Adapter can neither forget a reply nor invent a
// shape for one.
//
// No kernel, no motors, no config storage, no transport: bytes in via
// feed(), bytes out via Sink.
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
// (not malformed). The `id`, where a verb carries one, is always the
// LAST token of the line — self-marking, so an omitted optional field
// never shifts it into a data position. This was a colon-delimited,
// positional grammar before the 2026-08-20 stakeholder decision (commit
// 5a5b6da); this file is the post-cutover rewrite. See
// docs/design/protocol.md §9.1/§9.6 for the resolution history.
//
// ---- Resolved by the grammar itself (no longer this file's own call) ----
//
// The colon grammar's ambiguity between an OMITTED id and an id
// EXPLICITLY WRITTEN AS "0" is gone by CONSTRUCTION under the space
// grammar, not by a rule this file invented: omitted and `#0` are
// visibly different wire forms.
// - id OMITTED (verb whose id is optional, and the trailing token is
//   not present at all): the command still executes, and its `ok`/`err`
//   is sent once, BARE — no `#id` token in the reply at all (`ok`,
//   `err 2`), so a human at a terminal gets confirmation without
//   inventing an id.
// - id explicitly `#0`: executes SILENTLY, no reply of any kind — the
//   ack-suppression spelling for a lossy link that doesn't want an ack
//   for every line (docs/design/protocol.md §2.2). Legal only where the
//   id is optional (SET, WHEELS in this library's scope); on STOP,
//   whose id is REQUIRED, `#0` is itself malformed.
//
// GET's unknown-field-name silence is stated directly in
// docs/design/protocol.md §6 ("unknown name → silent, no reply, not
// counted malformed"), so it is design text this file implements, not
// an ambiguity this file resolves on its own.
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
//    Unaffected by the grammar migration; carried forward verbatim.
//
// 2. The generic malformed-line recovery (docs/design/protocol.md §2.3)
//    — "if the line's last token is a well-formed nonzero `#id`, reply
//    `err #<id> <code>`; otherwise no reply" — is written with NO
//    carve-out for a verb whose own grammar has no id concept at all
//    (HELLO, PING, ID, VER, STATUS, HELP, GET, TLM in this library's
//    scope: none of their rows in §6 have an id column). Read
//    literally, and confirmed by "including unknown verbs" in §2.3,
//    this rule is verb-agnostic: it fires on ANY malformed line
//    (unknown verb, wrong arity, or an unparseable field) whenever the
//    line's raw last token happens to parse as `#[0-9]+` and is
//    nonzero — regardless of whether the matched verb's own grammar
//    would ever have consumed that token as an id. This file
//    implements it that way (see rejectMalformed()/findLastFieldToken()
//    in the .cpp), with exactly ONE deliberate exception: ESTOP,
//    treated as the more specific rule (§2.3) winning over the generic
//    recovery mechanism — a malformed ESTOP line (e.g. `ESTOP #5`,
//    wrong arity) increments the malformed counter and replies with
//    NOTHING, even though `#5` would otherwise be a perfectly good
//    recoverable id. This is this file's own resolution of a tension
//    between the general rule and ESTOP's own stronger one; §2.3
//    states the resolution but the collision itself is not spelled out
//    anywhere else.
//
// 3. The id's own numeric grammar (`id ::= '#' [0-9]+`) is STRICTER
//    than the general "every wire value is a base-10 ASCII integer,
//    optionally signed" rule for ordinary integer fields (WHEELS's
//    `duration`, etc.): the id grammar allows ONLY decimal digits after
//    the `#`, no sign at all — not even a leading `+`, which C's
//    strtoul() would otherwise accept as valid syntax. This file parses
//    ids with a dedicated digit-only scan (parseIdDigits() in the .cpp)
//    rather than reusing the general unsigned-field parser,
//    specifically so `#+5` is rejected as not-an-id (falls through to
//    "ordinary malformed field", not "id 5").
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

  // thdr: once, on the first call and again whenever the column set
  // changes (spec §6.2); t: every call. See adapter.h's Snapshot/Column
  // for the caller's side of this contract.
  void emitTelemetry(const Snapshot& snapshot);

  // Lines dropped as unknown verb, wrong arity, or an unparseable field
  // (spec §2's malformed counter, flags bit 9). A lowercase-led inbound
  // verb — another robot's reply on a shared channel, spec §2.1 — is
  // dropped silently and does NOT increment this. Neither does a blank
  // or all-whitespace line (spec §2).
  uint32_t malformedCount() const { return malformedCount_; }

 private:
  // A per-verb handler receives:
  //   fields          — pointers to the verb's own field tokens, NOT
  //                      including a trailing id-shaped token (spec
  //                      §8.2's self-marking id is stripped from this
  //                      array by the caller wherever a verb's grammar
  //                      says the last token IS its id; see dispatch()
  //                      and each handler's own field-count check).
  //   fieldCount       — the TRUE number of field tokens the line had
  //                      after the verb (uncapped — see tokenizeLine()
  //                      in the .cpp for why an arity check on this is
  //                      still correct even past the fields[] array's
  //                      fixed storage cap).
  //   lastFieldToken   — the line's raw LAST token (nullptr if the line
  //                      was just the verb, nothing after it),
  //                      independent of `fields`' own capacity —
  //                      spec §2's "the line's last token is a
  //                      well-formed nonzero #id" recovery rule reads
  //                      THIS, not `fields[fieldCount-1]`, precisely so
  //                      it stays correct on an adversarial line with
  //                      more junk fields than `fields` has room to
  //                      store pointers for.
  using VerbHandler = void (ProtocolHandler::*)(char** fields,
                                                 size_t fieldCount,
                                                 const char* lastFieldToken);
  struct VerbEntry {
    const char* name;
    VerbHandler handler;
  };

  static const VerbEntry kCommandTable[12];

  // Field-token storage cap for one line, verb-exclusive: the largest
  // arity any in-scope verb declares is WHEELS's 4 (left, right,
  // duration, optional #id), so this leaves headroom without being a
  // firmware-unfriendly allocation. A line with MORE real tokens than
  // this is always wrong-arity for every verb this library knows about
  // (none has arity >= this cap), so capping storage here never turns
  // a truly-too-long line into a falsely-accepted one — see
  // tokenizeLine()'s own comment.
  static constexpr size_t kMaxFieldTokens = 8;

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

  // ---- dispatch ----
  void dispatch(char* verb, char** fields, size_t fieldCount,
                const char* lastFieldToken);
  void replyOk(uint32_t id);      // "ok #<id>\n"
  void replyOkBare();             // "ok\n"          -- id OMITTED (§8.1)
  void replyErr(uint32_t id, uint8_t code);  // "err #<id> <code>\n"
  void replyErrBare(uint8_t code);           // "err <code>\n"
  // Malformed-line recovery (spec §2 / ambiguity note #2 above): counts
  // the line malformed, then replies `err #<id> <code>` IF the line's
  // raw last token is a well-formed nonzero `#id` -- otherwise no
  // reply. Used for unknown verbs and every handler's own wrong-arity /
  // unparseable-field rejection EXCEPT ESTOP, which never calls this.
  void rejectMalformed(const char* lastFieldToken, uint8_t code);
  void writeLine(const char* text);  // one Sink::write() per line
  static uint8_t resultCode(Result result);

  // ---- per-verb handlers -- see VerbHandler's own comment above for
  // the (fields, fieldCount, lastFieldToken) contract every one shares.
  void handleHello(char** fields, size_t fieldCount,
                   const char* lastFieldToken);
  void handlePing(char** fields, size_t fieldCount,
                  const char* lastFieldToken);
  void handleVer(char** fields, size_t fieldCount,
                 const char* lastFieldToken);
  void handleId(char** fields, size_t fieldCount,
                const char* lastFieldToken);
  void handleStatus(char** fields, size_t fieldCount,
                     const char* lastFieldToken);
  void handleHelp(char** fields, size_t fieldCount,
                  const char* lastFieldToken);
  void handleGet(char** fields, size_t fieldCount,
                 const char* lastFieldToken);
  void handleSet(char** fields, size_t fieldCount,
                 const char* lastFieldToken);
  void handleTlm(char** fields, size_t fieldCount,
                 const char* lastFieldToken);
  void handleWheels(char** fields, size_t fieldCount,
                    const char* lastFieldToken);
  void handleStop(char** fields, size_t fieldCount,
                  const char* lastFieldToken);
  void handleEstop(char** fields, size_t fieldCount,
                   const char* lastFieldToken);

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

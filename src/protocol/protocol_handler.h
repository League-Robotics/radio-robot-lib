// protocol_handler.h — Protocol::ProtocolHandler: the v6 ASCII line-
// grammar codec (docs/protocol-v6-spec.md §2-§8) behind the Sink/Adapter
// seams docs/design/protocol.md §1-§2 defines. This is the ONLY class in
// this library that ever touches a wire byte: feed() reassembles
// arbitrary byte blocks into '\n'-terminated lines, splits each line in
// place on ':' (spec §11.1/§11.2 — no allocation, no std::string, no
// exceptions), dispatches to the Adapter, and formats the reply — once,
// per verb, so the Adapter can neither forget a reply nor invent a shape
// for one.
//
// No kernel, no motors, no config storage, no transport: bytes in via
// feed(), bytes out via Sink. See docs/plan.md Step 3.
//
// ---- Ambiguities in the wire spec this file had to resolve ----
//
// 1. Optional trailing `id` (SET, WHEELS) vs. spec §8.2's "Id 0 means no
//    ack wanted, legal on any verb with an optional id": §7.1's own
//    worked example is `SET:wheel_control.pid_kp:0.03 -> ok:0` — an id
//    field that is ABSENT still gets an ack, with id 0 in the reply. That
//    directly contradicts a literal reading of §8.2 if "absent" and
//    "explicit 0" are the same thing. This handler treats them as
//    DIFFERENT: an absent id field defaults to 0 and IS acked (matching
//    §7.1's example); an id field EXPLICITLY WRITTEN AS "0" on the wire
//    means "no ack wanted" and suppresses the reply (matching §8.2's
//    literal words). See resolveOptionalId() in the .cpp. This rule is
//    applied only to verbs whose id is genuinely OPTIONAL (SET, WHEELS)
//    — STOP's id is REQUIRED (spec §3.1's `STOP | id`, no brackets), so
//    it is always acked regardless of value.
//
// 2. GET's unknown field name has no wire outcome defined at all: GET
//    never carries an id (`GET | [name]`, no `[:id]`), so there is no
//    channel to carry an `err` on even though SET's symmetric case
//    (`onSet` returning kUnknown) plainly does. This handler treats an
//    onGet() that returns false as fully silent — no reply, and NOT
//    counted malformed (the line parsed fine; the name is a semantic
//    lookup miss, which is exactly the class of thing
//    docs/design/protocol.md §6 assigns to the adapter, not the line
//    parser).
//
// 3. WHEELS's documented "ceiling 5000" (spec §5.2) is stated in prose at
//    the verb-definition level, not in the Adapter interface
//    (docs/design/protocol.md §3) or anywhere this handler owns a bounds
//    table for. Per §6's "the handler holds no field table, no bounds,
//    no storage" — generalized here from config bounds to motion bounds
//    for consistency — this handler does NOT enforce the ceiling itself;
//    it passes `duration` through unchecked and leaves the ceiling to
//    whatever adapter a future step supplies (its `onWheels` can return
//    kRange). Flagged, not silently assumed.
//
// 4. `Snapshot`/`Column` (adapter.h) are this file's own invention: the
//    design doc names emitTelemetry(const Snapshot&) but never defines
//    the type. The shape here — column name/value/hex-flag triples — is
//    the minimal thing that reproduces spec §6.2's example
//    (`thdr:seq:now:flags:...` / `t:412:38472:d8:...`) with §6.5's one
//    hex exception, and nothing more.
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
  //   - a line longer than the 240-byte maximum: discarded to the next
  //     '\n' and counted malformed — NEVER truncated into a prefix that
  //     might still parse as a command the host never sent.
  void feed(const char* data, size_t length);

  // Unsolicited emissions the app drives, not the wire (spec §4).
  void sendBanner();  // device:NEZHA2:robot:<name>:<serial>
  void sendReady();   // ready

  // thdr: once, on the first call and again whenever the column set
  // changes (spec §6.2); t: every call. See adapter.h's Snapshot/Column
  // for the caller's side of this contract.
  void emitTelemetry(const Snapshot& snapshot);

  // Lines dropped as unknown verb, wrong arity, or an unparseable field
  // (spec §2's malformed counter, flags bit 9). A lowercase-led inbound
  // verb — another robot's reply on a shared channel, spec §2.1 — is
  // dropped silently and does NOT increment this.
  uint32_t malformedCount() const { return malformedCount_; }

 private:
  struct VerbEntry {
    const char* name;
    void (ProtocolHandler::*handler)(char* rest);
  };

  static const VerbEntry kCommandTable[12];

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

  // ---- dispatch ----
  void dispatch(char* verb, char* rest);
  static size_t splitFields(char* rest, char** fields, size_t maxFields);
  void replyOk(uint32_t id);
  void replyErr(uint32_t id, uint8_t code);
  void writeLine(const char* text);  // one Sink::write() per line
  static uint8_t resultCode(Result result);

  // ---- per-verb handlers. Each receives the mutable remainder of the
  // line AFTER the verb's own terminating ':' (nullptr if the verb had
  // no ':' at all — genuinely zero fields), splits it on ':' itself, and
  // owns that verb's arity check, decode, adapter call, and reply. ----
  void handleHello(char* rest);
  void handlePing(char* rest);
  void handleVer(char* rest);
  void handleId(char* rest);
  void handleStatus(char* rest);
  void handleHelp(char* rest);
  void handleGet(char* rest);
  void handleSet(char* rest);
  void handleTlm(char* rest);
  void handleWheels(char* rest);
  void handleStop(char* rest);
  void handleEstop(char* rest);

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

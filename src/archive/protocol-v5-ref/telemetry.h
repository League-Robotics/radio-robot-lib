#pragma once

#include <cstdint>

#include "control/differential_drive.h"
#include "core/comms.h"
#include "firm/types/robot_state.h"
#include "messages/telemetry.h"

// Control::DifferentialDrive is an ALIAS for the DiffDrive package class
// (control/differential_drive.h) -- an alias cannot be forward-declared,
// so this header includes it instead of the `class DifferentialDrive;`
// declaration it used to carry.

namespace Core {


constexpr uint32_t kFlagOtosPresent = 1u << 0;
constexpr uint32_t kFlagOtosConnected = 1u << 1;
constexpr uint32_t kFlagActive = 1u << 2;
constexpr uint32_t kFlagConnLeft = 1u << 3;
constexpr uint32_t kFlagConnRight = 1u << 4;
// Bit 5 pairs with kFlagLinePresent (13) / kFlagColorPresent (14): the
// Present bits say a reading is on the wire at all, these say it was
// re-read on THIS cycle. Only one perception leaf is sampled per cycle, so
// without the split the untouched sensor would have to be dropped from the
// frame rather than allowed to go one alternation stale.
constexpr uint32_t kFlagLineFresh = 1u << 5;
constexpr uint32_t kFlagFaultI2CSafetyNet = 1u << 6;
constexpr uint32_t kFlagFaultWedgeLatch = 1u << 7;
constexpr uint32_t kFlagFaultI2CNak = 1u << 8;
constexpr uint32_t kFlagFaultCommsMalformed = 1u << 9;
constexpr uint32_t kFlagEventDeadmanExpired = 1u << 10;
constexpr uint32_t kFlagEventConfigApplied = 1u << 12;
constexpr uint32_t kFlagLinePresent = 1u << 13;
constexpr uint32_t kFlagColorPresent = 1u << 14;
constexpr uint32_t kFlagFaultMoveTimeout = 1u << 15;
constexpr uint32_t kFlagFaultShapingDisabled = 1u << 16;
constexpr uint32_t kFlagFaultPositionClamped = 1u << 17;
constexpr uint32_t kFlagFaultCommandsDropped = 1u << 18;
constexpr uint32_t kFlagFaultWheelFrozenLeft = 1u << 19;
constexpr uint32_t kFlagFaultWheelFrozenRight = 1u << 20;
constexpr uint32_t kFlagFaultWheelDeficitLeft = 1u << 21;
constexpr uint32_t kFlagFaultWheelDeficitRight = 1u << 22;
// NOT bit 11: that is protocol v4's kFlagEventBootReady, which this firmware
// no longer SETS but which docs/protocol-v4.md still documents and host
// protocol.py still declares -- reusing it would have silently aliased colour
// freshness onto a boot-ready read. Bit 5 IS genuinely free (v4's deleted
// kFlagAckFresh, docs/protocol-v5.md §8.2); 23 is the first bit no protocol
// generation has ever assigned.
constexpr uint32_t kFlagColorFresh = 1u << 23;  // see kFlagLineFresh (bit 5)
// Bits 24/25 -- STALL. The drivetrain was commanded to move, the encoders say
// it did not, and the encoders were healthy enough to be believed, so
// Core::RobotLoop halted the robot. Distinct from bits 19/20 (kFlagFaultWheelFrozen*,
// an ENCODER fault where the wheel may be spinning fine) and bits 21/22
// (kFlagFaultWheelDeficit*, where the wheel IS turning but under its target).
// These are the only wheel-fault bits that mean the robot stopped itself.
// LATCHED until the host commands a new motion -- see RobotState::Health::stallLeft.
constexpr uint32_t kFlagFaultStallLeft = 1u << 24;
constexpr uint32_t kFlagFaultStallRight = 1u << 25;

// The kernel fiber stopped advancing its heartbeat while motion was
// commanded, and RobotLoop's sentinel force-stopped both motors. STICKY
// for the rest of the session: a kernel that died once is not to be
// trusted again without a reboot, and a bit that cleared itself would let
// the event scroll past unseen in a telemetry log.
constexpr uint32_t kFlagFaultKernelStalled = 1u << 26;

// [ms] primary-frame emit floor, deliberately BELOW Core::RobotLoop::kCycle
// (32) so every cycle clears it and telemetry stays ONE FRAME PER CYCLE
// (~31 fps). It was 40 against the old 50 ms kCycle for exactly the same
// reason -- the value tracks kCycle down, it is not an independent rate.
//
// A NOTE ON THE PERCEPTION ALTERNATION, now defused. robot_loop.cpp's pace
// block ticks only ONE perception leaf per cycle (line on odd cycles, colour
// on even). Publishing was once gated on that same per-cycle freshness, so
// an emit floor longer than one cycle ALIASED with the parity and one sensor
// vanished from the wire entirely -- measured on tovez 2026-08-07 at
// kPrimaryPeriod=40/kCycle=32: `line_present 93, color_present 0` over 93
// idle frames. Both readings now ride EVERY frame, with validity latched and
// a separate kFlagLineFresh/kFlagColorFresh saying which was re-read this
// cycle, so the emit rate no longer decides whether a sensor is reportable.
// The rate is now a pure sample-rate choice, which is what it should be.
//
// KNOWN, MEASURED COST -- read before raising the frame rate further. The
// nRF link is HALF DUPLEX, so outbound airtime eats the window in which the
// robot can hear the host. Over the getez relay (channel 3, 2026-08-07):
//
//   kPrimaryPeriod   telemetry   radio_bench_gate   move_wheels   0x0A repro
//   25 (this)        31.4 fps    30/35              command LOST  8/10
//   40               15.8 fps    31/35              ok            9/10
//
// The loss is INBOUND (host->robot commands), not outbound: outbound is
// 99.2% ok with zero unparseable and zero CRC mismatches at BOTH rates, and
// the lost move_wheels was proven to be the command itself, not its ack --
// the encoders never moved (338,358 before and after). Acks are already
// redundant against frame loss (kAckRepeats below: every ack rides three
// consecutive frames), so a dropped telemetry frame cannot lose one.
//
// Halving the emit rate DOES buy back that inbound reliability, and it was
// tried -- but it is a blunt workaround pointed at the wrong direction, and
// it breaks the perception alternation above. The real fix is inbound
// reliability (sequence + NACK + retransmit-from-N, host->robot; the COBS
// framing and CRC needed for it already exist) -- see clasi/issues/
// inbound-command-loss-needs-retransmit-not-a-slower-telemetry-stream.md.
// Until that lands, the relay path carries a known inbound-loss risk that
// pre-dates this constant (clasi/issues/later/radio-bench-gate-fault-latch-
// check-contradicts-inbound-loss-budget.md, sprint 128's own 31/35 run).
// USB is unaffected -- 31 fps is ~2400 B/s, ~21% of the link, and every
// move_protocol_bench scenario passes over it.
constexpr uint32_t kPrimaryPeriod = 25;

constexpr uint8_t kAckRingDepth = 12;

constexpr uint8_t kAckRepeats = 3;

enum class TlmMode : uint8_t { kOff, kAuto, kOn };

constexpr uint32_t kCoastHoldoff = 2000;  // [ms]

class Telemetry {
 public:
  struct Frame {
    msg::DriveMode mode = msg::DriveMode::IDLE;

    msg::EncoderReading encLeft{};
    msg::EncoderReading encRight{};

    msg::OtosReading otos{};

    msg::Pose2D pose{};
    msg::BodyTwist3 twist{};

    uint32_t line = 0;
    uint32_t color = 0;

    uint32_t cycleBusy = 0;  // [us] cycleStart -> frame-staging instant, THIS cycle
    uint32_t cyclePeriod = 0;  // [us] this cycle's own cycleStart minus the previous cycle's

    float dutyPerSpeedLeft = 0.0f;  // [duty/(mm/s)]
    float dutyPerSpeedRight = 0.0f;  // [duty/(mm/s)]
    float biasLeft = 0.0f;  // [mm/s] Stage C's adapted parameter
    float biasRight = 0.0f;  // [mm/s]
    float pidLeft = 0.0f;  // [mm/s] Stage B's last-computed output
    float pidRight = 0.0f;  // [mm/s]
  };

  explicit Telemetry(Comms& comms);

  void update(const Types::RobotState& state, const Control::DifferentialDrive& drive);

  void setLiveFlag(uint32_t bit, bool active);

  uint32_t flags() const { return flags_; }

  void setMode(TlmMode mode) { mode_ = mode; }
  TlmMode mode() const { return mode_; }

  bool applyAction(Comms::TlmAction action);

  void ack(uint32_t corrId, uint32_t errCode);

  void emit(uint32_t now, bool force = false);

  uint32_t primaryEmitCount() const { return primaryEmitCount_; }
  uint32_t lastPrimaryEmit() const { return lastPrimaryEmit_; }  // [ms]

 private:
  bool primaryDue(uint32_t now) const;
  bool pendingAckDeliveries() const;
  void emitPrimary(uint32_t now);
  void pushAckRing(uint32_t corrId, uint32_t errCode);

  void setFlag(uint32_t bit, bool active);

  static uint32_t ageOf(uint32_t now, uint32_t sampleTime);  // [ms] [ms] -> [ms], clamped to 255

  Comms& comms_;

  Frame frame_;

  uint32_t flags_ = 0;

  uint32_t ackRing_[kAckRingDepth]{};
  uint8_t ackRingHead_ = 0;
  uint8_t ackRingCount_ = 0;

  uint8_t ackSends_[kAckRingDepth]{};

  TlmMode mode_ = TlmMode::kAuto;

  bool everMoved_ = false;

  uint32_t lastActivity_ = 0;  // [ms]

  uint32_t seq_ = 0;

  bool everEmittedPrimary_ = false;
  uint32_t lastPrimaryEmit_ = 0;  // [ms]
  uint32_t primaryEmitCount_ = 0;
};

}

// mock_adapter.h -- Protocol::Adapter test double for the protocol host
// test harness (Step 3). Records which methods fired and
// with what arguments so a ctypes shim can read them back; canned
// return values are plain public fields a test sets before feed()ing a
// line. Test scaffolding only: nothing under src/ knows this file
// exists, and it is never linked into anything but this test's own
// shared library.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "adapter.h"

class MockAdapter : public Protocol::Adapter {
 public:
  static constexpr size_t kMaxFields = 4;

  // ---- canned responses, set by the test before feed() ----
  Protocol::Identity identityToReturn;
  Protocol::StatusFields statusToReturn;
  uint32_t nowToReturn = 0;
  Protocol::Result wheelsResult = Protocol::Result::kOk;   // WHEELS_V
  Protocol::Result wheelsXResult = Protocol::Result::kOk;
  Protocol::Result moveXResult = Protocol::Result::kOk;
  Protocol::Result moveVResult = Protocol::Result::kOk;
  Protocol::Result goToRResult = Protocol::Result::kOk;
  Protocol::Result goToWResult = Protocol::Result::kOk;
  Protocol::Result stopResult = Protocol::Result::kOk;
  Protocol::Result setResult = Protocol::Result::kOk;
  Protocol::Result tlmResult = Protocol::Result::kOk;
  Protocol::Result runResult = Protocol::Result::kOk;
  bool runHasResult = false;

  // The reliability layer's completion channel (2026-08-22,
  // docs/design/protocol.md §8.8) -- now Adapter-owned, polled by the
  // handler on every ack/nack. A test drives these directly to exercise
  // the piggyback; the default (0 / kNone) matches this library's own
  // "nothing has completed yet" wire spelling.
  uint32_t lastDoneToReturn = 0;
  Protocol::DoneReason lastDoneReasonToReturn = Protocol::DoneReason::kNone;
  // Text copied verbatim into onRun()'s `result` buffer when
  // runHasResult is true -- a test can point this at a string
  // containing '\n'/'\r' to exercise ProtocolHandler's own sanitize
  // pass on the ADAPTER's returned text (docs/design/protocol.md's RUN
  // section: sanitized exactly like sendDebug()'s text). Borrowed, not
  // copied -- same outlive-the-call contract as every other canned
  // string field on this mock.
  const char* runResultText = "";

  // A small fixed config table -- the source of truth for both a named
  // GET and a bare GET's dump. A name not in this table is the "unknown
  // field" case (onGet() returns false, per protocol_handler.h's
  // ambiguity note #2).
  const char* fieldNames[kMaxFields] = {"group.alpha", "group.beta",
                                         "group.gamma", "group.delta"};
  float fieldValues[kMaxFields] = {1.5f, -2.25f, 0.0f, 100.0f};
  size_t numFields = kMaxFields;

  // A single extra name/value pair a test can point at an ARBITRARY
  // field name (e.g. spec S7.1's own "wheel_control.pid_kp" example),
  // checked before the fixed table above. nullptr (the default) means
  // no override is armed.
  const char* overrideName = nullptr;
  float overrideValue = 0.0f;

  // ---- call counts ----
  mutable int identityCalls = 0;
  mutable int nowCalls = 0;
  mutable int statusCalls = 0;
  int wheelsCalls = 0;   // WHEELS_V (onWheelsV)
  int wheelsXCalls = 0;
  int moveXCalls = 0;
  int moveVCalls = 0;
  int goToRCalls = 0;
  int goToWCalls = 0;
  int stopCalls = 0;
  int estopCalls = 0;
  mutable int getCalls = 0;
  int setCalls = 0;
  int tlmCalls = 0;
  int runCalls = 0;

  // ---- last-call arguments ----
  float lastWheelsLeft = 0.0f;    // WHEELS_V
  float lastWheelsRight = 0.0f;
  uint32_t lastWheelsDuration = 0;
  uint32_t lastWheelsId = 0;

  float lastWheelsXLeft = 0.0f;
  float lastWheelsXRight = 0.0f;
  float lastWheelsXCruise = 0.0f;
  uint32_t lastWheelsXTimeout = 0;

  float lastMoveXDistance = 0.0f;
  float lastMoveXRotation = 0.0f;
  float lastMoveXCruise = 0.0f;
  uint32_t lastMoveXTimeout = 0;

  float lastMoveVVx = 0.0f;
  float lastMoveVOmega = 0.0f;
  uint32_t lastMoveVDuration = 0;

  float lastGoToRX = 0.0f;
  float lastGoToRY = 0.0f;
  float lastGoToRSpeed = 0.0f;
  float lastGoToRArrive = 0.0f;
  uint32_t lastGoToRTimeout = 0;

  float lastGoToWX = 0.0f;
  float lastGoToWY = 0.0f;
  float lastGoToWSpeed = 0.0f;
  float lastGoToWArrive = 0.0f;
  uint32_t lastGoToWTimeout = 0;

  uint32_t lastStopId = 0;
  bool lastStopImmediate = false;
  mutable char lastGetName[64] = {};
  char lastSetName[64] = {};
  float lastSetValue = 0.0f;
  uint32_t lastSetId = 0;
  Protocol::TlmMode lastTlmMode = Protocol::TlmMode::kOff;

  // ---- RUN's own last-call recording ----
  static constexpr size_t kMaxRecordedRunArgs = 16;
  char lastRunName[64] = {};
  size_t lastRunArgc = 0;
  char lastRunArgs[kMaxRecordedRunArgs][64] = {};

  void identity(Protocol::Identity& out) const override {
    ++identityCalls;
    out = identityToReturn;
  }
  uint32_t now() const override {
    ++nowCalls;
    return nowToReturn;
  }
  void status(Protocol::StatusFields& out) const override {
    ++statusCalls;
    out = statusToReturn;
  }
  Protocol::Result onWheelsV(float left, float right, uint32_t duration,
                            uint32_t id) override {
    ++wheelsCalls;
    lastWheelsLeft = left;
    lastWheelsRight = right;
    lastWheelsDuration = duration;
    lastWheelsId = id;
    return wheelsResult;
  }
  Protocol::Result onWheelsX(float left, float right, float cruise,
                            uint32_t timeout, uint32_t /*id*/) override {
    ++wheelsXCalls;
    lastWheelsXLeft = left;
    lastWheelsXRight = right;
    lastWheelsXCruise = cruise;
    lastWheelsXTimeout = timeout;
    return wheelsXResult;
  }
  Protocol::Result onMoveX(float distance, float rotation, float cruise,
                           uint32_t timeout, uint32_t /*id*/) override {
    ++moveXCalls;
    lastMoveXDistance = distance;
    lastMoveXRotation = rotation;
    lastMoveXCruise = cruise;
    lastMoveXTimeout = timeout;
    return moveXResult;
  }
  Protocol::Result onMoveV(float v_x, float omega, uint32_t duration,
                           uint32_t /*id*/) override {
    ++moveVCalls;
    lastMoveVVx = v_x;
    lastMoveVOmega = omega;
    lastMoveVDuration = duration;
    return moveVResult;
  }
  Protocol::Result onGoToR(float x, float y, float speed, float arrive,
                          uint32_t timeout, uint32_t /*id*/) override {
    ++goToRCalls;
    lastGoToRX = x;
    lastGoToRY = y;
    lastGoToRSpeed = speed;
    lastGoToRArrive = arrive;
    lastGoToRTimeout = timeout;
    return goToRResult;
  }
  Protocol::Result onGoToW(float x, float y, float speed, float arrive,
                          uint32_t timeout, uint32_t /*id*/) override {
    ++goToWCalls;
    lastGoToWX = x;
    lastGoToWY = y;
    lastGoToWSpeed = speed;
    lastGoToWArrive = arrive;
    lastGoToWTimeout = timeout;
    return goToWResult;
  }
  Protocol::Result onStop(bool immediate, uint32_t id) override {
    ++stopCalls;
    lastStopId = id;
    lastStopImmediate = immediate;
    return stopResult;
  }
  void onEstop() override { ++estopCalls; }
  uint32_t lastDone() const override { return lastDoneToReturn; }
  Protocol::DoneReason lastDoneReason() const override {
    return lastDoneReasonToReturn;
  }
  bool onGet(const char* name, float& out) const override {
    ++getCalls;
    std::snprintf(lastGetName, sizeof(lastGetName), "%s", name);
    if (overrideName != nullptr && std::strcmp(name, overrideName) == 0) {
      out = overrideValue;
      return true;
    }
    for (size_t i = 0; i < numFields; ++i) {
      if (std::strcmp(name, fieldNames[i]) == 0) {
        out = fieldValues[i];
        return true;
      }
    }
    return false;
  }
  Protocol::Result onSet(const char* name, float value,
                         uint32_t id) override {
    ++setCalls;
    std::snprintf(lastSetName, sizeof(lastSetName), "%s", name);
    lastSetValue = value;
    lastSetId = id;
    return setResult;
  }
  size_t fieldCount() const override { return numFields; }
  const char* fieldName(size_t index) const override {
    return index < numFields ? fieldNames[index] : "";
  }
  Protocol::Result onRun(const char* name, const char* const* argv,
                         size_t argc, char* result, size_t resultCapacity,
                         bool& hasResult) override {
    ++runCalls;
    std::snprintf(lastRunName, sizeof(lastRunName), "%s", name);
    lastRunArgc = argc;
    for (size_t i = 0; i < argc && i < kMaxRecordedRunArgs; ++i) {
      std::snprintf(lastRunArgs[i], sizeof(lastRunArgs[i]), "%s", argv[i]);
    }
    hasResult = runHasResult;
    if (runHasResult) {
      std::snprintf(result, resultCapacity, "%s", runResultText);
    }
    return runResult;
  }
  Protocol::Result onTlm(Protocol::TlmMode mode) override {
    ++tlmCalls;
    lastTlmMode = mode;
    return tlmResult;
  }
};

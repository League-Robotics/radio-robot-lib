// mock_adapter.h -- Protocol::Adapter test double for the protocol host
// test harness (docs/plan.md Step 3). Records which methods fired and
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
  Protocol::Result wheelsResult = Protocol::Result::kOk;
  Protocol::Result stopResult = Protocol::Result::kOk;
  Protocol::Result setResult = Protocol::Result::kOk;
  Protocol::Result tlmResult = Protocol::Result::kOk;

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
  int wheelsCalls = 0;
  int stopCalls = 0;
  int estopCalls = 0;
  mutable int getCalls = 0;
  int setCalls = 0;
  int tlmCalls = 0;

  // ---- last-call arguments ----
  float lastWheelsLeft = 0.0f;
  float lastWheelsRight = 0.0f;
  uint32_t lastWheelsDuration = 0;
  uint32_t lastWheelsId = 0;
  uint32_t lastStopId = 0;
  mutable char lastGetName[64] = {};
  char lastSetName[64] = {};
  float lastSetValue = 0.0f;
  uint32_t lastSetId = 0;
  Protocol::TlmMode lastTlmMode = Protocol::TlmMode::kOff;

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
  Protocol::Result onWheels(float left, float right, uint32_t duration,
                            uint32_t id) override {
    ++wheelsCalls;
    lastWheelsLeft = left;
    lastWheelsRight = right;
    lastWheelsDuration = duration;
    lastWheelsId = id;
    return wheelsResult;
  }
  Protocol::Result onStop(uint32_t id) override {
    ++stopCalls;
    lastStopId = id;
    return stopResult;
  }
  void onEstop() override { ++estopCalls; }
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
  Protocol::Result onTlm(Protocol::TlmMode mode) override {
    ++tlmCalls;
    lastTlmMode = mode;
    return tlmResult;
  }
};

// wire_v6_format.cpp -- Config::WireV6::formatFixed(). See wire_v6_format.h
// for the module's boundary and the spec section 7.2 examples it must
// reproduce exactly.
#include "config/wire_v6_format.h"

#include <cstdio>

namespace Config::WireV6 {

void formatFixed(float value, uint8_t decimals, char* out, size_t cap) {
  if (out == nullptr || cap == 0) return;
  if (decimals > 6) decimals = 6;  // spec section 7.2: always 6 on the wire

  uint32_t divisor = 1;
  for (uint8_t i = 0; i < decimals; ++i) divisor *= 10;

  const bool neg = value < 0.0f;
  const float absVal = neg ? -value : value;

  // Round to the nearest scaled integer using float arithmetic only --
  // newlib-nano's printf lacks %f, but ordinary float math is unaffected
  // (the nRF52833 has a single-precision hardware FPU). Clamp before the
  // cast so an out-of-range value (a mis-set field, not anything this
  // table's own declared bounds would normally allow through) degrades to
  // a large-but-defined number instead of undefined behavior on the
  // float -> uint32_t conversion.
  constexpr float kMaxScaled = 4294967040.0f;  // largest float < UINT32_MAX
  float scaled = absVal * static_cast<float>(divisor) + 0.5f;
  if (scaled > kMaxScaled) scaled = kMaxScaled;
  const uint32_t scaledInt = static_cast<uint32_t>(scaled);

  const uint32_t intPart = scaledInt / divisor;
  const uint32_t fracPart = scaledInt % divisor;

  std::snprintf(out, cap, "%s%lu.%0*lu", neg ? "-" : "",
                static_cast<unsigned long>(intPart), static_cast<int>(decimals),
                static_cast<unsigned long>(fracPart));
}

}  // namespace Config::WireV6

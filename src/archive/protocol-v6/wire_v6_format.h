// wire_v6_format.h -- protocol v6's ONE float formatter (spec section 7.2:
// "That ~15-line helper is the only float formatting in the firmware").
//
// GET's outbound config values are decimal, human-typed/human-read (spec
// section 7.2: "Config values are decimal, not scaled integers, because a
// human types and reads them") -- but newlib-nano's printf has no %f, so
// formatFixed() renders a float as fixed-point ASCII using integer
// arithmetic instead of snprintf's own "%f". Used ONLY by GET
// (Core::RobotLoop::applyConfigRequest(), sprint 137 ticket 002); SET's
// inbound parse uses strtof directly (newlib-nano provides it -- already
// used by Core::Comms::stageSeed()), so no formatter is needed on that
// side. Telemetry (spec section 6) never needs this: every telemetry
// column is a pre-scaled integer specifically so the 31 fps hot path never
// has to format a float at all.
//
// Hand-written, NOT generated -- unlike wire_v6_config_fields.h (ticket
// 001's table), this is logic, not data derived from robot_config.proto.
#pragma once

#include <cstddef>
#include <cstdint>

namespace Config::WireV6 {

// formatFixed() -- writes `value` into `out` (NUL-terminated, at most
// `cap` bytes including the terminator) as a fixed-point decimal with
// exactly `decimals` fractional digits, always present, no exponent:
// formatFixed(0.02f, 6, ...) -> "0.020000", formatFixed(-51.5f, 6, ...)
// -> "-51.500000" (spec's own section 7.2 examples). `decimals` is
// clamped to 6 -- config values are always formatted to 6 fractional
// digits on the wire (spec section 7.2); the parameter stays because the
// spec's own signature is `formatFixed(value, decimals)`.
void formatFixed(float value, uint8_t decimals, char* out, size_t cap);

}  // namespace Config::WireV6

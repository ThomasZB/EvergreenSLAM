/**
 * @file time.h
 * @author hang chen (chen@hang.plus)
 * @brief Timestamps and durations as distinct types.
 * @version 0.1
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_COMMON_TIME_H_
#define EVERGREENSLAM_COMMON_TIME_H_

#include <chrono>
#include <cstdint>
#include <ostream>
#include <ratio>

namespace evergreenslam::common {

using Duration = std::chrono::duration<int64_t, std::nano>;

// std::chrono only guarantees the Unix epoch for system_clock from C++20; every platform this
// builds for already uses it.
using Time = std::chrono::time_point<std::chrono::system_clock, Duration>;

constexpr int64_t kNanosPerSecond = 1000000000;

constexpr Duration FromSeconds(double seconds) {
  return Duration(static_cast<int64_t>(seconds * 1e9 + (seconds < 0 ? -0.5 : 0.5)));
}

constexpr double ToSeconds(Duration duration) {
  return std::chrono::duration<double>(duration).count();
}

// Scales the fraction on its own. A real stamp times 1e9 needs 61 bits, past
// what a double mantissa holds, so the whole seconds must not go through the
// multiplication or the result lands ~200 ns off.
constexpr Time FromUnixSeconds(double seconds) {
  const int64_t whole = static_cast<int64_t>(seconds);
  const double fraction = seconds - static_cast<double>(whole);
  return Time(Duration(whole * kNanosPerSecond + FromSeconds(fraction).count()));
}

// Lossy at a real stamp for the same reason, ~0.2 us; prefer ToUnixNanos.
constexpr double ToUnixSeconds(Time time) { return ToSeconds(time.time_since_epoch()); }

constexpr Time FromUnixNanos(int64_t nanos) { return Time(Duration(nanos)); }

constexpr int64_t ToUnixNanos(Time time) { return time.time_since_epoch().count(); }

inline Time Now() {
  return std::chrono::time_point_cast<Duration>(std::chrono::system_clock::now());
}

// ISO 8601 UTC with microseconds, e.g. 2026-07-27T09:15:04.250000Z.
std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace evergreenslam::common

#endif  // EVERGREENSLAM_COMMON_TIME_H_

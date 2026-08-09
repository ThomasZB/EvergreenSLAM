/**
 * @file time.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "common/time.h"

#include <cstdio>
#include <ctime>

namespace evergreenslam::common {

std::ostream& operator<<(std::ostream& os, const Time time) {
  int64_t seconds = ToUnixNanos(time) / kNanosPerSecond;
  int64_t nanos = ToUnixNanos(time) % kNanosPerSecond;
  // Integer division truncates toward zero, which would put a pre-epoch time in
  // the second after the one it belongs to.
  if (nanos < 0) {
    seconds -= 1;
    nanos += kNanosPerSecond;
  }

  const std::time_t as_time_t = static_cast<std::time_t>(seconds);
  std::tm utc = {};
  if (gmtime_r(&as_time_t, &utc) == nullptr) {
    return os << ToUnixNanos(time) << "ns";
  }

  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.%06lldZ", utc.tm_year + 1900,
                utc.tm_mon + 1, utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec,
                static_cast<long long>(nanos / 1000));
  return os << buffer;
}

}  // namespace evergreenslam::common

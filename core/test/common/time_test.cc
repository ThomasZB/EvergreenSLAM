/**
 * @file time_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "common/time.h"

#include <gtest/gtest.h>

#include <sstream>
#include <type_traits>

namespace evergreenslam::common {
namespace {

TEST(TimeTest, UnixSecondsRoundTrip) {
  const double seconds = 1785000000.25;
  EXPECT_DOUBLE_EQ(ToUnixSeconds(FromUnixSeconds(seconds)), seconds);
}

// The whole seconds must not go through the scaling; before the split this
// landed 200 ns away and grows with the epoch distance.
TEST(TimeTest, UnixSecondsIsExactToTheNanosecondAtARealStamp) {
  EXPECT_EQ(ToUnixNanos(FromUnixSeconds(1785110400.25)), 1785110400250000000LL);
  EXPECT_EQ(ToUnixNanos(FromUnixSeconds(-0.5)), -500000000LL);
}

TEST(TimeTest, UnixNanosRoundTrip) {
  const int64_t nanos = 1785000000123456789LL;
  EXPECT_EQ(ToUnixNanos(FromUnixNanos(nanos)), nanos);
}

// The reason for the type: as double seconds a nanosecond at a real stamp is
// below the mantissa, so two distinct scan times could compare equal.
TEST(TimeTest, KeepsNanosecondsApartAtARealStamp) {
  const Time time = FromUnixNanos(1785000000000000000LL);
  EXPECT_NE(time, time + Duration(1));
  EXPECT_EQ(ToUnixNanos(time + Duration(1)) - ToUnixNanos(time), 1);
}

TEST(TimeTest, DurationArithmetic) {
  const Time start = FromUnixSeconds(100.0);
  const Time end = start + FromSeconds(0.25);
  EXPECT_DOUBLE_EQ(ToSeconds(end - start), 0.25);
  EXPECT_LT(start, end);
}

// A Time is not a Duration and neither is a number, which is the whole point of
// the change; these would all have compiled when both were double.
TEST(TimeTest, TimeAndDurationDoNotMix) {
  EXPECT_FALSE((std::is_convertible_v<double, Time>));
  EXPECT_FALSE((std::is_convertible_v<double, Duration>));
  EXPECT_FALSE((std::is_convertible_v<Duration, Time>));
  EXPECT_FALSE((std::is_convertible_v<Time, double>));
}

TEST(TimeTest, PrintsIso8601Utc) {
  std::ostringstream stream;
  // 2026-07-27T00:00:00Z is 1785110400 s since the Unix epoch.
  stream << FromUnixSeconds(1785110400.25);
  EXPECT_EQ(stream.str(), "2026-07-27T00:00:00.250000Z");
}

TEST(TimeTest, PrintsPreEpochTimesInTheRightSecond) {
  std::ostringstream stream;
  stream << FromUnixNanos(-500000000LL);
  EXPECT_EQ(stream.str(), "1969-12-31T23:59:59.500000Z");
}

}  // namespace
}  // namespace evergreenslam::common

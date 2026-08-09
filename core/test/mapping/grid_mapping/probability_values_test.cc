/**
 * @file probability_values_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/probability_values.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace evergreenslam::mapping {
namespace {

// One quantisation step of the uint8 encoding, plus slack for float error.
constexpr float kValueSpan = static_cast<float>(kMaxValue - kMinValue);
constexpr float kStep = (kMaxProbability - kMinProbability) / kValueSpan;
constexpr float kHalfStep = 0.5f * kStep + 1e-5f;

TEST(ProbabilityValuesTest, RoundTripWithinOneQuantisationStep) {
  for (int i = 0; i <= 1000; ++i) {
    const float probability = static_cast<float>(i) / 1000.f;
    const uint8_t value = ProbabilityToValue(probability);
    // Inputs outside [kMinProbability, kMaxProbability] can only come back
    // clamped, so that is what the round trip is measured against.
    EXPECT_NEAR(ValueToProbability(value), ClampProbability(probability), kHalfStep)
        << "probability " << probability;
  }
}

TEST(ProbabilityValuesTest, ValuesStayInRange) {
  for (int i = -100; i <= 1100; ++i) {
    const float probability = static_cast<float>(i) / 1000.f;
    const int value = static_cast<int>(ProbabilityToValue(probability));
    EXPECT_GE(value, static_cast<int>(kMinValue)) << "probability " << probability;
    EXPECT_LE(value, static_cast<int>(kMaxValue)) << "probability " << probability;
  }
}

TEST(ProbabilityValuesTest, MarkedValueReadsAsUnmarked) {
  for (int value = kMinValue; value <= kMaxValue; ++value) {
    const uint8_t unmarked = static_cast<uint8_t>(value);
    const uint8_t marked = static_cast<uint8_t>(value + kUpdateMarker);
    EXPECT_FLOAT_EQ(ValueToProbability(marked), ValueToProbability(unmarked)) << "value " << value;
    EXPECT_TRUE(IsKnownValue(marked)) << "value " << value;
  }
  EXPECT_FALSE(IsKnownValue(kUnknownValue));
  EXPECT_FALSE(IsKnownValue(kUnknownValue + kUpdateMarker));
}

TEST(ProbabilityValuesTest, UnknownIsNoBetterThanFreeSpace) {
  EXPECT_FLOAT_EQ(ValueToProbability(kUnknownValue), kUnknownProbability);
  // The brute force matcher sums GetValue() over a candidate's points, so
  // ranking unknown above free space would pay it to slide off the map.
  EXPECT_LE(kUnknownProbability, kMinProbability + 1e-6f);
  for (int value = kMinValue; value <= kMaxValue; ++value) {
    EXPECT_LE(kUnknownProbability, ValueToProbability(static_cast<uint8_t>(value)) + 1e-6f)
        << "value " << value;
  }
}

TEST(ProbabilityValuesTest, LookupTablesStayInValueRange) {
  const std::vector<uint8_t> hit_table =
      ComputeLookupTableToApplyOdds(Odds(kDefaultHitProbability));
  const std::vector<uint8_t> miss_table =
      ComputeLookupTableToApplyOdds(Odds(kDefaultMissProbability));
  ASSERT_EQ(hit_table.size(), static_cast<size_t>(kValueCount));
  ASSERT_EQ(miss_table.size(), static_cast<size_t>(kValueCount));
  for (int value = 0; value < kValueCount; ++value) {
    for (const std::vector<uint8_t>* table : {&hit_table, &miss_table}) {
      const int entry = static_cast<int>((*table)[value]);
      EXPECT_GE(entry, static_cast<int>(kUpdateMarker)) << "value " << value;
      const int unmarked = entry & (kUpdateMarker - 1);
      EXPECT_GE(unmarked, static_cast<int>(kMinValue)) << "value " << value;
      EXPECT_LE(unmarked, static_cast<int>(kMaxValue)) << "value " << value;
    }
  }
}

TEST(ProbabilityValuesTest, HitTableIsMonotoneAndSaturates) {
  const std::vector<uint8_t> table = ComputeLookupTableToApplyOdds(Odds(kDefaultHitProbability));
  uint8_t value = kUnknownValue;
  float previous = ValueToProbability(value);
  for (int i = 0; i < 100; ++i) {
    // The table is indexed by the unmarked value; feeding a marked one back in
    // would run off the end of it.
    value = table[value & (kUpdateMarker - 1)];
    const int unmarked = static_cast<int>(value) & (kUpdateMarker - 1);
    ASSERT_GE(unmarked, static_cast<int>(kMinValue)) << "iteration " << i;
    ASSERT_LE(unmarked, static_cast<int>(kMaxValue)) << "iteration " << i;
    const float probability = ValueToProbability(value);
    EXPECT_GE(probability, previous - 1e-6f) << "iteration " << i;
    EXPECT_LE(probability, kMaxProbability + 1e-6f) << "iteration " << i;
    previous = probability;
  }
  EXPECT_NEAR(previous, kMaxProbability, 1e-5f);
  EXPECT_EQ(static_cast<int>(value) & (kUpdateMarker - 1), static_cast<int>(kMaxValue));
}

TEST(ProbabilityValuesTest, MissTableIsMonotoneAndSaturates) {
  const std::vector<uint8_t> table = ComputeLookupTableToApplyOdds(Odds(kDefaultMissProbability));
  uint8_t value = kUnknownValue;
  float previous = ValueToProbability(value);
  for (int i = 0; i < 100; ++i) {
    value = table[value & (kUpdateMarker - 1)];
    const int unmarked = static_cast<int>(value) & (kUpdateMarker - 1);
    ASSERT_GE(unmarked, static_cast<int>(kMinValue)) << "iteration " << i;
    ASSERT_LE(unmarked, static_cast<int>(kMaxValue)) << "iteration " << i;
    const float probability = ValueToProbability(value);
    // The first miss starts from the 0.5 prior, so it rises above the 0.1 an
    // unknown cell reads as before it starts falling.
    if (i > 0) {
      EXPECT_LE(probability, previous + 1e-6f) << "iteration " << i;
    }
    EXPECT_GE(probability, kMinProbability - 1e-6f) << "iteration " << i;
    previous = probability;
  }
  EXPECT_NEAR(previous, kMinProbability, 1e-5f);
  EXPECT_EQ(static_cast<int>(value) & (kUpdateMarker - 1), static_cast<int>(kMinValue));
}

TEST(ProbabilityValuesTest, OddsRoundTrip) {
  for (int i = 1; i < 100; ++i) {
    const float probability = static_cast<float>(i) / 100.f;
    EXPECT_NEAR(ProbabilityFromOdds(Odds(probability)), probability, 1e-5f);
  }
}

}  // namespace
}  // namespace evergreenslam::mapping

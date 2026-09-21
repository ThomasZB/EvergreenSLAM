/**
 * @file probability_values.h
 * @author hang chen (chen@hang.plus)
 * @brief uint8 encoding of occupancy probability and the odds update tables.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_GRID_MAPPING_PROBABILITY_VALUES_H_
#define EVERGREENSLAM_MAPPING_GRID_MAPPING_PROBABILITY_VALUES_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace evergreenslam::mapping {

constexpr double kMinProbability = 0.1;
constexpr double kMaxProbability = 1.0 - kMinProbability;

constexpr uint8_t kUnknownValue = 0;
constexpr uint8_t kMinValue = 1;
constexpr uint8_t kMaxValue = 127;
constexpr uint8_t kUpdateMarker = 128;
constexpr int kValueCount = kUpdateMarker;
constexpr int kReadTableSize = 256;

constexpr double kUnknownProbability = kMinProbability;

constexpr double kDefaultHitProbability = 0.55;
constexpr double kDefaultMissProbability = 0.49;

inline double Odds(double probability) { return probability / (1.0 - probability); }
inline double ProbabilityFromOdds(double odds) { return odds / (odds + 1.0); }
inline double ClampProbability(double probability) {
  return std::clamp(probability, kMinProbability, kMaxProbability);
}
inline uint8_t ProbabilityToValue(double probability) {
  return static_cast<uint8_t>(
      std::lround((ClampProbability(probability) - kMinProbability) * (kMaxValue - kMinValue) /
                  (kMaxProbability - kMinProbability)) +
      kMinValue);
}

namespace internal {

inline std::vector<double> BuildValueToProbabilityTable() {
  std::vector<double> table(kReadTableSize);
  table[kUnknownValue] = kUnknownProbability;
  for (int value = kMinValue; value <= kMaxValue; ++value) {
    table[value] = kMinProbability + (kMaxProbability - kMinProbability) * (value - kMinValue) /
                                         (kMaxValue - kMinValue);
  }
  for (int value = kValueCount; value < kReadTableSize; ++value) {
    table[value] = table[value - kValueCount];
  }
  return table;
}

}  // namespace internal

inline const std::vector<double>& ValueToProbabilityTable() {
  static const std::vector<double> table = internal::BuildValueToProbabilityTable();
  return table;
}
inline double ValueToProbability(uint8_t value) { return ValueToProbabilityTable()[value]; }
inline bool IsKnownValue(uint8_t value) { return (value & (kUpdateMarker - 1)) != kUnknownValue; }

// Distance from 0.5, not from `kUnknownProbability`: one update out of unknown lands right next
// to 0.5 in either direction.
inline int ValueConfidence(uint8_t value) {
  static const int neutral = ProbabilityToValue(0.5);
  return std::abs(static_cast<int>(value) - neutral);
}

// Entry 0 starts from a prior of 0.5.
inline std::vector<uint8_t> ComputeLookupTableToApplyOdds(double odds) {
  std::vector<uint8_t> table(kValueCount);
  table[kUnknownValue] = ProbabilityToValue(ProbabilityFromOdds(odds)) + kUpdateMarker;
  for (int value = kMinValue; value <= kMaxValue; ++value) {
    table[value] = ProbabilityToValue(ProbabilityFromOdds(
                       odds * Odds(ValueToProbability(static_cast<uint8_t>(value))))) +
                   kUpdateMarker;
  }
  return table;
}

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_GRID_MAPPING_PROBABILITY_VALUES_H_

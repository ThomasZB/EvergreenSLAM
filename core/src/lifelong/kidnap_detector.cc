/**
 * @file kidnap_detector.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/kidnap_detector.h"

#include <algorithm>

namespace evergreenslam::lifelong {
namespace {

// The mark is recomputed from two path sums each keyframe; only a real closure moves it this far.
constexpr double kMarkEpsilon = 1e-6;

}  // namespace

KidnapDetector::KidnapDetector(const KidnapDetectorOption& option)
    : option_(option), cooldown_keyframes_(option.cooldown_keyframes) {}

std::optional<KidnapEvidence> KidnapDetector::Observe(const KidnapSignals& signals,
                                                      bool relocalization_open) {
  if (!option_.enabled) {
    return std::nullopt;
  }
  if (signals.since_base_match.has_value()) {
    const double mark = signals.path_length - *signals.since_base_match;
    if (!base_match_mark_.has_value() || mark > *base_match_mark_ + kMarkEpsilon) {
      lowest_match_score_.reset();
      base_match_mark_ = mark;
    }
  }
  if (signals.match_score.has_value()) {
    lowest_match_score_ = std::min(lowest_match_score_.value_or(*signals.match_score),
                                   *signals.match_score);
  }
  status_.cooldown_keyframes_left = std::max(0, status_.cooldown_keyframes_left - 1);
  status_.continuity_broken =
      !signals.since_base_match.has_value() ||
      (lowest_match_score_.has_value() && *lowest_match_score_ <= option_.max_match_score);
  if (relocalization_open) {
    status_.suspect_keyframes = 0;
    return std::nullopt;
  }

  const double distance = signals.since_base_match.value_or(signals.path_length);
  const bool disagrees = signals.base.has_value() && signals.base->score.has_value() &&
                         signals.base->coverage >= option_.min_base_coverage &&
                         *signals.base->score < option_.max_base_score;
  if (!status_.continuity_broken || !disagrees ||
      distance < option_.min_distance_without_base_match) {
    status_.suspect_keyframes = 0;
    return std::nullopt;
  }
  ++status_.suspect_keyframes;
  if (status_.suspect_keyframes < option_.min_suspect_keyframes ||
      status_.cooldown_keyframes_left > 0) {
    return std::nullopt;
  }

  KidnapEvidence evidence;
  evidence.keyframes = status_.suspect_keyframes;
  evidence.lowest_match_score =
      signals.since_base_match.has_value() ? lowest_match_score_ : std::nullopt;
  evidence.base_score = *signals.base->score;
  evidence.base_coverage = signals.base->coverage;
  evidence.distance_without_base_match = distance;
  status_.suspect_keyframes = 0;
  return evidence;
}

void KidnapDetector::OnSearchSettled(RelocalizationState state) {
  if (state == RelocalizationState::MATCHED) {
    cooldown_keyframes_ = option_.cooldown_keyframes;
  }
  status_.cooldown_keyframes_left = cooldown_keyframes_;
  if (state == RelocalizationState::NO_MATCH) {
    cooldown_keyframes_ = std::min(2 * cooldown_keyframes_, option_.max_cooldown_keyframes);
  }
}

void KidnapDetector::Reset() {
  status_.suspect_keyframes = 0;
  status_.continuity_broken = false;
  lowest_match_score_.reset();
  base_match_mark_.reset();
}

}  // namespace evergreenslam::lifelong

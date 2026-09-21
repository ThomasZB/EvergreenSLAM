/**
 * @file constraint_sampler.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/constraint_sampler.h"

#include <algorithm>
#include <tuple>

namespace evergreenslam::lifelong {

ConstraintSampler::ConstraintSampler(const ConstraintSamplerOption& option) : option_(option) {}

bool ConstraintSampler::ShouldSampleRound() {
  ++num_rounds_;
  if (static_cast<double>(num_sampled_) <
      option_.sampling_ratio * static_cast<double>(num_rounds_)) {
    ++num_sampled_;
    return true;
  }
  return false;
}

std::vector<LoopCandidate> ConstraintSampler::Select(std::vector<LoopCandidate> candidates,
                                                     int budget) const {
  const int limit = budget < 0 ? option_.max_matches_per_round : budget;
  std::sort(candidates.begin(), candidates.end(),
            [](const LoopCandidate& a, const LoopCandidate& b) {
              return std::make_tuple(!a.cross_session, a.distance, a.node_id, a.submap_id) <
                     std::make_tuple(!b.cross_session, b.distance, b.node_id, b.submap_id);
            });
  if (static_cast<int>(candidates.size()) > std::max(limit, 0)) {
    candidates.resize(static_cast<size_t>(std::max(limit, 0)));
  }
  return candidates;
}

}  // namespace evergreenslam::lifelong

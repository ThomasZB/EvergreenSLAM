/**
 * @file constraint_sampler.h
 * @author hang chen (chen@hang.plus)
 * @brief Fixed-budget selection of loop closure candidates. No adaptive feedback (ROADMAP #25).
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_SAMPLER_H_
#define EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_SAMPLER_H_

#include <cstdint>
#include <vector>

#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

struct ConstraintSamplerOption {
  // Counter-based, not RNG: the decision must be a pure function of the round index.
  double sampling_ratio = 1.0;
  int max_matches_per_round = 8;
};

struct LoopCandidate {
  NodeId node_id;
  SubmapId submap_id;
  double distance = 0.0;
  // The coarse window must grow by this too, or a far candidate never reaches its true pose.
  double search_slack = 0.0;
  bool cross_session = false;
};

class ConstraintSampler {
 public:
  explicit ConstraintSampler(const ConstraintSamplerOption& option = ConstraintSamplerOption());

  bool ShouldSampleRound();

  // Cross-session first (they anchor a session for freezing), then nearer, then ids as a
  // deterministic tie break. budget < 0 means the option default.
  std::vector<LoopCandidate> Select(std::vector<LoopCandidate> candidates, int budget = -1) const;

  const ConstraintSamplerOption& option() const { return option_; }

 private:
  ConstraintSamplerOption option_;
  int64_t num_rounds_ = 0;
  int64_t num_sampled_ = 0;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_CONSTRAINTS_CONSTRAINT_SAMPLER_H_

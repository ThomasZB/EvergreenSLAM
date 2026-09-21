/**
 * @file trim_selector.h
 * @author hang chen (chen@hang.plus)
 * @brief Which submaps stopped adding information: coverage by newer submaps and newer nodes.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_TRIM_SELECTOR_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_TRIM_SELECTOR_H_

#include <cstdint>
#include <map>
#include <vector>

#include "lifelong/pose_graph_data.h"
#include "lifelong/pose_graph_trimmer/pose_graph_trimmer_option.h"

namespace evergreenslam::lifelong {

// Returns oldest first, respecting the per-session survivor floor.
class TrimSelector {
 public:
  explicit TrimSelector(const TrimSelectorOption& option = TrimSelectorOption());

  std::vector<SubmapId> Select(const PoseGraphData& graph) const;

  const std::vector<std::int64_t>& Footprint(const SubmapRecord& record) const;

  const TrimSelectorOption& option() const { return option_; }

 private:
  TrimSelectorOption option_;

  // Keyed by the pose it was computed at: once the farthest known cell has moved over half a
  // coarse cell, by translation or rotation, the entry is stale.
  struct CachedFootprint {
    Eigen::Affine2d pose = Eigen::Affine2d::Identity();
    double radius = 0.0;
    std::vector<std::int64_t> cells;
  };
  mutable std::map<SubmapId, CachedFootprint> footprint_cache_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_TRIM_SELECTOR_H_

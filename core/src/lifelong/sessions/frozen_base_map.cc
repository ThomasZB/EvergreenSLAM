/**
 * @file frozen_base_map.cc
 * @author hang chen (chen@hang.plus)
 * @brief One occupancy grid of every frozen session, and how well a scan agrees with it.
 * @version 0.1
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/frozen_base_map.h"

#include <vector>

#include "lifelong/global_grid/global_map.h"
#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::lifelong {

FrozenBaseMap::FrozenBaseMap()
    : grid_(std::vector<uint8_t>(), 0, 0, 0.05, 0.0, 0.0, mapping::kUnknownValue) {}

void FrozenBaseMap::Rebuild(const PoseGraphData& graph) {
  grid_ = AssembleGlobalMap(graph, 0.0, /*only_finished=*/true,
                            [&graph](SessionId id) { return graph.session(id).frozen(); });
}

FrozenBaseMap::Agreement FrozenBaseMap::Evaluate(const sensor::PointCloud& points_in_global) const {
  Agreement agreement;
  if (points_in_global.empty()) {
    return agreement;
  }
  int known = 0;
  double probability_sum = 0.0;
  for (const sensor::Point2d& point : points_in_global) {
    const uint8_t value = grid_.GetValueAtPoint(point.point);
    if (!mapping::IsKnownValue(value)) {
      continue;
    }
    ++known;
    probability_sum += mapping::ValueToProbability(value);
  }
  agreement.coverage = static_cast<double>(known) / static_cast<double>(points_in_global.size());
  if (known > 0) {
    agreement.score = probability_sum / static_cast<double>(known);
  }
  return agreement;
}

}  // namespace evergreenslam::lifelong

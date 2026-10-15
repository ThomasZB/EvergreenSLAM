/**
 * @file frozen_base_map.h
 * @author hang chen (chen@hang.plus)
 * @brief One occupancy grid of every frozen session, and how well a scan agrees with it.
 * @version 0.1
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_BASE_MAP_H_
#define EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_BASE_MAP_H_

#include <optional>

#include "lifelong/pose_graph_data.h"
#include "mapping/grid_mapping/grid_map.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::lifelong {

class FrozenBaseMap {
 public:
  struct Agreement {
    double coverage = 0.0;
    std::optional<double> score;
  };

  FrozenBaseMap();

  void Rebuild(const PoseGraphData& graph);
  Agreement Evaluate(const sensor::PointCloud& points_in_global) const;

  const mapping::GridMapu8& grid() const { return grid_; }

 private:
  mapping::GridMapu8 grid_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_BASE_MAP_H_

/**
 * @file submap.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/submap.h"

#include <glog/logging.h>

namespace evergreenslam::mapping {

Submap::Submap(const SubmapId& id, const Eigen::Affine2d& local_pose, double resolution)
    : id_(id), local_pose_(local_pose), grid_(resolution) {}

void Submap::InsertScan(const Eigen::Vector2d& origin, const sensor::PointCloud& point_cloud,
                        const CastRaysMapping& inserter) {
  CHECK(!finished_) << "scan inserted into a finished submap";
  inserter.Insert(origin, point_cloud, grid_);
  ++num_scans_;
  snapshot_.reset();
}

const GridMapu8& Submap::Snapshot() const {
  if (!snapshot_.has_value()) {
    snapshot_ = grid_.ToSnapshot();
  }
  return *snapshot_;
}

void Submap::Finish() {
  finished_ = true;
  Snapshot();
}

}  // namespace evergreenslam::mapping

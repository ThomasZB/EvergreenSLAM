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

#include <utility>

namespace evergreenslam::mapping {

Submap::Submap(int local_index, const Eigen::Affine2d& local_pose, double resolution)
    : local_index_(local_index), local_pose_(local_pose), grid_(resolution) {}

Submap::Submap(int local_index, const Eigen::Affine2d& local_pose, ProbabilityGrid grid,
               int num_scans, bool finished)
    : local_index_(local_index),
      local_pose_(local_pose),
      grid_(std::move(grid)),
      num_scans_(num_scans),
      finished_(finished) {
  // Same contract as Finish(): a finished submap's snapshot exists before anyone can see it.
  if (finished) {
    snapshot_ = grid_.ToSnapshot();
  }
}

void Submap::InsertScan(const Eigen::Affine2d& scan_local_pose,
                        const sensor::PointCloud& point_cloud, const CastRaysMapping& inserter) {
  CHECK(!finished()) << "scan inserted into a finished submap";
  const Eigen::Affine2d scan_in_submap = Eigen::Affine2d(local_pose_.inverse() * scan_local_pose);
  std::lock_guard<std::mutex> lock(insert_mutex_);
  inserter.Insert(scan_in_submap.translation(),
                  sensor::TransformPointCloud(point_cloud, scan_in_submap), grid_);
  num_scans_.fetch_add(1, std::memory_order_relaxed);
  snapshot_.reset();
}

const GridMapu8& Submap::Snapshot() const {
  if (!snapshot_.has_value()) {
    snapshot_ = grid_.ToSnapshot();
  }
  return *snapshot_;
}

std::shared_ptr<Submap> Submap::FinishedCopy() const {
  const std::lock_guard<std::mutex> lock(insert_mutex_);
  return std::make_shared<Submap>(local_index_, local_pose_, grid_, num_scans(), true);
}

GridMapu8 Submap::SnapshotCopy() const {
  if (finished()) {
    return Snapshot();
  }
  std::lock_guard<std::mutex> lock(insert_mutex_);
  return grid_.ToSnapshot();
}

void Submap::Finish() {
  // Snapshot before the release store, so finished() == true implies the snapshot exists.
  Snapshot();
  finished_.store(true, std::memory_order_release);
}

}  // namespace evergreenslam::mapping

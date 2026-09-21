/**
 * @file active_map.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/laser_odometry/active_map.h"

#include <glog/logging.h>

namespace evergreenslam::mapping {

ActiveMap::ActiveMap(const ActiveMapOption& option)
    : option_(option), inserter_(option.inserter_option) {
  CHECK_GT(option_.num_scans_per_submap, 0);
}

std::vector<std::shared_ptr<const Submap>> ActiveMap::InsertScan(
    const Eigen::Affine2d& local_pose, const sensor::PointCloud& point_cloud) {
  if (submaps_.empty() || submaps_.back()->num_scans() >= option_.num_scans_per_submap) {
    AddSubmap(local_pose);
  }
  for (auto& submap : submaps_) {
    submap->InsertScan(local_pose, point_cloud, inserter_);
  }
  if (submaps_.front()->num_scans() >= 2 * option_.num_scans_per_submap &&
      !submaps_.front()->finished()) {
    submaps_.front()->Finish();
  }
  return {submaps_.begin(), submaps_.end()};
}

std::shared_ptr<const Submap> ActiveMap::matching_submap() const {
  if (submaps_.empty()) {
    return nullptr;
  }
  return submaps_.front();
}

void ActiveMap::AddSubmap(const Eigen::Affine2d& local_pose) {
  // Every finished front goes before the window grows, so an insert never reaches a finished grid.
  while (!submaps_.empty() && submaps_.front()->finished()) {
    submaps_.erase(submaps_.begin());
  }
  CHECK_LT(submaps_.size(), 2u) << "growing a full window whose front never finished";
  submaps_.push_back(std::make_shared<Submap>(next_local_index_++, local_pose, option_.resolution));
}

}  // namespace evergreenslam::mapping

/**
 * @file motion_filter.cc
 * @author hang chen (chen@hang.plus)
 * @brief Drops scans taken too close to the last one that was kept.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/filters/motion_filter.h"

#include <cmath>

#include "utils/transform/transform.h"

namespace evergreenslam::utils::filters {

bool MotionFilter::IsSimilar(common::Time time, const Eigen::Affine2d& pose) {
  ++num_total_;

  if (has_reference_) {
    const double angle_delta =
        transform::NormalizeAngle(transform::GetYaw(pose) - transform::GetYaw(last_pose_));
    const bool moved_enough =
        common::ToSeconds(time - last_time_) > option_.max_time_seconds ||
        (pose.translation() - last_pose_.translation()).norm() > option_.max_distance_meters ||
        std::abs(angle_delta) > option_.max_angle_radians;
    if (!moved_enough) {
      return true;
    }
  }

  last_time_ = time;
  last_pose_ = pose;
  has_reference_ = true;
  ++num_kept_;
  return false;
}

void MotionFilter::Reset() { has_reference_ = false; }

}  // namespace evergreenslam::utils::filters

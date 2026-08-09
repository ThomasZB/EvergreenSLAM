/**
 * @file pose_extrapolator.cc
 * @author hang chen (chen@hang.plus)
 * @brief Constant velocity prediction from the pose history alone.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/filters/pose_extrapolator.h"

#include "utils/transform/transform.h"

namespace evergreenslam::utils::filters {

void PoseExtrapolator::AddPose(common::Time time, const Eigen::Affine2d& pose) {
  time_[0] = time_[1];
  pose_[0] = pose_[1];
  time_[1] = time;
  pose_[1] = pose;

  // Only 0, 1 and "at least 2" are distinguished, so saturate.
  if (num_poses_ < 2) {
    ++num_poses_;
  }
}

Eigen::Affine2d PoseExtrapolator::ExtrapolatePose(common::Time time) const {
  if (num_poses_ == 0) {
    return Eigen::Affine2d::Identity();
  }
  if (num_poses_ == 1) {
    return pose_[1];
  }

  // Seconds, not Duration: dividing one Duration by another is integer
  // division, which silently rounds the interpolation factor to 0 or 1.
  const double time_span = common::ToSeconds(time_[1] - time_[0]);
  // Two observations sharing a stamp carry no velocity.
  if (time_span <= 0.0) {
    return pose_[1];
  }

  return transform::Interpolate(pose_[0], pose_[1], common::ToSeconds(time - time_[0]) / time_span);
}

void PoseExtrapolator::Reset() { num_poses_ = 0; }

}  // namespace evergreenslam::utils::filters

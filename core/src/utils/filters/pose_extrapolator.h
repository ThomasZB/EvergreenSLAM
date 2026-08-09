/**
 * @file pose_extrapolator.h
 * @author hang chen (chen@hang.plus)
 * @brief Constant velocity prediction from the pose history alone.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_FILTERS_POSE_EXTRAPOLATOR_H_
#define EVERGREENSLAM_UTILS_FILTERS_POSE_EXTRAPOLATOR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/time.h"

namespace evergreenslam::utils::filters {

// Superseded by GenericTrackingFilter. No production caller left; kept because two of the
// tracking filter's tests measure against it as the constant velocity baseline.
class PoseExtrapolator {
 public:
  // Times must be non-decreasing.
  void AddPose(common::Time time, const Eigen::Affine2d& pose);
  Eigen::Affine2d ExtrapolatePose(common::Time time) const;

  void Reset();

  bool empty() const { return num_poses_ == 0; }
  common::Time latest_time() const { return time_[1]; }
  const Eigen::Affine2d& latest_pose() const { return pose_[1]; }

 private:
  common::Time time_[2] = {};
  Eigen::Affine2d pose_[2] = {Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()};
  int num_poses_ = 0;
};

}  // namespace evergreenslam::utils::filters

#endif  // EVERGREENSLAM_UTILS_FILTERS_POSE_EXTRAPOLATOR_H_

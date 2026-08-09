/**
 * @file pose_optimization.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/laser_odometry/pose_optimization.h"

namespace evergreenslam::mapping {

PoseOptimization::PoseOptimization(const PoseOptimizationOption& option)
    : option_(option),
      correlative_matcher_(option.linear_search_window, option.angular_search_window),
      ceres_matcher_(option.grid_match_weight) {
  if (option.translation_weight > 0.0 && option.rotation_weight > 0.0) {
    ceres_matcher_.SetPosePrior(option.translation_weight, option.rotation_weight);
    correlative_matcher_.SetMotionPenalty(option.translation_weight, option.rotation_weight);
  }
}

double PoseOptimization::Match(const sensor::PointCloud& point_cloud, const GridMapu8& grid_map,
                               Eigen::Affine2d& pose, Eigen::Affine2d* coarse_pose) {
  if (point_cloud.empty()) {
    return 0.0;
  }

  const Eigen::Affine2d predicted_pose = pose;
  const double score = correlative_matcher_.Match(point_cloud, grid_map, pose);
  if (coarse_pose != nullptr) {
    *coarse_pose = pose;
  }
  ceres_matcher_.Match(point_cloud, grid_map, pose, predicted_pose);
  return score;
}

}  // namespace evergreenslam::mapping

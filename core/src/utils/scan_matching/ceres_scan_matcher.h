/**
 * @file ceres_scan_matcher.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef EVERGREENSLAM_UTILS_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define EVERGREENSLAM_UTILS_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mapping/grid_mapping/grid_map.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::utils::scan_matching {

class CeresScanMatcher {
 public:
  explicit CeresScanMatcher(double grid_match_weight = 100.0);
  ~CeresScanMatcher() = default;

  void SetPosePrior(double translation_weight, double rotation_weight);
  // The prior anchors to prior_pose, not to initial_pose: initial_pose is usually a coarse
  // match result, and a prior on it would ratify whatever the coarse stage picked instead of
  // pulling towards the motion prediction.
  void Match(const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
             Eigen::Affine2d& initial_pose, const Eigen::Affine2d& prior_pose);
  void Match(const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
             Eigen::Affine2d& initial_pose);

 private:
  double grid_match_weight_;

  bool use_pose_prior_;
  double translation_weight_;
  double rotation_weight_;
};

}  // namespace evergreenslam::utils::scan_matching

#endif  // EVERGREENSLAM_UTILS_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

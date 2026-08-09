/**
 * @file pose_optimization.h
 * @author hang chen (chen@hang.plus)
 * @brief Coarse-then-fine scan to map alignment, the core of laser odometry.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_LASER_ODOMETRY_POSE_OPTIMIZATION_H_
#define EVERGREENSLAM_MAPPING_LASER_ODOMETRY_POSE_OPTIMIZATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/laser_odometry/pose_optimization_option.h"
#include "sensor/point_cloud.h"
#include "utils/scan_matching/ceres_scan_matcher.h"
#include "utils/scan_matching/real_time_correlative_scan_matcher.h"

namespace evergreenslam::mapping {

class PoseOptimization {
 public:
  explicit PoseOptimization(const PoseOptimizationOption& option = PoseOptimizationOption());

  double Match(const sensor::PointCloud& point_cloud, const GridMapu8& grid_map,
               Eigen::Affine2d& pose, Eigen::Affine2d* coarse_pose = nullptr);

 private:
  PoseOptimizationOption option_;
  utils::scan_matching::RealTimeCorrelativeScanMatcher correlative_matcher_;
  utils::scan_matching::CeresScanMatcher ceres_matcher_;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_LASER_ODOMETRY_POSE_OPTIMIZATION_H_

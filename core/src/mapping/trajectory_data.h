/**
 * @file trajectory_data.h
 * @author hang chen (chen@hang.plus)
 * @brief Records local mapping hands to the pose graph.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_TRAJECTORY_DATA_H_
#define EVERGREENSLAM_MAPPING_TRAJECTORY_DATA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/time.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {

struct TrajectoryNode {
  common::Time time;
  Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();
  sensor::PointCloud point_cloud;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_TRAJECTORY_DATA_H_

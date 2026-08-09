/**
 * @file local_trajectory_builder_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_LOCAL_TRAJECTORY_BUILDER_OPTION_H_
#define EVERGREENSLAM_MAPPING_LOCAL_TRAJECTORY_BUILDER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>
#include <string>
#include <vector>

#include "mapping/laser_odometry/active_map_option.h"
#include "mapping/laser_odometry/pose_optimization_option.h"
#include "sensor/adaptive_voxel_filter_option.h"
#include "utils/filters/generic_tracking_filter_option.h"
#include "utils/filters/motion_filter_option.h"

namespace evergreenslam::mapping {

struct LocalTrajectoryBuilderOption {
  double min_range = 0.3;     // m, anything below this is usually the robot itself
  double max_range = 30.0;    // m
  double voxel_size = 0.025;  // m, thins the cloud that goes into the grid

  sensor::AdaptiveVoxelFilterOption adaptive_voxel_filter_option;
  utils::filters::GenericTrackingFilterOption tracking_filter_option;
  PoseOptimizationOption pose_optimization_option;
  utils::filters::MotionFilterOption motion_filter_option;
  ActiveMapOption active_map_option;

  friend std::ostream& operator<<(std::ostream& os, const LocalTrajectoryBuilderOption& option) {
    os << "LocalTrajectoryBuilderOption:" << std::endl;
    os << "  min_range: " << option.min_range << std::endl;
    os << "  max_range: " << option.max_range << std::endl;
    os << "  voxel_size: " << option.voxel_size << std::endl;
    os << option.adaptive_voxel_filter_option;
    os << option.tracking_filter_option;
    os << option.pose_optimization_option;
    os << option.motion_filter_option;
    os << option.active_map_option;
    return os;
  }
};

LocalTrajectoryBuilderOption LoadLocalTrajectoryBuilderOption(const YAML::Node& node);
std::vector<std::string> FindUnknownLocalTrajectoryBuilderKeys(const YAML::Node& node);
LocalTrajectoryBuilderOption LoadLocalTrajectoryBuilderOptionFromFile(const std::string& path);

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_LOCAL_TRAJECTORY_BUILDER_OPTION_H_

/**
 * @file pose_optimization_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_LASER_ODOMETRY_POSE_OPTIMIZATION_OPTION_H_
#define EVERGREENSLAM_MAPPING_LASER_ODOMETRY_POSE_OPTIMIZATION_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::mapping {

struct PoseOptimizationOption {
  double linear_search_window = 0.15;  // m
  double angular_search_window = 0.2;  // rad
  double grid_match_weight = 100.0;    // weight of the grid term in the Ceres stage
  // Small but nonzero: the prior mostly serves to break ties on a young map's flat score
  // plateaus, where an unanchored match can slide a whole search step and poison the map.
  double translation_weight = 0.01;
  double rotation_weight = 0.04;

  friend std::ostream& operator<<(std::ostream& os, const PoseOptimizationOption& option) {
    os << "PoseOptimizationOption:" << std::endl;
    os << "  linear_search_window: " << option.linear_search_window << std::endl;
    os << "  angular_search_window: " << option.angular_search_window << std::endl;
    os << "  grid_match_weight: " << option.grid_match_weight << std::endl;
    os << "  translation_weight: " << option.translation_weight << std::endl;
    os << "  rotation_weight: " << option.rotation_weight << std::endl;
    return os;
  }
};

PoseOptimizationOption LoadPoseOptimizationOption(const YAML::Node& node);

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_LASER_ODOMETRY_POSE_OPTIMIZATION_OPTION_H_

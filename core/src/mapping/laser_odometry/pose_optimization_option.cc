/**
 * @file pose_optimization_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/laser_odometry/pose_optimization_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::mapping {

using utils::config::LoadOption;

PoseOptimizationOption LoadPoseOptimizationOption(const YAML::Node& node) {
  PoseOptimizationOption option;
  option.linear_search_window =
      LoadOption(node, "linear_search_window", option.linear_search_window);
  option.angular_search_window =
      LoadOption(node, "angular_search_window", option.angular_search_window);
  option.grid_match_weight = LoadOption(node, "grid_match_weight", option.grid_match_weight);
  option.translation_weight = LoadOption(node, "translation_weight", option.translation_weight);
  option.rotation_weight = LoadOption(node, "rotation_weight", option.rotation_weight);
  return option;
}

}  // namespace evergreenslam::mapping

/**
 * @file adaptive_voxel_filter_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sensor/adaptive_voxel_filter_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::sensor {

using utils::config::LoadOption;

AdaptiveVoxelFilterOption LoadAdaptiveVoxelFilterOption(const YAML::Node& node) {
  AdaptiveVoxelFilterOption option;
  option.max_length = LoadOption(node, "max_length", option.max_length);
  option.min_num_points = LoadOption(node, "min_num_points", option.min_num_points);
  return option;
}

}  // namespace evergreenslam::sensor

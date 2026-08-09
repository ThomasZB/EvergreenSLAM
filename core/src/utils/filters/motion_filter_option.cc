/**
 * @file motion_filter_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/filters/motion_filter_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::utils::filters {

using config::LoadOption;

MotionFilterOption LoadMotionFilterOption(const YAML::Node& node) {
  MotionFilterOption option;
  option.max_time_seconds = LoadOption(node, "max_time_seconds", option.max_time_seconds);
  option.max_distance_meters = LoadOption(node, "max_distance_meters", option.max_distance_meters);
  option.max_angle_radians = LoadOption(node, "max_angle_radians", option.max_angle_radians);
  return option;
}

}  // namespace evergreenslam::utils::filters

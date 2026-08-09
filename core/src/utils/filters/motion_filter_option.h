/**
 * @file motion_filter_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_FILTERS_MOTION_FILTER_OPTION_H_
#define EVERGREENSLAM_UTILS_FILTERS_MOTION_FILTER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::utils::filters {

// These three thresholds are what makes the pose graph's fixed sqrt_info = 1e2 valid: they keep
// the error magnitude between adjacent keyframes constant, trading weight tuning for keyframe
// selection.
struct MotionFilterOption {
  double max_time_seconds = 5.0;
  double max_distance_meters = 0.2;
  double max_angle_radians = 0.0175;

  friend std::ostream& operator<<(std::ostream& os, const MotionFilterOption& option) {
    os << "MotionFilterOption:" << std::endl;
    os << "  max_time_seconds: " << option.max_time_seconds << std::endl;
    os << "  max_distance_meters: " << option.max_distance_meters << std::endl;
    os << "  max_angle_radians: " << option.max_angle_radians << std::endl;
    return os;
  }
};

MotionFilterOption LoadMotionFilterOption(const YAML::Node& node);

}  // namespace evergreenslam::utils::filters

#endif  // EVERGREENSLAM_UTILS_FILTERS_MOTION_FILTER_OPTION_H_

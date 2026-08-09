/**
 * @file adaptive_voxel_filter_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_SENSOR_ADAPTIVE_VOXEL_FILTER_OPTION_H_
#define EVERGREENSLAM_SENSOR_ADAPTIVE_VOXEL_FILTER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::sensor {

struct AdaptiveVoxelFilterOption {
  double max_length = 0.5;   // m, the coarsest voxel, so it caps how many points survive
  int min_num_points = 200;  // the voxel shrinks until at least this many points survive

  friend std::ostream& operator<<(std::ostream& os, const AdaptiveVoxelFilterOption& option) {
    os << "AdaptiveVoxelFilterOption:" << std::endl;
    os << "  max_length: " << option.max_length << std::endl;
    os << "  min_num_points: " << option.min_num_points << std::endl;
    return os;
  }
};

AdaptiveVoxelFilterOption LoadAdaptiveVoxelFilterOption(const YAML::Node& node);

}  // namespace evergreenslam::sensor

#endif  // EVERGREENSLAM_SENSOR_ADAPTIVE_VOXEL_FILTER_OPTION_H_

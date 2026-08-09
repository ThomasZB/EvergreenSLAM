/**
 * @file active_map_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_LASER_ODOMETRY_ACTIVE_MAP_OPTION_H_
#define EVERGREENSLAM_MAPPING_LASER_ODOMETRY_ACTIVE_MAP_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

#include "mapping/grid_mapping/castrays_mapping_option.h"

namespace evergreenslam::mapping {

struct ActiveMapOption {
  double resolution = 0.05;  // m/cell
  // A larger submap drifts more internally but leaves fewer pose graph nodes.
  int num_scans_per_submap = 90;
  CastRaysMappingOption inserter_option;

  friend std::ostream& operator<<(std::ostream& os, const ActiveMapOption& option) {
    os << "ActiveMapOption:" << std::endl;
    os << "  resolution: " << option.resolution << std::endl;
    os << "  num_scans_per_submap: " << option.num_scans_per_submap << std::endl;
    os << option.inserter_option;
    return os;
  }
};

ActiveMapOption LoadActiveMapOption(const YAML::Node& node);

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_LASER_ODOMETRY_ACTIVE_MAP_OPTION_H_

/**
 * @file active_map_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/laser_odometry/active_map_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::mapping {

using utils::config::Child;
using utils::config::LoadOption;

ActiveMapOption LoadActiveMapOption(const YAML::Node& node) {
  ActiveMapOption option;
  option.resolution = LoadOption(node, "resolution", option.resolution);
  option.num_scans_per_submap =
      LoadOption(node, "num_scans_per_submap", option.num_scans_per_submap);
  option.inserter_option = LoadCastRaysMappingOption(Child(node, "castrays_mapping"));
  return option;
}

}  // namespace evergreenslam::mapping

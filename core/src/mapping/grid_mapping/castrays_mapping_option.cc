/**
 * @file castrays_mapping_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/castrays_mapping_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::mapping {

using utils::config::LoadOption;

CastRaysMappingOption LoadCastRaysMappingOption(const YAML::Node& node) {
  CastRaysMappingOption option;
  option.hit_probability = LoadOption(node, "hit_probability", option.hit_probability);
  option.miss_probability = LoadOption(node, "miss_probability", option.miss_probability);
  option.insert_free_space = LoadOption(node, "insert_free_space", option.insert_free_space);
  return option;
}

}  // namespace evergreenslam::mapping

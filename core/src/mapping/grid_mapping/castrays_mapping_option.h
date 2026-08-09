/**
 * @file castrays_mapping_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_GRID_MAPPING_CASTRAYS_MAPPING_OPTION_H_
#define EVERGREENSLAM_MAPPING_GRID_MAPPING_CASTRAYS_MAPPING_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::mapping {

struct CastRaysMappingOption {
  double hit_probability = kDefaultHitProbability;    // evidence from a single hit
  double miss_probability = kDefaultMissProbability;  // evidence from a single pass-through
  bool insert_free_space = true;                      // off maps obstacles only, no clearing

  friend std::ostream& operator<<(std::ostream& os, const CastRaysMappingOption& option) {
    os << "CastRaysMappingOption:" << std::endl;
    os << "  hit_probability: " << option.hit_probability << std::endl;
    os << "  miss_probability: " << option.miss_probability << std::endl;
    os << "  insert_free_space: " << option.insert_free_space << std::endl;
    return os;
  }
};

CastRaysMappingOption LoadCastRaysMappingOption(const YAML::Node& node);

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_GRID_MAPPING_CASTRAYS_MAPPING_OPTION_H_

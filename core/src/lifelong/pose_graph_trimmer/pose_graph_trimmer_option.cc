/**
 * @file pose_graph_trimmer_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/pose_graph_trimmer_option.h"

#include <glog/logging.h>

#include <string>

#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {

TrimSelectorOption LoadTrimSelectorOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  TrimSelectorOption option;
  option.submap_coverage_resolution =
      LoadOption(node, "submap_coverage_resolution", option.submap_coverage_resolution);
  option.node_coverage_resolution =
      LoadOption(node, "node_coverage_resolution", option.node_coverage_resolution);
  option.min_covered_fraction =
      LoadOption(node, "min_covered_fraction", option.min_covered_fraction);
  option.min_covering_newer_submaps =
      LoadOption(node, "min_covering_newer_submaps", option.min_covering_newer_submaps);
  option.min_covered_node_fraction =
      LoadOption(node, "min_covered_node_fraction", option.min_covered_node_fraction);
  const std::string mode =
      LoadOption(node, "coverage_mode", std::string(ToString(option.coverage_mode)));
  if (mode == "any") {
    option.coverage_mode = CoverageMode::kAny;
  } else if (mode == "all") {
    option.coverage_mode = CoverageMode::kAll;
  } else {
    LOG(FATAL) << "coverage_mode must be \"any\" or \"all\", got: " << mode;
  }
  option.min_surviving_submaps =
      LoadOption(node, "min_surviving_submaps", option.min_surviving_submaps);
  option.keep_newest_submaps = LoadOption(node, "keep_newest_submaps", option.keep_newest_submaps);
  return option;
}

PoseGraphTrimmerOption LoadPoseGraphTrimmerOption(const YAML::Node& node) {
  using utils::config::Child;
  using utils::config::LoadOption;
  PoseGraphTrimmerOption option;
  option.selector = LoadTrimSelectorOption(Child(node, "selector"));
  option.max_submaps_per_round =
      LoadOption(node, "max_submaps_per_round", option.max_submaps_per_round);
  option.min_recovered_stddev =
      LoadOption(node, "min_recovered_stddev", option.min_recovered_stddev);
  option.max_binary_constraint_length =
      LoadOption(node, "max_binary_constraint_length", option.max_binary_constraint_length);
  option.prior_max_translation_stddev =
      LoadOption(node, "prior_max_translation_stddev", option.prior_max_translation_stddev);
  option.prior_max_rotation_stddev =
      LoadOption(node, "prior_max_rotation_stddev", option.prior_max_rotation_stddev);
  return option;
}

}  // namespace evergreenslam::lifelong

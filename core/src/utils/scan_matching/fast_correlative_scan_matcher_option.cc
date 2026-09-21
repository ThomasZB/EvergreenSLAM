/**
 * @file fast_correlative_scan_matcher_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/scan_matching/fast_correlative_scan_matcher_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::utils::scan_matching {

using utils::config::LoadOption;

FastCorrelativeScanMatcherOption LoadFastCorrelativeScanMatcherOption(const YAML::Node& node) {
  FastCorrelativeScanMatcherOption option;
  option.angular_search_window =
      LoadOption(node, "angular_search_window", option.angular_search_window);
  option.branch_and_bound_depth =
      LoadOption(node, "branch_and_bound_depth", option.branch_and_bound_depth);
  return option;
}

}  // namespace evergreenslam::utils::scan_matching

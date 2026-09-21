/**
 * @file fast_correlative_scan_matcher_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_OPTION_H_
#define EVERGREENSLAM_UTILS_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::utils::scan_matching {

struct FastCorrelativeScanMatcherOption {
  // A match with no prior ignores this and searches the whole submap over all headings.
  double angular_search_window = 0.52;  // rad, ~30 degrees
  int branch_and_bound_depth = 7;       // the coarsest level pools 2^(depth - 1) cells

  friend std::ostream& operator<<(std::ostream& os,
                                  const FastCorrelativeScanMatcherOption& option) {
    os << "FastCorrelativeScanMatcherOption:" << std::endl;
    os << "  angular_search_window: " << option.angular_search_window << std::endl;
    os << "  branch_and_bound_depth: " << option.branch_and_bound_depth << std::endl;
    return os;
  }
};

FastCorrelativeScanMatcherOption LoadFastCorrelativeScanMatcherOption(const YAML::Node& node);

}  // namespace evergreenslam::utils::scan_matching

#endif  // EVERGREENSLAM_UTILS_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_OPTION_H_

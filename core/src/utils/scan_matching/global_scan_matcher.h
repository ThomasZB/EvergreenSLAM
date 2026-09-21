/**
 * @file global_scan_matcher.h
 * @author hang chen (chen@hang.plus)
 * @brief Candidate and result types for pose recovery with weak or no prior, shared by loop
 * closure and boot relocalization.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_SCAN_MATCHING_GLOBAL_SCAN_MATCHER_H_
#define EVERGREENSLAM_UTILS_SCAN_MATCHING_GLOBAL_SCAN_MATCHER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>

#include "mapping/grid_mapping/grid_map.h"

namespace evergreenslam::utils::scan_matching {

// The caller owns the grid and keeps it alive for the call.
struct CandidateSubmap {
  // Maps the grid's own frame to the caller's global frame.
  Eigen::Affine2d grid_to_global = Eigen::Affine2d::Identity();
  std::reference_wrapper<const mapping::GridMapu8> grid;
};

struct GlobalMatchResult {
  // Sensor pose in the caller's global frame.
  Eigen::Affine2d pose = Eigen::Affine2d::Identity();
  // Mean occupancy probability, the same scale the local matchers report.
  double score = 0.0;
};

}  // namespace evergreenslam::utils::scan_matching

#endif  // EVERGREENSLAM_UTILS_SCAN_MATCHING_GLOBAL_SCAN_MATCHER_H_

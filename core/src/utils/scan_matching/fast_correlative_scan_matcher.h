/**
 * @file fast_correlative_scan_matcher.h
 * @author hang chen (chen@hang.plus)
 * @brief Branch-and-bound global matcher over max-pooled precomputation grids.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
#define EVERGREENSLAM_UTILS_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <vector>

#include "sensor/point_cloud.h"
#include "utils/scan_matching/fast_correlative_scan_matcher_option.h"
#include "utils/scan_matching/global_scan_matcher.h"
#include "utils/scan_matching/precomputation_grid.h"

namespace evergreenslam::utils::scan_matching {

// Multi-resolution exhaustive search (Hess et al., "Real-Time Loop Closure in 2D LIDAR SLAM").
// Stateless: the precomputation stack depends only on the grid and the depth, so a caller with
// immutable grids can build it once and hand it back through MatchParams.
class FastCorrelativeScanMatcher {
 public:
  // The linear window and score floor differ per pair, so they are the caller's to set.
  struct MatchParams {
    double linear_search_window;  // m; ignored by a whole-grid match without a prior
    double min_score;             // mean occupancy probability below this is a failed match
    std::optional<double> angular_search_window;
    // Must come from BuildPrecomputationStack on this candidate's own grid.
    std::shared_ptr<const PrecomputationGridStack> stack;
  };

  explicit FastCorrelativeScanMatcher(
      const FastCorrelativeScanMatcherOption& option = FastCorrelativeScanMatcherOption());

  // point_cloud is in the sensor frame. Without a prior every candidate is searched in full
  // over all headings.
  std::optional<GlobalMatchResult> Match(const sensor::PointCloud& point_cloud,
                                         const std::vector<CandidateSubmap>& candidates,
                                         const std::optional<Eigen::Affine2d>& prior_pose,
                                         const MatchParams& params) const;

  std::shared_ptr<const PrecomputationGridStack> BuildPrecomputationStack(
      const mapping::GridMapu8& grid) const;

  // Bitwise equal to Match() by construction. Exhaustive and slow; tests only.
  std::optional<GlobalMatchResult> MatchBruteForce(const sensor::PointCloud& point_cloud,
                                                   const std::vector<CandidateSubmap>& candidates,
                                                   const std::optional<Eigen::Affine2d>& prior_pose,
                                                   const MatchParams& params) const;

 private:
  std::optional<GlobalMatchResult> MatchInternal(const sensor::PointCloud& point_cloud,
                                                 const std::vector<CandidateSubmap>& candidates,
                                                 const std::optional<Eigen::Affine2d>& prior_pose,
                                                 const MatchParams& params,
                                                 bool use_branch_and_bound) const;

  FastCorrelativeScanMatcherOption option_;
};

}  // namespace evergreenslam::utils::scan_matching

#endif  // EVERGREENSLAM_UTILS_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

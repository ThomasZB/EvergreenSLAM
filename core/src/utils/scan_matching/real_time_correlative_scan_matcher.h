/**
 * @file real_time_correlative_scan_matcher.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef EVERGREENSLAM_UTILS_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
#define EVERGREENSLAM_UTILS_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::utils::scan_matching {

class RealTimeCorrelativeScanMatcher {
 public:
  RealTimeCorrelativeScanMatcher(double linear_search_window, double angular_search_window);
  ~RealTimeCorrelativeScanMatcher() = default;

  void SetManualResolution(double angular_resolution, double linear_resolution);

  // Subtracts weight * offset-from-initial-pose from every candidate's score before picking the
  // winner. On a young map the raw score plateaus across whole cells and the argmax is
  // effectively arbitrary; the penalty breaks those ties towards the motion prediction.
  void SetMotionPenalty(double translation_weight, double rotation_weight);

  // Returns the mean occupancy probability under the best candidate.
  double Match(const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
               Eigen::Affine2d& initial_pose);

 private:
  // Whole-cell shifts away from the initial pose, not absolute cell indices,
  // so the sub-cell part of that pose survives by simply being added to.
  struct CandidateResult {
    int x_offset;
    int y_offset;
    int theta_index;

    double score;

    bool operator<(const CandidateResult& other) const { return score < other.score; }
    bool operator>(const CandidateResult& other) const { return score > other.score; }
  };

  void UpdateSearchParameters(const mapping::GridMapu8& grid_map,
                              const sensor::PointCloud& point_cloud);

  size_t NumAngularCandidates() const;

  std::vector<std::vector<Eigen::Array2i>> PrecomputeRotatedPoints(
      const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
      const Eigen::Affine2d& initial_pose);

  std::vector<CandidateResult> GenerateAndScoreCandidates(
      const std::vector<std::vector<Eigen::Array2i>>& rotated_points,
      const mapping::GridMapu8& grid_map);

  double GetBestFromCandidates(const std::vector<CandidateResult>& candidates,
                               Eigen::Affine2d& initial_pose);

  double ComputeScore(const std::vector<Eigen::Array2i>& points, const mapping::GridMapu8& grid_map,
                      int x_offset, int y_offset);

  double linear_search_window_;
  double angular_search_window_;

  bool use_manual_resolution_;
  double angular_resolution_ = 0.0001;
  double linear_resolution_ = 0.05;

  double translation_penalty_weight_ = 0.0;  // score per metre of candidate offset
  double rotation_penalty_weight_ = 0.0;     // score per radian of candidate offset
};

}  // namespace evergreenslam::utils::scan_matching

#endif  // EVERGREENSLAM_UTILS_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_

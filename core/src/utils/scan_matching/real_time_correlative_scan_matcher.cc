/**
 * @file real_time_correlative_scan_matcher.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "utils/scan_matching/real_time_correlative_scan_matcher.h"

#include <glog/logging.h>
#include <omp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::utils::scan_matching {

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher(double linear_search_window,
                                                               double angular_search_window)
    : linear_search_window_(linear_search_window),
      angular_search_window_(angular_search_window),
      use_manual_resolution_(false) {}

void RealTimeCorrelativeScanMatcher::SetManualResolution(double angular_resolution,
                                                         double linear_resolution) {
  use_manual_resolution_ = true;
  angular_resolution_ = angular_resolution;
  linear_resolution_ = linear_resolution;
}

void RealTimeCorrelativeScanMatcher::SetMotionPenalty(double translation_weight,
                                                      double rotation_weight) {
  translation_penalty_weight_ = translation_weight;
  rotation_penalty_weight_ = rotation_weight;
}

double RealTimeCorrelativeScanMatcher::Match(const sensor::PointCloud& point_cloud,
                                             const mapping::GridMapu8& grid_map,
                                             Eigen::Affine2d& initial_pose) {
  UpdateSearchParameters(grid_map, point_cloud);

  const auto rotated_points = PrecomputeRotatedPoints(point_cloud, grid_map, initial_pose);
  const std::vector<CandidateResult> candidates =
      GenerateAndScoreCandidates(rotated_points, grid_map);

  return GetBestFromCandidates(candidates, initial_pose);
}

void RealTimeCorrelativeScanMatcher::UpdateSearchParameters(const mapping::GridMapu8& grid_map,
                                                            const sensor::PointCloud& point_cloud) {
  DCHECK_EQ(grid_map.unknown_value(), mapping::kUnknownValue);
  if (use_manual_resolution_) {
    return;
  }
  linear_resolution_ = grid_map.resolution();
  double max_range = 3.0 * linear_resolution_;
  for (const auto& point : point_cloud) {
    max_range = std::max(max_range, point.point.norm());
  }
  // The angle at which the farthest point moves by one cell, so that no
  // angular candidate is skipped and none is wasted.
  angular_resolution_ = 0.999 * std::acos(1 - 0.5 * (linear_resolution_ * linear_resolution_) /
                                                  (max_range * max_range));
}

size_t RealTimeCorrelativeScanMatcher::NumAngularCandidates() const {
  return static_cast<size_t>(2 * angular_search_window_ / angular_resolution_) + 1;
}

std::vector<std::vector<Eigen::Array2i>> RealTimeCorrelativeScanMatcher::PrecomputeRotatedPoints(
    const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
    const Eigen::Affine2d& initial_pose) {
  const size_t size = NumAngularCandidates();
  std::vector<std::vector<Eigen::Array2i>> rotated_points;
  rotated_points.reserve(size);

  double delta_theta = -angular_search_window_;
  for (size_t i = 0; i < size; i++, delta_theta += angular_resolution_) {
    // Rotation before translation, i.e. about the sensor itself, which is what
    // the reported pose has to mean.
    Eigen::Affine2d transform = Eigen::Affine2d::Identity();
    transform.linear() = Eigen::Rotation2Dd(delta_theta).toRotationMatrix() * initial_pose.linear();
    transform.translation() = initial_pose.translation();

    std::vector<Eigen::Array2i> cells;
    cells.reserve(point_cloud.size());
    for (const auto& point : point_cloud) {
      // Discretised by the grid itself, so these are exactly the cells the map was written with.
      // Doing it relative to the sensor and adding the candidate's cell index is only valid with
      // floor; truncating put 75% of a 360 beam scan one cell off and made every score noise.
      cells.push_back(grid_map.ToCell(transform * point.point));
    }
    rotated_points.push_back(std::move(cells));
  }
  return rotated_points;
}

std::vector<RealTimeCorrelativeScanMatcher::CandidateResult>
RealTimeCorrelativeScanMatcher::GenerateAndScoreCandidates(
    const std::vector<std::vector<Eigen::Array2i>>& rotated_points,
    const mapping::GridMapu8& grid_map) {
  const int half_linear = static_cast<int>(linear_search_window_ / linear_resolution_);
  const int size_linear = 2 * half_linear + 1;
  const int size_angular = static_cast<int>(rotated_points.size());

  std::vector<CandidateResult> candidates(static_cast<size_t>(size_linear) * size_linear *
                                          size_angular);

  // Perfectly nested, which collapse(3) requires.
  const int num_threads = 4;
#pragma omp parallel for collapse(3) num_threads(num_threads)
  for (int x_index = 0; x_index < size_linear; x_index++) {
    for (int y_index = 0; y_index < size_linear; y_index++) {
      for (int theta_index = 0; theta_index < size_angular; theta_index++) {
        const size_t candidate_index =
            (static_cast<size_t>(x_index) * size_linear + y_index) * size_angular + theta_index;
        CandidateResult& candidate = candidates[candidate_index];
        candidate.x_offset = x_index - half_linear;
        candidate.y_offset = y_index - half_linear;
        candidate.theta_index = theta_index;
        candidate.score = ComputeScore(rotated_points[theta_index], grid_map, candidate.x_offset,
                                       candidate.y_offset);
      }
    }
  }
  return candidates;
}

double RealTimeCorrelativeScanMatcher::GetBestFromCandidates(
    const std::vector<CandidateResult>& candidates, Eigen::Affine2d& initial_pose) {
  const CandidateResult* best_candidate = &candidates.front();
  double best_penalized = std::numeric_limits<double>::lowest();
  for (const CandidateResult& candidate : candidates) {
    const double penalized =
        candidate.score -
        translation_penalty_weight_ * std::hypot(candidate.x_offset, candidate.y_offset) *
            linear_resolution_ -
        rotation_penalty_weight_ *
            std::abs(-angular_search_window_ + candidate.theta_index * angular_resolution_);
    if (penalized > best_penalized) {
      best_penalized = penalized;
      best_candidate = &candidate;
    }
  }

  const double delta_theta =
      -angular_search_window_ + best_candidate->theta_index * angular_resolution_;

  initial_pose.translation() += Eigen::Vector2d(best_candidate->x_offset * linear_resolution_,
                                                best_candidate->y_offset * linear_resolution_);
  initial_pose.linear() =
      (Eigen::Rotation2Dd(delta_theta).toRotationMatrix() * initial_pose.linear()).eval();
  return best_candidate->score;
}

double RealTimeCorrelativeScanMatcher::ComputeScore(const std::vector<Eigen::Array2i>& points,
                                                    const mapping::GridMapu8& grid_map,
                                                    int x_offset, int y_offset) {
  const std::vector<double>& probability = mapping::ValueToProbabilityTable();
  double score = 0;
  for (const auto& point : points) {
    score += probability[grid_map.GetValue(point[0] + x_offset, point[1] + y_offset)];
  }
  return score / points.size();
}

}  // namespace evergreenslam::utils::scan_matching

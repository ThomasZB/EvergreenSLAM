/**
 * @file fast_correlative_scan_matcher.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/scan_matching/fast_correlative_scan_matcher.h"

#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>

#include "mapping/grid_mapping/probability_values.h"
#include "utils/scan_matching/precomputation_grid.h"

namespace evergreenslam::utils::scan_matching {
namespace {

// Clamped so unknown scores like minimum probability. The mean is then affine in the integer
// sum, so maximizing the sum maximizes the mean exactly, ties included.
inline int64_t PointScore(uint8_t value) { return std::max<int64_t>(value, mapping::kMinValue); }

double ScoreFromSum(int64_t sum, size_t num_points) {
  return mapping::kMinProbability +
         (static_cast<double>(sum) / static_cast<double>(num_points) - mapping::kMinValue) *
             (mapping::kMaxProbability - mapping::kMinProbability) /
             (mapping::kMaxValue - mapping::kMinValue);
}

// Integer space, never re-compared as doubles, so branch and bound and brute force cannot
// disagree at the boundary.
int64_t ScoreSumThreshold(double min_score, size_t num_points) {
  const double per_point =
      mapping::kMinValue + (min_score - mapping::kMinProbability) *
                               (mapping::kMaxValue - mapping::kMinValue) /
                               (mapping::kMaxProbability - mapping::kMinProbability);
  return static_cast<int64_t>(std::ceil(per_point * static_cast<double>(num_points)));
}

struct SearchParameters {
  Eigen::Affine2d base_pose = Eigen::Affine2d::Identity();  // in the grid's frame
  double start_delta_theta = 0.0;
  double angular_step = 1.0;
  int num_angles = 0;
  int min_x = 0;  // cell offsets from the base pose, inclusive bounds
  int max_x = 0;
  int min_y = 0;
  int max_y = 0;
};

SearchParameters ComputeSearchParameters(const sensor::PointCloud& point_cloud,
                                         const mapping::GridMapu8& grid,
                                         const std::optional<Eigen::Affine2d>& prior_in_grid,
                                         double angular_search_window,
                                         double linear_search_window) {
  SearchParameters parameters;
  const double resolution = grid.resolution();
  double max_range = 3.0 * resolution;
  for (const auto& point : point_cloud) {
    max_range = std::max(max_range, point.point.norm());
  }
  // The angle at which the farthest point moves by one cell: no candidate skipped or wasted.
  parameters.angular_step =
      0.999 * std::acos(1 - 0.5 * (resolution * resolution) / (max_range * max_range));

  double angular_window = 0.0;
  if (prior_in_grid.has_value()) {
    parameters.base_pose = *prior_in_grid;
    angular_window = angular_search_window;
    const int half_cells = static_cast<int>(std::ceil(linear_search_window / resolution));
    parameters.min_x = -half_cells;
    parameters.max_x = half_cells;
    parameters.min_y = -half_cells;
    parameters.max_y = half_cells;
  } else {
    parameters.base_pose.translation() =
        Eigen::Vector2d(grid.origin_x() + 0.5 * grid.width() * resolution,
                        grid.origin_y() + 0.5 * grid.height() * resolution);
    angular_window = M_PI;
    parameters.min_x = -(grid.width() / 2 + 1);
    parameters.max_x = grid.width() / 2 + 1;
    parameters.min_y = -(grid.height() / 2 + 1);
    parameters.max_y = grid.height() / 2 + 1;
  }
  parameters.num_angles = static_cast<int>(2.0 * angular_window / parameters.angular_step) + 1;
  parameters.start_delta_theta = -angular_window;
  return parameters;
}

// Adding the candidate's whole-cell offset is only valid with floor discretization.
std::vector<std::vector<Eigen::Array2i>> DiscretizeScans(const sensor::PointCloud& point_cloud,
                                                         const mapping::GridMapu8& grid,
                                                         const SearchParameters& parameters) {
  std::vector<std::vector<Eigen::Array2i>> scans;
  scans.reserve(parameters.num_angles);
  for (int angle_index = 0; angle_index < parameters.num_angles; ++angle_index) {
    const double delta_theta = parameters.start_delta_theta + angle_index * parameters.angular_step;
    Eigen::Affine2d transform = Eigen::Affine2d::Identity();
    transform.linear() =
        Eigen::Rotation2Dd(delta_theta).toRotationMatrix() * parameters.base_pose.linear();
    transform.translation() = parameters.base_pose.translation();
    std::vector<Eigen::Array2i> cells;
    cells.reserve(point_cloud.size());
    for (const auto& point : point_cloud) {
      cells.push_back(grid.ToCell(transform * point.point));
    }
    scans.push_back(std::move(cells));
  }
  return scans;
}

struct Candidate {
  int angle_index = 0;
  int x = 0;
  int y = 0;
  int64_t score_sum = 0;

  bool EarlierThan(const Candidate& other) const {
    return std::tie(angle_index, x, y) < std::tie(other.angle_index, other.x, other.y);
  }
};

int64_t ScoreCandidate(const PrecomputationGrid& grid, const std::vector<Eigen::Array2i>& cells,
                       int x_offset, int y_offset) {
  int64_t sum = 0;
  for (const Eigen::Array2i& cell : cells) {
    sum += PointScore(grid.GetValue(cell.x() + x_offset, cell.y() + y_offset));
  }
  return sum;
}

void ScoreCandidates(const PrecomputationGrid& grid,
                     const std::vector<std::vector<Eigen::Array2i>>& scans,
                     std::vector<Candidate>& candidates) {
  for (Candidate& candidate : candidates) {
    candidate.score_sum =
        ScoreCandidate(grid, scans[candidate.angle_index], candidate.x, candidate.y);
  }
}

struct BestCandidate {
  bool found = false;
  Candidate candidate;
};

// Highest sum at or above the threshold, ties towards the smallest (angle index, x, y).
void ConsiderLeaf(const Candidate& leaf, int64_t threshold, BestCandidate& best) {
  if (!best.found) {
    if (leaf.score_sum >= threshold) {
      best.found = true;
      best.candidate = leaf;
    }
    return;
  }
  if (leaf.score_sum > best.candidate.score_sum ||
      (leaf.score_sum == best.candidate.score_sum && leaf.EarlierThan(best.candidate))) {
    best.candidate = leaf;
  }
}

// Subtrees whose bound equals the current best are still explored: one may hold an equal-score
// leaf that wins the deterministic tie-break.
void BranchAndBound(const PrecomputationGridStack& stack,
                    const std::vector<std::vector<Eigen::Array2i>>& scans,
                    const SearchParameters& parameters, std::vector<Candidate>& candidates,
                    int level, int64_t threshold, BestCandidate& best) {
  std::stable_sort(
      candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) { return a.score_sum > b.score_sum; });
  for (const Candidate& candidate : candidates) {
    const int64_t required = best.found ? best.candidate.score_sum : threshold;
    if (candidate.score_sum < required) {
      break;  // sorted descending, nothing later can reach the bar either
    }
    if (level == 0) {
      ConsiderLeaf(candidate, threshold, best);
      continue;
    }
    const int stride = 1 << (level - 1);
    std::vector<Candidate> children;
    children.reserve(4);
    for (int dy : {0, stride}) {
      for (int dx : {0, stride}) {
        if (candidate.x + dx > parameters.max_x || candidate.y + dy > parameters.max_y) {
          continue;
        }
        children.push_back(Candidate{candidate.angle_index, candidate.x + dx, candidate.y + dy, 0});
      }
    }
    ScoreCandidates(stack.Get(level - 1), scans, children);
    BranchAndBound(stack, scans, parameters, children, level - 1, threshold, best);
  }
}

Eigen::Affine2d CandidateToPose(const Candidate& candidate, const SearchParameters& parameters,
                                double resolution) {
  const double delta_theta =
      parameters.start_delta_theta + candidate.angle_index * parameters.angular_step;
  Eigen::Affine2d pose = Eigen::Affine2d::Identity();
  pose.linear() =
      Eigen::Rotation2Dd(delta_theta).toRotationMatrix() * parameters.base_pose.linear();
  pose.translation() = parameters.base_pose.translation() +
                       Eigen::Vector2d(candidate.x * resolution, candidate.y * resolution);
  return pose;
}

}  // namespace

FastCorrelativeScanMatcher::FastCorrelativeScanMatcher(
    const FastCorrelativeScanMatcherOption& option)
    : option_(option) {
  CHECK_GT(option_.branch_and_bound_depth, 0);
}

std::optional<GlobalMatchResult> FastCorrelativeScanMatcher::Match(
    const sensor::PointCloud& point_cloud, const std::vector<CandidateSubmap>& candidates,
    const std::optional<Eigen::Affine2d>& prior_pose, const MatchParams& params) const {
  return MatchInternal(point_cloud, candidates, prior_pose, params, true);
}

std::shared_ptr<const PrecomputationGridStack> FastCorrelativeScanMatcher::BuildPrecomputationStack(
    const mapping::GridMapu8& grid) const {
  return std::make_shared<const PrecomputationGridStack>(grid, option_.branch_and_bound_depth);
}

std::optional<GlobalMatchResult> FastCorrelativeScanMatcher::MatchBruteForce(
    const sensor::PointCloud& point_cloud, const std::vector<CandidateSubmap>& candidates,
    const std::optional<Eigen::Affine2d>& prior_pose, const MatchParams& params) const {
  return MatchInternal(point_cloud, candidates, prior_pose, params, false);
}

std::optional<GlobalMatchResult> FastCorrelativeScanMatcher::MatchInternal(
    const sensor::PointCloud& point_cloud, const std::vector<CandidateSubmap>& candidates,
    const std::optional<Eigen::Affine2d>& prior_pose, const MatchParams& params,
    bool use_branch_and_bound) const {
  if (point_cloud.empty()) {
    return std::nullopt;
  }
  CHECK(params.stack == nullptr || candidates.size() == 1);
  const double linear_search_window = params.linear_search_window;
  const double angular_search_window =
      params.angular_search_window.value_or(option_.angular_search_window);
  const int64_t threshold = ScoreSumThreshold(params.min_score, point_cloud.size());

  bool found = false;
  int64_t best_sum = 0;
  Eigen::Affine2d best_pose = Eigen::Affine2d::Identity();

  for (const CandidateSubmap& submap : candidates) {
    const mapping::GridMapu8& grid = submap.grid.get();
    if (grid.width() <= 0 || grid.height() <= 0) {
      continue;
    }
    std::optional<Eigen::Affine2d> prior_in_grid;
    if (prior_pose.has_value()) {
      prior_in_grid = submap.grid_to_global.inverse() * (*prior_pose);
    }
    const SearchParameters parameters = ComputeSearchParameters(
        point_cloud, grid, prior_in_grid, angular_search_window, linear_search_window);
    const std::vector<std::vector<Eigen::Array2i>> scans =
        DiscretizeScans(point_cloud, grid, parameters);

    BestCandidate best_in_submap;
    if (use_branch_and_bound) {
      const std::shared_ptr<const PrecomputationGridStack> owned =
          params.stack != nullptr ? params.stack : BuildPrecomputationStack(grid);
      const PrecomputationGridStack& stack = *owned;
      CHECK_EQ(stack.max_depth() + 1, option_.branch_and_bound_depth);
      const int max_level = stack.max_depth();
      const int stride = 1 << max_level;
      std::vector<Candidate> top_candidates;
      for (int angle_index = 0; angle_index < parameters.num_angles; ++angle_index) {
        for (int x = parameters.min_x; x <= parameters.max_x; x += stride) {
          for (int y = parameters.min_y; y <= parameters.max_y; y += stride) {
            top_candidates.push_back(Candidate{angle_index, x, y, 0});
          }
        }
      }
      ScoreCandidates(stack.Get(max_level), scans, top_candidates);
      BranchAndBound(stack, scans, parameters, top_candidates, max_level, threshold,
                     best_in_submap);
    } else {
      const PrecomputationGrid full_resolution = PrecomputationGrid::FromGrid(grid);
      for (int angle_index = 0; angle_index < parameters.num_angles; ++angle_index) {
        for (int x = parameters.min_x; x <= parameters.max_x; ++x) {
          for (int y = parameters.min_y; y <= parameters.max_y; ++y) {
            Candidate leaf{angle_index, x, y, 0};
            leaf.score_sum = ScoreCandidate(full_resolution, scans[angle_index], x, y);
            ConsiderLeaf(leaf, threshold, best_in_submap);
          }
        }
      }
    }

    // Strictly greater, so a cross-submap tie keeps the earlier candidate submap.
    if (best_in_submap.found && (!found || best_in_submap.candidate.score_sum > best_sum)) {
      found = true;
      best_sum = best_in_submap.candidate.score_sum;
      const Eigen::Affine2d pose_in_grid =
          CandidateToPose(best_in_submap.candidate, parameters, grid.resolution());
      best_pose = submap.grid_to_global * pose_in_grid;
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return GlobalMatchResult{best_pose, ScoreFromSum(best_sum, point_cloud.size())};
}

}  // namespace evergreenslam::utils::scan_matching

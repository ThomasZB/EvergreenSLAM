/**
 * @file ceres_scan_matcher.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "utils/scan_matching/ceres_scan_matcher.h"

#include <cmath>

#include "utils/cost_functor/grid_match_cost_function.h"
#include "utils/cost_functor/pose_prior_cost_function.h"
#include "utils/transform/transform.h"

namespace evergreenslam::utils::scan_matching {

CeresScanMatcher::CeresScanMatcher(double grid_match_weight)
    : grid_match_weight_(grid_match_weight),
      use_pose_prior_(false),
      translation_weight_(1.0),
      rotation_weight_(1.0) {}

void CeresScanMatcher::SetPosePrior(double translation_weight, double rotation_weight) {
  use_pose_prior_ = true;
  translation_weight_ = translation_weight;
  rotation_weight_ = rotation_weight;
}

void CeresScanMatcher::Match(const sensor::PointCloud& point_cloud,
                             const mapping::GridMapu8& grid_map, Eigen::Affine2d& initial_pose) {
  Match(point_cloud, grid_map, initial_pose, initial_pose);
}

void CeresScanMatcher::Match(const sensor::PointCloud& point_cloud,
                             const mapping::GridMapu8& grid_map, Eigen::Affine2d& initial_pose,
                             const Eigen::Affine2d& prior_pose) {
  std::array<double, 3> initial_pose_array = transform::ToArray3(initial_pose);

  ceres::Problem problem;
  problem.AddParameterBlock(initial_pose_array.data(), 3);
  problem.AddResidualBlock(
      cost_functor::CreateGridMatchCostFunction(
          point_cloud, grid_map,
          grid_match_weight_ / std::sqrt(static_cast<double>(point_cloud.size()))),
      nullptr, initial_pose_array.data());
  if (use_pose_prior_) {
    problem.AddResidualBlock(cost_functor::CreatePosePriorCostFunction(
                                 prior_pose, translation_weight_, rotation_weight_),
                             nullptr, initial_pose_array.data());
  }

  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = false;
  options.max_num_iterations = 10;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  initial_pose = transform::FromArray3(initial_pose_array);
}

}  // namespace evergreenslam::utils::scan_matching

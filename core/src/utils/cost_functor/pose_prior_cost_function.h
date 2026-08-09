/**
 * @file pose_prior_cost_function.h
 * @author hang chen (chen@hang.plus)
 * @brief Residual between a pose being solved for and a fixed prior pose.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_COST_FUNCTOR_POSE_PRIOR_COST_FUNCTION_H_
#define EVERGREENSLAM_UTILS_COST_FUNCTOR_POSE_PRIOR_COST_FUNCTION_H_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace evergreenslam::utils::cost_functor {

ceres::CostFunction* CreatePosePriorCostFunction(const Eigen::Affine2d& prior_pose,
                                                 const double translation_weight,
                                                 const double rotation_weight);

}  // namespace evergreenslam::utils::cost_functor

#endif  // EVERGREENSLAM_UTILS_COST_FUNCTOR_POSE_PRIOR_COST_FUNCTION_H_

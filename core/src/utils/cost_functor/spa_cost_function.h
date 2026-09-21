/**
 * @file spa_cost_function.h
 * @author hang chen (chen@hang.plus)
 * @brief Pose graph residuals: a relative pose between two poses, and an absolute pose prior.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_COST_FUNCTOR_SPA_COST_FUNCTION_H_
#define EVERGREENSLAM_UTILS_COST_FUNCTOR_SPA_COST_FUNCTION_H_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace evergreenslam::utils::cost_functor {

// Both parameter blocks are [x, y, theta] in the same frame.
ceres::CostFunction* CreateSpaRelativePoseCostFunction(const Eigen::Affine2d& relative_pose,
                                                       const Eigen::Matrix3d& sqrt_information);

// Not interchangeable with CreatePosePriorCostFunction, which weights per degree of freedom.
ceres::CostFunction* CreateSpaAbsolutePoseCostFunction(const Eigen::Affine2d& absolute_pose,
                                                       const Eigen::Matrix3d& sqrt_information);

}  // namespace evergreenslam::utils::cost_functor

#endif  // EVERGREENSLAM_UTILS_COST_FUNCTOR_SPA_COST_FUNCTION_H_

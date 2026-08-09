/**
 * @file pose_prior_cost_function.cc
 * @author hang chen (chen@hang.plus)
 * @brief Residual between a pose being solved for and a fixed prior pose.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/cost_functor/pose_prior_cost_function.h"

#include "utils/transform/transform.h"

namespace evergreenslam::utils::cost_functor {
namespace {

class PosePriorCostFunction {
 public:
  PosePriorCostFunction(const Eigen::Affine2d& prior_pose, const double translation_weight,
                        const double rotation_weight)
      : prior_x_(prior_pose.translation().x()),
        prior_y_(prior_pose.translation().y()),
        prior_theta_(Eigen::Rotation2Dd(prior_pose.rotation()).angle()),
        translation_weight_(translation_weight),
        rotation_weight_(rotation_weight) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = translation_weight_ * (pose[0] - prior_x_);
    residual[1] = translation_weight_ * (pose[1] - prior_y_);
    residual[2] = rotation_weight_ * transform::NormalizeAngle(pose[2] - T(prior_theta_));
    return true;
  }

 private:
  const double prior_x_;
  const double prior_y_;
  const double prior_theta_;
  const double translation_weight_;
  const double rotation_weight_;
};

}  // namespace

ceres::CostFunction* CreatePosePriorCostFunction(const Eigen::Affine2d& prior_pose,
                                                 const double translation_weight,
                                                 const double rotation_weight) {
  return new ceres::AutoDiffCostFunction<PosePriorCostFunction, 3 /* residuals */,
                                         3 /* pose variables */>(
      new PosePriorCostFunction(prior_pose, translation_weight, rotation_weight));
}

}  // namespace evergreenslam::utils::cost_functor

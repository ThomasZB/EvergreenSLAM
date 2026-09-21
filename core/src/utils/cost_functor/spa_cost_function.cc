/**
 * @file spa_cost_function.cc
 * @author hang chen (chen@hang.plus)
 * @brief Pose graph residuals: a relative pose between two poses, and an absolute pose prior.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/cost_functor/spa_cost_function.h"

#include <array>

#include "utils/transform/transform.h"

namespace evergreenslam::utils::cost_functor {
namespace {

std::array<double, 9> ToRowMajor(const Eigen::Matrix3d& matrix) {
  std::array<double, 9> values{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      values[3 * row + col] = matrix(row, col);
    }
  }
  return values;
}

// Eigen refuses a double matrix times a Jet vector, so the weighting is spelled out.
template <typename T>
void ApplySqrtInformation(const std::array<double, 9>& sqrt_information, const T* const error,
                          T* residual) {
  for (int row = 0; row < 3; ++row) {
    residual[row] = T(sqrt_information[3 * row]) * error[0] +
                    T(sqrt_information[3 * row + 1]) * error[1] +
                    T(sqrt_information[3 * row + 2]) * error[2];
  }
}

class SpaRelativePoseCostFunction {
 public:
  SpaRelativePoseCostFunction(const Eigen::Affine2d& relative_pose,
                              const Eigen::Matrix3d& sqrt_information)
      : relative_x_(relative_pose.translation().x()),
        relative_y_(relative_pose.translation().y()),
        relative_theta_(transform::GetYaw(relative_pose)),
        sqrt_information_(ToRowMajor(sqrt_information)) {}

  template <typename T>
  bool operator()(const T* const from, const T* const to, T* residual) const {
    const T cos_from = ceres::cos(from[2]);
    const T sin_from = ceres::sin(from[2]);
    const T delta_x = to[0] - from[0];
    const T delta_y = to[1] - from[1];

    T error[3];
    error[0] = cos_from * delta_x + sin_from * delta_y - T(relative_x_);
    error[1] = -sin_from * delta_x + cos_from * delta_y - T(relative_y_);
    error[2] = transform::NormalizeAngle(to[2] - from[2] - T(relative_theta_));

    ApplySqrtInformation(sqrt_information_, error, residual);
    return true;
  }

 private:
  const double relative_x_;
  const double relative_y_;
  const double relative_theta_;
  const std::array<double, 9> sqrt_information_;
};

class SpaAbsolutePoseCostFunction {
 public:
  SpaAbsolutePoseCostFunction(const Eigen::Affine2d& absolute_pose,
                              const Eigen::Matrix3d& sqrt_information)
      : absolute_x_(absolute_pose.translation().x()),
        absolute_y_(absolute_pose.translation().y()),
        absolute_theta_(transform::GetYaw(absolute_pose)),
        sqrt_information_(ToRowMajor(sqrt_information)) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    T error[3];
    error[0] = pose[0] - T(absolute_x_);
    error[1] = pose[1] - T(absolute_y_);
    error[2] = transform::NormalizeAngle(pose[2] - T(absolute_theta_));

    ApplySqrtInformation(sqrt_information_, error, residual);
    return true;
  }

 private:
  const double absolute_x_;
  const double absolute_y_;
  const double absolute_theta_;
  const std::array<double, 9> sqrt_information_;
};

}  // namespace

ceres::CostFunction* CreateSpaRelativePoseCostFunction(const Eigen::Affine2d& relative_pose,
                                                       const Eigen::Matrix3d& sqrt_information) {
  return new ceres::AutoDiffCostFunction<SpaRelativePoseCostFunction, 3 /* residuals */,
                                         3 /* from */, 3 /* to */>(
      new SpaRelativePoseCostFunction(relative_pose, sqrt_information));
}

ceres::CostFunction* CreateSpaAbsolutePoseCostFunction(const Eigen::Affine2d& absolute_pose,
                                                       const Eigen::Matrix3d& sqrt_information) {
  return new ceres::AutoDiffCostFunction<SpaAbsolutePoseCostFunction, 3 /* residuals */,
                                         3 /* pose */>(
      new SpaAbsolutePoseCostFunction(absolute_pose, sqrt_information));
}

}  // namespace evergreenslam::utils::cost_functor

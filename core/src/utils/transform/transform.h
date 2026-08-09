/**
 * @file transform.h
 * @author hang chen (chen@hang.plus)
 * @brief 2D rigid transform helpers.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_TRANSFORM_TRANSFORM_H_
#define EVERGREENSLAM_UTILS_TRANSFORM_TRANSFORM_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>

namespace evergreenslam::utils::transform {

inline double GetYaw(const Eigen::Affine2d& transform) {
  return Eigen::Rotation2Dd(transform.linear()).angle();
}

template <typename T>
T NormalizeAngle(T angle) {
  while (angle > T(M_PI)) {
    angle -= T(2.0 * M_PI);
  }
  while (angle < -T(M_PI)) {
    angle += T(2.0 * M_PI);
  }
  return angle;
}

inline Eigen::Affine2d FromXYTheta(double x, double y, double theta) {
  Eigen::Affine2d pose = Eigen::Affine2d::Identity();
  pose.linear() = Eigen::Rotation2Dd(theta).toRotationMatrix();
  pose.translation() = Eigen::Vector2d(x, y);
  return pose;
}

inline std::array<double, 3> ToArray3(const Eigen::Affine2d& pose) {
  return {pose.translation().x(), pose.translation().y(), GetYaw(pose)};
}

inline Eigen::Matrix<double, 3, 1> ToVector3(const Eigen::Affine2d& pose) {
  Eigen::Matrix<double, 3, 1> vec;
  vec << pose.translation().x(), pose.translation().y(), GetYaw(pose);
  return vec;
}

inline Eigen::Affine2d FromArray3(const std::array<double, 3>& pose_array) {
  return FromXYTheta(pose_array[0], pose_array[1], pose_array[2]);
}

// Extrapolates when factor is outside [0, 1], which is how the constant
// velocity model predicts forward.
inline Eigen::Affine2d Interpolate(const Eigen::Affine2d& start, const Eigen::Affine2d& end,
                                   double factor) {
  const Eigen::Vector2d translation =
      start.translation() + factor * (end.translation() - start.translation());
  const double start_yaw = GetYaw(start);
  const double delta_yaw = NormalizeAngle(GetYaw(end) - start_yaw);
  return FromXYTheta(translation.x(), translation.y(),
                     NormalizeAngle(start_yaw + factor * delta_yaw));
}

}  // namespace evergreenslam::utils::transform

#endif  // EVERGREENSLAM_UTILS_TRANSFORM_TRANSFORM_H_

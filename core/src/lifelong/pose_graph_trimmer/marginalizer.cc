/**
 * @file marginalizer.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/marginalizer.h"

#include <glog/logging.h>

#include <cmath>

#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr int kDim = 3;
// Stands in for I -> infinity; still finite so the tree search stays well defined.
constexpr double kSaturatedMutualInformation = 1e12;

}  // namespace

Eigen::MatrixXd ComputeMutualInformationMatrix(const Eigen::MatrixXd& joint_covariance) {
  CHECK_EQ(joint_covariance.rows(), joint_covariance.cols());
  CHECK_EQ(joint_covariance.rows() % kDim, 0);
  const int n = static_cast<int>(joint_covariance.rows()) / kDim;

  Eigen::MatrixXd mutual_information = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const Eigen::Matrix<double, 6, 6> pair = ExtractPairCovariance(joint_covariance, i, j);
      const double det_ii = pair.topLeftCorner<kDim, kDim>().determinant();
      const double det_jj = pair.bottomRightCorner<kDim, kDim>().determinant();
      const double det_joint = pair.determinant();
      double information = kSaturatedMutualInformation;
      if (det_joint > 0.0 && det_ii > 0.0 && det_jj > 0.0) {
        information = 0.5 * std::log(det_ii * det_jj / det_joint);
        // Independence can come out at -1e-16 numerically; mutual information is nonnegative.
        information = std::max(information, 0.0);
        information = std::min(information, kSaturatedMutualInformation);
      }
      mutual_information(i, j) = information;
      mutual_information(j, i) = information;
    }
  }
  return mutual_information;
}

Eigen::Matrix<double, 6, 6> ExtractPairCovariance(const Eigen::MatrixXd& joint_covariance, int i,
                                                  int j) {
  CHECK_EQ(joint_covariance.rows(), joint_covariance.cols());
  const int n = static_cast<int>(joint_covariance.rows()) / kDim;
  CHECK_GE(i, 0);
  CHECK_GE(j, 0);
  CHECK_LT(i, n);
  CHECK_LT(j, n);
  CHECK_NE(i, j);

  Eigen::Matrix<double, 6, 6> pair;
  pair.topLeftCorner<kDim, kDim>() = joint_covariance.block<kDim, kDim>(i * kDim, i * kDim);
  pair.topRightCorner<kDim, kDim>() = joint_covariance.block<kDim, kDim>(i * kDim, j * kDim);
  pair.bottomLeftCorner<kDim, kDim>() = joint_covariance.block<kDim, kDim>(j * kDim, i * kDim);
  pair.bottomRightCorner<kDim, kDim>() = joint_covariance.block<kDim, kDim>(j * kDim, j * kDim);
  return pair;
}

RelativePoseDistribution ComputeRelativePoseDistribution(
    const Eigen::Affine2d& pose_i, const Eigen::Affine2d& pose_j,
    const Eigen::Matrix<double, 6, 6>& pair_covariance) {
  const double theta_i = transform::GetYaw(pose_i);
  const double cos_i = std::cos(theta_i);
  const double sin_i = std::sin(theta_i);
  const double dx = pose_j.translation().x() - pose_i.translation().x();
  const double dy = pose_j.translation().y() - pose_i.translation().y();

  Eigen::Matrix<double, 3, 6> jacobian;
  jacobian << -cos_i, -sin_i, -sin_i * dx + cos_i * dy, cos_i, sin_i, 0.0,  //
      sin_i, -cos_i, -cos_i * dx - sin_i * dy, -sin_i, cos_i, 0.0,          //
      0.0, 0.0, -1.0, 0.0, 0.0, 1.0;

  RelativePoseDistribution result;
  result.mean = pose_i.inverse() * pose_j;
  result.covariance = jacobian * pair_covariance * jacobian.transpose();
  return result;
}

std::optional<Eigen::Matrix3d> SqrtInformationFromCovariance(const Eigen::Matrix3d& covariance) {
  // Positive definiteness is checked by the factorization itself.
  const Eigen::LLT<Eigen::Matrix3d> llt(covariance);
  if (llt.info() != Eigen::Success) {
    return std::nullopt;
  }
  const Eigen::Matrix3d lower = llt.matrixL();
  const Eigen::Matrix3d lower_inverse =
      lower.triangularView<Eigen::Lower>().solve(Eigen::Matrix3d::Identity());
  if (!lower_inverse.allFinite()) {
    return std::nullopt;
  }
  return lower_inverse;
}

std::optional<Eigen::Matrix3d> FlooredSqrtInformation(const Eigen::Matrix3d& covariance,
                                                      double min_stddev) {
  CHECK_GE(min_stddev, 0.0);
  return SqrtInformationFromCovariance(covariance +
                                       min_stddev * min_stddev * Eigen::Matrix3d::Identity());
}

std::optional<MarginalConstraint> MakeMarginalConstraint(const std::vector<Eigen::Affine2d>& poses,
                                                         const Eigen::MatrixXd& joint_covariance,
                                                         int i, int j, double min_stddev) {
  CHECK_LT(i, static_cast<int>(poses.size()));
  CHECK_LT(j, static_cast<int>(poses.size()));

  const Eigen::Matrix<double, 6, 6> pair = ExtractPairCovariance(joint_covariance, i, j);
  // Trace mixes m^2 and rad^2 but only ranks the two ends.
  const double trace_i = pair.topLeftCorner<kDim, kDim>().trace();
  const double trace_j = pair.bottomRightCorner<kDim, kDim>().trace();
  const bool i_is_from = trace_i <= trace_j;
  const int from = i_is_from ? i : j;
  const int to = i_is_from ? j : i;
  const Eigen::Matrix<double, 6, 6> oriented_pair =
      i_is_from ? pair : ExtractPairCovariance(joint_covariance, j, i);

  const RelativePoseDistribution distribution =
      ComputeRelativePoseDistribution(poses[from], poses[to], oriented_pair);
  const std::optional<Eigen::Matrix3d> sqrt_information =
      FlooredSqrtInformation(distribution.covariance, min_stddev);
  if (!sqrt_information.has_value()) {
    return std::nullopt;
  }

  MarginalConstraint constraint;
  constraint.from = from;
  constraint.to = to;
  constraint.relative_pose = distribution.mean;
  constraint.sqrt_information = *sqrt_information;
  return constraint;
}

}  // namespace evergreenslam::lifelong

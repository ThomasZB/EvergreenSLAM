/**
 * @file marginalizer.h
 * @author hang chen (chen@hang.plus)
 * @brief Mutual information and recovered relative-pose constraints, on compact indices.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_MARGINALIZER_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_MARGINALIZER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <vector>

namespace evergreenslam::lifelong {

// I(i, j) = 0.5 * log(|Sigma_ii| |Sigma_jj| / |Sigma_joint|).
Eigen::MatrixXd ComputeMutualInformationMatrix(const Eigen::MatrixXd& joint_covariance);

Eigen::Matrix<double, 6, 6> ExtractPairCovariance(const Eigen::MatrixXd& joint_covariance, int i,
                                                  int j);

struct RelativePoseDistribution {
  Eigen::Affine2d mean = Eigen::Affine2d::Identity();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
};

// Linearized under additive [x, y, theta] perturbations, matching the Ceres parameter block.
RelativePoseDistribution ComputeRelativePoseDistribution(
    const Eigen::Affine2d& pose_i, const Eigen::Affine2d& pose_j,
    const Eigen::Matrix<double, 6, 6>& pair_covariance);

// Returns S = L^-1 with covariance = L L^T, so S^T S = covariance^-1 and |S * error|^2 is the
// Mahalanobis distance. Empty when the covariance is not positive definite.
std::optional<Eigen::Matrix3d> SqrtInformationFromCovariance(const Eigen::Matrix3d& covariance);

// The per-axis floor every constraint the trimmer writes goes through, binary or prior.
std::optional<Eigen::Matrix3d> FlooredSqrtInformation(const Eigen::Matrix3d& covariance,
                                                      double min_stddev);

struct MarginalConstraint {
  int from = 0;
  int to = 0;
  Eigen::Affine2d relative_pose = Eigen::Affine2d::Identity();
  Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
};

// `from` is the smaller-covariance end. Empty when the floored covariance cannot be inverted.
std::optional<MarginalConstraint> MakeMarginalConstraint(const std::vector<Eigen::Affine2d>& poses,
                                                         const Eigen::MatrixXd& joint_covariance,
                                                         int i, int j, double min_stddev);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_MARGINALIZER_H_

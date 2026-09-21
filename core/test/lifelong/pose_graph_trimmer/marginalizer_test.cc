/**
 * @file marginalizer_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The three brute-force comparisons the plan mandates: Schur, sampled MI, Monte-Carlo
 *        covariance propagation.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/marginalizer.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Portable Gaussian sampler: mt19937 is bit-exact across platforms, std::normal_distribution
// is not, and the tolerances below assume the same draws everywhere.
class PortableGaussian {
 public:
  explicit PortableGaussian(std::uint32_t seed) : engine_(seed) {}

  double Uniform() { return (static_cast<double>(engine_()) + 0.5) / 4294967296.0; }

  double Sample() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    const double radius = std::sqrt(-2.0 * std::log(Uniform()));
    const double angle = 2.0 * M_PI * Uniform();
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return radius * std::cos(angle);
  }

  Eigen::VectorXd SampleVector(int size) {
    Eigen::VectorXd vector(size);
    for (int i = 0; i < size; ++i) {
      vector(i) = Sample();
    }
    return vector;
  }

 private:
  std::mt19937 engine_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

// A cross block with positive determinant, so the design doc's wrong formula produces a finite
// and very different value.
Matrix6d TestJointCovariance() {
  Eigen::Matrix3d sigma_ii = Eigen::Vector3d(1.0, 0.7, 0.4).asDiagonal();
  Eigen::Matrix3d sigma_jj = Eigen::Vector3d(0.9, 0.6, 0.3).asDiagonal();
  Eigen::Matrix3d sigma_ij;
  sigma_ij << 0.2, 0.05, 0.0,  //
      0.0, 0.15, 0.02,         //
      0.01, 0.0, 0.1;
  Matrix6d joint;
  joint.topLeftCorner<3, 3>() = sigma_ii;
  joint.topRightCorner<3, 3>() = sigma_ij;
  joint.bottomLeftCorner<3, 3>() = sigma_ij.transpose();
  joint.bottomRightCorner<3, 3>() = sigma_jj;
  return joint;
}

double GaussianLogDensity(const Eigen::VectorXd& x, const Eigen::MatrixXd& information,
                          double log_det_covariance) {
  const double k = static_cast<double>(x.size());
  return -0.5 * (x.transpose() * information * x)(0, 0) -
         0.5 * (k * std::log(2.0 * M_PI) + log_det_covariance);
}

TEST(MarginalizerTest, MutualInformationMatchesMonteCarloAndTheWrongFormulaDoesNot) {
  const Matrix6d joint = TestJointCovariance();
  ASSERT_EQ(Eigen::LLT<Matrix6d>(joint).info(), Eigen::Success)
      << "the test covariance has to be positive definite";

  const Eigen::Matrix3d sigma_ii = joint.topLeftCorner<3, 3>();
  const Eigen::Matrix3d sigma_jj = joint.bottomRightCorner<3, 3>();
  const Eigen::Matrix3d sigma_ij = joint.topRightCorner<3, 3>();

  const Eigen::MatrixXd mutual_information = ComputeMutualInformationMatrix(joint);
  ASSERT_EQ(mutual_information.rows(), 2);
  const double closed_form = mutual_information(0, 1);
  EXPECT_EQ(mutual_information(0, 1), mutual_information(1, 0));
  EXPECT_EQ(mutual_information(0, 0), 0.0);
  EXPECT_EQ(mutual_information(1, 1), 0.0);
  EXPECT_DOUBLE_EQ(closed_form, 0.5 * std::log(sigma_ii.determinant() * sigma_jj.determinant() /
                                               joint.determinant()));

  // Monte Carlo estimate of the mutual information that does not go through either formula.
  const Matrix6d lower = Eigen::LLT<Matrix6d>(joint).matrixL();
  const Eigen::MatrixXd joint_information = joint.inverse();
  const Eigen::Matrix3d information_ii = sigma_ii.inverse();
  const Eigen::Matrix3d information_jj = sigma_jj.inverse();
  const double log_det_joint = std::log(joint.determinant());
  const double log_det_ii = std::log(sigma_ii.determinant());
  const double log_det_jj = std::log(sigma_jj.determinant());

  PortableGaussian random(20260809);
  constexpr int kNumSamples = 300000;
  double sum = 0.0;
  for (int n = 0; n < kNumSamples; ++n) {
    const Eigen::VectorXd sample = lower * random.SampleVector(6);
    const Eigen::VectorXd head = sample.head(3);
    const Eigen::VectorXd tail = sample.tail(3);
    sum += GaussianLogDensity(sample, joint_information, log_det_joint) -
           GaussianLogDensity(head, information_ii, log_det_ii) -
           GaussianLogDensity(tail, information_jj, log_det_jj);
  }
  const double monte_carlo = sum / kNumSamples;

  EXPECT_NEAR(closed_form, monte_carlo, 0.03);

  // Regression guard: the design doc's formula divides by |Sigma_ij| instead of the joint
  // determinant, and must NOT match the sampled truth.
  const double wrong_formula =
      0.5 * std::log(sigma_ii.determinant() * sigma_jj.determinant() / sigma_ij.determinant());
  EXPECT_GT(std::abs(wrong_formula - monte_carlo), 0.5);
  EXPECT_GT(std::abs(wrong_formula - monte_carlo), 10.0 * std::abs(closed_form - monte_carlo));
}

TEST(MarginalizerTest, PairCovarianceMatchesDenseSchurComplementMarginalization) {
  constexpr int kNumVariables = 5;
  constexpr int kDims = kNumVariables * 3;
  PortableGaussian random(97);
  Eigen::MatrixXd factor(kDims, kDims);
  for (int r = 0; r < kDims; ++r) {
    for (int c = 0; c < kDims; ++c) {
      factor(r, c) = random.Sample();
    }
  }
  const Eigen::MatrixXd information =
      factor * factor.transpose() + 0.5 * Eigen::MatrixXd::Identity(kDims, kDims);
  const Eigen::MatrixXd covariance = information.inverse();

  const std::vector<std::pair<int, int>> pairs = {{0, 3}, {1, 4}, {2, 3}, {0, 1}};
  for (const auto& [i, j] : pairs) {
    std::vector<int> rest;
    for (int v = 0; v < kNumVariables; ++v) {
      if (v != i && v != j) {
        rest.push_back(v);
      }
    }
    Eigen::MatrixXd lambda_pp(6, 6);
    Eigen::MatrixXd lambda_pr(6, kDims - 6);
    Eigen::MatrixXd lambda_rr(kDims - 6, kDims - 6);
    const std::vector<int> kept = {i, j};
    for (int a = 0; a < 2; ++a) {
      for (int b = 0; b < 2; ++b) {
        lambda_pp.block<3, 3>(a * 3, b * 3) = information.block<3, 3>(kept[a] * 3, kept[b] * 3);
      }
      for (size_t b = 0; b < rest.size(); ++b) {
        lambda_pr.block<3, 3>(a * 3, static_cast<int>(b) * 3) =
            information.block<3, 3>(kept[a] * 3, rest[b] * 3);
      }
    }
    for (size_t a = 0; a < rest.size(); ++a) {
      for (size_t b = 0; b < rest.size(); ++b) {
        lambda_rr.block<3, 3>(static_cast<int>(a) * 3, static_cast<int>(b) * 3) =
            information.block<3, 3>(rest[a] * 3, rest[b] * 3);
      }
    }
    const Eigen::MatrixXd schur =
        lambda_pp - lambda_pr * lambda_rr.inverse() * lambda_pr.transpose();
    const Eigen::MatrixXd brute_force = schur.inverse();

    const Matrix6d extracted = ExtractPairCovariance(covariance, i, j);
    EXPECT_LT((extracted - brute_force).norm(), 1e-9 * brute_force.norm())
        << "pair (" << i << ", " << j << ")";
  }
}

TEST(MarginalizerTest, RelativePoseCovarianceMatchesMonteCarloPropagation) {
  const Eigen::Affine2d pose_i = transform::FromXYTheta(1.213, -0.707, 0.351);
  const Eigen::Affine2d pose_j = transform::FromXYTheta(2.117, 0.409, -0.523);

  // Centimetre-grade uncertainty, small enough that the linearization dominates its own error.
  PortableGaussian random(4242);
  Matrix6d factor;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      factor(r, c) = 0.01 * random.Sample();
    }
  }
  const Matrix6d pair_covariance = factor * factor.transpose() + 1e-6 * Matrix6d::Identity();

  const RelativePoseDistribution predicted =
      ComputeRelativePoseDistribution(pose_i, pose_j, pair_covariance);
  const Eigen::Vector3d mean_prediction = transform::ToVector3(predicted.mean);
  const Eigen::Vector3d direct = transform::ToVector3(Eigen::Affine2d(pose_i.inverse() * pose_j));
  EXPECT_LT((mean_prediction - direct).norm(), 1e-12);

  const Matrix6d lower = Eigen::LLT<Matrix6d>(pair_covariance).matrixL();
  constexpr int kNumSamples = 200000;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();
  for (int n = 0; n < kNumSamples; ++n) {
    const Eigen::VectorXd delta = lower * random.SampleVector(6);
    const Eigen::Affine2d perturbed_i = transform::FromXYTheta(
        pose_i.translation().x() + delta(0), pose_i.translation().y() + delta(1),
        transform::GetYaw(pose_i) + delta(2));
    const Eigen::Affine2d perturbed_j = transform::FromXYTheta(
        pose_j.translation().x() + delta(3), pose_j.translation().y() + delta(4),
        transform::GetYaw(pose_j) + delta(5));
    const Eigen::Affine2d relative = perturbed_i.inverse() * perturbed_j;
    Eigen::Vector3d error = transform::ToVector3(relative) - direct;
    error(2) = transform::NormalizeAngle(error(2));
    sum += error;
    sum_outer += error * error.transpose();
  }
  const Eigen::Vector3d mean = sum / kNumSamples;
  const Eigen::Matrix3d monte_carlo = sum_outer / kNumSamples - mean * mean.transpose();

  EXPECT_LT((monte_carlo - predicted.covariance).norm(), 0.05 * predicted.covariance.norm());
}

TEST(MarginalizerTest, SqrtInformationSatisfiesItsConvention) {
  Eigen::Matrix3d covariance;
  covariance << 4e-4, 1e-4, 0.0,  //
      1e-4, 9e-4, 2e-5,           //
      0.0, 2e-5, 1e-4;
  const std::optional<Eigen::Matrix3d> sqrt_information = SqrtInformationFromCovariance(covariance);
  ASSERT_TRUE(sqrt_information.has_value());
  const Eigen::Matrix3d reconstructed = sqrt_information->transpose() * (*sqrt_information);
  const Eigen::Matrix3d information = covariance.inverse();
  EXPECT_LT((reconstructed - information).norm(), 1e-9 * information.norm());

  EXPECT_FALSE(SqrtInformationFromCovariance(Eigen::Matrix3d::Zero()).has_value());
  EXPECT_FALSE(SqrtInformationFromCovariance(-Eigen::Matrix3d::Identity()).has_value());
}

TEST(MarginalizerTest, FlooredSqrtInformationAddsTheFloorBeforeInverting) {
  Eigen::Matrix3d covariance;
  covariance << 4e-4, 1e-4, 0.0,  //
      1e-4, 9e-4, 2e-5,           //
      0.0, 2e-5, 1e-4;
  const std::optional<Eigen::Matrix3d> floored = FlooredSqrtInformation(covariance, 0.05);
  ASSERT_TRUE(floored.has_value());
  const Eigen::Matrix3d recovered = (floored->transpose() * (*floored)).inverse();
  const Eigen::Matrix3d expected = covariance + 0.05 * 0.05 * Eigen::Matrix3d::Identity();
  EXPECT_LT((recovered - expected).norm(), 1e-9 * expected.norm());

  // A zero covariance is not invertible on its own; the floor makes it so.
  const std::optional<Eigen::Matrix3d> from_zero =
      FlooredSqrtInformation(Eigen::Matrix3d::Zero(), 0.05);
  ASSERT_TRUE(from_zero.has_value());
  EXPECT_LT(
      (from_zero->transpose() * (*from_zero) - Eigen::Matrix3d::Identity() / (0.05 * 0.05)).norm(),
      1e-6 / (0.05 * 0.05));
  EXPECT_FALSE(FlooredSqrtInformation(Eigen::Matrix3d::Zero(), 0.0).has_value());
}

TEST(MarginalizerTest, MakeMarginalConstraintPutsFromAtTheSmallerCovarianceEnd) {
  // Variable 0 tight, variable 1 loose: from must be 0 whichever way the pair is asked for.
  Eigen::MatrixXd joint = Eigen::MatrixXd::Zero(6, 6);
  joint.topLeftCorner<3, 3>() = 1e-4 * Eigen::Matrix3d::Identity();
  joint.bottomRightCorner<3, 3>() = 1e-2 * Eigen::Matrix3d::Identity();
  joint.topRightCorner<3, 3>() = 1e-5 * Eigen::Matrix3d::Identity();
  joint.bottomLeftCorner<3, 3>() = 1e-5 * Eigen::Matrix3d::Identity();

  const std::vector<Eigen::Affine2d> poses = {transform::FromXYTheta(0.4, 0.2, 0.1),
                                              transform::FromXYTheta(1.7, -0.3, 0.9)};
  for (const auto& [i, j] : std::vector<std::pair<int, int>>{{0, 1}, {1, 0}}) {
    const std::optional<MarginalConstraint> constraint =
        MakeMarginalConstraint(poses, joint, i, j, 1e-2);
    ASSERT_TRUE(constraint.has_value());
    EXPECT_EQ(constraint->from, 0);
    EXPECT_EQ(constraint->to, 1);
    const Eigen::Vector3d expected =
        transform::ToVector3(Eigen::Affine2d(poses[0].inverse() * poses[1]));
    EXPECT_LT((transform::ToVector3(constraint->relative_pose) - expected).norm(), 1e-12);
    // The floor keeps any recovered constraint at or below the raw-measurement weight.
    const Eigen::Matrix3d recovered_covariance =
        (constraint->sqrt_information.transpose() * constraint->sqrt_information).inverse();
    EXPECT_GE(recovered_covariance(0, 0), 1e-4 - 1e-12);
  }
}

}  // namespace
}  // namespace evergreenslam::lifelong

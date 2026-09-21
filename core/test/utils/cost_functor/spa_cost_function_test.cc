/**
 * @file spa_cost_function_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Sign, frame and weighting conventions of the pose graph residuals.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/cost_functor/spa_cost_function.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>

#include "utils/transform/transform.h"

namespace evergreenslam::utils::cost_functor {
namespace {

constexpr double kEpsilon = 1e-12;

std::array<double, 3> EvaluateRelative(const Eigen::Affine2d& relative_pose,
                                       const Eigen::Matrix3d& sqrt_information,
                                       const Eigen::Affine2d& from, const Eigen::Affine2d& to) {
  const std::unique_ptr<ceres::CostFunction> cost(
      CreateSpaRelativePoseCostFunction(relative_pose, sqrt_information));
  std::array<double, 3> from_block = transform::ToArray3(from);
  std::array<double, 3> to_block = transform::ToArray3(to);
  const double* blocks[] = {from_block.data(), to_block.data()};
  std::array<double, 3> residual{};
  EXPECT_TRUE(cost->Evaluate(blocks, residual.data(), nullptr));
  return residual;
}

std::array<double, 3> EvaluateAbsolute(const Eigen::Affine2d& absolute_pose,
                                       const Eigen::Matrix3d& sqrt_information,
                                       const std::array<double, 3>& pose_block) {
  const std::unique_ptr<ceres::CostFunction> cost(
      CreateSpaAbsolutePoseCostFunction(absolute_pose, sqrt_information));
  const double* blocks[] = {pose_block.data()};
  std::array<double, 3> residual{};
  EXPECT_TRUE(cost->Evaluate(blocks, residual.data(), nullptr));
  return residual;
}

TEST(SpaCostFunction, RelativeResidualVanishesOnTheMeasurement) {
  const Eigen::Affine2d from = transform::FromXYTheta(3.0, -1.5, 0.7);
  const Eigen::Affine2d relative_pose = transform::FromXYTheta(1.2, 0.4, -0.3);
  const Eigen::Affine2d to = from * relative_pose;

  const std::array<double, 3> residual =
      EvaluateRelative(relative_pose, Eigen::Matrix3d::Identity() * 1e2, from, to);
  EXPECT_NEAR(residual[0], 0.0, 1e-10);
  EXPECT_NEAR(residual[1], 0.0, 1e-10);
  EXPECT_NEAR(residual[2], 0.0, 1e-10);
}

// The error has to live in the `from` frame, or a rotated pose graph would not optimize the
// same way as an axis aligned one.
TEST(SpaCostFunction, TranslationErrorIsExpressedInTheFromFrame) {
  const Eigen::Affine2d from = transform::FromXYTheta(1.0, 2.0, M_PI / 2.0);
  const Eigen::Affine2d to = transform::FromXYTheta(1.0 + 0.3, 2.0 + 0.7, M_PI / 2.0);

  const std::array<double, 3> residual =
      EvaluateRelative(Eigen::Affine2d::Identity(), Eigen::Matrix3d::Identity(), from, to);
  // R(pi/2)^T * (0.3, 0.7) = (0.7, -0.3).
  EXPECT_NEAR(residual[0], 0.7, 1e-12);
  EXPECT_NEAR(residual[1], -0.3, 1e-12);
  EXPECT_NEAR(residual[2], 0.0, 1e-12);
}

// Without wrapping this residual would be about -6 rad and drag the solution the long way round.
TEST(SpaCostFunction, AngleErrorWrapsAcrossPi) {
  const Eigen::Affine2d from = transform::FromXYTheta(0.0, 0.0, 3.0);
  const Eigen::Affine2d to = transform::FromXYTheta(0.0, 0.0, -3.0);

  const std::array<double, 3> residual =
      EvaluateRelative(Eigen::Affine2d::Identity(), Eigen::Matrix3d::Identity(), from, to);
  EXPECT_NEAR(residual[2], -6.0 + 2.0 * M_PI, 1e-12);
}

TEST(SpaCostFunction, SqrtInformationScalesEachDegreeOfFreedom) {
  const Eigen::Affine2d from = Eigen::Affine2d::Identity();
  const Eigen::Affine2d to = transform::FromXYTheta(0.5, -0.25, 0.1);
  Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Zero();
  sqrt_information.diagonal() << 2.0, 3.0, 4.0;

  const std::array<double, 3> weighted =
      EvaluateRelative(Eigen::Affine2d::Identity(), sqrt_information, from, to);
  const std::array<double, 3> plain =
      EvaluateRelative(Eigen::Affine2d::Identity(), Eigen::Matrix3d::Identity(), from, to);
  EXPECT_NEAR(weighted[0], 2.0 * plain[0], kEpsilon);
  EXPECT_NEAR(weighted[1], 3.0 * plain[1], kEpsilon);
  EXPECT_NEAR(weighted[2], 4.0 * plain[2], kEpsilon);
}

// A full matrix, not just its diagonal: the trimmer's marginalized constraints are correlated.
TEST(SpaCostFunction, OffDiagonalSqrtInformationMixesDegreesOfFreedom) {
  Eigen::Matrix3d sqrt_information = Eigen::Matrix3d::Identity();
  sqrt_information(0, 2) = 5.0;

  const Eigen::Affine2d to = transform::FromXYTheta(0.0, 0.0, 0.2);
  const std::array<double, 3> residual = EvaluateRelative(
      Eigen::Affine2d::Identity(), sqrt_information, Eigen::Affine2d::Identity(), to);
  EXPECT_NEAR(residual[0], 5.0 * 0.2, 1e-12);
  EXPECT_NEAR(residual[2], 0.2, 1e-12);
}

TEST(SpaCostFunction, AbsoluteResidualVanishesOnThePrior) {
  const Eigen::Affine2d prior = transform::FromXYTheta(-4.0, 6.0, 1.3);
  const std::array<double, 3> residual =
      EvaluateAbsolute(prior, Eigen::Matrix3d::Identity() * 1e2, transform::ToArray3(prior));
  EXPECT_NEAR(residual[0], 0.0, 1e-10);
  EXPECT_NEAR(residual[1], 0.0, 1e-10);
  EXPECT_NEAR(residual[2], 0.0, 1e-10);
}

// The unary error stays in the global frame: the prior says where the pose is, not where it is
// relative to itself.
TEST(SpaCostFunction, AbsoluteResidualIsNotRotatedIntoThePoseFrame) {
  const Eigen::Affine2d prior = transform::FromXYTheta(0.0, 0.0, M_PI / 2.0);
  const std::array<double, 3> pose_block = {0.3, 0.7, M_PI / 2.0};
  const std::array<double, 3> residual =
      EvaluateAbsolute(prior, Eigen::Matrix3d::Identity(), pose_block);
  EXPECT_NEAR(residual[0], 0.3, 1e-12);
  EXPECT_NEAR(residual[1], 0.7, 1e-12);
  EXPECT_NEAR(residual[2], 0.0, 1e-12);
}

TEST(SpaCostFunction, AbsoluteAngleErrorWrapsAcrossPi) {
  const Eigen::Affine2d prior = transform::FromXYTheta(0.0, 0.0, 3.0);
  const std::array<double, 3> pose_block = {0.0, 0.0, -3.0};
  const std::array<double, 3> residual =
      EvaluateAbsolute(prior, Eigen::Matrix3d::Identity(), pose_block);
  EXPECT_NEAR(residual[2], -6.0 + 2.0 * M_PI, 1e-12);
}

}  // namespace
}  // namespace evergreenslam::utils::cost_functor

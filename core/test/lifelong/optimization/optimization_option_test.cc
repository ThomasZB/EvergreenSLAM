/**
 * @file optimization_option_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Constraint weights and the loop closure loss knee.
 * @version 0.1
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/optimization/optimization_option.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>

namespace evergreenslam::lifelong {
namespace {

// The shipped defaults must reproduce the old hardcoded 1e2 * I and Huber(10.0) exactly: a ULP
// tolerance would hide the drift this test guards.
TEST(ConstraintWeightOptionTest, DefaultsReproduceTheFormerConstants) {
  const ConstraintWeightOption option;
  const Eigen::Matrix3d expected = Eigen::Matrix3d::Identity() * 1e2;
  EXPECT_TRUE((OdometrySqrtInformation(option) - expected).isZero(0.0));
  EXPECT_TRUE((LoopClosureSqrtInformation(option) - expected).isZero(0.0));
  EXPECT_EQ(LoopClosureHuberDelta(option), 10.0);
}

TEST(ConstraintWeightOptionTest, SqrtInformationInvertsTheStddevsPerAxis) {
  ConstraintWeightOption option;
  option.odometry_translation_stddev = 0.02;
  option.odometry_rotation_stddev = 0.005;
  option.loop_closure_translation_stddev = 0.5;
  option.loop_closure_rotation_stddev = 0.25;

  const Eigen::Matrix3d odometry = OdometrySqrtInformation(option);
  EXPECT_EQ(odometry(0, 0), 50.0);
  EXPECT_EQ(odometry(1, 1), 50.0);
  EXPECT_EQ(odometry(2, 2), 200.0);
  EXPECT_TRUE((odometry - Eigen::Matrix3d(odometry.diagonal().asDiagonal())).isZero(0.0));

  const Eigen::Matrix3d loop = LoopClosureSqrtInformation(option);
  EXPECT_EQ(loop(0, 0), 2.0);
  EXPECT_EQ(loop(2, 2), 4.0);
}

// The residual the loss sees is already whitened, so the knee is a distance divided by the
// translation stddev.
TEST(ConstraintWeightOptionTest, HuberDeltaIsInResidualUnits) {
  ConstraintWeightOption option;
  option.loop_closure_translation_stddev = 0.02;
  option.loop_closure_huber_distance = 0.5;
  EXPECT_EQ(LoopClosureHuberDelta(option), 25.0);
}

TEST(ConstraintWeightOptionTest, LeverArmScalesOnlyTheTranslationRows) {
  const ConstraintWeightOption option;  // stddevs 0.01 m / 0.01 rad
  const Eigen::Matrix3d base = LoopClosureSqrtInformation(option);
  EXPECT_TRUE((LeverArmSqrtInformation(base, 0.0) - base).isZero(0.0));

  // sigma_t_eff^2 = 0.01^2 + (0.01 * 10)^2: the translation weight drops to 1/sqrt(101).
  const Eigen::Matrix3d far = LeverArmSqrtInformation(base, 10.0);
  EXPECT_DOUBLE_EQ(far(0, 0), base(0, 0) / std::sqrt(101.0));
  EXPECT_DOUBLE_EQ(far(1, 1), base(1, 1) / std::sqrt(101.0));
  EXPECT_EQ(far(2, 2), base(2, 2));
  EXPECT_TRUE((far - Eigen::Matrix3d(far.diagonal().asDiagonal())).isZero(0.0));
}

TEST(ConstraintWeightOptionTest, LoadsFromYaml) {
  const YAML::Node node = YAML::Load(R"(
odometry_translation_stddev: 0.03
loop_closure_rotation_stddev: 0.07
loop_closure_huber_distance: 0.4
)");
  const ConstraintWeightOption option = LoadConstraintWeightOption(node);
  EXPECT_DOUBLE_EQ(option.odometry_translation_stddev, 0.03);
  EXPECT_DOUBLE_EQ(option.loop_closure_rotation_stddev, 0.07);
  EXPECT_DOUBLE_EQ(option.loop_closure_huber_distance, 0.4);
  const ConstraintWeightOption defaults;
  EXPECT_DOUBLE_EQ(option.odometry_rotation_stddev, defaults.odometry_rotation_stddev);
  EXPECT_DOUBLE_EQ(option.loop_closure_translation_stddev,
                   defaults.loop_closure_translation_stddev);
}

TEST(ConstraintWeightOptionTest, NonPositiveStddevIsFatal) {
  ConstraintWeightOption option;
  option.odometry_rotation_stddev = 0.0;
  EXPECT_DEATH(OdometrySqrtInformation(option), "rotation_stddev");
}

}  // namespace
}  // namespace evergreenslam::lifelong

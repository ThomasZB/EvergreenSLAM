/**
 * @file transform_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/transform/transform.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"

namespace evergreenslam::utils::transform {
namespace {

// The local frame drifts without bound, so poses and grid origins reach the
// kilometre range. float carries ~1 mm of resolution at 12 km, which is
// coarser than the map itself; these tolerances are below what float can
// represent there and so fail if anything on the path narrows to float.
constexpr double kFarX = 12345.678901;
constexpr double kFarY = -9876.543210;

TEST(TransformTest, PoseRoundTripsThroughArray3FarFromTheOrigin) {
  const Eigen::Affine2d pose = FromXYTheta(kFarX, kFarY, 1.234567);
  const Eigen::Affine2d round_tripped = FromArray3(ToArray3(pose));

  EXPECT_NEAR(round_tripped.translation().x(), kFarX, 1e-9);
  EXPECT_NEAR(round_tripped.translation().y(), kFarY, 1e-9);
  EXPECT_NEAR(GetYaw(round_tripped), 1.234567, 1e-12);
}

TEST(TransformTest, ComposingWithTheInverseIsExactFarFromTheOrigin) {
  const Eigen::Affine2d pose = FromXYTheta(kFarX, kFarY, 0.5);
  const Eigen::Affine2d delta = FromXYTheta(0.001, -0.002, 1e-5);
  const Eigen::Affine2d moved = pose * delta;
  const Eigen::Affine2d recovered = pose.inverse() * moved;

  EXPECT_NEAR(recovered.translation().x(), 0.001, 1e-9);
  EXPECT_NEAR(recovered.translation().y(), -0.002, 1e-9);
  EXPECT_NEAR(GetYaw(recovered), 1e-5, 1e-12);
}

TEST(TransformTest, GridCellRoundTripsFarFromTheOrigin) {
  const mapping::GridMapu8 grid(std::vector<uint8_t>(100 * 100, 0), 100, 100, 0.05, kFarX, kFarY,
                                0);
  for (int y = 0; y < 100; y += 7) {
    for (int x = 0; x < 100; x += 7) {
      const Eigen::Array2i cell(x, y);
      const Eigen::Array2i round_tripped = grid.ToCell(grid.ToCenter(cell));
      ASSERT_EQ(round_tripped.x(), x) << "cell " << x << ", " << y;
      ASSERT_EQ(round_tripped.y(), y) << "cell " << x << ", " << y;
    }
  }
}

TEST(TransformTest, NormalizeAngleWrapsBothWays) {
  // The interval is closed at both ends, so +pi and -pi are both valid; only
  // the magnitude is pinned.
  EXPECT_NEAR(std::abs(NormalizeAngle(3.0 * M_PI)), M_PI, 1e-12);
  EXPECT_NEAR(std::abs(NormalizeAngle(-3.0 * M_PI)), M_PI, 1e-12);
  EXPECT_NEAR(NormalizeAngle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(NormalizeAngle(2.0 * M_PI + 0.5), 0.5, 1e-12);
  EXPECT_NEAR(NormalizeAngle(-2.0 * M_PI - 0.5), -0.5, 1e-12);
}

TEST(TransformTest, InterpolateCrossesTheAngleWrap) {
  const Eigen::Affine2d start = FromXYTheta(0.0, 0.0, 3.0);
  const Eigen::Affine2d end = FromXYTheta(2.0, 4.0, -3.0);

  const Eigen::Affine2d half = Interpolate(start, end, 0.5);
  EXPECT_NEAR(half.translation().x(), 1.0, 1e-12);
  EXPECT_NEAR(half.translation().y(), 2.0, 1e-12);
  // The short way round passes through pi, not through zero.
  EXPECT_NEAR(std::abs(GetYaw(half)), M_PI, 1e-9);

  const Eigen::Affine2d extrapolated = Interpolate(start, end, 2.0);
  EXPECT_NEAR(extrapolated.translation().x(), 4.0, 1e-12);
  EXPECT_NEAR(extrapolated.translation().y(), 8.0, 1e-12);
}

}  // namespace
}  // namespace evergreenslam::utils::transform

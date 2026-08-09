/**
 * @file undistortion_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sensor/undistortion.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::sensor {
namespace {

using utils::transform::FromXYTheta;
using utils::transform::Interpolate;

constexpr double kScanDuration = 0.08;

TEST(UndistortionTest, EmptyCloudStaysEmpty) {
  const PointCloud undistorted =
      Undistort(TimedPointCloud(), FromXYTheta(1.0, 2.0, 0.5), FromXYTheta(3.0, 4.0, 1.5));
  EXPECT_TRUE(undistorted.empty());
}

TEST(UndistortionTest, StaticPosesOnlyStripTheOffsets) {
  const Eigen::Affine2d pose = FromXYTheta(0.7, -1.3, 0.4);
  TimedPointCloud cloud;
  cloud.push_back({{1.23, -0.45}, common::FromSeconds(-kScanDuration)});
  cloud.push_back({{-2.31, 0.87}, common::FromSeconds(0.0)});

  const PointCloud undistorted = Undistort(cloud, pose, pose);

  ASSERT_EQ(undistorted.size(), cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_NEAR((undistorted[i].point - cloud[i].point).norm(), 0.0, 1e-12);
  }
}

TEST(UndistortionTest, ZeroSpanFallsBackToTheEndPose) {
  TimedPointCloud cloud;
  cloud.push_back({{1.5, 2.5}, common::FromSeconds(0.0)});

  const PointCloud undistorted =
      Undistort(cloud, FromXYTheta(5.0, 5.0, 1.0), FromXYTheta(9.0, -3.0, 2.0));

  ASSERT_EQ(undistorted.size(), 1u);
  EXPECT_NEAR((undistorted[0].point - Eigen::Vector2d(1.5, 2.5)).norm(), 0.0, 1e-12);
}

TEST(UndistortionTest, RecoversLandmarksUnderAConstantVelocitySweep) {
  // Deliberately off any tidy values so nothing cancels by accident.
  const std::vector<Eigen::Vector2d> landmarks = {{1.37, 0.83}, {-2.11, 1.59}, {0.42, -3.27}};
  const Eigen::Affine2d start_pose = FromXYTheta(0.31, -0.22, 0.17);
  const Eigen::Affine2d end_pose = FromXYTheta(0.52, 0.09, 0.58);

  constexpr int kNumPoints = 30;
  TimedPointCloud cloud;
  for (int i = 0; i < kNumPoints; ++i) {
    const double factor = static_cast<double>(i) / (kNumPoints - 1);
    const Eigen::Affine2d body_pose = Interpolate(start_pose, end_pose, factor);
    const Eigen::Vector2d measured = body_pose.inverse() * landmarks[i % landmarks.size()];
    cloud.push_back({measured, common::FromSeconds((factor - 1.0) * kScanDuration)});
  }

  const PointCloud undistorted = Undistort(cloud, start_pose, end_pose);

  ASSERT_EQ(undistorted.size(), cloud.size());
  const Eigen::Affine2d from_local = end_pose.inverse();
  double max_distortion = 0.0;
  for (int i = 0; i < kNumPoints; ++i) {
    const Eigen::Vector2d expected = from_local * landmarks[i % landmarks.size()];
    // Offsets are quantized to whole nanoseconds, so recovery is exact only to ~1e-9.
    EXPECT_NEAR((undistorted[i].point - expected).norm(), 0.0, 1e-6);
    max_distortion = std::max(max_distortion, (cloud[i].point - expected).norm());
  }
  // The raw scan must actually be distorted, or this test proves nothing.
  EXPECT_GT(max_distortion, 0.1);
}

}  // namespace
}  // namespace evergreenslam::sensor

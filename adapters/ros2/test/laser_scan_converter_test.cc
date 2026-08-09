/**
 * @file laser_scan_converter_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "laser_scan_converter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "utils/transform/transform.h"

namespace evergreenslam::ros2 {
namespace {

sensor_msgs::msg::LaserScan MakeScan(const std::vector<float>& ranges) {
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = M_PI_2;
  scan.range_min = 0.2;
  scan.range_max = 10.0;
  scan.ranges = ranges;
  return scan;
}

TEST(LaserScanConverterTest, DropsBeamsWithoutAReturn) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  // A range comparison alone keeps NaN, since every comparison against it is false.
  const auto cloud =
      FromLaserScan(MakeScan({1.0f, nan, inf, 2.0f}), Eigen::Affine2d::Identity()).point_cloud;
  ASSERT_EQ(cloud.size(), 2u);
  EXPECT_NEAR(cloud[0].point.x(), 1.0, 1e-9);
  EXPECT_NEAR(cloud[1].point.y(), -2.0, 1e-9);
}

TEST(LaserScanConverterTest, DropsBeamsOutsideTheScannerRange) {
  const auto cloud =
      FromLaserScan(MakeScan({0.1f, 1.0f, 20.0f}), Eigen::Affine2d::Identity()).point_cloud;
  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_NEAR(cloud[0].point.y(), 1.0, 1e-9);
}

TEST(LaserScanConverterTest, PutsPointsInTheBaseFrame) {
  const Eigen::Affine2d base_from_laser = utils::transform::FromXYTheta(0.275, 0.0, M_PI_2);
  const auto cloud = FromLaserScan(MakeScan({2.0f}), base_from_laser).point_cloud;
  ASSERT_EQ(cloud.size(), 1u);
  // The beam points along laser +x, which the extrinsic turns into base +y.
  EXPECT_NEAR(cloud[0].point.x(), 0.275, 1e-9);
  EXPECT_NEAR(cloud[0].point.y(), 2.0, 1e-9);
}

TEST(LaserScanConverterTest, StampsTheCloudAtTheLastBeam) {
  sensor_msgs::msg::LaserScan scan =
      MakeScan({1.0f, std::numeric_limits<float>::quiet_NaN(), 2.0f, 1.0f});
  scan.header.stamp.sec = 1285622989;
  scan.header.stamp.nanosec = 551713839;
  scan.time_increment = 1e-4f;

  const TimedScan timed_scan = FromLaserScan(scan, Eigen::Affine2d::Identity());

  // header.stamp is the first beam; the scan time is the last one, dropped beams included.
  EXPECT_NEAR(common::ToSeconds(timed_scan.time - FromRosTime(scan.header.stamp)), 3e-4, 1e-9);
  ASSERT_EQ(timed_scan.point_cloud.size(), 3u);
  EXPECT_NEAR(common::ToSeconds(timed_scan.point_cloud[0].offset), -3e-4, 1e-9);
  EXPECT_NEAR(common::ToSeconds(timed_scan.point_cloud[1].offset), -1e-4, 1e-9);
  EXPECT_NEAR(common::ToSeconds(timed_scan.point_cloud[2].offset), 0.0, 1e-9);
}

TEST(LaserScanConverterTest, KeepsNanosecondsThroughTheStampConversion) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1285622989;
  stamp.nanosec = 551713839;
  EXPECT_EQ(common::ToUnixNanos(FromRosTime(stamp)), 1285622989551713839LL);
}

}  // namespace
}  // namespace evergreenslam::ros2

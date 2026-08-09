/**
 * @file adaptive_voxel_filter_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sensor/adaptive_voxel_filter.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sensor/voxel_filter.h"

namespace evergreenslam::sensor {
namespace {

// Same room as the local mapping end to end test, and the bounds are off the grid resolution for
// the same reason: on aligned walls every point sits on a voxel boundary, where lround() decides
// the cell on rounding noise rather than on geometry.
constexpr double kRoomMinX = 0.013;
constexpr double kRoomMaxX = 10.007;
constexpr double kRoomMinY = 0.021;
constexpr double kRoomMaxY = 8.003;

double RangeToWall(const Eigen::Vector2d& point, double angle) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double range = std::numeric_limits<double>::max();
  if (dx > 1e-6) {
    range = std::min(range, (kRoomMaxX - point.x()) / dx);
  } else if (dx < -1e-6) {
    range = std::min(range, (kRoomMinX - point.x()) / dx);
  }
  if (dy > 1e-6) {
    range = std::min(range, (kRoomMaxY - point.y()) / dy);
  } else if (dy < -1e-6) {
    range = std::min(range, (kRoomMinY - point.y()) / dy);
  }
  return range;
}

PointCloud SimulateScan(const Eigen::Vector2d& position, int num_beams) {
  PointCloud cloud;
  for (int i = 0; i < num_beams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / num_beams;
    const double range = RangeToWall(position, bearing);
    cloud.push_back(Point2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing))});
  }
  return cloud;
}

// The point of the filter: matching cost must follow the config, not the sensor. A 16x denser
// lidar in the same room has to come out at the same size.
TEST(AdaptiveVoxelFilterTest, OutputSizeFollowsTheBudgetNotTheBeamCount) {
  AdaptiveVoxelFilterOption option;
  const AdaptiveVoxelFilter filter(option);
  const Eigen::Vector2d position(3.3, 2.7);

  for (const int num_beams : {360, 1440, 5760}) {
    const PointCloud filtered = filter.Filter(SimulateScan(position, num_beams));
    EXPECT_GE(filtered.size(), static_cast<size_t>(option.min_num_points)) << num_beams << " beams";
    // The bisection stops at a 10% length interval, so the count can overshoot; what must not
    // happen is the raw beam count coming straight through.
    EXPECT_LT(filtered.size(), static_cast<size_t>(2 * option.min_num_points))
        << num_beams << " beams";
  }
}

TEST(AdaptiveVoxelFilterTest, StopsAtMaxLengthWhenThatAlreadyMeetsTheBudget) {
  AdaptiveVoxelFilterOption option;
  option.min_num_points = 20;
  const PointCloud scan = SimulateScan(Eigen::Vector2d(3.3, 2.7), 1440);

  const PointCloud filtered = AdaptiveVoxelFilter(option).Filter(scan);
  const PointCloud fixed = VoxelFilter(option.max_length).Filter(scan);
  EXPECT_EQ(filtered.size(), fixed.size());
}

TEST(AdaptiveVoxelFilterTest, SparseScanPassesThroughUntouched) {
  const PointCloud scan = SimulateScan(Eigen::Vector2d(3.3, 2.7), 90);
  const PointCloud filtered = AdaptiveVoxelFilter().Filter(scan);

  ASSERT_EQ(filtered.size(), scan.size());
  for (size_t i = 0; i < scan.size(); ++i) {
    EXPECT_TRUE(filtered[i].point.isApprox(scan[i].point));
  }
}

// The budget is a floor, not a promise. Points packed tighter than the finest voxel cannot reach
// it however far the search goes, and what comes back must still not be empty: AddScan() drops an
// empty cloud, so returning nothing here would silently lose the scan.
TEST(AdaptiveVoxelFilterTest, BudgetFinerThanTheDataStillReturnsPoints) {
  AdaptiveVoxelFilterOption option;
  option.min_num_points = 200;
  PointCloud cluster;
  for (int i = 0; i < 500; ++i) {
    cluster.push_back(Point2d{Eigen::Vector2d(1.0 + 0.02 * i / 500.0, 2.0)});
  }

  const PointCloud filtered = AdaptiveVoxelFilter(option).Filter(cluster);
  EXPECT_FALSE(filtered.empty());
  EXPECT_LT(filtered.size(), static_cast<size_t>(option.min_num_points));
}

TEST(AdaptiveVoxelFilterTest, IsDeterministic) {
  const PointCloud scan = SimulateScan(Eigen::Vector2d(3.3, 2.7), 1440);
  const AdaptiveVoxelFilter filter;

  const PointCloud first = filter.Filter(scan);
  const PointCloud second = filter.Filter(scan);
  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i) {
    EXPECT_TRUE(first[i].point.isApprox(second[i].point));
  }
}

}  // namespace
}  // namespace evergreenslam::sensor

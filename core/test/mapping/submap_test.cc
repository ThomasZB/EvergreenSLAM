/**
 * @file submap_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The grid lives in the submap frame: world = global_pose * grid_point.
 * @version 0.1
 * @date 2026-09-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/submap.h"

#include <gtest/gtest.h>

#include <cmath>

#include "mapping/grid_mapping/probability_values.h"
#include "utils/transform/transform.h"

namespace evergreenslam::mapping {
namespace {

constexpr double kResolution = 0.05;

// Off the resolution lattice on purpose.
sensor::PointCloud RingScan(double radius) {
  sensor::PointCloud cloud;
  for (int i = 0; i < 72; ++i) {
    const double angle = 2.0 * M_PI * i / 72.0 + 0.011;
    cloud.push_back(sensor::Point2d{radius * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  return cloud;
}

TEST(SubmapTest, GridStartedFarFromTheLocalOriginStaysCompact) {
  const Eigen::Affine2d local_pose = utils::transform::FromXYTheta(100.0, 100.0, 0.7);
  const Eigen::Affine2d scan_pose =
      Eigen::Affine2d(local_pose * utils::transform::FromXYTheta(0.5, 0.2, 0.1));
  Submap submap(0, local_pose, kResolution);
  const CastRaysMapping inserter;
  submap.InsertScan(scan_pose, RingScan(2.013), inserter);

  // A 2 m ring plus growth padding, never the 100 m back to the local origin.
  EXPECT_LT(submap.grid().width(), 400);
  EXPECT_LT(submap.grid().height(), 400);

  const GridMapu8& snapshot = submap.Snapshot();
  const double min_x = snapshot.origin_x();
  const double min_y = snapshot.origin_y();
  const double max_x = min_x + snapshot.width() * snapshot.resolution();
  const double max_y = min_y + snapshot.height() * snapshot.resolution();
  EXPECT_GT(min_x, -3.0);
  EXPECT_GT(min_y, -3.0);
  EXPECT_LT(max_x, 3.5);
  EXPECT_LT(max_y, 3.5);
}

TEST(SubmapTest, ScanAtALargeLocalPoseLandsWhereGlobalPoseTimesGridPointSays) {
  const Eigen::Affine2d local_pose = utils::transform::FromXYTheta(100.0, 100.0, 0.7);
  const Eigen::Affine2d scan_pose =
      Eigen::Affine2d(local_pose * utils::transform::FromXYTheta(0.5, 0.2, 0.1));
  Submap submap(0, local_pose, kResolution);
  const CastRaysMapping inserter;
  const sensor::PointCloud cloud = RingScan(2.013);
  submap.InsertScan(scan_pose, cloud, inserter);

  // An arbitrary optimized pose: the placement must not involve local_pose at all.
  const Eigen::Affine2d global_pose = utils::transform::FromXYTheta(-7.0, 3.0, -1.3);
  const Eigen::Affine2d grid_to_world = global_pose;
  const Eigen::Affine2d world_to_grid = grid_to_world.inverse();
  const Eigen::Affine2d scan_in_world =
      Eigen::Affine2d(global_pose * local_pose.inverse() * scan_pose);

  int occupied = 0;
  int free = 0;
  for (const auto& point : cloud) {
    const Eigen::Vector2d hit_world = scan_in_world * point.point;
    const Eigen::Vector2d hit_grid = world_to_grid * hit_world;
    if (submap.grid().GetProbability(submap.grid().ToCell(hit_grid)) > 0.5) {
      ++occupied;
    }
    const Eigen::Vector2d midway_world = scan_in_world * Eigen::Vector2d(0.5 * point.point);
    const Eigen::Vector2d midway_grid = world_to_grid * midway_world;
    if (submap.grid().IsKnown(submap.grid().ToCell(midway_grid)) &&
        submap.grid().GetProbability(submap.grid().ToCell(midway_grid)) < 0.5) {
      ++free;
    }
  }
  EXPECT_GT(occupied, static_cast<int>(0.9 * cloud.size()));
  // Ray casting skips a few cells at this beam spacing; the frame is what is under test.
  EXPECT_GT(free, static_cast<int>(0.7 * cloud.size()));

  // Where the old convention would have put it: nothing there.
  const Eigen::Vector2d stale = scan_pose * cloud[0].point;
  EXPECT_FALSE(submap.grid().IsKnown(submap.grid().ToCell(stale)));
}

// Deserialization builds a finished submap without calling Finish(); finished() must still mean
// the snapshot is there, or the first reader builds it racing the second.
TEST(SubmapTest, AFinishedSubmapIsConstructedWithItsSnapshot) {
  Submap source(0, Eigen::Affine2d::Identity(), kResolution);
  const CastRaysMapping inserter;
  source.InsertScan(utils::transform::FromXYTheta(0.3, -0.2, 0.15), RingScan(2.013), inserter);

  const Submap loaded(0, source.local_pose(), source.grid(), source.num_scans(), true);
  EXPECT_TRUE(loaded.finished());
  const GridMapu8& snapshot = loaded.Snapshot();
  const GridMapu8 expected = loaded.grid().ToSnapshot();
  ASSERT_GT(expected.width(), 0);
  EXPECT_EQ(snapshot.width(), expected.width());
  EXPECT_EQ(snapshot.height(), expected.height());
  EXPECT_DOUBLE_EQ(snapshot.origin_x(), expected.origin_x());
  EXPECT_DOUBLE_EQ(snapshot.origin_y(), expected.origin_y());
  EXPECT_EQ(snapshot.data(), expected.data());

  const Submap unfinished(0, source.local_pose(), source.grid(), source.num_scans(), false);
  EXPECT_FALSE(unfinished.finished());
}

}  // namespace
}  // namespace evergreenslam::mapping

/**
 * @file active_map_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The two-submap rotation schedule.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/laser_odometry/active_map.h"

#include <gtest/gtest.h>

#include <cmath>

#include "utils/transform/transform.h"

namespace evergreenslam::mapping {
namespace {

// Enough to make the grid non-empty; the shape does not matter.
sensor::PointCloud RingAround(const Eigen::Vector2d& centre) {
  sensor::PointCloud cloud;
  for (int i = 0; i < 32; ++i) {
    const double angle = 2.0 * M_PI * i / 32;
    cloud.push_back(
        sensor::Point2d{centre + 2.0 * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  return cloud;
}

ActiveMapOption SmallSubmapOptions(int num_scans_per_submap) {
  ActiveMapOption options;
  options.num_scans_per_submap = num_scans_per_submap;
  return options;
}

TEST(ActiveMapTest, CreatesFinishesAndReleasesSubmapsOnSchedule) {
  constexpr int kNumScansPerSubmap = 4;
  ActiveMap active_map(SmallSubmapOptions(kNumScansPerSubmap));

  EXPECT_EQ(active_map.matching_submap(), nullptr);

  for (int i = 0; i < 4 * kNumScansPerSubmap; ++i) {
    const Eigen::Vector2d origin(0.05 * i, 0.0);
    const auto submaps =
        active_map.InsertScan(utils::transform::FromXYTheta(origin.x(), origin.y(), 0.0),
                              RingAround(Eigen::Vector2d::Zero()));

    EXPECT_FALSE(submaps.empty());
    // Never more than two submaps take a scan, or insertion cost and the intra-submap constraint
    // count both grow without bound.
    EXPECT_LE(submaps.size(), 2u);
    EXPECT_EQ(submaps.front().get(), active_map.matching_submap().get());
  }

  const auto& submaps = active_map.submaps();
  ASSERT_EQ(submaps.size(), 2u);
  EXPECT_TRUE(submaps.front()->finished());
  EXPECT_FALSE(submaps.back()->finished());
  EXPECT_EQ(submaps.front()->num_scans(), 2 * kNumScansPerSubmap);
  EXPECT_EQ(submaps.back()->num_scans(), kNumScansPerSubmap);
  EXPECT_EQ(submaps.front()->local_index(), 2);
  EXPECT_EQ(submaps.back()->local_index(), 3);

  // A finished submap keeps a usable snapshot, which is what makes it safe to hand to another
  // thread.
  EXPECT_GT(submaps.front()->Snapshot().width(), 0);
}

TEST(ActiveMapTest, SubmapStartedFarFromTheLocalOriginStaysCompact) {
  ActiveMap active_map(SmallSubmapOptions(4));
  const Eigen::Affine2d pose = utils::transform::FromXYTheta(100.0, 100.0, 0.3);
  active_map.InsertScan(pose, RingAround(Eigen::Vector2d::Zero()));

  const auto& submap = active_map.submaps().back();
  EXPECT_LT(submap->grid().width(), 400);
  EXPECT_LT(submap->grid().height(), 400);
}

}  // namespace
}  // namespace evergreenslam::mapping

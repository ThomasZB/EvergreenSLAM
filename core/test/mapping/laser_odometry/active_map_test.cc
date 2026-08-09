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
    const auto submaps = active_map.InsertScan(
        origin, RingAround(origin), utils::transform::FromXYTheta(origin.x(), origin.y(), 0.0));

    EXPECT_FALSE(submaps.empty());
    // Never more than two submaps take a scan, otherwise insertion cost and
    // the number of intra-submap constraints both grow without bound.
    EXPECT_LE(submaps.size(), 2u);
    EXPECT_EQ(submaps.front().get(), active_map.matching_submap().get());
  }

  // 16 scans at 4 per submap. Submaps are created at scans 0, 4, 8 and 12 and
  // finished four scans after the next one starts, so by scan 15 submaps 0 and
  // 1 have been finished and released and 2 and 3 are the live pair.
  const auto& submaps = active_map.submaps();
  ASSERT_EQ(submaps.size(), 2u);
  EXPECT_TRUE(submaps.front()->finished());
  EXPECT_FALSE(submaps.back()->finished());
  EXPECT_EQ(submaps.front()->num_scans(), 2 * kNumScansPerSubmap);
  EXPECT_EQ(submaps.back()->num_scans(), kNumScansPerSubmap);
  EXPECT_EQ(submaps.front()->id().submap_index, 2);
  EXPECT_EQ(submaps.back()->id().submap_index, 3);

  // A finished submap keeps a usable snapshot, which is what makes it safe to
  // hand to another thread.
  EXPECT_GT(submaps.front()->Snapshot().width(), 0);
}

TEST(ActiveMapTest, StartNewSessionFinishesAndRestartsIndexing) {
  ActiveMap active_map(SmallSubmapOptions(4));
  for (int i = 0; i < 6; ++i) {
    const Eigen::Vector2d origin(0.05 * i, 0.0);
    active_map.InsertScan(origin, RingAround(origin), Eigen::Affine2d::Identity());
  }
  ASSERT_FALSE(active_map.submaps().empty());

  active_map.StartNewSession(7);
  EXPECT_TRUE(active_map.submaps().empty());
  EXPECT_EQ(active_map.session_id(), 7);
  EXPECT_EQ(active_map.matching_submap(), nullptr);

  active_map.InsertScan(Eigen::Vector2d::Zero(), RingAround(Eigen::Vector2d::Zero()),
                        Eigen::Affine2d::Identity());
  ASSERT_EQ(active_map.submaps().size(), 1u);
  EXPECT_EQ(active_map.submaps().front()->id().session_id, 7);
  EXPECT_EQ(active_map.submaps().front()->id().submap_index, 0);
}

}  // namespace
}  // namespace evergreenslam::mapping

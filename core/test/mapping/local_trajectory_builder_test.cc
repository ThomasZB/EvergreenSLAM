/**
 * @file local_trajectory_builder_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief End to end check: a simulated lidar driven through a closed room.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/local_trajectory_builder.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <set>

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::mapping {
namespace {

namespace transform = utils::transform;

// Scans at 10 Hz from a stamp of the same magnitude a real bag carries.
common::Time ScanTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// An empty axis aligned room, fully enclosed so every scan sees all four walls.
//
// The bounds are deliberately not multiples of the 0.05 m grid resolution: on aligned walls
// every point sits exactly on a cell boundary, and a rotation of a few ten-thousandths of a
// radian flips them all together, collapsing the score for reasons unrelated to the code.
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

// Every beam is cast from the same pose, so the scan is instantaneous and all offsets are zero;
// nonzero offsets would make undistortion warp a scan that carries no real distortion.
sensor::TimedPointCloud SimulateScan(const Eigen::Affine2d& world_pose, int num_beams) {
  const double yaw = transform::GetYaw(world_pose);
  sensor::TimedPointCloud cloud;
  for (int i = 0; i < num_beams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / num_beams;
    const double range = RangeToWall(world_pose.translation(), yaw + bearing);
    cloud.push_back(
        sensor::TimedPoint2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing)),
                             common::Duration::zero()});
  }
  return cloud;
}

Eigen::Affine2d WorldPoseAt(int scan_index, int num_scans) {
  const double s = static_cast<double>(scan_index) / (num_scans - 1);
  return transform::FromXYTheta(2.0 + 4.0 * s, 2.0 + 1.5 * s, 0.6 * s);
}

TEST(LocalTrajectoryBuilderTest, TracksASimulatedRobotInAClosedRoom) {
  constexpr int kNumScans = 200;
  constexpr int kNumBeams = 360;

  LocalTrajectoryBuilderOption options;
  LocalTrajectoryBuilder builder(options);

  int num_keyframes = 0;
  double worst_score = 1.0;
  double last_score = 0.0;
  for (int i = 0; i < kNumScans; ++i) {
    const auto result =
        builder.AddScan(ScanTime(i), SimulateScan(WorldPoseAt(i, kNumScans), kNumBeams));
    if (result.has_value()) {
      ++num_keyframes;
      // The first keyframe has no map to match against and scores 0 by
      // definition, so it is excluded.
      if (num_keyframes > 1) {
        last_score = result->matching_result.match_score;
        worst_score = std::min(worst_score, last_score);
      }
      EXPECT_FALSE(result->insertion_submaps.empty());
      // What the node carries is the adaptively filtered cloud, so its size follows the point
      // budget rather than the beam count the sensor happened to produce.
      EXPECT_GE(result->node.point_cloud.size(),
                static_cast<size_t>(options.adaptive_voxel_filter_option.min_num_points));
      EXPECT_LT(result->node.point_cloud.size(), static_cast<size_t>(kNumBeams));
    }
  }
  EXPECT_GT(num_keyframes, 5);

  // The local frame is defined by the first scan, so ground truth is the
  // motion since then, not the world pose.
  const Eigen::Affine2d first_world_pose = WorldPoseAt(0, kNumScans);
  const Eigen::Affine2d expected =
      first_world_pose.inverse() * WorldPoseAt(kNumScans - 1, kNumScans);
  const Eigen::Affine2d actual = builder.local_pose();

  // Over ~4.3 m with no odometry and no noise, error should be at the grid resolution, not at
  // the metre scale a broken pipeline gives.
  EXPECT_LT((actual.translation() - expected.translation()).norm(), 0.10)
      << "expected " << expected.translation().transpose() << ", got "
      << actual.translation().transpose();
  EXPECT_LT(
      std::abs(transform::NormalizeAngle(transform::GetYaw(actual) - transform::GetYaw(expected))),
      0.05);

  // Early keyframes are low by construction: after a single insertion a hit cell only holds
  // kDefaultHitProbability. What matters is that none is near kMinProbability, which would mean
  // matching against unknown space, and that a mature map scores close to kMaxProbability.
  EXPECT_GT(worst_score, 0.3);
  EXPECT_GT(last_score, 0.7);
}

// With the default 90 scans per submap the run above never fills one, so the handoff is never
// exercised: the mature submap is released and matching switches to a younger one with half the
// scans. That transition is where a frame or lifetime bug would sit.
TEST(LocalTrajectoryBuilderTest, KeepsTrackingAcrossSubmapRotations) {
  constexpr int kNumScans = 200;
  constexpr int kNumBeams = 360;

  LocalTrajectoryBuilderOption options;
  options.active_map_option.num_scans_per_submap = 8;
  LocalTrajectoryBuilder builder(options);

  std::set<int> submap_indices;
  for (int i = 0; i < kNumScans; ++i) {
    const auto result =
        builder.AddScan(ScanTime(i), SimulateScan(WorldPoseAt(i, kNumScans), kNumBeams));
    if (result.has_value()) {
      for (const auto& submap : result->insertion_submaps) {
        submap_indices.insert(submap->id().submap_index);
      }
    }
  }
  EXPECT_GE(submap_indices.size(), 3u);
  EXPECT_LE(builder.active_map().submaps().size(), 2u);

  const Eigen::Affine2d expected =
      WorldPoseAt(0, kNumScans).inverse() * WorldPoseAt(kNumScans - 1, kNumScans);
  const Eigen::Affine2d actual = builder.local_pose();

  // No score assertion here: the score legitimately dips just after each
  // rotation, because the new matching submap has seen half as many scans.
  EXPECT_LT((actual.translation() - expected.translation()).norm(), 0.10)
      << "expected " << expected.translation().transpose() << ", got "
      << actual.translation().transpose();
  EXPECT_LT(
      std::abs(transform::NormalizeAngle(transform::GetYaw(actual) - transform::GetYaw(expected))),
      0.05);
}

// A new session is a new bag, and nothing forces its stamps to come after the previous one's.
// The tracking filter rejects anything older than its whole history, so without a reset the
// first session would swallow every measurement of the second.
//
// The path keeps a constant heading: session two's local frame is session one's final pose, and
// a rotated frame puts the walls diagonally across the grid, which costs centimetres of drift
// for reasons unrelated to sessions.
Eigen::Affine2d StraightPoseAt(int scan_index, int num_scans) {
  const double s = static_cast<double>(scan_index) / (num_scans - 1);
  return transform::FromXYTheta(2.0 + 4.0 * s, 2.0 + 1.5 * s, 0.0);
}

TEST(LocalTrajectoryBuilderTest, KeepsTrackingWhenANewSessionRewindsTheClock) {
  constexpr int kNumScans = 60;
  constexpr int kNumBeams = 360;

  LocalTrajectoryBuilder builder{LocalTrajectoryBuilderOption()};
  for (int i = 0; i < kNumScans; ++i) {
    builder.AddScan(ScanTime(i) + common::FromSeconds(900.0),
                    SimulateScan(StraightPoseAt(i, kNumScans), kNumBeams));
  }
  const Eigen::Affine2d after_first_session = builder.local_pose();

  builder.StartNewSession(1);
  for (int i = 0; i < kNumScans; ++i) {
    builder.AddScan(ScanTime(i), SimulateScan(StraightPoseAt(i, kNumScans), kNumBeams));
  }

  // local_pose_ carries over, so the second session picks up where the first left off and the
  // expected total is the path driven twice.
  const Eigen::Affine2d session_motion =
      StraightPoseAt(0, kNumScans).inverse() * StraightPoseAt(kNumScans - 1, kNumScans);
  const Eigen::Affine2d expected = after_first_session * session_motion;
  const Eigen::Affine2d actual = builder.local_pose();

  EXPECT_LT((actual.translation() - expected.translation()).norm(), 0.10)
      << "expected " << expected.translation().transpose() << ", got "
      << actual.translation().transpose();
  EXPECT_LT(
      std::abs(transform::NormalizeAngle(transform::GetYaw(actual) - transform::GetYaw(expected))),
      0.05);
}

TEST(LocalTrajectoryBuilderTest, BuildsAMapWithWallsAndFreeSpace) {
  constexpr int kNumScans = 200;
  constexpr int kNumBeams = 360;

  LocalTrajectoryBuilder builder{LocalTrajectoryBuilderOption()};
  for (int i = 0; i < kNumScans; ++i) {
    builder.AddScan(ScanTime(i), SimulateScan(WorldPoseAt(i, kNumScans), kNumBeams));
  }

  const std::shared_ptr<const Submap> submap = builder.active_map().matching_submap();
  ASSERT_NE(submap, nullptr);
  const GridMapu8& snapshot = submap->Snapshot();
  ASSERT_GT(snapshot.width(), 0);

  const Eigen::Affine2d to_local = WorldPoseAt(0, kNumScans).inverse();

  // The left wall, sampled along the stretch the robot had a clear view of.
  // Individual cells can be missed at this beam spacing, so the assertion is
  // on the fraction hit rather than on any one cell.
  int num_wall_samples = 0;
  int num_occupied = 0;
  for (float y = 1.f; y <= 4.f; y += 0.05) {
    ++num_wall_samples;
    if (ValueToProbability(snapshot.GetValueAtPoint(to_local * Eigen::Vector2d(kRoomMinX, y))) >
        0.5) {
      ++num_occupied;
    }
  }
  EXPECT_GT(num_occupied, num_wall_samples * 0.7)
      << num_occupied << " of " << num_wall_samples << " wall cells occupied";

  for (double x = 3.0; x <= 5.0; x += 0.5) {
    const Eigen::Vector2d free_point = to_local * Eigen::Vector2d(x, 3.f);
    EXPECT_LT(ValueToProbability(snapshot.GetValueAtPoint(free_point)), 0.4)
        << "free space at world x " << x << " not carved out";
  }

  const Eigen::Vector2d outside = to_local * Eigen::Vector2d(kRoomMaxX + 2.0, 4.f);
  EXPECT_NEAR(ValueToProbability(snapshot.GetValueAtPoint(outside)), kUnknownProbability, 1e-6);
}

class RecordingSink : public debug::DebugSink {
 public:
  void PublishScanMatch(common::Time, const std::unordered_map<std::string, Eigen::Affine2d>& poses,
                        const sensor::PointCloud& point_cloud, const GridMapu8& grid_map,
                        double score) override {
    ++calls;
    last_poses = poses;
    last_points = point_cloud.size();
    last_cells = grid_map.data().size();
    last_score = score;
  }

  int calls = 0;
  std::unordered_map<std::string, Eigen::Affine2d> last_poses;
  size_t last_points = 0;
  size_t last_cells = 0;
  double last_score = 0.0;
};

TEST(LocalTrajectoryBuilderTest, AttachingADebugSinkChangesNothing) {
  constexpr int kNumScans = 120;
  constexpr int kNumBeams = 360;

  auto run = [](std::shared_ptr<debug::DebugSink> sink) {
    LocalTrajectoryBuilderOption options;
    LocalTrajectoryBuilder builder(options);
    builder.SetDebugSink(sink);
    std::vector<Eigen::Affine2d> poses;
    for (int i = 0; i < kNumScans; ++i) {
      builder.AddScan(ScanTime(i), SimulateScan(WorldPoseAt(i, kNumScans), kNumBeams));
      poses.push_back(builder.local_pose());
    }
    return poses;
  };

  const std::vector<Eigen::Affine2d> without_sink = run(nullptr);
  auto sink = std::make_shared<RecordingSink>();
  const std::vector<Eigen::Affine2d> with_sink = run(sink);

  ASSERT_EQ(without_sink.size(), with_sink.size());
  for (size_t i = 0; i < without_sink.size(); ++i) {
    // Exact, not near: an observer that moves the result by an ulp is still an observer that
    // changed the run, and every number this project reports would then depend on who was
    // watching.
    EXPECT_TRUE((without_sink[i].matrix().array() == with_sink[i].matrix().array()).all())
        << "scan " << i << " moved:\n"
        << without_sink[i].matrix() << "\nversus\n"
        << with_sink[i].matrix();
  }

  EXPECT_GT(sink->calls, 0);
  EXPECT_EQ(sink->last_poses.count("predicted"), 1u);
  EXPECT_EQ(sink->last_poses.count("coarse"), 1u);
  EXPECT_EQ(sink->last_poses.count("matched"), 1u);
  EXPECT_GT(sink->last_points, 0u);
  EXPECT_GT(sink->last_cells, 0u);
  EXPECT_GT(sink->last_score, 0.0);
}

}  // namespace
}  // namespace evergreenslam::mapping

/**
 * @file fast_correlative_scan_matcher_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief End to end check: relocalize with no prior in maps built by the real mapping pipeline.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "common/time.h"
#include "mapping/local_trajectory_builder.h"
#include "utils/scan_matching/fast_correlative_scan_matcher.h"
#include "utils/transform/transform.h"

namespace evergreenslam::utils::scan_matching {
namespace {

namespace transform = utils::transform;

// The relocalization-grade window and floor the constraint builder uses for wide pairs.
FastCorrelativeScanMatcher::MatchParams WideParams() {
  FastCorrelativeScanMatcher::MatchParams params;
  params.linear_search_window = 7.0;
  params.min_score = 0.55;
  return params;
}

// Wall coordinates deliberately not multiples of the 0.05 m grid resolution: on aligned walls
// every point sits exactly on a cell boundary.
struct World {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  // An off-center obstacle breaks the 180-degree symmetry a bare rectangle has.
  std::optional<Eigen::AlignedBox2d> pillar;
};

double RangeToBox(const Eigen::Vector2d& origin, double angle, const Eigen::AlignedBox2d& box) {
  const Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
  double t_min = 0.0;
  double t_max = std::numeric_limits<double>::max();
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(direction[axis]) < 1e-9) {
      if (origin[axis] < box.min()[axis] || origin[axis] > box.max()[axis]) {
        return std::numeric_limits<double>::max();
      }
      continue;
    }
    double t1 = (box.min()[axis] - origin[axis]) / direction[axis];
    double t2 = (box.max()[axis] - origin[axis]) / direction[axis];
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
  }
  if (t_min > t_max || t_min <= 0.0) {
    return std::numeric_limits<double>::max();
  }
  return t_min;
}

double RangeToBoundary(const World& world, const Eigen::Vector2d& origin, double angle) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double range = std::numeric_limits<double>::max();
  if (dx > 1e-6) {
    range = std::min(range, (world.max_x - origin.x()) / dx);
  } else if (dx < -1e-6) {
    range = std::min(range, (world.min_x - origin.x()) / dx);
  }
  if (dy > 1e-6) {
    range = std::min(range, (world.max_y - origin.y()) / dy);
  } else if (dy < -1e-6) {
    range = std::min(range, (world.min_y - origin.y()) / dy);
  }
  if (world.pillar.has_value()) {
    range = std::min(range, RangeToBox(origin, angle, *world.pillar));
  }
  return range;
}

// Every beam is cast from the same pose, so the scan is instantaneous and all offsets are zero.
sensor::TimedPointCloud SimulateTimedScan(const World& world, const Eigen::Affine2d& world_pose,
                                          int num_beams) {
  const double yaw = transform::GetYaw(world_pose);
  sensor::TimedPointCloud cloud;
  for (int i = 0; i < num_beams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / num_beams;
    const double range = RangeToBoundary(world, world_pose.translation(), yaw + bearing);
    cloud.push_back(
        sensor::TimedPoint2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing)),
                             common::Duration::zero()});
  }
  return cloud;
}

sensor::PointCloud SimulateScan(const World& world, const Eigen::Affine2d& world_pose,
                                int num_beams) {
  sensor::PointCloud cloud;
  for (const auto& point : SimulateTimedScan(world, world_pose, num_beams)) {
    cloud.push_back(sensor::Point2d{point.point});
  }
  return cloud;
}

common::Time ScanTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

double TranslationError(const Eigen::Affine2d& a, const Eigen::Affine2d& b) {
  return (a.translation() - b.translation()).norm();
}

double RotationError(const Eigen::Affine2d& a, const Eigen::Affine2d& b) {
  return std::abs(transform::NormalizeAngle(transform::GetYaw(a) - transform::GetYaw(b)));
}

// The builder's local frame is defined by the first scan, so grid_to_global is that first pose.
struct BuiltMap {
  std::unique_ptr<mapping::LocalTrajectoryBuilder> builder;
  std::vector<CandidateSubmap> candidates;
};

template <typename PoseAt>
BuiltMap BuildMap(const World& world, const PoseAt& pose_at, int num_scans, int num_beams) {
  BuiltMap built;
  built.builder =
      std::make_unique<mapping::LocalTrajectoryBuilder>(mapping::LocalTrajectoryBuilderOption());
  for (int i = 0; i < num_scans; ++i) {
    built.builder->AddScan(ScanTime(i), SimulateTimedScan(world, pose_at(i), num_beams));
  }
  const Eigen::Affine2d grid_to_global = pose_at(0);
  for (const auto& submap : built.builder->active_map().submaps()) {
    built.candidates.push_back(CandidateSubmap{grid_to_global, submap->Snapshot()});
  }
  return built;
}

TEST(FastCorrelativeScanMatcherE2eTest, RelocalizesWithNoPriorInAMappedRoom) {
  constexpr int kNumScans = 200;
  constexpr int kNumBeams = 360;
  const World world{0.013, 10.007, 0.021, 8.003,
                    Eigen::AlignedBox2d(Eigen::Vector2d(6.31, 1.52), Eigen::Vector2d(7.12, 2.33))};
  const auto pose_at = [](int i) {
    const double s = static_cast<double>(i) / (kNumScans - 1);
    return transform::FromXYTheta(2.0 + 4.0 * s, 2.0 + 1.5 * s, 0.6 * s);
  };
  const BuiltMap built = BuildMap(world, pose_at, kNumScans, kNumBeams);
  ASSERT_FALSE(built.candidates.empty());

  // A pose well off the driven trajectory, with a heading no scan was taken at.
  const Eigen::Affine2d query = transform::FromXYTheta(7.51, 5.52, 2.0);
  const sensor::PointCloud cloud = SimulateScan(world, query, kNumBeams);

  const FastCorrelativeScanMatcher matcher;
  const std::optional<GlobalMatchResult> result =
      matcher.Match(cloud, built.candidates, std::nullopt, WideParams());

  ASSERT_TRUE(result.has_value());
  EXPECT_GT(result->score, 0.6);
  EXPECT_LT(TranslationError(result->pose, query), 0.15)
      << "expected " << query.translation().transpose() << ", got "
      << result->pose.translation().transpose();
  EXPECT_LT(RotationError(result->pose, query), 0.05);
}

// A bare rectangular corridor is 180-degree symmetric, so with no prior the matcher may return
// the true pose or its rotated twin. The along-corridor coordinate is deliberately not pinned:
// grazing side walls are translation-invariant along that axis, so the best candidate can sit
// metres away (observed 3.1 m at score 0.78 on Linux, the true pose on macOS).
TEST(FastCorrelativeScanMatcherE2eTest, SymmetricCorridorGivesOneOfTheTwoModesAndPriorPicksOne) {
  constexpr int kNumScans = 150;
  constexpr int kNumBeams = 360;
  const World world{0.017, 9.213, 3.011, 4.522, std::nullopt};
  const Eigen::Vector2d center(0.5 * (world.min_x + world.max_x),
                               0.5 * (world.min_y + world.max_y));
  const auto pose_at = [&](int i) {
    const double s = static_cast<double>(i) / (kNumScans - 1);
    return transform::FromXYTheta(1.23 + 6.77 * s, center.y(), 0.0);
  };
  const BuiltMap built = BuildMap(world, pose_at, kNumScans, kNumBeams);
  ASSERT_FALSE(built.candidates.empty());

  // If odometry itself lost track, everything below would fail for unrelated reasons.
  const Eigen::Affine2d expected_motion = pose_at(0).inverse() * pose_at(kNumScans - 1);
  ASSERT_LT(TranslationError(built.builder->local_pose(), expected_motion), 0.15);

  const Eigen::Affine2d query = transform::FromXYTheta(2.53, 3.71, 0.07);
  Eigen::Affine2d twin = Eigen::Affine2d::Identity();
  twin.linear() =
      (Eigen::Rotation2Dd(M_PI) * Eigen::Rotation2Dd(transform::GetYaw(query))).toRotationMatrix();
  twin.translation() = 2.0 * center - query.translation();
  const sensor::PointCloud cloud = SimulateScan(world, query, kNumBeams);

  const FastCorrelativeScanMatcher matcher;
  const std::optional<GlobalMatchResult> no_prior =
      matcher.Match(cloud, built.candidates, std::nullopt, WideParams());
  ASSERT_TRUE(no_prior.has_value());
  EXPECT_GT(no_prior->score, 0.6);
  const double error_true = TranslationError(no_prior->pose, query);
  const double error_twin = TranslationError(no_prior->pose, twin);
  EXPECT_LT(std::min(error_true, error_twin), 0.15)
      << "matched neither mode, got " << no_prior->pose.translation().transpose();
  EXPECT_LT(std::min(RotationError(no_prior->pose, query), RotationError(no_prior->pose, twin)),
            0.05);

  const Eigen::Affine2d prior = transform::FromXYTheta(2.84, 3.60, 0.20);
  const std::optional<GlobalMatchResult> with_prior =
      matcher.Match(cloud, built.candidates, prior, WideParams());
  ASSERT_TRUE(with_prior.has_value());
  EXPECT_LT(RotationError(with_prior->pose, query), 0.05)
      << "prior did not exclude the flipped mode, got yaw " << transform::GetYaw(with_prior->pose);
  EXPECT_LT(std::abs(with_prior->pose.translation().y() - query.translation().y()), 0.15);
  // Along the corridor only the search window bounds the result (see the comment above).
  EXPECT_LT(std::abs(with_prior->pose.translation().x() - prior.translation().x()),
            WideParams().linear_search_window + 0.1);
}

}  // namespace
}  // namespace evergreenslam::utils::scan_matching

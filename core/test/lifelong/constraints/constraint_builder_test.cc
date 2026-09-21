/**
 * @file constraint_builder_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Candidate eligibility, budget, gates and the facade write path, on small real grids.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/constraint_builder.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "common/time.h"
#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// For the slack and footprint tests: the origin-distance gate would decide them first.
ConstraintBuilderOption UngatedOption() {
  ConstraintBuilderOption option;
  option.max_constraint_distance = std::numeric_limits<double>::infinity();
  return option;
}

// Wall coordinates deliberately off the grid-resolution lattice.
struct World {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
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

sensor::PointCloud SimulateScan(const World& world, const Eigen::Affine2d& world_pose,
                                int num_beams) {
  const double yaw = transform::GetYaw(world_pose);
  sensor::PointCloud cloud;
  for (int i = 0; i < num_beams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / num_beams;
    const double range = RangeToBoundary(world, world_pose.translation(), yaw + bearing);
    cloud.push_back(
        sensor::Point2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing))});
  }
  return cloud;
}

// These tests keep the session-local frame equal to the world frame.
std::shared_ptr<mapping::Submap> BuildSubmap(const World& world, int local_index,
                                             const Eigen::Affine2d& local_pose,
                                             const std::vector<Eigen::Affine2d>& scan_poses,
                                             bool finish) {
  static const mapping::CastRaysMapping inserter;
  auto submap = std::make_shared<mapping::Submap>(local_index, local_pose, kResolution);
  for (const Eigen::Affine2d& pose : scan_poses) {
    const sensor::PointCloud cloud = SimulateScan(world, pose, 240);
    submap->InsertScan(pose, cloud, inserter);
  }
  if (finish) {
    submap->Finish();
  }
  return submap;
}

std::vector<Eigen::Affine2d> ScanPosesAround(const Eigen::Affine2d& center, int count) {
  std::vector<Eigen::Affine2d> poses;
  for (int i = 0; i < count; ++i) {
    poses.push_back(Eigen::Affine2d(center * transform::FromXYTheta(0.07 * i, 0.05 * i, 0.03 * i)));
  }
  return poses;
}

testing::ExplicitInsertion MakeInsertion(
    const NodeId& node_id, const Eigen::Affine2d& local_pose, const sensor::PointCloud& cloud,
    const std::vector<std::pair<SubmapId, std::shared_ptr<const mapping::Submap>>>& submaps) {
  testing::ExplicitInsertion insertion;
  insertion.node_id = node_id;
  insertion.node.time = TestTime(node_id.node_index);
  insertion.node.local_pose = local_pose;
  insertion.node.point_cloud = cloud;
  insertion.insertion_submaps = submaps;
  return insertion;
}

const World kRoom{0.013, 8.011, 0.017, 6.005,
                  Eigen::AlignedBox2d(Eigen::Vector2d(5.11, 1.52), Eigen::Vector2d(5.92, 2.33))};
// Same session frame as kRoom, 22 m of nothing in between.
const World kFarRoom{30.013, 38.011, 0.017, 6.005, std::nullopt};
const World kHall{0.013, 40.011, 0.017, 6.005, std::nullopt};

// One finished submap plus a query node elsewhere in it, prior deliberately off the true pose.
struct RoomFixture {
  PoseGraph pose_graph;
  SessionId session;
  SubmapId room_submap_id;
  NodeId query_node_id;
  Eigen::Affine2d true_node_pose = Eigen::Affine2d::Identity();
  Eigen::Affine2d submap_local_pose = Eigen::Affine2d::Identity();
  std::shared_ptr<const mapping::Submap> unfinished_submap;
};

// The query scan comes from query_world: the map is always kRoom, so a different world here is
// the scene having changed since the map was built.
void FeedRoomFixture(RoomFixture& fixture, const World& query_world = kRoom,
                     int num_map_scans = 12) {
  fixture.session = fixture.pose_graph.StartNewSession(TestTime(0));
  const int session_index = fixture.session.session_index;
  fixture.room_submap_id = SubmapId{session_index, 0};
  fixture.query_node_id = NodeId{session_index, 1};
  fixture.submap_local_pose = transform::FromXYTheta(2.1, 2.6, 0.1);

  const std::shared_ptr<mapping::Submap> room_submap =
      BuildSubmap(kRoom, 0, fixture.submap_local_pose,
                  ScanPosesAround(fixture.submap_local_pose, num_map_scans), true);
  testing::EnqueueExplicitInsertion(
      fixture.pose_graph, MakeInsertion(NodeId{session_index, 0}, fixture.submap_local_pose,
                                        SimulateScan(kRoom, fixture.submap_local_pose, 240),
                                        {{fixture.room_submap_id, room_submap}}));

  fixture.true_node_pose = transform::FromXYTheta(5.53, 4.42, 1.0);
  const Eigen::Affine2d prior_pose =
      Eigen::Affine2d(fixture.true_node_pose * transform::FromXYTheta(0.3, -0.2, 0.05));
  fixture.unfinished_submap = std::make_shared<const mapping::Submap>(1, prior_pose, kResolution);
  testing::EnqueueExplicitInsertion(
      fixture.pose_graph, MakeInsertion(fixture.query_node_id, prior_pose,
                                        SimulateScan(query_world, fixture.true_node_pose, 240),
                                        {{SubmapId{session_index, 1}, fixture.unfinished_submap}}));
  fixture.pose_graph.WaitUntilQuiescent();
}

NodeId AddQueryNode(RoomFixture& fixture, int node_index, const Eigen::Affine2d& true_pose) {
  const NodeId node_id{fixture.session.session_index, node_index};
  testing::EnqueueExplicitInsertion(
      fixture.pose_graph,
      MakeInsertion(node_id, true_pose, SimulateScan(kRoom, true_pose, 240),
                    {{SubmapId{fixture.session.session_index, 1}, fixture.unfinished_submap}}));
  fixture.pose_graph.WaitUntilQuiescent();
  return node_id;
}

TEST(ConstraintBuilder, AcceptsATrueLoopAndWritesThroughTheFacade) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());

  const int residuals_before = fixture.pose_graph.optimization().num_residual_blocks();
  builder.SearchForNode(fixture.query_node_id);
  fixture.pose_graph.WaitUntilQuiescent();

  EXPECT_EQ(builder.num_matches_attempted(), 1);
  ASSERT_EQ(builder.num_constraints_added(), 1);

  const std::vector<Constraint>& constraints = fixture.pose_graph.graph().constraints();
  const auto it = std::find_if(constraints.begin(), constraints.end(), [](const Constraint& c) {
    return c.type == Constraint::Type::INTER_SUBMAP;
  });
  ASSERT_TRUE(it != constraints.end());
  EXPECT_TRUE(it->from == VariableId::Of(fixture.room_submap_id));
  ASSERT_TRUE(it->to.has_value());
  EXPECT_TRUE(*it->to == VariableId::Of(fixture.query_node_id));
  EXPECT_TRUE(it->sqrt_information.isApprox(
      LeverArmSqrtInformation(LoopClosureSqrtInformation(ConstraintWeightOption()),
                              it->relative_pose.translation().norm())));

  const Eigen::Affine2d expected =
      Eigen::Affine2d(fixture.submap_local_pose.inverse() * fixture.true_node_pose);
  EXPECT_LT((it->relative_pose.translation() - expected.translation()).norm(), 0.1);
  EXPECT_LT(std::abs(transform::NormalizeAngle(transform::GetYaw(it->relative_pose) -
                                               transform::GetYaw(expected))),
            0.05);

  EXPECT_EQ(fixture.pose_graph.optimization().num_residual_blocks(), residuals_before + 1);
}

// The closure weight is configuration, not a constant baked into the match path.
TEST(ConstraintBuilder, ClosureCarriesTheConfiguredWeight) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  ConstraintWeightOption weight;
  weight.loop_closure_translation_stddev = 0.02;
  weight.loop_closure_rotation_stddev = 0.005;
  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption(), weight);

  builder.SearchForNode(fixture.query_node_id);
  fixture.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(builder.num_constraints_added(), 1);

  const std::vector<Constraint>& constraints = fixture.pose_graph.graph().constraints();
  const auto it = std::find_if(constraints.begin(), constraints.end(), [](const Constraint& c) {
    return c.type == Constraint::Type::INTER_SUBMAP;
  });
  ASSERT_TRUE(it != constraints.end());
  // Translation rows shrink with the lever arm: 50 / sqrt(1 + (50 * 0.005 * L)^2).
  const double lever_arm = it->relative_pose.translation().norm();
  ASSERT_GT(lever_arm, 0.1) << "the fixture has to close over a real lever arm";
  const double expected_translation = 50.0 / std::hypot(1.0, 50.0 * 0.005 * lever_arm);
  EXPECT_DOUBLE_EQ(it->sqrt_information(0, 0), expected_translation);
  EXPECT_DOUBLE_EQ(it->sqrt_information(1, 1), expected_translation);
  EXPECT_LT(it->sqrt_information(0, 0), 50.0);
  EXPECT_EQ(it->sqrt_information(2, 2), 200.0);
}

TEST(ConstraintBuilder, SearchForSubmapFindsTheSameLoopFromTheOtherDirection) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());

  builder.SearchForSubmap(fixture.room_submap_id);
  fixture.pose_graph.WaitUntilQuiescent();

  EXPECT_EQ(builder.num_matches_attempted(), 1);
  EXPECT_EQ(builder.num_constraints_added(), 1);
}

TEST(ConstraintBuilder, DoesNotRetryAnAttemptedPair) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());

  builder.SearchForNode(fixture.query_node_id);
  fixture.pose_graph.WaitUntilQuiescent();
  builder.SearchForNode(fixture.query_node_id);
  builder.SearchForSubmap(fixture.room_submap_id);
  fixture.pose_graph.WaitUntilQuiescent();

  EXPECT_EQ(builder.num_matches_attempted(), 1);
  EXPECT_EQ(builder.num_constraints_added(), 1);
}

TEST(ConstraintBuilder, SkipsOwnUnfinishedAndFarSubmaps) {
  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  const int session_index = session.session_index;

  const Eigen::Affine2d own_pose = transform::FromXYTheta(2.1, 2.6, 0.0);
  const std::shared_ptr<mapping::Submap> own_submap =
      BuildSubmap(kRoom, 0, own_pose, ScanPosesAround(own_pose, 4), true);
  const Eigen::Affine2d open_pose = transform::FromXYTheta(4.1, 3.1, 0.2);
  const std::shared_ptr<mapping::Submap> open_submap =
      BuildSubmap(kRoom, 1, open_pose, ScanPosesAround(open_pose, 4), false);

  const NodeId node_id{session_index, 0};
  testing::EnqueueExplicitInsertion(
      pose_graph, MakeInsertion(node_id, own_pose, SimulateScan(kRoom, own_pose, 240),
                                {{SubmapId{session_index, 0}, own_submap},
                                 {SubmapId{session_index, 1}, open_submap}}));
  pose_graph.WaitUntilQuiescent();

  ConstraintBuilderOption no_slack;
  no_slack.max_candidate_slack = 0.0;
  ConstraintBuilder builder(pose_graph, no_slack);
  builder.SearchForNode(node_id);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 0);

  open_submap->Finish();
  const Eigen::Affine2d far_pose = transform::FromXYTheta(32.5, 3.1, 0.4);
  const std::shared_ptr<mapping::Submap> far_submap =
      BuildSubmap(kFarRoom, 2, far_pose, ScanPosesAround(far_pose, 4), true);
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(NodeId{session_index, 1}, far_pose, SimulateScan(kFarRoom, far_pose, 240),
                    {{SubmapId{session_index, 2}, far_submap}}));
  pose_graph.WaitUntilQuiescent();

  builder.SearchForNode(node_id);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 0);
}

// With a fixed margin around the footprint a node whose estimate drifted beyond it can never see
// the submap that would close the loop; the margin must grow with the pair's odometry path length.
TEST(ConstraintBuilder, DriftSlackGrowsTheFootprintMarginAndTheSearchWindow) {
  const auto run = [](double max_candidate_slack) {
    PoseGraph pose_graph;
    const SessionId session = pose_graph.StartNewSession(TestTime(0));
    const int session_index = session.session_index;

    const Eigen::Affine2d submap_pose = transform::FromXYTheta(2.1, 2.6, 0.1);
    const std::shared_ptr<mapping::Submap> room_submap =
        BuildSubmap(kRoom, 0, submap_pose, ScanPosesAround(submap_pose, 12), true);
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(NodeId{session_index, 0}, submap_pose, SimulateScan(kRoom, submap_pose, 240),
                      {{SubmapId{session_index, 0}, room_submap}}));

    ConstraintBuilderOption option = UngatedOption();
    option.candidate_slack_per_meter = 1.0;  // Test scale: metres of walk, not kilometres.
    option.max_candidate_slack = max_candidate_slack;
    ConstraintBuilder builder(pose_graph, option);
    pose_graph.WaitUntilQuiescent();
    builder.SearchForNode(NodeId{session_index, 0});
    pose_graph.WaitUntilQuiescent();

    // The walk nodes carry empty clouds, so they only accumulate path and never match.
    const auto walk_submap =
        std::make_shared<const mapping::Submap>(1, Eigen::Affine2d::Identity(), kResolution);
    int next_node_index = 1;
    for (int i = 0; i < 10; ++i) {
      const double along = i < 5 ? 4.0 * (i + 1) : 4.0 * (10 - i);
      const NodeId walk_id{session_index, next_node_index++};
      testing::EnqueueExplicitInsertion(
          pose_graph, MakeInsertion(walk_id, transform::FromXYTheta(2.1 + along, 30.0, 0.0), {},
                                    {{SubmapId{session_index, 1}, walk_submap}}));
      builder.SearchForNode(walk_id);
      pose_graph.WaitUntilQuiescent();
    }

    // The query node is truly back in the room, but its estimate has drifted 10 m past its walls.
    const Eigen::Affine2d true_pose = transform::FromXYTheta(5.53, 4.42, 1.0);
    const Eigen::Affine2d drifted_estimate =
        Eigen::Affine2d(transform::FromXYTheta(13.0, 1.5, 0.0) * true_pose);
    const NodeId query_id{session_index, next_node_index};
    testing::EnqueueExplicitInsertion(
        pose_graph, MakeInsertion(query_id, drifted_estimate, SimulateScan(kRoom, true_pose, 240),
                                  {{SubmapId{session_index, 1}, walk_submap}}));
    builder.SearchForNode(query_id);
    pose_graph.WaitUntilQuiescent();
    return builder.num_constraints_added();
  };

  // A fixed margin (slack capped at 0) is the deadlock: the loop is never even tried.
  EXPECT_EQ(run(0.0), 0);
  EXPECT_EQ(run(30.0), 1);
}

// A loaded frozen map has no path record; the cap that covers an unknown drift belongs to
// floating sessions only, or every frozen submap sits inside a 40 m candidate radius.
TEST(ConstraintBuilder, AFrozenEndpointAddsNoDriftSlack) {
  const auto attempted = [](bool freeze_base) {
    PoseGraph pose_graph;
    const SessionId base = pose_graph.StartNewSession(TestTime(0));
    const Eigen::Affine2d base_pose = transform::FromXYTheta(2.1, 2.6, 0.1);
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(NodeId{base.session_index, 0}, base_pose, SimulateScan(kRoom, base_pose, 240),
                      {{SubmapId{base.session_index, 0},
                        BuildSubmap(kRoom, 0, base_pose, ScanPosesAround(base_pose, 12), true)}}));
    if (freeze_base) {
      pose_graph.FreezeSession(base);
    }
    pose_graph.WaitUntilQuiescent();

    const SessionId fed = pose_graph.StartNewSession(TestTime(1));
    // 9 m past the room's footprint: only the 30 m cap reaches it.
    const Eigen::Affine2d far_pose = transform::FromXYTheta(17.1, 2.6, 0.0);
    const NodeId query{fed.session_index, 0};
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(query, far_pose, SimulateScan(kRoom, far_pose, 240),
                      {{SubmapId{fed.session_index, 0},
                        std::make_shared<const mapping::Submap>(0, far_pose, kResolution)}}));
    pose_graph.WaitUntilQuiescent();

    ConstraintBuilder builder(pose_graph, UngatedOption());
    builder.SearchForNode(query);
    pose_graph.WaitUntilQuiescent();
    return builder.num_matches_attempted();
  };

  EXPECT_EQ(attempted(/*freeze_base=*/false), 1) << "a floating base without a path gets the cap";
  EXPECT_EQ(attempted(/*freeze_base=*/true), 0);
}

// A closure ties the trajectory before it, so drift restarts there. The walk nodes carry empty
// clouds and only accumulate path; the query's estimate sits 2.5 m past the east wall, which the
// whole walk reaches at 0.25 m per metre but the leg since the closure does not.
int QueryAttemptsAfterClosureWalk(bool frozen_base, int min_loop_node_gap, bool closing_scan) {
  PoseGraph pose_graph;
  const SessionId base = pose_graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d room_pose = transform::FromXYTheta(2.1, 2.6, 0.0);
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(NodeId{base.session_index, 0}, room_pose, SimulateScan(kRoom, room_pose, 240),
                    {{SubmapId{base.session_index, 0},
                      BuildSubmap(kRoom, 0, room_pose, ScanPosesAround(room_pose, 12), true)}}));
  SessionId session = base;
  if (frozen_base) {
    pose_graph.FreezeSession(base);
    pose_graph.WaitUntilQuiescent();
    session = pose_graph.StartNewSession(TestTime(1));
  }
  const int session_index = session.session_index;

  ConstraintBuilderOption option;
  option.candidate_slack_per_meter = 0.25;
  option.min_loop_node_gap = min_loop_node_gap;
  ConstraintBuilder builder(pose_graph, option);
  pose_graph.WaitUntilQuiescent();
  int next_index = 0;
  if (!frozen_base) {
    builder.SearchForNode(NodeId{session_index, next_index++});
  }
  const auto walk_submap =
      std::make_shared<const mapping::Submap>(1, Eigen::Affine2d::Identity(), kResolution);
  const auto feed = [&](const Eigen::Affine2d& estimate, const sensor::PointCloud& cloud) {
    const NodeId node_id{session_index, next_index++};
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(node_id, estimate, cloud, {{SubmapId{session_index, 1}, walk_submap}}));
    builder.SearchForNode(node_id);
    pose_graph.WaitUntilQuiescent();
  };
  for (int i = 0; i < 5; ++i) {
    feed(transform::FromXYTheta(i % 2 == 0 ? 7.0 : 3.0, 2.6, 0.0), {});
  }
  const Eigen::Affine2d closing_pose = transform::FromXYTheta(5.53, 4.42, 1.0);
  feed(Eigen::Affine2d(closing_pose * transform::FromXYTheta(0.3, -0.2, 0.05)),
       closing_scan ? SimulateScan(kRoom, closing_pose, 240) : sensor::PointCloud{});
  EXPECT_EQ(builder.num_constraints_added(), closing_scan ? 1 : 0);

  const int before = builder.num_matches_attempted();
  feed(transform::FromXYTheta(10.5, 4.0, 1.0), SimulateScan(kRoom, closing_pose, 240));
  return builder.num_matches_attempted() - before;
}

TEST(ConstraintBuilder, AValidSameSessionClosureResetsTheDrift) {
  // The closing node is six nodes past the submap's only node.
  EXPECT_EQ(QueryAttemptsAfterClosureWalk(/*frozen_base=*/false, /*min_loop_node_gap=*/6,
                                          /*closing_scan=*/true),
            0);
  EXPECT_EQ(QueryAttemptsAfterClosureWalk(/*frozen_base=*/false, /*min_loop_node_gap=*/6,
                                          /*closing_scan=*/false),
            1);
}

TEST(ConstraintBuilder, AClosureBelowTheNodeGapIsNotALoop) {
  EXPECT_EQ(QueryAttemptsAfterClosureWalk(/*frozen_base=*/false, /*min_loop_node_gap=*/7,
                                          /*closing_scan=*/true),
            1);
}

TEST(ConstraintBuilder, ACrossSessionClosureOntoAFrozenBaseResetsTheDrift) {
  EXPECT_EQ(QueryAttemptsAfterClosureWalk(/*frozen_base=*/true, /*min_loop_node_gap=*/100,
                                          /*closing_scan=*/true),
            0);
  EXPECT_EQ(QueryAttemptsAfterClosureWalk(/*frozen_base=*/true, /*min_loop_node_gap=*/100,
                                          /*closing_scan=*/false),
            1);
}

// The base is never searched by this builder, so it has no path record, like a session loaded
// from disk: its drift is unknown until a closure ties it.
TEST(ConstraintBuilder, APathlessFloatingSessionIsTiedByItsFirstClosure) {
  PoseGraph pose_graph;
  const SessionId base = pose_graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d base_pose = transform::FromXYTheta(2.1, 2.6, 0.1);
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(NodeId{base.session_index, 0}, base_pose, SimulateScan(kRoom, base_pose, 240),
                    {{SubmapId{base.session_index, 0},
                      BuildSubmap(kRoom, 0, base_pose, ScanPosesAround(base_pose, 12), true)}}));
  pose_graph.WaitUntilQuiescent();
  const SessionId fed = pose_graph.StartNewSession(TestTime(1));
  const int fed_index = fed.session_index;
  const auto fed_submap =
      std::make_shared<const mapping::Submap>(0, Eigen::Affine2d::Identity(), kResolution);
  const auto feed = [&](int index, const Eigen::Affine2d& estimate,
                        const sensor::PointCloud& cloud) {
    testing::EnqueueExplicitInsertion(pose_graph,
                                      MakeInsertion(NodeId{fed_index, index}, estimate, cloud,
                                                    {{SubmapId{fed_index, 0}, fed_submap}}));
  };
  // 9 m past the room's footprint: only the cap reaches it.
  const Eigen::Affine2d far_pose = transform::FromXYTheta(17.1, 2.6, 0.0);
  const Eigen::Affine2d true_pose = transform::FromXYTheta(5.53, 4.42, 1.0);
  feed(0, far_pose, {});
  feed(1, Eigen::Affine2d(true_pose * transform::FromXYTheta(0.3, -0.2, 0.05)),
       SimulateScan(kRoom, true_pose, 240));
  feed(2, far_pose, {});
  pose_graph.WaitUntilQuiescent();

  ConstraintBuilder builder(pose_graph, UngatedOption());
  builder.SearchForNode(NodeId{fed_index, 0});
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 1);
  builder.SearchForNode(NodeId{fed_index, 1});
  pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(builder.num_constraints_added(), 1);
  builder.SearchForNode(NodeId{fed_index, 2});
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 2);
}

// The submap's origin is not the measure: a long hall is one submap, and a node at its far end
// is inside the evidence the submap holds.
TEST(ConstraintBuilder, ANodeFarFromTheOriginButInsideTheFootprintIsACandidate) {
  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  const int session_index = session.session_index;

  const Eigen::Affine2d submap_pose = transform::FromXYTheta(2.1, 2.6, 0.1);
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(
          NodeId{session_index, 0}, submap_pose, SimulateScan(kHall, submap_pose, 240),
          {{SubmapId{session_index, 0},
            BuildSubmap(kHall, 0, submap_pose, ScanPosesAround(submap_pose, 12), true)}}));
  const Eigen::Affine2d far_end = transform::FromXYTheta(36.5, 3.1, 0.4);
  const NodeId query{session_index, 1};
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(query, far_end, SimulateScan(kHall, far_end, 240),
                    {{SubmapId{session_index, 1},
                      std::make_shared<const mapping::Submap>(1, far_end, kResolution)}}));
  pose_graph.WaitUntilQuiescent();

  ConstraintBuilderOption no_slack = UngatedOption();
  no_slack.max_candidate_slack = 0.0;
  ConstraintBuilder builder(pose_graph, no_slack);
  builder.SearchForNode(query);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 1);
}

TEST(ConstraintBuilder, SlackIsMeasuredFromTheFootprintEdge) {
  // The room submap never records a path, so every pair against it gets exactly the cap. Its
  // frame is unrotated: the footprint box is axis-aligned in the grid frame, and a rotated room
  // would pad it.
  const auto attempted = [](double estimate_x) {
    PoseGraph pose_graph;
    const SessionId session = pose_graph.StartNewSession(TestTime(0));
    const int session_index = session.session_index;

    const Eigen::Affine2d submap_pose = transform::FromXYTheta(2.1, 2.6, 0.0);
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(
            NodeId{session_index, 0}, submap_pose, SimulateScan(kRoom, submap_pose, 240),
            {{SubmapId{session_index, 0},
              BuildSubmap(kRoom, 0, submap_pose, ScanPosesAround(submap_pose, 12), true)}}));
    const Eigen::Affine2d true_pose = transform::FromXYTheta(5.53, 4.42, 1.0);
    const Eigen::Affine2d estimate = transform::FromXYTheta(estimate_x, 3.0, 1.0);
    const NodeId query{session_index, 1};
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(query, estimate, SimulateScan(kRoom, true_pose, 240),
                      {{SubmapId{session_index, 1},
                        std::make_shared<const mapping::Submap>(1, estimate, kResolution)}}));
    pose_graph.WaitUntilQuiescent();

    ConstraintBuilderOption option;
    option.max_candidate_slack = 1.0;
    ConstraintBuilder builder(pose_graph, option);
    builder.SearchForNode(query);
    pose_graph.WaitUntilQuiescent();
    return builder.num_matches_attempted();
  };

  // The east wall is at x = 8.011; the footprint ends within one cell of it.
  EXPECT_EQ(attempted(8.7), 1);
  EXPECT_EQ(attempted(9.3), 0);
}

TEST(ConstraintBuilder, SearchForSubmapAndSearchForNodeAgreeOnPairs) {
  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  const int session_index = session.session_index;

  const auto add_finished = [&](int index, const World& world, const Eigen::Affine2d& pose) {
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(NodeId{session_index, index}, pose, SimulateScan(world, pose, 240),
                      {{SubmapId{session_index, index},
                        BuildSubmap(world, index, pose, ScanPosesAround(pose, 12), true)}}));
  };
  add_finished(0, kRoom, transform::FromXYTheta(2.1, 2.6, 0.1));
  add_finished(1, kRoom, transform::FromXYTheta(5.7, 3.3, -0.4));
  add_finished(2, kFarRoom, transform::FromXYTheta(32.5, 3.1, 0.4));
  const auto live_submap =
      std::make_shared<const mapping::Submap>(3, Eigen::Affine2d::Identity(), kResolution);
  for (int i = 0; i < 2; ++i) {
    const Eigen::Affine2d pose = transform::FromXYTheta(4.53 + 0.3 * i, 4.42, 1.0);
    testing::EnqueueExplicitInsertion(
        pose_graph,
        MakeInsertion(NodeId{session_index, 3 + i}, pose, SimulateScan(kRoom, pose, 240),
                      {{SubmapId{session_index, 3}, live_submap}}));
  }
  pose_graph.WaitUntilQuiescent();

  ConstraintBuilderOption no_slack;
  no_slack.max_candidate_slack = 0.0;
  ConstraintBuilder by_node(pose_graph, no_slack);
  for (int index = 0; index < 5; ++index) {
    by_node.SearchForNode(NodeId{session_index, index});
  }
  pose_graph.WaitUntilQuiescent();
  ConstraintBuilder by_submap(pose_graph, no_slack);
  for (int index = 0; index < 3; ++index) {
    by_submap.SearchForSubmap(SubmapId{session_index, index});
  }
  pose_graph.WaitUntilQuiescent();

  // Nodes 0/1 against each other's room submap, the two live nodes against both; the far room
  // reaches nothing and nothing reaches it.
  EXPECT_EQ(by_node.num_matches_attempted(), 6);
  EXPECT_EQ(by_submap.num_matches_attempted(), 6);
  EXPECT_EQ(by_submap.num_constraints_added(), by_node.num_constraints_added());
}

// Moved furniture: a fifth of the scan lands in cells the map observed free. A window that
// cannot alias far may accept the changed scene; the same pair under a wide window may not.
// Twelve mapping scans leave free cells near the 0.4 threshold, so the map here is denser.
// The room submap never records a path, so the pair's slack is exactly the cap.
TEST(ConstraintBuilder, ATightWindowToleratesAChangedSceneAndAWideOneDoesNot) {
  const World moved{kRoom.min_x, kRoom.max_x, kRoom.min_y, kRoom.max_y,
                    Eigen::AlignedBox2d(Eigen::Vector2d(6.2, 3.95), Eigen::Vector2d(7.4, 4.9))};
  const auto added = [&moved](double max_candidate_slack, double tight_window_max) {
    RoomFixture fixture;
    FeedRoomFixture(fixture, moved, /*num_map_scans=*/40);
    ConstraintBuilderOption option;
    option.search_window_floor = 0.5;
    option.max_candidate_slack = max_candidate_slack;
    option.tight_window_max = tight_window_max;
    ConstraintBuilder builder(fixture.pose_graph, option);
    builder.SearchForNode(fixture.query_node_id);
    fixture.pose_graph.WaitUntilQuiescent();
    EXPECT_EQ(builder.num_matches_attempted(), 1);
    if (builder.num_constraints_added() == 1) {
      const std::vector<Constraint>& constraints = fixture.pose_graph.graph().constraints();
      const auto it = std::find_if(constraints.begin(), constraints.end(), [](const Constraint& c) {
        return c.type == Constraint::Type::INTER_SUBMAP;
      });
      const Eigen::Affine2d expected =
          Eigen::Affine2d(fixture.submap_local_pose.inverse() * fixture.true_node_pose);
      EXPECT_LT((it->relative_pose.translation() - expected.translation()).norm(), 0.1);
    }
    return builder.num_constraints_added();
  };

  // The window is the 0.5 m floor plus the slack: 0.9 m takes the tight gates, 1.1 m the wide.
  EXPECT_EQ(added(/*max_candidate_slack=*/0.4, /*tight_window_max=*/1.0), 1);
  EXPECT_EQ(added(/*max_candidate_slack=*/0.6, /*tight_window_max=*/1.0), 0);
  // The wide window found the same pose; only the gate set rejected it.
  EXPECT_EQ(added(/*max_candidate_slack=*/0.6, /*tight_window_max=*/1.5), 1);
}

TEST(ConstraintBuilder, FrozenAgainstFrozenIsNeverMatched) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  fixture.pose_graph.FreezeSession(fixture.session);
  fixture.pose_graph.WaitUntilQuiescent();

  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());
  builder.SearchForNode(fixture.query_node_id);
  fixture.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 0);
}

TEST(ConstraintBuilder, SamplingRatioGatesNodeRoundsButNotSubmapRounds) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);

  ConstraintBuilderOption option;
  option.sampler_option.sampling_ratio = 0.5;
  ConstraintBuilder builder(fixture.pose_graph, option);

  // Rounds 1 and 3 run; the pair dedupe makes the second running round attempt nothing new.
  for (int round = 0; round < 4; ++round) {
    builder.SearchForNode(fixture.query_node_id);
  }
  fixture.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 1);

  // The submap direction is not round-gated: a skipped node round does not eat its budget.
  ConstraintBuilderOption never = ConstraintBuilderOption();
  never.sampler_option.sampling_ratio = 0.0;
  RoomFixture second;
  FeedRoomFixture(second);
  ConstraintBuilder gated(second.pose_graph, never);
  gated.SearchForNode(second.query_node_id);
  second.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(gated.num_matches_attempted(), 0);
  gated.SearchForSubmap(second.room_submap_id);
  second.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(gated.num_matches_attempted(), 1);
}

// A finished submap's grid never changes, so its branch-and-bound stack is worth keeping across
// pairs; rebuilding it is the dominant per-pair cost.
TEST(ConstraintBuilder, ReusesOnePrecomputationStackPerSubmap) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  const NodeId second_query = AddQueryNode(fixture, 2, transform::FromXYTheta(2.53, 4.42, -0.7));
  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());

  builder.SearchForNode(fixture.query_node_id);
  fixture.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 1);
  EXPECT_EQ(builder.num_precomputation_builds(), 1);
  EXPECT_EQ(builder.num_cached_precomputations(), 1);

  builder.SearchForNode(second_query);
  fixture.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_matches_attempted(), 2);
  EXPECT_EQ(builder.num_precomputation_builds(), 1);
  EXPECT_EQ(builder.num_cached_precomputations(), 1);
}

int PrecomputationBuildsOverTwoRounds(int cache_size) {
  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  const int session_index = session.session_index;

  const Eigen::Affine2d pose_a = transform::FromXYTheta(2.1, 2.6, 0.1);
  const Eigen::Affine2d pose_b = transform::FromXYTheta(5.7, 3.3, -0.4);
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(NodeId{session_index, 0}, pose_a, SimulateScan(kRoom, pose_a, 240),
                    {{SubmapId{session_index, 0},
                      BuildSubmap(kRoom, 0, pose_a, ScanPosesAround(pose_a, 12), true)}}));
  testing::EnqueueExplicitInsertion(
      pose_graph,
      MakeInsertion(NodeId{session_index, 1}, pose_b, SimulateScan(kRoom, pose_b, 240),
                    {{SubmapId{session_index, 1},
                      BuildSubmap(kRoom, 1, pose_b, ScanPosesAround(pose_b, 12), true)}}));

  const auto live_submap =
      std::make_shared<const mapping::Submap>(2, Eigen::Affine2d::Identity(), kResolution);
  ConstraintBuilderOption option;
  option.precomputation_cache_size = cache_size;
  ConstraintBuilder builder(pose_graph, option);
  pose_graph.WaitUntilQuiescent();

  for (int i = 0; i < 2; ++i) {
    const Eigen::Affine2d query_pose = transform::FromXYTheta(4.53 + 0.3 * i, 4.42, 1.0);
    const NodeId query_id{session_index, 2 + i};
    testing::EnqueueExplicitInsertion(
        pose_graph, MakeInsertion(query_id, query_pose, SimulateScan(kRoom, query_pose, 240),
                                  {{SubmapId{session_index, 2}, live_submap}}));
    builder.SearchForNode(query_id);
    pose_graph.WaitUntilQuiescent();
  }
  EXPECT_EQ(builder.num_matches_attempted(), 4);
  EXPECT_LE(builder.num_cached_precomputations(), cache_size);
  return builder.num_precomputation_builds();
}

TEST(ConstraintBuilder, EvictsTheLeastRecentlyUsedPrecomputation) {
  // Two submaps alternating: a cache of two holds both, a cache of one thrashes, zero disables.
  EXPECT_EQ(PrecomputationBuildsOverTwoRounds(2), 2);
  EXPECT_EQ(PrecomputationBuildsOverTwoRounds(1), 4);
  EXPECT_EQ(PrecomputationBuildsOverTwoRounds(0), 0);
}

TEST(ConstraintBuilder, SweepDropsPrecomputationForVanishedSubmaps) {
  RoomFixture fixture;
  FeedRoomFixture(fixture);
  ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());
  builder.SearchForNode(fixture.query_node_id);
  fixture.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(builder.num_cached_precomputations(), 1);

  TrimRequest request;
  request.deleted_submap_ids.push_back(fixture.room_submap_id);
  fixture.pose_graph.ApplyTrim(request);
  fixture.pose_graph.WaitUntilQuiescent();
  ASSERT_FALSE(fixture.pose_graph.graph().HasSubmap(fixture.room_submap_id));
  ASSERT_TRUE(fixture.pose_graph.graph().HasNode(fixture.query_node_id));

  for (int i = 0; i < 256; ++i) {
    builder.SearchForNode(fixture.query_node_id);
  }
  fixture.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(builder.num_cached_precomputations(), 0);
}

TEST(ConstraintBuilder, IdenticalInputYieldsBitwiseIdenticalConstraints) {
  std::vector<std::vector<double>> runs;
  for (int run = 0; run < 2; ++run) {
    RoomFixture fixture;
    FeedRoomFixture(fixture);
    ConstraintBuilder builder(fixture.pose_graph, ConstraintBuilderOption());
    builder.SearchForNode(fixture.query_node_id);
    fixture.pose_graph.WaitUntilQuiescent();

    std::vector<double> flattened;
    for (const Constraint& constraint : fixture.pose_graph.graph().constraints()) {
      if (constraint.type != Constraint::Type::INTER_SUBMAP) {
        continue;
      }
      flattened.push_back(constraint.relative_pose.translation().x());
      flattened.push_back(constraint.relative_pose.translation().y());
      flattened.push_back(transform::GetYaw(constraint.relative_pose));
    }
    runs.push_back(flattened);
  }
  ASSERT_FALSE(runs[0].empty());
  EXPECT_EQ(runs[0], runs[1]);
}

}  // namespace
}  // namespace evergreenslam::lifelong

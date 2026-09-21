/**
 * @file constraint_builder_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief End to end: a drifted simulated loop must close, and negative samples must not.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "common/time.h"
#include "lifelong/constraints/constraint_builder.h"
#include "lifelong/pose_graph.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/scan_matching/fast_correlative_scan_matcher.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;
constexpr int kNodesPerSubmap = 10;
constexpr int kNumBeams = 360;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// Wall coordinates deliberately off the grid-resolution lattice.
struct World {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  std::vector<Eigen::AlignedBox2d> pillars;
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
  for (const Eigen::AlignedBox2d& pillar : world.pillars) {
    range = std::min(range, RangeToBox(origin, angle, pillar));
  }
  return range;
}

// max_range models the sensor, which is what makes a long corridor genuinely self-similar
// instead of pinned by far end caps.
sensor::PointCloud SimulateScan(const World& world, const Eigen::Affine2d& world_pose,
                                double max_range) {
  const double yaw = transform::GetYaw(world_pose);
  sensor::PointCloud cloud;
  for (int i = 0; i < kNumBeams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / kNumBeams;
    const double range = RangeToBoundary(world, world_pose.translation(), yaw + bearing);
    if (range > max_range) {
      continue;
    }
    cloud.push_back(
        sensor::Point2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing))});
  }
  return cloud;
}

// Feeding drifted odometry bakes the drift into the grids exactly like the real pipeline does.
class SessionFeeder {
 public:
  SessionFeeder(PoseGraph& pose_graph, SessionId session, const World& world, double max_range)
      : pose_graph_(pose_graph), session_(session), world_(world), max_range_(max_range) {}

  void FeedNode(int index, const Eigen::Affine2d& true_pose, const Eigen::Affine2d& local_pose) {
    const int newest = index / kNodesPerSubmap;
    while (static_cast<int>(submaps_.size()) <= newest) {
      const int k = static_cast<int>(submaps_.size());
      submaps_.push_back(std::make_shared<mapping::Submap>(k, local_pose, kResolution));
      if (k >= 2) {
        submaps_[k - 2]->Finish();
      }
    }

    const sensor::PointCloud cloud = SimulateScan(world_, true_pose, max_range_);
    testing::ExplicitInsertion insertion;
    insertion.node_id = NodeId{session_.session_index, index};
    insertion.node.time = TestTime(index);
    insertion.node.local_pose = local_pose;
    insertion.node.point_cloud = cloud;
    for (int k = std::max(0, newest - 1); k <= newest; ++k) {
      submaps_[k]->InsertScan(local_pose, cloud, inserter_);
      insertion.insertion_submaps.emplace_back(SubmapId{session_.session_index, k}, submaps_[k]);
    }
    testing::EnqueueExplicitInsertion(pose_graph_, std::move(insertion));
  }

  void FinishAll() {
    for (const auto& submap : submaps_) {
      if (!submap->finished()) {
        submap->Finish();
      }
    }
  }

 private:
  PoseGraph& pose_graph_;
  SessionId session_;
  World world_;
  double max_range_;
  mapping::CastRaysMapping inserter_;
  std::vector<std::shared_ptr<mapping::Submap>> submaps_;
};

template <typename GroundTruthAt>
double MeanNodeError(const PoseGraphData& graph, SessionId session, const GroundTruthAt& truth_at) {
  double total = 0.0;
  int count = 0;
  for (const NodeId& id : graph.session(session).node_ids) {
    total +=
        (graph.node(id).global_pose.translation() - truth_at(id.node_index).translation()).norm();
    ++count;
  }
  return count == 0 ? 0.0 : total / static_cast<double>(count);
}

std::vector<Constraint> InterConstraints(const PoseGraphData& graph) {
  std::vector<Constraint> inter;
  for (const Constraint& constraint : graph.constraints()) {
    if (constraint.type == Constraint::Type::INTER_SUBMAP) {
      inter.push_back(constraint);
    }
  }
  return inter;
}

// Zero false constraints: every accepted constraint is checked against the simulation truth.
template <typename SubmapTruthAt, typename NodeTruthAt>
void ExpectAllConstraintsTrue(const std::vector<Constraint>& inter,
                              const SubmapTruthAt& submap_truth, const NodeTruthAt& node_truth,
                              double translation_tolerance, double rotation_tolerance) {
  for (const Constraint& constraint : inter) {
    ASSERT_TRUE(constraint.to.has_value());
    const Eigen::Affine2d expected = Eigen::Affine2d(
        submap_truth(constraint.from.submap_id()).inverse() * node_truth(constraint.to->node_id()));
    EXPECT_LT((constraint.relative_pose.translation() - expected.translation()).norm(),
              translation_tolerance)
        << "false constraint: submap " << constraint.from.session_id << "." << constraint.from.index
        << " -> node " << constraint.to->session_id << "." << constraint.to->index;
    EXPECT_LT(std::abs(transform::NormalizeAngle(transform::GetYaw(constraint.relative_pose) -
                                                 transform::GetYaw(expected))),
              rotation_tolerance);
  }
}

std::vector<std::array<double, 6>> SnapshotSession(const PoseGraphData& graph, SessionId session) {
  std::vector<std::array<double, 6>> poses;
  const auto append = [&poses](const Eigen::Affine2d& pose) {
    poses.push_back({pose.linear()(0, 0), pose.linear()(0, 1), pose.linear()(1, 0),
                     pose.linear()(1, 1), pose.translation().x(), pose.translation().y()});
  };
  for (const SubmapId& id : graph.session(session).submap_ids) {
    append(graph.submap(id).global_pose);
  }
  for (const NodeId& id : graph.session(session).node_ids) {
    append(graph.node(id).global_pose);
  }
  return poses;
}

// Three pillars, so no pose in the room is rotationally or translationally ambiguous.
const World kLoopRoom{
    0.013,
    13.007,
    0.021,
    10.003,
    {Eigen::AlignedBox2d(Eigen::Vector2d(1.51, 1.52), Eigen::Vector2d(2.32, 2.33)),
     Eigen::AlignedBox2d(Eigen::Vector2d(10.61, 7.52), Eigen::Vector2d(11.42, 8.33)),
     Eigen::AlignedBox2d(Eigen::Vector2d(6.01, 4.62), Eigen::Vector2d(6.82, 5.43))}};

constexpr int kNumLoopNodes = 80;
constexpr double kLoopRadius = 3.1;

Eigen::Affine2d LoopGroundTruth(int index) {
  const double phi = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(kNumLoopNodes);
  return transform::FromXYTheta(6.4 + kLoopRadius * std::sin(phi),
                                1.9 + kLoopRadius * (1.0 - std::cos(phi)), phi);
}

// A constant yaw bias per step: the trajectory opens up and never comes back on its own.
std::vector<Eigen::Affine2d> DriftedLoopOdometry(double yaw_bias_per_step) {
  std::vector<Eigen::Affine2d> poses;
  poses.push_back(LoopGroundTruth(0));
  for (int i = 1; i < kNumLoopNodes; ++i) {
    const Eigen::Affine2d increment =
        Eigen::Affine2d(LoopGroundTruth(i - 1).inverse() * LoopGroundTruth(i));
    poses.push_back(Eigen::Affine2d(poses.back() * increment *
                                    transform::FromXYTheta(0.0, 0.0, yaw_bias_per_step)));
  }
  return poses;
}

TEST(ConstraintBuilderE2eTest, DriftedLoopClosesWithZeroFalseConstraints) {
  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  SessionFeeder feeder(pose_graph, session, kLoopRoom, 30.0);

  const std::vector<Eigen::Affine2d> odometry = DriftedLoopOdometry(0.004);
  for (int i = 0; i < kNumLoopNodes; ++i) {
    feeder.FeedNode(i, LoopGroundTruth(i), odometry[i]);
  }
  pose_graph.WaitUntilQuiescent();
  feeder.FinishAll();

  const PoseGraphData& graph = pose_graph.graph();
  const auto truth = [](int index) { return LoopGroundTruth(index); };
  const double error_before = MeanNodeError(graph, session, truth);
  const double last_error_before =
      (graph.node(NodeId{session.session_index, kNumLoopNodes - 1}).global_pose.translation() -
       LoopGroundTruth(kNumLoopNodes - 1).translation())
          .norm();
  EXPECT_GT(error_before, 0.3) << "the simulated drift has to be worth correcting";
  EXPECT_GT(last_error_before, 0.8);

  ConstraintBuilder builder(pose_graph);
  for (int i = kNumLoopNodes - 10; i < kNumLoopNodes; ++i) {
    builder.SearchForNode(NodeId{session.session_index, i});
  }
  builder.SearchForSubmap(SubmapId{session.session_index, 7});
  pose_graph.WaitUntilQuiescent();

  const std::vector<Constraint> inter = InterConstraints(graph);
  ASSERT_GT(builder.num_constraints_added(), 0);
  ASSERT_EQ(static_cast<int>(inter.size()), builder.num_constraints_added());

  bool found_long_loop = false;
  for (const Constraint& constraint : inter) {
    const int submap_node = constraint.from.index * kNodesPerSubmap;
    found_long_loop = found_long_loop || std::abs(constraint.to->index - submap_node) >= 40;
  }
  EXPECT_TRUE(found_long_loop) << "no constraint spans the loop";

  const auto submap_truth = [](const SubmapId& id) {
    return LoopGroundTruth(id.submap_index * kNodesPerSubmap);
  };
  const auto node_truth = [](const NodeId& id) { return LoopGroundTruth(id.node_index); };
  ExpectAllConstraintsTrue(inter, submap_truth, node_truth, 0.35, 0.1);

  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  const double error_after = MeanNodeError(graph, session, truth);
  const double last_error_after =
      (graph.node(NodeId{session.session_index, kNumLoopNodes - 1}).global_pose.translation() -
       LoopGroundTruth(kNumLoopNodes - 1).translation())
          .norm();
  EXPECT_LT(error_after, error_before / 2.0)
      << "before " << error_before << " after " << error_after;
  EXPECT_LT(last_error_after, 0.4);
}

// The cache is pure performance: the same run with it off must produce the same constraints.
TEST(ConstraintBuilderE2eTest, PrecomputationCacheDoesNotChangeTheConstraints) {
  const auto run = [](int cache_size) {
    PoseGraph pose_graph;
    const SessionId session = pose_graph.StartNewSession(TestTime(0));
    SessionFeeder feeder(pose_graph, session, kLoopRoom, 30.0);

    const std::vector<Eigen::Affine2d> odometry = DriftedLoopOdometry(0.004);
    for (int i = 0; i < kNumLoopNodes; ++i) {
      feeder.FeedNode(i, LoopGroundTruth(i), odometry[i]);
    }
    pose_graph.WaitUntilQuiescent();
    feeder.FinishAll();

    ConstraintBuilderOption option;
    option.precomputation_cache_size = cache_size;
    ConstraintBuilder builder(pose_graph, option);
    for (int i = kNumLoopNodes - 10; i < kNumLoopNodes; ++i) {
      builder.SearchForNode(NodeId{session.session_index, i});
    }
    builder.SearchForSubmap(SubmapId{session.session_index, 7});
    pose_graph.WaitUntilQuiescent();

    std::vector<double> flattened;
    for (const Constraint& constraint : InterConstraints(pose_graph.graph())) {
      flattened.push_back(static_cast<double>(constraint.from.index));
      flattened.push_back(static_cast<double>(constraint.to->index));
      flattened.push_back(constraint.relative_pose.translation().x());
      flattened.push_back(constraint.relative_pose.translation().y());
      flattened.push_back(transform::GetYaw(constraint.relative_pose));
    }
    return flattened;
  };

  const std::vector<double> uncached = run(0);
  ASSERT_FALSE(uncached.empty());
  EXPECT_EQ(uncached, run(32));
}

// With the far caps out of range the along-corridor coordinate is unobservable and the coarse
// matcher reports high scores metres from truth. None of those may become a constraint.
TEST(ConstraintBuilderE2eTest, SelfSimilarCorridorProducesHighCoarseScoresButNoConstraints) {
  const World corridor{0.017, 20.213, 3.011, 4.522, {}};
  constexpr int kNumNodes = 60;
  constexpr double kMaxRange = 6.0;
  const auto truth_at = [](int i) { return transform::FromXYTheta(7.1 + 0.09 * i, 3.77, 0.0); };
  // A 30 percent odometry scale error along the corridor: the classic aliasing setup.
  const auto local_at = [](int i) { return transform::FromXYTheta(0.117 * i, 0.0, 0.0); };

  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0), truth_at(0));
  SessionFeeder feeder(pose_graph, session, corridor, kMaxRange);
  for (int i = 0; i < kNumNodes; ++i) {
    feeder.FeedNode(i, truth_at(i), local_at(i));
  }
  pose_graph.WaitUntilQuiescent();
  feeder.FinishAll();

  const PoseGraphData& graph = pose_graph.graph();

  // Gate 1 alone would poison the graph here: the coarse matcher accepts a corridor match more
  // than a metre off along the axis.
  const SubmapRecord& target = graph.submap(SubmapId{session.session_index, 0});
  const Eigen::Affine2d& grid_to_global = target.global_pose;
  const Node& probe = graph.node(NodeId{session.session_index, 40});
  EXPECT_GT((probe.global_pose.translation() - truth_at(40).translation()).norm(), 0.8)
      << "the scenario has to put the prior well off the truth";
  const utils::scan_matching::FastCorrelativeScanMatcher coarse_only;
  utils::scan_matching::FastCorrelativeScanMatcher::MatchParams wide;
  wide.linear_search_window = 7.0;
  wide.min_score = 0.55;
  const std::optional<utils::scan_matching::GlobalMatchResult> coarse_result = coarse_only.Match(
      probe.constant_data.point_cloud,
      {utils::scan_matching::CandidateSubmap{grid_to_global, target.submap->Snapshot()}},
      probe.global_pose, wide);
  ASSERT_TRUE(coarse_result.has_value());
  EXPECT_GT(coarse_result->score, 0.55);

  ConstraintBuilder builder(pose_graph);
  for (int i = 30; i < kNumNodes; ++i) {
    builder.SearchForNode(NodeId{session.session_index, i});
  }
  pose_graph.WaitUntilQuiescent();

  EXPECT_GT(builder.num_matches_attempted(), 10);
  EXPECT_EQ(builder.num_constraints_added(), 0);
  EXPECT_TRUE(InterConstraints(graph).empty());
}

// Geometry that is not in the map: nothing may be accepted whatever the coarse stage makes of it.
TEST(ConstraintBuilderE2eTest, ScanFromDisjointGeometryIsRejected) {
  const World room_a{
      0.013,
      8.011,
      0.017,
      6.005,
      {Eigen::AlignedBox2d(Eigen::Vector2d(5.11, 1.52), Eigen::Vector2d(5.92, 2.33))}};
  const World room_b{0.021, 4.013, 0.019, 3.207, {}};

  PoseGraph pose_graph;
  const SessionId first = pose_graph.StartNewSession(TestTime(0));
  SessionFeeder first_feeder(pose_graph, first, room_a, 30.0);
  const auto first_pose_at = [](int i) {
    return transform::FromXYTheta(2.0 + 0.15 * i, 2.3 + 0.08 * i, 0.04 * i);
  };
  for (int i = 0; i < 20; ++i) {
    first_feeder.FeedNode(i, first_pose_at(i), first_pose_at(i));
  }
  pose_graph.WaitUntilQuiescent();
  first_feeder.FinishAll();

  // The second session lives in room B but its alignment claims room A, which is what a wrong
  // loop closure would need to be true.
  const SessionId second =
      pose_graph.StartNewSession(TestTime(100), transform::FromXYTheta(3.5, 3.0, 0.5));
  SessionFeeder second_feeder(pose_graph, second, room_b, 30.0);
  const auto second_local_at = [](int i) {
    return transform::FromXYTheta(0.05 * i, 0.03 * i, 0.02 * i);
  };
  const auto second_true_at = [&second_local_at](int i) {
    return Eigen::Affine2d(transform::FromXYTheta(2.0, 1.6, 0.0) * second_local_at(i));
  };
  for (int i = 0; i < 3; ++i) {
    second_feeder.FeedNode(i, second_true_at(i), second_local_at(i));
  }
  pose_graph.WaitUntilQuiescent();

  ConstraintBuilder builder(pose_graph);
  for (int i = 0; i < 3; ++i) {
    builder.SearchForNode(NodeId{second.session_index, i});
  }
  pose_graph.WaitUntilQuiescent();

  for (const Constraint& c : InterConstraints(pose_graph.graph())) {
    const Eigen::Affine2d node_in_submap = c.relative_pose;
    std::cout << "accepted: submap " << c.from.session_id << "." << c.from.index << " node "
              << c.to->session_id << "." << c.to->index << " rel ("
              << node_in_submap.translation().x() << ", " << node_in_submap.translation().y()
              << ", " << transform::GetYaw(node_in_submap) << ")\n";
  }
  EXPECT_GT(builder.num_matches_attempted(), 0);
  EXPECT_EQ(builder.num_constraints_added(), 0);
  EXPECT_TRUE(InterConstraints(pose_graph.graph()).empty());
}

// A misaligned new session gets pulled onto the frozen base by cross-session constraints, and
// the frozen base does not move.
TEST(ConstraintBuilderE2eTest, CrossSessionLoopsAnchorASecondSessionOntoAFrozenBase) {
  PoseGraph pose_graph;
  const SessionId first = pose_graph.StartNewSession(TestTime(0));
  SessionFeeder first_feeder(pose_graph, first, kLoopRoom, 30.0);
  constexpr int kFirstNodes = 40;
  for (int i = 0; i < kFirstNodes; ++i) {
    first_feeder.FeedNode(i, LoopGroundTruth(i), LoopGroundTruth(i));
  }
  pose_graph.WaitUntilQuiescent();
  first_feeder.FinishAll();
  pose_graph.FreezeSession(first);
  pose_graph.WaitUntilQuiescent();

  const PoseGraphData& graph = pose_graph.graph();
  const std::vector<std::array<double, 6>> frozen_before = SnapshotSession(graph, first);
  ASSERT_FALSE(frozen_before.empty());

  const Eigen::Affine2d misalignment = transform::FromXYTheta(0.45, -0.35, 0.06);
  const SessionId second = pose_graph.StartNewSession(
      TestTime(kFirstNodes), Eigen::Affine2d(LoopGroundTruth(0) * misalignment));
  SessionFeeder second_feeder(pose_graph, second, kLoopRoom, 30.0);
  constexpr int kSecondNodes = 20;
  const auto second_local_at = [](int i) {
    return Eigen::Affine2d(LoopGroundTruth(0).inverse() * LoopGroundTruth(i));
  };
  for (int i = 0; i < kSecondNodes; ++i) {
    second_feeder.FeedNode(i, LoopGroundTruth(i), second_local_at(i));
  }
  pose_graph.WaitUntilQuiescent();

  const auto truth = [](int index) { return LoopGroundTruth(index); };
  const double error_before = MeanNodeError(graph, second, truth);
  EXPECT_GT(error_before, 0.3);

  ConstraintBuilder builder(pose_graph);
  for (int i = 2; i < kSecondNodes; i += 3) {
    builder.SearchForNode(NodeId{second.session_index, i}, 2);
  }
  pose_graph.WaitUntilQuiescent();

  const std::vector<Constraint> inter = InterConstraints(graph);
  ASSERT_GE(static_cast<int>(inter.size()), 2);
  for (const Constraint& constraint : inter) {
    EXPECT_NE(constraint.from.session_id, constraint.to->session_id)
        << "every loop here has to be cross-session";
  }
  const auto submap_truth = [](const SubmapId& id) {
    return LoopGroundTruth(id.submap_index * kNodesPerSubmap);
  };
  const auto node_truth = [](const NodeId& id) { return LoopGroundTruth(id.node_index); };
  ExpectAllConstraintsTrue(inter, submap_truth, node_truth, 0.35, 0.1);

  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();

  const double error_after = MeanNodeError(graph, second, truth);
  EXPECT_LT(error_after, error_before / 2.0)
      << "before " << error_before << " after " << error_after;
  EXPECT_LT(error_after, 0.2);

  const std::vector<std::array<double, 6>> frozen_after = SnapshotSession(graph, first);
  ASSERT_EQ(frozen_before.size(), frozen_after.size());
  for (size_t i = 0; i < frozen_before.size(); ++i) {
    for (size_t j = 0; j < frozen_before[i].size(); ++j) {
      EXPECT_EQ(frozen_before[i][j], frozen_after[i][j])
          << "frozen pose " << i << " coefficient " << j << " moved";
    }
  }
}

}  // namespace
}  // namespace evergreenslam::lifelong

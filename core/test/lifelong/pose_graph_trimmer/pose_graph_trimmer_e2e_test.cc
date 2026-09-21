/**
 * @file pose_graph_trimmer_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Thirty loops over the same room: bounded counts with trimming, linear growth without,
 *        bounded accuracy loss, and a frozen base that never moves a bit.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "common/time.h"
#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_trimmer/pose_graph_trimmer.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;
constexpr int kNodesPerSubmap = 10;
constexpr int kNumBeams = 360;
constexpr int kNodesPerLoop = 40;
constexpr int kNumLoops = 30;
constexpr double kMaxRange = 30.0;
// Accuracy after trimming may cost at most 5 cm of mean last-loop error over the untrimmed
// baseline.
constexpr double kMaxDegradation = 0.05;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// Walls deliberately off the grid-resolution lattice.
struct World {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  std::vector<Eigen::AlignedBox2d> pillars;
};

const World kLoopRoom{
    0.013,
    13.007,
    0.021,
    10.003,
    {Eigen::AlignedBox2d(Eigen::Vector2d(1.51, 1.52), Eigen::Vector2d(2.32, 2.33)),
     Eigen::AlignedBox2d(Eigen::Vector2d(10.61, 7.52), Eigen::Vector2d(11.42, 8.33)),
     Eigen::AlignedBox2d(Eigen::Vector2d(6.01, 4.62), Eigen::Vector2d(6.82, 5.43))}};

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

sensor::PointCloud SimulateScan(const World& world, const Eigen::Affine2d& world_pose) {
  const double yaw = transform::GetYaw(world_pose);
  sensor::PointCloud cloud;
  for (int i = 0; i < kNumBeams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / kNumBeams;
    const double range = RangeToBoundary(world, world_pose.translation(), yaw + bearing);
    if (range > kMaxRange) {
      continue;
    }
    cloud.push_back(
        sensor::Point2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing))});
  }
  return cloud;
}

// The index is not taken modulo anything; the angles wrap.
Eigen::Affine2d LoopGroundTruth(int index) {
  const double phi = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(kNodesPerLoop);
  return transform::FromXYTheta(6.4 + 3.1 * std::sin(phi), 1.9 + 3.1 * (1.0 - std::cos(phi)), phi);
}

std::vector<Eigen::Affine2d> DriftedOdometry(int num_nodes, double yaw_bias_per_step) {
  std::vector<Eigen::Affine2d> poses;
  poses.push_back(LoopGroundTruth(0));
  for (int i = 1; i < num_nodes; ++i) {
    const Eigen::Affine2d increment =
        Eigen::Affine2d(LoopGroundTruth(i - 1).inverse() * LoopGroundTruth(i));
    poses.push_back(Eigen::Affine2d(poses.back() * increment *
                                    transform::FromXYTheta(0.0, 0.0, yaw_bias_per_step)));
  }
  return poses;
}

// Rasterizes what local mapping would have built, drift baked into the grids.
class SessionFeeder {
 public:
  SessionFeeder(PoseGraph& pose_graph, SessionId session, int time_offset)
      : pose_graph_(pose_graph), session_(session), time_offset_(time_offset) {}

  void FeedNode(int index, const Eigen::Affine2d& true_pose, const Eigen::Affine2d& local_pose) {
    const int newest = index / kNodesPerSubmap;
    while (static_cast<int>(submaps_.size()) <= newest) {
      const int k = static_cast<int>(submaps_.size());
      submaps_.push_back(std::make_shared<mapping::Submap>(k, local_pose, kResolution));
      if (k >= 2) {
        submaps_[k - 2]->Finish();
      }
    }

    const sensor::PointCloud cloud = SimulateScan(kLoopRoom, true_pose);
    testing::ExplicitInsertion insertion;
    insertion.node_id = NodeId{session_.session_index, index};
    insertion.node.time = TestTime(time_offset_ + index);
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

  int num_submaps_created() const { return static_cast<int>(submaps_.size()); }

 private:
  PoseGraph& pose_graph_;
  SessionId session_;
  int time_offset_;
  mapping::CastRaysMapping inserter_;
  std::vector<std::shared_ptr<mapping::Submap>> submaps_;
};

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

struct ScenarioResult {
  int submaps_created = 0;
  int submaps_remaining = 0;
  int nodes_remaining = 0;
  int constraints_final = 0;
  int constraints_mid_run = 0;
  double last_loop_error = 0.0;
};

// Session 1 maps a loop, closes it and freezes; session 2 re-drives the same circle with injected
// yaw drift, anchored by handcrafted-correct closures to keep this about the trimmer.
ScenarioResult RunScenario(bool with_trimming) {
  PoseGraphOption option;
  // The freeze here is manual and the story is the trimmer, so judging is held back.
  option.session_manager.auto_freeze = false;
  PoseGraph pose_graph(option);
  const PoseGraphData& graph = pose_graph.graph();

  const SessionId first = pose_graph.StartNewSession(TestTime(0));
  {
    SessionFeeder feeder(pose_graph, first, 0);
    const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(kNodesPerLoop, 0.001);
    for (int i = 0; i < kNodesPerLoop; ++i) {
      feeder.FeedNode(i, LoopGroundTruth(i), odometry[i]);
    }
    pose_graph.WaitUntilQuiescent();
    feeder.FinishAll();
    for (int i = kNodesPerLoop - 10; i < kNodesPerLoop; ++i) {
      Constraint constraint;
      constraint.type = Constraint::Type::INTER_SUBMAP;
      constraint.from = VariableId::Of(SubmapId{first.session_index, 0});
      constraint.to = VariableId::Of(NodeId{first.session_index, i});
      constraint.relative_pose = LoopGroundTruth(0).inverse() * LoopGroundTruth(i);
      constraint.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
      pose_graph.AddConstraint(constraint);
    }
    pose_graph.Optimize();
    pose_graph.FreezeSession(first);
    pose_graph.WaitUntilQuiescent();
  }
  const std::vector<std::array<double, 6>> frozen_snapshot = SnapshotSession(graph, first);

  const SessionId second = pose_graph.StartNewSession(TestTime(50));
  SessionFeeder feeder(pose_graph, second, 100);
  PoseGraphTrimmer trimmer(pose_graph);
  const int num_nodes = kNumLoops * kNodesPerLoop;
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(num_nodes, 0.002);

  ScenarioResult result;
  for (int i = 0; i < num_nodes; ++i) {
    feeder.FeedNode(i, LoopGroundTruth(i), odometry[i]);
    if (i % kNodesPerSubmap == 5) {
      // Built from the frozen pose actually in the graph, so the constraint is consistent with
      // the base as frozen, not with a truth nobody stored.
      const int frozen_index = ((i % kNodesPerLoop) / kNodesPerSubmap);
      const SubmapId anchor{first.session_index, frozen_index};
      Constraint constraint;
      constraint.type = Constraint::Type::INTER_SUBMAP;
      constraint.from = VariableId::Of(anchor);
      constraint.to = VariableId::Of(NodeId{second.session_index, i});
      constraint.relative_pose = graph.submap(anchor).global_pose.inverse() * LoopGroundTruth(i);
      constraint.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
      pose_graph.AddConstraint(constraint);
    }
    if ((i + 1) % kNodesPerLoop == 0) {
      pose_graph.Optimize();
      pose_graph.WaitUntilQuiescent();
      if (with_trimming) {
        // One selector refill plus two trim rounds keeps pace with the four submaps a loop
        // creates.
        for (int round = 0; round < 3; ++round) {
          trimmer.TrimOnce();
        }
        pose_graph.WaitUntilQuiescent();
      }
      if ((i + 1) == kNumLoops * kNodesPerLoop / 2) {
        result.constraints_mid_run = static_cast<int>(graph.constraints().size());
      }
    }
  }
  feeder.FinishAll();
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();

  // Trimming never cut the session loose from the frozen base: no gauge datum of its own.
  for (const SubmapId& id : graph.session(second).submap_ids) {
    EXPECT_FALSE(pose_graph.optimization().IsVariableConstant(VariableId::Of(id)));
  }
  for (const NodeId& id : graph.session(second).node_ids) {
    EXPECT_FALSE(pose_graph.optimization().IsVariableConstant(VariableId::Of(id)));
  }

  // The frozen base never moved a bit through any of it, and the trimmer never touched it.
  const std::vector<std::array<double, 6>> frozen_after = SnapshotSession(graph, first);
  EXPECT_EQ(frozen_snapshot.size(), frozen_after.size());
  for (size_t s = 0; s < frozen_snapshot.size(); ++s) {
    for (size_t c = 0; c < frozen_snapshot[s].size(); ++c) {
      EXPECT_EQ(frozen_snapshot[s][c], frozen_after[s][c]) << "frozen pose " << s;
    }
  }

  std::set<SubmapId> deleted_submaps;
  std::set<NodeId> deleted_nodes;
  for (const TrimReport& report : trimmer.reports()) {
    for (const TrimReport::SubmapSuccession& succession : report.submap_successions) {
      EXPECT_EQ(succession.deleted_submap_id.session_id, second.session_index)
          << "only the active session may be trimmed";
      EXPECT_TRUE(succession.successor_submap_id.has_value());
      EXPECT_TRUE(deleted_submaps.insert(succession.deleted_submap_id).second)
          << "a submap can only be deleted once";
      EXPECT_FALSE(graph.HasSubmap(succession.deleted_submap_id));
    }
    for (const NodeId& node_id : report.deleted_node_ids) {
      EXPECT_TRUE(deleted_nodes.insert(node_id).second);
      EXPECT_FALSE(graph.HasNode(node_id));
    }
  }
  EXPECT_EQ(feeder.num_submaps_created() - static_cast<int>(deleted_submaps.size()),
            static_cast<int>(graph.session(second).submap_ids.size()));
  EXPECT_EQ(num_nodes - static_cast<int>(deleted_nodes.size()),
            static_cast<int>(graph.session(second).node_ids.size()));

  result.submaps_created = feeder.num_submaps_created();
  result.submaps_remaining = static_cast<int>(graph.session(second).submap_ids.size());
  result.nodes_remaining = static_cast<int>(graph.session(second).node_ids.size());
  result.constraints_final = static_cast<int>(graph.constraints().size());
  double error_sum = 0.0;
  for (int i = num_nodes - kNodesPerLoop; i < num_nodes; ++i) {
    const NodeId id{second.session_index, i};
    EXPECT_TRUE(graph.HasNode(id)) << "recent nodes must survive trimming";
    error_sum +=
        (graph.node(id).global_pose.translation() - LoopGroundTruth(i).translation()).norm();
  }
  result.last_loop_error = error_sum / kNodesPerLoop;
  return result;
}

TEST(PoseGraphTrimmerE2eTest, ThirtyLoopsStayBoundedAndBarelyLoseAccuracy) {
  const ScenarioResult baseline = RunScenario(false);
  const ScenarioResult trimmed = RunScenario(true);

  printf("METRICS baseline: created=%d remaining=%d nodes=%d constraints=%d mid=%d err=%.4f\n",
         baseline.submaps_created, baseline.submaps_remaining, baseline.nodes_remaining,
         baseline.constraints_final, baseline.constraints_mid_run, baseline.last_loop_error);
  printf("METRICS trimmed:  created=%d remaining=%d nodes=%d constraints=%d mid=%d err=%.4f\n",
         trimmed.submaps_created, trimmed.submaps_remaining, trimmed.nodes_remaining,
         trimmed.constraints_final, trimmed.constraints_mid_run, trimmed.last_loop_error);

  // Space, not time: the untrimmed graph grows linearly with the loop count, the trimmed one
  // flattens out.
  EXPECT_EQ(baseline.submaps_created, kNumLoops * kNodesPerLoop / kNodesPerSubmap);
  EXPECT_EQ(baseline.submaps_remaining, baseline.submaps_created);
  EXPECT_LE(trimmed.submaps_remaining, 20);
  EXPECT_GE(trimmed.submaps_remaining, PoseGraphTrimmerOption().selector.min_surviving_submaps)
      << "the surviving-submap floor has to hold";
  EXPECT_LT(trimmed.submaps_remaining, baseline.submaps_remaining / 4);
  EXPECT_LT(trimmed.nodes_remaining, baseline.nodes_remaining / 4);

  // The accuracy cost of trimming stays under the threshold.
  EXPECT_LT(baseline.last_loop_error, 0.10) << "the baseline itself has to be healthy";
  EXPECT_LT(trimmed.last_loop_error, baseline.last_loop_error + kMaxDegradation)
      << "baseline " << baseline.last_loop_error << " trimmed " << trimmed.last_loop_error;
  EXPECT_LT(trimmed.last_loop_error, 0.15);

  // The Chow-Liu tree keeps the constraint count from densifying.
  EXPECT_LT(trimmed.constraints_final, trimmed.constraints_mid_run * 13 / 10);
  EXPECT_LT(trimmed.constraints_final, baseline.constraints_final / 2);
}

}  // namespace
}  // namespace evergreenslam::lifelong

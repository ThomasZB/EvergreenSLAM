/**
 * @file pose_graph_trimmer_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief One trim under a microscope: report equals the graph diff, Problem stays in lockstep.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/pose_graph_trimmer.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
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
constexpr int kNodesPerSubmap = 3;
constexpr int kNumSubmaps = 8;
constexpr int kNumNodes = kNodesPerSubmap * kNumSubmaps;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

sensor::PointCloud RingScan(const Eigen::Vector2d& center, double radius) {
  sensor::PointCloud cloud;
  for (int i = 0; i < 72; ++i) {
    const double angle = 2.0 * M_PI * i / 72.0;
    cloud.push_back(
        sensor::Point2d{center + radius * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  return cloud;
}

Eigen::Affine2d NodePose(int index) {
  return transform::FromXYTheta(0.4 * index + 0.013, 0.257, 0.0);
}

// With `overlap`, each node goes into the newest submap and the one before it, so submaps chain
// through shared nodes and a trimmed submap orphans a node only once its second home is gone too.
std::vector<std::shared_ptr<mapping::Submap>> FeedPath(PoseGraph& pose_graph, SessionId session,
                                                       const std::vector<Eigen::Affine2d>& poses,
                                                       int time_offset, bool overlap) {
  std::vector<std::shared_ptr<mapping::Submap>> submaps;
  mapping::CastRaysMapping inserter;
  for (size_t i = 0; i < poses.size(); ++i) {
    const int newest = static_cast<int>(i) / kNodesPerSubmap;
    const Eigen::Affine2d& pose = poses[i];
    if (static_cast<int>(submaps.size()) <= newest) {
      submaps.push_back(std::make_shared<mapping::Submap>(newest, pose, kResolution));
    }

    testing::ExplicitInsertion insertion;
    insertion.node_id = NodeId{session.session_index, static_cast<int>(i)};
    insertion.node.time = TestTime(time_offset + static_cast<int>(i));
    insertion.node.local_pose = pose;
    for (int k = overlap ? std::max(0, newest - 1) : newest; k <= newest; ++k) {
      submaps[k]->InsertScan(pose, RingScan(Eigen::Vector2d::Zero(), 2.013), inserter);
      insertion.insertion_submaps.emplace_back(SubmapId{session.session_index, k}, submaps[k]);
    }
    testing::EnqueueExplicitInsertion(pose_graph, std::move(insertion));
  }
  pose_graph.WaitUntilQuiescent();
  for (const auto& submap : submaps) {
    submap->Finish();
  }
  return submaps;
}

std::vector<std::shared_ptr<mapping::Submap>> FeedCorridor(PoseGraph& pose_graph, SessionId session,
                                                           int time_offset) {
  std::vector<Eigen::Affine2d> poses;
  for (int i = 0; i < kNumNodes; ++i) {
    poses.push_back(NodePose(i));
  }
  return FeedPath(pose_graph, session, poses, time_offset, /*overlap=*/true);
}

// A loop-closure-weight edge consistent with the graph's current global poses.
void AddInterConstraint(PoseGraph& pose_graph, const VariableId& from, const VariableId& to) {
  const PoseGraphData& graph = pose_graph.graph();
  const auto global_pose = [&graph](const VariableId& variable) {
    return variable.kind == VariableId::Kind::SUBMAP
               ? graph.submap(variable.submap_id()).global_pose
               : graph.node(variable.node_id()).global_pose;
  };
  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.from = from;
  constraint.to = to;
  constraint.relative_pose = global_pose(from).inverse() * global_pose(to);
  constraint.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
  pose_graph.AddConstraint(constraint);
}

struct TestGraph {
  PoseGraph pose_graph;
  SessionId session;
  std::vector<std::shared_ptr<mapping::Submap>> submaps;

  TestGraph() {
    session = pose_graph.StartNewSession(TestTime(0));
    submaps = FeedCorridor(pose_graph, session, 0);
    for (const auto& [submap_index, node_index] :
         std::vector<std::pair<int, int>>{{0, 8}, {3, 2}}) {
      Constraint constraint;
      constraint.type = Constraint::Type::INTER_SUBMAP;
      constraint.from = VariableId::Of(SubmapId{session.session_index, submap_index});
      constraint.to = VariableId::Of(NodeId{session.session_index, node_index});
      constraint.relative_pose =
          submaps[submap_index]->local_pose().inverse() * NodePose(node_index);
      constraint.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
      pose_graph.AddConstraint(constraint);
    }
    pose_graph.Optimize();
    pose_graph.WaitUntilQuiescent();
  }
};

bool AnyVariableConstant(const PoseGraphData& graph, const Optimization& optimization,
                         SessionId session) {
  for (const SubmapId& id : graph.session(session).submap_ids) {
    if (optimization.IsVariableConstant(VariableId::Of(id))) {
      return true;
    }
  }
  for (const NodeId& id : graph.session(session).node_ids) {
    if (optimization.IsVariableConstant(VariableId::Of(id))) {
      return true;
    }
  }
  return false;
}

// Linked, the second session hangs off frozen submap 2 by one edge and carries no datum;
// unlinked, it floats on a datum of its own.
SessionId OpenSecondSession(TestGraph& test, bool linked) {
  PoseGraph& pose_graph = test.pose_graph;
  const PoseGraphData& graph = pose_graph.graph();
  pose_graph.FreezeSession(test.session);
  pose_graph.WaitUntilQuiescent();
  const SessionId second =
      pose_graph.StartNewSession(TestTime(100), transform::FromXYTheta(0.0, 5.0, 0.0));
  FeedCorridor(pose_graph, second, 100);
  if (linked) {
    const SubmapId frozen{test.session.session_index, 2};
    const SubmapId link{second.session_index, 0};
    Constraint anchor;
    anchor.type = Constraint::Type::INTER_SUBMAP;
    anchor.from = VariableId::Of(frozen);
    anchor.to = VariableId::Of(link);
    anchor.relative_pose =
        graph.submap(frozen).global_pose.inverse() * graph.submap(link).global_pose;
    anchor.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
    pose_graph.AddConstraint(anchor);
  }
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(AnyVariableConstant(graph, pose_graph.mutable_optimization(), second), !linked);
  return second;
}

Eigen::Matrix3d RecoveredCovariance(const Constraint& constraint) {
  return (constraint.sqrt_information.transpose() * constraint.sqrt_information).inverse();
}

bool Touches(const Constraint& constraint, const std::set<VariableId>& variables) {
  return variables.count(constraint.from) > 0 ||
         (constraint.to.has_value() && variables.count(*constraint.to) > 0);
}

bool SameEndpoints(const Constraint& a, const Constraint& b) {
  return a.type == b.type && a.from == b.from && a.to.has_value() == b.to.has_value() &&
         (!a.to.has_value() || *a.to == *b.to);
}

// Trims `target` and asserts the report is exactly the graph diff. Returns the report.
TrimReport TrimAndVerify(TestGraph& test, PoseGraphTrimmer& trimmer, const SubmapId& target) {
  PoseGraph& pose_graph = test.pose_graph;
  const PoseGraphData& graph = pose_graph.graph();
  Optimization& optimization = pose_graph.mutable_optimization();

  std::set<VariableId> expected_removed = {VariableId::Of(target)};
  std::set<NodeId> expected_orphans;
  for (const NodeId& node_id : graph.submap(target).node_ids) {
    const std::vector<SubmapId> containing = graph.ContainingSubmapIds(node_id);
    if (containing.size() == 1 && containing.front() == target) {
      expected_orphans.insert(node_id);
      expected_removed.insert(VariableId::Of(node_id));
    }
  }

  std::set<VariableId> expected_blanket;
  std::set<VariableId> expected_anchors;
  bool expected_prior = false;
  for (const Constraint& constraint : graph.constraints()) {
    const bool from_removed = expected_removed.count(constraint.from) > 0;
    if (!constraint.to.has_value()) {
      expected_prior = expected_prior || from_removed;
      continue;
    }
    const bool to_removed = expected_removed.count(*constraint.to) > 0;
    if (from_removed == to_removed) {
      continue;
    }
    const VariableId& survivor = from_removed ? *constraint.to : constraint.from;
    if (optimization.IsVariableConstant(survivor)) {
      expected_anchors.insert(survivor);
    } else {
      expected_blanket.insert(survivor);
    }
  }
  EXPECT_GE(expected_blanket.size(), 2u);
  const size_t expected_added = expected_blanket.size() - 1 + (expected_anchors.empty() ? 0u : 1u) +
                                (expected_prior ? 1u : 0u);

  const std::vector<Constraint> constraints_before = graph.constraints();
  const int touching_before =
      static_cast<int>(std::count_if(constraints_before.begin(), constraints_before.end(),
                                     [&expected_removed](const Constraint& constraint) {
                                       return Touches(constraint, expected_removed);
                                     }));
  const Eigen::Affine2d deleted_global = graph.submap(target).global_pose;
  const int variables_before = optimization.num_variables();
  const size_t nodes_before = graph.nodes().size();
  const size_t submaps_before = graph.submaps().size();
  const size_t reports_before = trimmer.reports().size();

  trimmer.EnqueueForTrim(target);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();

  EXPECT_EQ(trimmer.reports().size(), reports_before + 1);
  const TrimReport& report = trimmer.reports().back();

  EXPECT_FALSE(graph.HasSubmap(target));
  const std::set<NodeId> reported_nodes(report.deleted_node_ids.begin(),
                                        report.deleted_node_ids.end());
  EXPECT_EQ(reported_nodes, expected_orphans);
  for (const NodeId& id : report.deleted_node_ids) {
    EXPECT_FALSE(graph.HasNode(id));
  }
  EXPECT_EQ(graph.nodes().size(), nodes_before - expected_orphans.size());
  EXPECT_EQ(graph.submaps().size(), submaps_before - 1);

  // The succession names a survivor and records the transform taken before deletion.
  EXPECT_EQ(report.submap_successions.size(), 1u);
  const TrimReport::SubmapSuccession& succession = report.submap_successions.front();
  EXPECT_EQ(succession.deleted_submap_id, target);
  EXPECT_TRUE(succession.successor_submap_id.has_value());
  if (succession.successor_submap_id.has_value()) {
    EXPECT_TRUE(graph.HasSubmap(*succession.successor_submap_id));
    const Eigen::Affine2d expected_succession =
        graph.submap(*succession.successor_submap_id).global_pose.inverse() * deleted_global;
    EXPECT_LT((transform::ToVector3(succession.successor_from_deleted) -
               transform::ToVector3(expected_succession))
                  .norm(),
              1e-9);
  }

  // Everything touching the removed set went, and the report lists exactly the added edges.
  EXPECT_EQ(report.added_constraints.size(), expected_added);
  const std::vector<Constraint>& constraints_after = graph.constraints();
  EXPECT_EQ(constraints_after.size(),
            constraints_before.size() - touching_before + report.added_constraints.size());
  for (const Constraint& constraint : constraints_after) {
    EXPECT_FALSE(Touches(constraint, expected_removed));
  }
  for (const Constraint& added : report.added_constraints) {
    EXPECT_TRUE(expected_blanket.count(added.from) > 0 || expected_anchors.count(added.from) > 0);
    if (added.to.has_value()) {
      EXPECT_TRUE(expected_blanket.count(*added.to) > 0);
    }
    EXPECT_TRUE(std::any_of(constraints_after.begin(), constraints_after.end(),
                            [&added](const Constraint& c) { return SameEndpoints(c, added); }));
  }

  EXPECT_EQ(optimization.num_variables(),
            variables_before - 1 - static_cast<int>(expected_orphans.size()));
  EXPECT_EQ(optimization.num_variables(),
            static_cast<int>(graph.nodes().size() + graph.submaps().size()));
  return report;
}

TEST(PoseGraphTrimmerTest, ReportMatchesTheGraphDiffAndTheProblemStaysInSync) {
  TestGraph test;
  PoseGraph& pose_graph = test.pose_graph;
  PoseGraphTrimmer trimmer(pose_graph);

  // Submap 1 shares every node with a neighbour, so this trim orphans nothing.
  const TrimReport first = TrimAndVerify(test, trimmer, SubmapId{test.session.session_index, 1});
  EXPECT_TRUE(first.deleted_node_ids.empty());
  for (const Constraint& added : first.added_constraints) {
    EXPECT_EQ(added.type, Constraint::Type::INTER_SUBMAP);
  }

  // With submap 1 gone, submap 2 is the last home of nodes 6..8: this trim deletes them, and
  // with node 8 goes the closure to submap 0, the gauge datum, so a survivor is re-attached.
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  const SubmapId datum{test.session.session_index, 0};
  ASSERT_TRUE(pose_graph.mutable_optimization().IsVariableConstant(VariableId::Of(datum)));
  const TrimReport second = TrimAndVerify(test, trimmer, SubmapId{test.session.session_index, 2});
  EXPECT_EQ(second.deleted_node_ids.size(), 3u);
  EXPECT_EQ(trimmer.num_submaps_trimmed(), 2);
  EXPECT_EQ(std::count_if(second.added_constraints.begin(), second.added_constraints.end(),
                          [&datum](const Constraint& c) {
                            return c.from == VariableId::Of(datum) && c.recovered;
                          }),
            1);

  // The Problem still solves and accepts new work; a desynced index would CHECK here.
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  Constraint late;
  late.type = Constraint::Type::INTER_SUBMAP;
  late.from = VariableId::Of(SubmapId{test.session.session_index, 3});
  late.to = VariableId::Of(NodeId{test.session.session_index, 20});
  late.relative_pose = test.submaps[3]->local_pose().inverse() * NodePose(20);
  late.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
  pose_graph.AddConstraint(late);
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
}

TEST(PoseGraphTrimmerTest, TheGaugeDatumIsSkippedNotTrimmed) {
  TestGraph test;
  PoseGraph& pose_graph = test.pose_graph;
  const SubmapId datum{test.session.session_index, 0};
  ASSERT_TRUE(pose_graph.mutable_optimization().IsVariableConstant(VariableId::Of(datum)))
      << "the first submap should carry the component's gauge datum after a solve";

  PoseGraphTrimmer trimmer(pose_graph);
  trimmer.EnqueueForTrim(datum);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();

  EXPECT_TRUE(trimmer.reports().empty());
  EXPECT_EQ(trimmer.num_submaps_trimmed(), 0);
  EXPECT_TRUE(pose_graph.graph().HasSubmap(datum));
}

TEST(PoseGraphTrimmerTest, LongEdgesBecomePriorsOnlyWhenTheMarginalIsFreezeGrade) {
  // Gate wide open and every edge "long": all recovered constraints become absolute priors.
  {
    TestGraph test;
    const SessionId second = OpenSecondSession(test, /*linked=*/true);
    PoseGraphTrimmerOption option;
    option.max_binary_constraint_length = 0.0;
    option.prior_max_translation_stddev = 1e6;
    option.prior_max_rotation_stddev = 1e6;
    PoseGraphTrimmer trimmer(test.pose_graph, option);
    trimmer.EnqueueForTrim(SubmapId{second.session_index, 1});
    trimmer.TrimOnce();
    test.pose_graph.WaitUntilQuiescent();

    ASSERT_EQ(trimmer.reports().size(), 1u);
    ASSERT_FALSE(trimmer.reports().front().added_constraints.empty());
    EXPECT_GT(trimmer.num_priors_added(), 0);
    for (const Constraint& constraint : trimmer.reports().front().added_constraints) {
      EXPECT_EQ(constraint.type, Constraint::Type::PRIOR);
      EXPECT_FALSE(constraint.to.has_value());
    }
    test.pose_graph.Optimize();
    test.pose_graph.WaitUntilQuiescent();
  }
  // Gate shut: the long edges stay binary; an absolute prior on a poorly-constrained variable is
  // what the caveat forbids.
  {
    TestGraph test;
    const SessionId second = OpenSecondSession(test, /*linked=*/true);
    PoseGraphTrimmerOption option;
    option.max_binary_constraint_length = 0.0;
    option.prior_max_translation_stddev = 1e-12;
    option.prior_max_rotation_stddev = 1e-12;
    PoseGraphTrimmer trimmer(test.pose_graph, option);
    trimmer.EnqueueForTrim(SubmapId{second.session_index, 1});
    trimmer.TrimOnce();
    test.pose_graph.WaitUntilQuiescent();

    ASSERT_EQ(trimmer.reports().size(), 1u);
    ASSERT_FALSE(trimmer.reports().front().added_constraints.empty());
    EXPECT_EQ(trimmer.num_priors_added(), 0);
    for (const Constraint& constraint : trimmer.reports().front().added_constraints) {
      EXPECT_EQ(constraint.type, Constraint::Type::INTER_SUBMAP);
      EXPECT_TRUE(constraint.to.has_value());
    }
  }
}

TEST(PoseGraphTrimmerTest, DeletingTheOnlyLinkToAFrozenSubmapReanchorsASurvivor) {
  TestGraph test;
  PoseGraph& pose_graph = test.pose_graph;
  const PoseGraphData& graph = pose_graph.graph();
  Optimization& optimization = pose_graph.mutable_optimization();
  pose_graph.FreezeSession(test.session);
  pose_graph.WaitUntilQuiescent();

  const SessionId second =
      pose_graph.StartNewSession(TestTime(100), transform::FromXYTheta(0.0, 5.0, 0.0));
  FeedCorridor(pose_graph, second, 100);
  const SubmapId frozen{test.session.session_index, 2};
  const SubmapId link{second.session_index, 1};
  Constraint anchor;
  anchor.type = Constraint::Type::INTER_SUBMAP;
  anchor.from = VariableId::Of(frozen);
  anchor.to = VariableId::Of(link);
  anchor.relative_pose =
      graph.submap(frozen).global_pose.inverse() * graph.submap(link).global_pose;
  anchor.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
  pose_graph.AddConstraint(anchor);
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  ASSERT_FALSE(AnyVariableConstant(graph, optimization, second)) << "anchored: no own datum";

  PoseGraphTrimmer trimmer(pose_graph);
  trimmer.EnqueueForTrim(link);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(trimmer.reports().size(), 1u);
  EXPECT_FALSE(graph.HasSubmap(link));

  std::vector<Constraint> reanchored;
  for (const Constraint& added : trimmer.reports().front().added_constraints) {
    if (added.from == VariableId::Of(frozen)) {
      reanchored.push_back(added);
    }
  }
  ASSERT_EQ(reanchored.size(), 1u);
  const Constraint& edge = reanchored.front();
  EXPECT_EQ(edge.type, Constraint::Type::INTER_SUBMAP);
  EXPECT_TRUE(edge.recovered);
  ASSERT_TRUE(edge.to.has_value());
  EXPECT_EQ(edge.to->session(), second);
  EXPECT_FALSE(optimization.IsVariableConstant(*edge.to));
  const Eigen::Affine2d expected_relative =
      graph.submap(frozen).global_pose.inverse() * optimization.GetVariablePose(*edge.to);
  EXPECT_LT(
      (transform::ToVector3(edge.relative_pose) - transform::ToVector3(expected_relative)).norm(),
      1e-9);
  const Eigen::Matrix3d covariance = RecoveredCovariance(edge);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_GE(covariance(axis, axis), 1e-4 - 1e-12);
  }

  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  EXPECT_FALSE(AnyVariableConstant(graph, optimization, second))
      << "the session floated off the frozen frame and was handed its own gauge datum";
}

// A trimmed head must hand its frozen tie, and the prior it carried, to the chain's new head,
// even when a revisit node of the same session sits nearer.
TEST(PoseGraphTrimmerTest, TrimmingTheHeadHandsItsFrozenTieAndPriorToTheChainsEarliestNode) {
  TestGraph test;
  PoseGraph& pose_graph = test.pose_graph;
  const PoseGraphData& graph = pose_graph.graph();
  Optimization& optimization = pose_graph.mutable_optimization();
  pose_graph.FreezeSession(test.session);
  pose_graph.WaitUntilQuiescent();

  // Out along one line and back along a parallel one: the last node ends half a metre from the
  // head, the head's chain successor 1.2 m away.
  std::vector<Eigen::Affine2d> poses;
  for (int i = 0; i < kNumNodes; ++i) {
    poses.push_back(i < kNumNodes / 2
                        ? transform::FromXYTheta(0.4 * i + 0.013, 5.257, 0.0)
                        : transform::FromXYTheta(0.4 * (kNumNodes - 1 - i) + 0.013, 5.757, M_PI));
  }
  const SessionId second = pose_graph.StartNewSession(TestTime(100));
  FeedPath(pose_graph, second, poses, 100, /*overlap=*/true);
  const SubmapId frozen{test.session.session_index, 2};
  const SubmapId head{second.session_index, 0};
  const NodeId successor{second.session_index, kNodesPerSubmap};
  const NodeId revisit{second.session_index, kNumNodes - 1};
  AddInterConstraint(pose_graph, VariableId::Of(frozen), VariableId::Of(head));
  AddInterConstraint(pose_graph, VariableId::Of(head), VariableId::Of(revisit));
  Constraint prior;
  prior.type = Constraint::Type::PRIOR;
  prior.from = VariableId::Of(head);
  prior.to = std::nullopt;
  prior.relative_pose = graph.submap(head).global_pose;
  prior.sqrt_information = Eigen::Matrix3d::Identity() * 1e3;
  pose_graph.AddConstraint(prior);
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  const Eigen::Vector2d head_position = graph.submap(head).global_pose.translation();
  ASSERT_LT((graph.node(revisit).global_pose.translation() - head_position).norm(),
            (graph.node(successor).global_pose.translation() - head_position).norm());

  PoseGraphTrimmer trimmer(pose_graph);
  trimmer.EnqueueForTrim(head);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(trimmer.reports().size(), 1u);
  EXPECT_FALSE(graph.HasSubmap(head));

  std::vector<Constraint> anchors;
  std::vector<Constraint> priors;
  for (const Constraint& added : trimmer.reports().front().added_constraints) {
    if (added.from == VariableId::Of(frozen)) {
      anchors.push_back(added);
    } else if (added.type == Constraint::Type::PRIOR) {
      priors.push_back(added);
    }
  }
  ASSERT_EQ(anchors.size(), 1u);
  ASSERT_TRUE(anchors.front().to.has_value());
  EXPECT_EQ(*anchors.front().to, VariableId::Of(successor));
  ASSERT_EQ(priors.size(), 1u);
  EXPECT_EQ(priors.front().from, VariableId::Of(successor));
  EXPECT_FALSE(optimization.IsVariableConstant(VariableId::Of(successor)));
}

// With no node of its own session in the blanket the tie goes to the nearest survivor: here a
// later submap that sits closer than the next one.
TEST(PoseGraphTrimmerTest, WithoutASameSessionNodeSurvivorTheFrozenTieGoesToTheNearest) {
  TestGraph test;
  PoseGraph& pose_graph = test.pose_graph;
  const PoseGraphData& graph = pose_graph.graph();
  pose_graph.FreezeSession(test.session);
  pose_graph.WaitUntilQuiescent();

  // Three submaps sharing no node: submap 1 starts 4 m from the head, submap 2 only 1.2 m.
  std::vector<Eigen::Affine2d> poses;
  for (int i = 0; i < 3 * kNodesPerSubmap; ++i) {
    const double start = i < kNodesPerSubmap ? 0.013 : (i < 2 * kNodesPerSubmap ? 4.013 : 1.213);
    poses.push_back(transform::FromXYTheta(start + 0.4 * (i % kNodesPerSubmap), 5.257, 0.0));
  }
  const SessionId second = pose_graph.StartNewSession(TestTime(100));
  FeedPath(pose_graph, second, poses, 100, /*overlap=*/false);
  const SubmapId frozen{test.session.session_index, 2};
  const SubmapId head{second.session_index, 0};
  const SubmapId far{second.session_index, 1};
  const SubmapId near{second.session_index, 2};
  AddInterConstraint(pose_graph, VariableId::Of(frozen), VariableId::Of(head));
  AddInterConstraint(pose_graph, VariableId::Of(head), VariableId::Of(far));
  AddInterConstraint(pose_graph, VariableId::Of(head), VariableId::Of(near));
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();

  PoseGraphTrimmer trimmer(pose_graph);
  trimmer.EnqueueForTrim(head);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(trimmer.reports().size(), 1u);
  EXPECT_FALSE(graph.HasSubmap(head));
  EXPECT_EQ(trimmer.reports().front().deleted_node_ids.size(), 3u) << "no node was shared";

  std::vector<Constraint> anchors;
  for (const Constraint& added : trimmer.reports().front().added_constraints) {
    if (added.from == VariableId::Of(frozen)) {
      anchors.push_back(added);
    }
  }
  ASSERT_EQ(anchors.size(), 1u);
  ASSERT_TRUE(anchors.front().to.has_value());
  EXPECT_EQ(*anchors.front().to, VariableId::Of(near));
}

TEST(PoseGraphTrimmerTest, DeletingASubmapWithAPriorMovesTheFlooredPriorToASurvivor) {
  TestGraph test;
  PoseGraph& pose_graph = test.pose_graph;
  const PoseGraphData& graph = pose_graph.graph();
  Optimization& optimization = pose_graph.mutable_optimization();
  const SessionId second = OpenSecondSession(test, /*linked=*/false);
  const SubmapId target{second.session_index, 1};

  Constraint prior;
  prior.type = Constraint::Type::PRIOR;
  prior.from = VariableId::Of(target);
  prior.to = std::nullopt;
  prior.relative_pose = graph.submap(target).global_pose;
  prior.sqrt_information = Eigen::Matrix3d::Identity() * 1e3;
  pose_graph.AddConstraint(prior);
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  ASSERT_FALSE(AnyVariableConstant(graph, optimization, second))
      << "a prior anchors the component, so no gauge datum";

  PoseGraphTrimmerOption option;
  option.min_recovered_stddev = 0.05;
  PoseGraphTrimmer trimmer(pose_graph, option);
  trimmer.EnqueueForTrim(target);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(trimmer.reports().size(), 1u);
  EXPECT_EQ(trimmer.num_priors_added(), 1);

  std::vector<Constraint> priors;
  for (const Constraint& added : trimmer.reports().front().added_constraints) {
    if (added.type == Constraint::Type::PRIOR) {
      priors.push_back(added);
    }
  }
  ASSERT_EQ(priors.size(), 1u);
  const Constraint& moved = priors.front();
  EXPECT_FALSE(moved.to.has_value());
  EXPECT_EQ(moved.from.kind, VariableId::Kind::NODE);
  EXPECT_TRUE(graph.HasVariable(moved.from));
  EXPECT_LT((transform::ToVector3(moved.relative_pose) -
             transform::ToVector3(optimization.GetVariablePose(moved.from)))
                .norm(),
            1e-9);
  const Eigen::Matrix3d covariance = RecoveredCovariance(moved);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_GE(covariance(axis, axis), 0.05 * 0.05 - 1e-12);
  }
  for (const Constraint& constraint : graph.constraints()) {
    EXPECT_FALSE(constraint.from == VariableId::Of(target));
  }

  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  EXPECT_FALSE(AnyVariableConstant(graph, optimization, second))
      << "the prior was dropped and the component fell back to a gauge datum";
}

TEST(PoseGraphTrimmerTest, LongEdgePriorsNeverClaimMoreCertaintyThanTheFloor) {
  TestGraph test;
  const SessionId second = OpenSecondSession(test, /*linked=*/true);
  PoseGraphTrimmerOption option;
  option.max_binary_constraint_length = 0.0;
  option.prior_max_translation_stddev = 1e6;
  option.prior_max_rotation_stddev = 1e6;
  option.min_recovered_stddev = 0.05;
  PoseGraphTrimmer trimmer(test.pose_graph, option);
  trimmer.EnqueueForTrim(SubmapId{second.session_index, 1});
  trimmer.TrimOnce();
  test.pose_graph.WaitUntilQuiescent();

  ASSERT_EQ(trimmer.reports().size(), 1u);
  ASSERT_GT(trimmer.num_priors_added(), 0);
  for (const Constraint& constraint : trimmer.reports().front().added_constraints) {
    ASSERT_EQ(constraint.type, Constraint::Type::PRIOR);
    const Eigen::Matrix3d covariance = RecoveredCovariance(constraint);
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_GE(covariance(axis, axis), 0.05 * 0.05 - 1e-12);
    }
  }
}

// A marginal measured against the session's own datum is tiny whatever the session's true
// uncertainty is; a prior built from it would look frozen-grade and the session could never be
// placed again.
TEST(PoseGraphTrimmerTest, NoPriorIsPlantedInASessionHoldingItsOwnDatum) {
  PoseGraphTrimmerOption option;
  option.max_binary_constraint_length = 0.0;
  option.prior_max_translation_stddev = 1e6;
  option.prior_max_rotation_stddev = 1e6;

  const auto expect_binary_only = [](const PoseGraphTrimmer& trimmer) {
    ASSERT_EQ(trimmer.reports().size(), 1u);
    ASSERT_FALSE(trimmer.reports().front().added_constraints.empty());
    EXPECT_EQ(trimmer.num_priors_added(), 0);
    for (const Constraint& constraint : trimmer.reports().front().added_constraints) {
      EXPECT_EQ(constraint.type, Constraint::Type::INTER_SUBMAP);
      EXPECT_TRUE(constraint.to.has_value());
    }
  };

  {
    TestGraph test;
    PoseGraph& pose_graph = test.pose_graph;
    ASSERT_TRUE(
        AnyVariableConstant(pose_graph.graph(), pose_graph.mutable_optimization(), test.session));
    PoseGraphTrimmer trimmer(pose_graph, option);
    trimmer.EnqueueForTrim(SubmapId{test.session.session_index, 1});
    trimmer.TrimOnce();
    pose_graph.WaitUntilQuiescent();
    expect_binary_only(trimmer);
    pose_graph.Optimize();
    pose_graph.WaitUntilQuiescent();
    EXPECT_TRUE(
        AnyVariableConstant(pose_graph.graph(), pose_graph.mutable_optimization(), test.session));
  }

  // A floating fed session next to frozen truth, with no constraint between the two.
  {
    PoseGraph pose_graph;
    pose_graph.Start(TestTime(100));
    const SessionId fed = *pose_graph.session_manager().fed_session();
    const SessionId base = pose_graph.StartNewSession(TestTime(0));
    FeedCorridor(pose_graph, base, 0);
    pose_graph.FreezeSession(base);
    pose_graph.WaitUntilQuiescent();
    FeedCorridor(pose_graph, fed, 100);
    pose_graph.Optimize();
    pose_graph.WaitUntilQuiescent();
    const PoseGraphData& graph = pose_graph.graph();
    Optimization& optimization = pose_graph.mutable_optimization();
    ASSERT_TRUE(graph.session(base).frozen());
    ASSERT_TRUE(AnyVariableConstant(graph, optimization, fed)) << "floating: own datum";

    PoseGraphTrimmer trimmer(pose_graph, option);
    trimmer.EnqueueForTrim(SubmapId{fed.session_index, 1});
    trimmer.TrimOnce();
    pose_graph.WaitUntilQuiescent();
    expect_binary_only(trimmer);
    for (const Constraint& constraint : graph.constraints()) {
      EXPECT_NE(constraint.type, Constraint::Type::PRIOR);
    }

    pose_graph.Optimize();
    pose_graph.WaitUntilQuiescent();
    EXPECT_TRUE(AnyVariableConstant(graph, optimization, fed))
        << "the trim must leave the session on its own datum";
  }
}

}  // namespace
}  // namespace evergreenslam::lifelong

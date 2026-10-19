/**
 * @file drift_tracker_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Odometer, closure validity and slack rules on a grid-free graph.
 * @version 0.1
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/drift_tracker.h"

#include <gtest/gtest.h>

#include <map>

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

constexpr double kPerMeter = 0.1;
constexpr double kCap = 3.0;

ConstraintBuilderOption Option(int min_loop_node_gap) {
  ConstraintBuilderOption option;
  option.candidate_slack_per_meter = kPerMeter;
  option.max_candidate_slack = kCap;
  option.min_loop_node_gap = min_loop_node_gap;
  return option;
}

SessionId StartSession(PoseGraphData& graph) {
  return graph.StartNewSession(common::FromUnixSeconds(1785000000.0));
}

SubmapId AddSubmap(PoseGraphData& graph, SessionId session) {
  SubmapRecord record;
  record.id = graph.AllocateSubmapId(session);
  graph.AddSubmap(record);
  return record.id;
}

NodeId AddNode(PoseGraphData& graph, SessionId session, double x, const SubmapId& submap) {
  Node node;
  node.id = graph.AllocateNodeId(session);
  node.constant_data.local_pose = utils::transform::FromXYTheta(x, 0.0, 0.0);
  graph.AddNode(node, {submap});
  return node.id;
}

NodeId RecordNode(PoseGraphData& graph, DriftTracker& tracker, SessionId session, double x,
                  const SubmapId& submap) {
  const NodeId id = AddNode(graph, session, x, submap);
  tracker.RecordNode(graph, id);
  return id;
}

TEST(DriftTracker, SameSessionSlackIsThePathWalkedBetweenTheEndpoints) {
  PoseGraphData graph;
  DriftTracker tracker(Option(100));
  const SessionId session = StartSession(graph);
  const SubmapId first = AddSubmap(graph, session);
  for (double x : {0.0, 1.0, 2.0}) {
    RecordNode(graph, tracker, session, x, first);
  }
  const SubmapId second = AddSubmap(graph, session);
  RecordNode(graph, tracker, session, 3.0, second);
  const NodeId ahead = RecordNode(graph, tracker, session, 4.0, second);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, ahead, first), kPerMeter * 2.0);

  // Path, not displacement: walking back over the same ground keeps drifting.
  const NodeId back = RecordNode(graph, tracker, session, 0.0, second);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, back, first), kPerMeter * 6.0);

  const NodeId far = RecordNode(graph, tracker, session, 60.0, second);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, far, first), kCap);
}

TEST(DriftTracker, AClosureAtOrPastTheNodeGapResetsBothEndpoints) {
  for (const int gap : {8, 9}) {
    PoseGraphData graph;
    DriftTracker tracker(Option(gap));
    const SessionId session = StartSession(graph);
    const SubmapId first = AddSubmap(graph, session);
    for (double x : {0.0, 1.0, 2.0}) {
      RecordNode(graph, tracker, session, x, first);
    }
    const SubmapId second = AddSubmap(graph, session);
    NodeId closing{};
    for (double x = 3.0; x <= 10.0; x += 1.0) {
      closing = RecordNode(graph, tracker, session, x, second);
    }
    EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, first), kPerMeter * 8.0);

    // Node 10 against a submap whose members end at node 2.
    tracker.NoteClosure(graph, closing, first);
    const NodeId next = RecordNode(graph, tracker, session, 11.0, second);
    if (gap == 8) {
      EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, first), 0.0);
      EXPECT_DOUBLE_EQ(tracker.Slack(graph, next, first), kPerMeter * 1.0);
    } else {
      EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, first), kPerMeter * 8.0);
      EXPECT_DOUBLE_EQ(tracker.Slack(graph, next, first), kPerMeter * 9.0);
    }
  }
}

TEST(DriftTracker, TheNodeGapCountsANodeBelowTheSubmapRangeToo) {
  for (const int gap : {1, 2}) {
    PoseGraphData graph;
    DriftTracker tracker(Option(gap));
    const SessionId session = StartSession(graph);
    const SubmapId first = AddSubmap(graph, session);
    RecordNode(graph, tracker, session, 0.0, first);
    const NodeId early = RecordNode(graph, tracker, session, 1.0, first);
    const SubmapId later = AddSubmap(graph, session);
    for (double x = 2.0; x <= 9.0; x += 1.0) {
      RecordNode(graph, tracker, session, x, later);
    }
    // Node 1 against a submap whose members start at node 2.
    tracker.NoteClosure(graph, early, later);
    const SubmapId newest = AddSubmap(graph, session);
    const NodeId next = RecordNode(graph, tracker, session, 10.0, newest);
    EXPECT_DOUBLE_EQ(tracker.Slack(graph, next, first),
                     gap == 1 ? kPerMeter * 1.0 : kPerMeter * 9.0);
  }
}

TEST(DriftTracker, AFrozenEndpointDriftsNothingAndACrossSessionClosureAlwaysCounts) {
  PoseGraphData graph;
  DriftTracker tracker(Option(100));
  const SessionId base = StartSession(graph);
  const SubmapId base_submap = AddSubmap(graph, base);
  for (double x : {0.0, 1.0, 2.0}) {
    RecordNode(graph, tracker, base, x, base_submap);
  }
  graph.FreezeSession(base);

  const SessionId session = StartSession(graph);
  const SubmapId submap = AddSubmap(graph, session);
  NodeId closing{};
  for (double x = 0.0; x <= 5.0; x += 1.0) {
    closing = RecordNode(graph, tracker, session, x, submap);
  }
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, base_submap), kPerMeter * 5.0);

  tracker.NoteClosure(graph, closing, base_submap);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, base_submap), 0.0);
  const NodeId next = RecordNode(graph, tracker, session, 6.0, submap);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, next, base_submap), kPerMeter * 1.0);
}

TEST(DriftTracker, APathlessIdleSessionGetsTheCapUntilAClosureTiesIt) {
  PoseGraphData graph;
  DriftTracker tracker(Option(100));
  // Loaded from disk: nodes exist but were never recorded.
  const SessionId loaded = StartSession(graph);
  const SubmapId loaded_submap = AddSubmap(graph, loaded);
  const NodeId loaded_node = AddNode(graph, loaded, 0.0, loaded_submap);

  const SessionId session = StartSession(graph);
  const SubmapId submap = AddSubmap(graph, session);
  RecordNode(graph, tracker, session, 0.0, submap);
  const NodeId closing = RecordNode(graph, tracker, session, 1.0, submap);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, loaded_submap), kCap);

  tracker.NoteClosure(graph, closing, loaded_submap);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, closing, loaded_submap), 0.0);
  RecordNode(graph, tracker, session, 2.0, submap);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, loaded_node, submap), kPerMeter * 1.0);
}

struct BaseScene {
  PoseGraphData graph;
  DriftTracker tracker{Option(1)};
  SessionId base;
  SubmapId base_submap;
  NodeId base_node;
  SessionId fed;
  SubmapId fed_submap;
};

void BuildBaseScene(BaseScene& scene) {
  scene.base = StartSession(scene.graph);
  scene.base_submap = AddSubmap(scene.graph, scene.base);
  scene.base_node = RecordNode(scene.graph, scene.tracker, scene.base, 0.0, scene.base_submap);
  scene.graph.FreezeSession(scene.base);
  scene.fed = StartSession(scene.graph);
  scene.fed_submap = AddSubmap(scene.graph, scene.fed);
}

TEST(DriftTracker, OnlyAClosureToAFrozenSessionSetsTheBaseMatch) {
  BaseScene scene;
  BuildBaseScene(scene);
  PoseGraphData& graph = scene.graph;
  DriftTracker& tracker = scene.tracker;
  const SessionId idle = StartSession(graph);
  const SubmapId idle_submap = AddSubmap(graph, idle);
  RecordNode(graph, tracker, idle, 0.0, idle_submap);

  NodeId node{};
  for (double x = 0.0; x <= 4.0; x += 1.0) {
    node = RecordNode(graph, tracker, scene.fed, x, scene.fed_submap);
  }
  const SubmapId later = AddSubmap(graph, scene.fed);
  const NodeId far = RecordNode(graph, tracker, scene.fed, 5.0, later);
  tracker.NoteClosure(graph, far, scene.fed_submap);
  tracker.NoteClosure(graph, node, idle_submap);
  EXPECT_FALSE(tracker.SinceBaseMatch(scene.fed).has_value());

  tracker.NoteClosure(graph, node, scene.base_submap);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 1.0);
  RecordNode(graph, tracker, scene.fed, 7.0, later);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 3.0);
  EXPECT_FALSE(tracker.SinceBaseMatch(scene.base).has_value());
  EXPECT_FALSE(tracker.SinceBaseMatch(idle).has_value());

  // Frozen node against a fed submap: the submap's path counts.
  tracker.NoteClosure(graph, scene.base_node, later);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 0.0);
}

TEST(DriftTracker, ALateBaseMatchNeverMovesTheMarkBack) {
  BaseScene scene;
  BuildBaseScene(scene);
  PoseGraphData& graph = scene.graph;
  DriftTracker& tracker = scene.tracker;
  const NodeId early = RecordNode(graph, tracker, scene.fed, 0.0, scene.fed_submap);
  RecordNode(graph, tracker, scene.fed, 2.0, scene.fed_submap);
  const NodeId newer = RecordNode(graph, tracker, scene.fed, 4.0, scene.fed_submap);
  RecordNode(graph, tracker, scene.fed, 6.0, scene.fed_submap);

  tracker.NoteClosure(graph, newer, scene.base_submap);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 2.0);
  tracker.NoteClosure(graph, early, scene.base_submap);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 2.0);
}

TEST(DriftTracker, AnUnrecordedNodeMatchesAtTheCurrentPath) {
  BaseScene scene;
  BuildBaseScene(scene);
  PoseGraphData& graph = scene.graph;
  DriftTracker& tracker = scene.tracker;
  RecordNode(graph, tracker, scene.fed, 0.0, scene.fed_submap);
  RecordNode(graph, tracker, scene.fed, 3.0, scene.fed_submap);
  const NodeId pending = AddNode(graph, scene.fed, 4.0, scene.fed_submap);
  tracker.NoteClosure(graph, pending, scene.base_submap);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 0.0);
}

TEST(DriftTracker, TheSuccessorContinuesThePredecessorsOdometerUnderTheNewIds) {
  BaseScene scene;
  BuildBaseScene(scene);
  PoseGraphData& graph = scene.graph;
  DriftTracker& tracker = scene.tracker;
  const SessionId old = scene.fed;
  const SubmapId finished = scene.fed_submap;
  NodeId matched{};
  for (double x = 0.0; x <= 3.0; x += 1.0) {
    matched = RecordNode(graph, tracker, old, x, finished);
  }
  const SubmapId open = AddSubmap(graph, old);
  NodeId newest{};
  for (double x = 4.0; x <= 6.0; x += 1.0) {
    newest = RecordNode(graph, tracker, old, x, open);
  }
  tracker.NoteClosure(graph, matched, scene.base_submap);
  ASSERT_DOUBLE_EQ(*tracker.SinceBaseMatch(old), 3.0);
  const double slack_before = tracker.Slack(graph, newest, finished);
  const double base_slack_before = tracker.Slack(graph, newest, scene.base_submap);

  const SessionId next = graph.StartNewSession(common::FromUnixSeconds(1785000001.0));
  const SubmapId adopted = graph.AllocateSubmapId(next);
  const std::map<NodeId, NodeId> renamed = graph.TransferSubmap(open, adopted);
  ASSERT_EQ(renamed.size(), 3u);
  tracker.InheritSession(old, next, {{open, adopted}}, renamed);
  graph.FreezeSession(old);

  const NodeId renamed_newest = renamed.at(newest);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(next), 3.0);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, renamed_newest, scene.base_submap), base_slack_before);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, renamed_newest, adopted), 0.0);
  EXPECT_DOUBLE_EQ(tracker.Slack(graph, renamed_newest, finished), slack_before);

  RecordNode(graph, tracker, next, 8.0, adopted);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(next), 5.0);
}

TEST(DriftTracker, PathLengthCountsFromTheFirstNodeWithOrWithoutABaseMatch) {
  BaseScene scene;
  BuildBaseScene(scene);
  PoseGraphData& graph = scene.graph;
  DriftTracker& tracker = scene.tracker;
  EXPECT_FALSE(tracker.PathLength(scene.fed).has_value());
  RecordNode(graph, tracker, scene.fed, 1.0, scene.fed_submap);
  EXPECT_DOUBLE_EQ(*tracker.PathLength(scene.fed), 0.0);
  const NodeId node = RecordNode(graph, tracker, scene.fed, 3.5, scene.fed_submap);
  EXPECT_DOUBLE_EQ(*tracker.PathLength(scene.fed), 2.5);
  tracker.NoteClosure(graph, node, scene.base_submap);
  RecordNode(graph, tracker, scene.fed, 4.5, scene.fed_submap);
  EXPECT_DOUBLE_EQ(*tracker.PathLength(scene.fed), 3.5);
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(scene.fed), 1.0);
}

TEST(DriftTracker, ASweptSessionLosesItsBaseMatch) {
  BaseScene scene;
  BuildBaseScene(scene);
  PoseGraphData& graph = scene.graph;
  DriftTracker& tracker = scene.tracker;
  const NodeId node = RecordNode(graph, tracker, scene.fed, 0.0, scene.fed_submap);
  tracker.NoteClosure(graph, node, scene.base_submap);
  ASSERT_TRUE(tracker.SinceBaseMatch(scene.fed).has_value());
  graph.RemoveSession(scene.fed);
  tracker.Sweep(graph);
  EXPECT_FALSE(tracker.SinceBaseMatch(scene.fed).has_value());
}

}  // namespace
}  // namespace evergreenslam::lifelong

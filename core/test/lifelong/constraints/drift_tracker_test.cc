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

TEST(DriftTracker, APathlessFloatingSessionGetsTheCapUntilAClosureTiesIt) {
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

}  // namespace
}  // namespace evergreenslam::lifelong

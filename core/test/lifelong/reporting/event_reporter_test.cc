/**
 * @file event_reporter_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Which backend changes become events, and with which fields.
 * @version 0.1
 * @date 2026-10-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/reporting/event_reporter.h"

#include <gtest/gtest.h>

#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph_data.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

NodeId AddNode(PoseGraphData& graph, SessionId session, int index, const Eigen::Affine2d& local) {
  Node node;
  node.id = NodeId{session.session_index, index};
  node.constant_data.time = TestTime(index);
  node.constant_data.local_pose = local;
  graph.AddNode(node, {});
  return node.id;
}

std::vector<PoseGraphEvent> Events(const EventReporter& reporter) {
  return reporter.Since(0).events;
}

TEST(EventReporter, APoseCorrectionNeedsTheSameSessionAndAMoveAboveEitherThreshold) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  AddNode(graph, session, 3, transform::FromXYTheta(1.0, 0.0, 0.0));
  EventReporter reporter(graph, ReportingOption());

  reporter.OnAlignmentPublished(session, Eigen::Affine2d::Identity());
  reporter.OnAlignmentPublished(session, transform::FromXYTheta(0.2, 0.0, 0.0));
  EXPECT_TRUE(Events(reporter).empty());

  reporter.OnAlignmentPublished(session, transform::FromXYTheta(0.7, 0.0, 0.0));
  std::vector<PoseGraphEvent> events = Events(reporter);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, PoseGraphEvent::Kind::POSE_CORRECTION);
  EXPECT_EQ(events[0].session, session);
  EXPECT_EQ(events[0].time, TestTime(3));
  ASSERT_TRUE(events[0].correction.has_value());
  EXPECT_NEAR(events[0].correction->translation, 0.5, 1e-9);

  // About the origin, so the node 1 m out also moves 0.25 m: the rotation alone trips it.
  reporter.OnAlignmentPublished(session, transform::FromXYTheta(0.7, 0.0, 0.25));
  events = Events(reporter);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_NEAR(events[1].correction->rotation, 0.25, 1e-9);

  const SessionId next = graph.StartNewSession(TestTime(4));
  AddNode(graph, next, 0, Eigen::Affine2d::Identity());
  reporter.OnAlignmentPublished(next, transform::FromXYTheta(30.0, 0.0, 0.0));
  EXPECT_EQ(Events(reporter).size(), 2u) << "a session handover is not a correction";
}

TEST(EventReporter, SessionChangesCarryTheNewestSensorTime) {
  PoseGraphData graph;
  const SessionId first = graph.StartNewSession(TestTime(0));
  const SessionId second = graph.StartNewSession(TestTime(0));
  AddNode(graph, first, 5, Eigen::Affine2d::Identity());
  AddNode(graph, second, 9, Eigen::Affine2d::Identity());
  EventReporter reporter(graph, ReportingOption());

  reporter.OnSessionFrozen(graph, first);
  reporter.OnSessionRemoved(second);
  reporter.OnFedSessionDropped(second, first, TestTime(7));

  const std::vector<PoseGraphEvent> events = Events(reporter);
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].kind, PoseGraphEvent::Kind::SESSION_FROZEN);
  EXPECT_EQ(events[0].session, first);
  EXPECT_EQ(events[0].time, TestTime(9));
  EXPECT_EQ(events[1].kind, PoseGraphEvent::Kind::SESSION_REMOVED);
  EXPECT_EQ(events[1].session, second);
  EXPECT_EQ(events[2].kind, PoseGraphEvent::Kind::FED_SESSION_DROPPED);
  EXPECT_EQ(events[2].session, second);
  EXPECT_EQ(events[2].fed_now, first);
  EXPECT_EQ(events[2].time, TestTime(7));
  EXPECT_EQ(events[2].seq, 2);
}

}  // namespace
}  // namespace evergreenslam::lifelong

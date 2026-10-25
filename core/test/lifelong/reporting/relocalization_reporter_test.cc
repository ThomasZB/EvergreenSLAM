/**
 * @file relocalization_reporter_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Which finished rounds become relocalization results, and what they say.
 * @version 0.1
 * @date 2026-10-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/reporting/relocalization_reporter.h"

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

struct Fixture {
  Fixture() : events(graph, ReportingOption()), reporter(graph, events) {
    session = graph.StartNewSession(TestTime(0));
    Node node;
    node.id = NodeId{session.session_index, 4};
    node.constant_data.time = TestTime(4);
    graph.AddNode(node, {});
    node_id = node.id;
    reporter.AddListener([this](const RelocalizationResult& result) { results.push_back(result); });
  }

  MatchRoundSummary Round(MatchRoundSummary::Source source) const {
    MatchRoundSummary summary;
    summary.source = source;
    summary.node_id = node_id;
    summary.candidates = 3;
    return summary;
  }

  PoseGraphData graph;
  EventReporter events;
  RelocalizationReporter reporter;
  SessionId session;
  NodeId node_id;
  std::vector<RelocalizationResult> results;
};

TEST(RelocalizationReporter, OrdinaryLoopClosureRoundsAreNotReported) {
  Fixture fixture;
  fixture.reporter.OnMatchRoundFinished(fixture.Round(MatchRoundSummary::Source::GRAPH));
  EXPECT_TRUE(fixture.results.empty());
  EXPECT_TRUE(fixture.events.Since(0).events.empty());
}

TEST(RelocalizationReporter, AnAcceptedRoundReportsHowFarTheNodeMoved) {
  Fixture fixture;
  MatchRoundSummary summary = fixture.Round(MatchRoundSummary::Source::EXTERNAL);
  summary.prior = transform::FromXYTheta(1.0, 2.0, 0.0);
  summary.constraints_added = 2;
  summary.best = ScoredPose{transform::FromXYTheta(3.0, 4.0, 0.5), 0.8};
  fixture.graph.SetNodeGlobalPose(fixture.node_id, transform::FromXYTheta(3.0, 4.0, 0.5));

  fixture.reporter.OnMatchRoundFinished(summary);

  ASSERT_EQ(fixture.results.size(), 1u);
  const RelocalizationResult& result = fixture.results[0];
  EXPECT_TRUE(result.node_id == fixture.node_id);
  EXPECT_EQ(result.time, TestTime(4));
  ASSERT_TRUE(result.prior.has_value());
  EXPECT_TRUE(result.prior->isApprox(*summary.prior));
  EXPECT_EQ(result.candidates, 3);
  EXPECT_EQ(result.constraints_added, 2);
  ASSERT_TRUE(result.correction.has_value());
  EXPECT_NEAR(result.correction->translation, 5.0, 1e-9);
  EXPECT_NEAR(result.correction->rotation, 0.5, 1e-9);
  ASSERT_TRUE(result.best.has_value());
  EXPECT_DOUBLE_EQ(result.best->score, 0.8);

  const std::vector<PoseGraphEvent> events = fixture.events.Since(0).events;
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].kind, PoseGraphEvent::Kind::RELOCALIZATION);
  EXPECT_EQ(events[0].candidates, 3);
  EXPECT_EQ(events[0].constraints_added, 2);
  ASSERT_TRUE(events[0].correction.has_value());
  EXPECT_NEAR(events[0].correction->translation, 5.0, 1e-9);
}

TEST(RelocalizationReporter, ARoundWithoutAConstraintReportsNoCorrection) {
  Fixture fixture;
  fixture.graph.SetNodeGlobalPose(fixture.node_id, transform::FromXYTheta(3.0, 4.0, 0.5));
  fixture.reporter.OnMatchRoundFinished(fixture.Round(MatchRoundSummary::Source::NONE));
  ASSERT_EQ(fixture.results.size(), 1u);
  EXPECT_FALSE(fixture.results[0].prior.has_value());
  EXPECT_FALSE(fixture.results[0].correction.has_value());
}

TEST(RelocalizationReporter, ARoundWhoseNodeIsGoneIsNotReported) {
  Fixture fixture;
  MatchRoundSummary summary = fixture.Round(MatchRoundSummary::Source::EXTERNAL);
  summary.node_id = NodeId{fixture.session.session_index + 1, 0};
  fixture.reporter.OnMatchRoundFinished(summary);
  EXPECT_TRUE(fixture.results.empty());
  EXPECT_TRUE(fixture.events.Since(0).events.empty());
}

}  // namespace
}  // namespace evergreenslam::lifelong

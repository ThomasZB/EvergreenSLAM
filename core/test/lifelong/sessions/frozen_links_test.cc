/**
 * @file frozen_links_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Which constraints count as a tie to the frozen layer, and for which submaps.
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/frozen_links.h"

#include <gtest/gtest.h>

#include <map>
#include <vector>

#include "common/time.h"

namespace evergreenslam::lifelong {
namespace {

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// Graph-only: links are about endpoints, never about cells or poses.
class Fixture {
 public:
  SessionId StartSession() { return graph_.StartNewSession(TestTime(0)); }

  SubmapId AddSubmap(SessionId session, int index) {
    SubmapRecord record;
    record.id = SubmapId{session.session_index, index};
    graph_.AddSubmap(record);
    return record.id;
  }

  NodeId AddNode(SessionId session, int index, const std::vector<SubmapId>& in) {
    Node node;
    node.id = NodeId{session.session_index, index};
    node.constant_data.time = TestTime(index);
    graph_.AddNode(node, in);
    return node.id;
  }

  void Intra(const SubmapId& submap, const NodeId& node) {
    Add(Constraint::Type::INTRA_SUBMAP, VariableId::Of(submap), VariableId::Of(node));
  }
  void Inter(const VariableId& from, const VariableId& to) {
    Add(Constraint::Type::INTER_SUBMAP, from, to);
  }
  void Prior(const VariableId& variable) {
    Constraint constraint;
    constraint.type = Constraint::Type::PRIOR;
    constraint.from = variable;
    graph_.AddConstraint(constraint);
  }

  PoseGraphData& graph() { return graph_; }

 private:
  void Add(Constraint::Type type, const VariableId& from, const VariableId& to) {
    Constraint constraint;
    constraint.type = type;
    constraint.from = from;
    constraint.to = to;
    graph_.AddConstraint(constraint);
  }

  PoseGraphData graph_;
};

// Session 0 with two overlapping submaps, then its unfinished second submap is rotated into
// session 1 and session 0 is frozen: the shape the manager leaves behind on every freeze.
struct Rotated {
  Fixture fixture;
  SessionId frozen;
  SessionId live;
  SubmapId frozen_submap;
  SubmapId transferred;
  NodeId frozen_only_node;
  NodeId shared_node;
  NodeId tail_node;

  Rotated() {
    frozen = fixture.StartSession();
    live = fixture.StartSession();
    frozen_submap = fixture.AddSubmap(frozen, 0);
    const SubmapId second = fixture.AddSubmap(frozen, 1);
    frozen_only_node = fixture.AddNode(frozen, 0, {frozen_submap});
    shared_node = fixture.AddNode(frozen, 1, {frozen_submap, second});
    tail_node = fixture.AddNode(frozen, 2, {second});
    fixture.Intra(frozen_submap, frozen_only_node);
    fixture.Intra(frozen_submap, shared_node);
    fixture.Intra(second, shared_node);
    fixture.Intra(second, tail_node);

    transferred = SubmapId{live.session_index, 0};
    fixture.graph().TransferSubmap(second, transferred);
    fixture.graph().FreezeSession(frozen);
  }
};

TEST(FrozenLinksTest, EverySubmapGetsAnEntryEvenWithoutConstraints) {
  Fixture fixture;
  const SessionId session = fixture.StartSession();
  const SubmapId first = fixture.AddSubmap(session, 0);
  const SubmapId second = fixture.AddSubmap(session, 1);

  const std::map<SubmapId, int> links = CountFrozenLinks(fixture.graph(), session);
  const std::map<SubmapId, int> expected{{first, 0}, {second, 0}};
  EXPECT_EQ(links, expected);
}

TEST(FrozenLinksTest, IntraOverlapLeftBehindByARotationCounts) {
  Rotated scene;
  const std::map<SubmapId, int> links = CountFrozenLinks(scene.fixture.graph(), scene.live);
  ASSERT_EQ(links.size(), 1u);
  // The shared and the tail node both stayed in the frozen session.
  EXPECT_EQ(links.at(scene.transferred), 2);
}

TEST(FrozenLinksTest, FrozenToFrozenDoesNotCount) {
  Rotated scene;
  // The shared node is held by the transferred submap, but both endpoints are frozen.
  scene.fixture.Inter(VariableId::Of(scene.frozen_submap), VariableId::Of(scene.tail_node));
  const std::map<SubmapId, int> links = CountFrozenLinks(scene.fixture.graph(), scene.live);
  EXPECT_EQ(links.at(scene.transferred), 2);

  const std::map<SubmapId, int> frozen_links =
      CountFrozenLinks(scene.fixture.graph(), scene.frozen);
  EXPECT_EQ(frozen_links.at(scene.frozen_submap), 0);
}

TEST(FrozenLinksTest, InterFromAFrozenSubmapCountsForEverySubmapHoldingTheNode) {
  Rotated scene;
  const SubmapId one = scene.fixture.AddSubmap(scene.live, 1);
  const SubmapId two = scene.fixture.AddSubmap(scene.live, 2);
  const NodeId node = scene.fixture.AddNode(scene.live, 0, {one, two});
  scene.fixture.Intra(one, node);
  scene.fixture.Intra(two, node);
  scene.fixture.Inter(VariableId::Of(scene.frozen_submap), VariableId::Of(node));

  const std::map<SubmapId, int> links = CountFrozenLinks(scene.fixture.graph(), scene.live);
  EXPECT_EQ(links.at(scene.transferred), 2);
  EXPECT_EQ(links.at(one), 1);
  EXPECT_EQ(links.at(two), 1);
}

TEST(FrozenLinksTest, SubmapToSubmapCountsInEitherDirection) {
  Rotated scene;
  const SubmapId one = scene.fixture.AddSubmap(scene.live, 1);
  const SubmapId two = scene.fixture.AddSubmap(scene.live, 2);
  scene.fixture.Inter(VariableId::Of(one), VariableId::Of(scene.frozen_submap));
  scene.fixture.Inter(VariableId::Of(scene.frozen_submap), VariableId::Of(two));

  const std::map<SubmapId, int> links = CountFrozenLinks(scene.fixture.graph(), scene.live);
  EXPECT_EQ(links.at(one), 1);
  EXPECT_EQ(links.at(two), 1);
}

TEST(FrozenLinksTest, PriorCounts) {
  Rotated scene;
  const SubmapId one = scene.fixture.AddSubmap(scene.live, 1);
  const SubmapId two = scene.fixture.AddSubmap(scene.live, 2);
  const NodeId node = scene.fixture.AddNode(scene.live, 0, {one, two});
  scene.fixture.Prior(VariableId::Of(one));
  scene.fixture.Prior(VariableId::Of(node));
  // A prior inside the frozen session is not a tie for anyone.
  scene.fixture.Prior(VariableId::Of(scene.tail_node));

  const std::map<SubmapId, int> links = CountFrozenLinks(scene.fixture.graph(), scene.live);
  EXPECT_EQ(links.at(scene.transferred), 2);
  EXPECT_EQ(links.at(one), 2);
  EXPECT_EQ(links.at(two), 1);
}

TEST(FrozenLinksTest, UnfrozenToUnfrozenDoesNotCount) {
  Rotated scene;
  const SubmapId one = scene.fixture.AddSubmap(scene.live, 1);
  const NodeId node = scene.fixture.AddNode(scene.live, 0, {one});
  scene.fixture.Intra(one, node);

  const SessionId other = scene.fixture.StartSession();
  const SubmapId other_submap = scene.fixture.AddSubmap(other, 0);
  const NodeId other_node = scene.fixture.AddNode(other, 0, {other_submap});
  scene.fixture.Inter(VariableId::Of(other_submap), VariableId::Of(node));
  scene.fixture.Inter(VariableId::Of(one), VariableId::Of(other_node));

  EXPECT_EQ(CountFrozenLinks(scene.fixture.graph(), scene.live).at(one), 0);
  EXPECT_EQ(CountFrozenLinks(scene.fixture.graph(), other).at(other_submap), 0);
}

}  // namespace
}  // namespace evergreenslam::lifelong

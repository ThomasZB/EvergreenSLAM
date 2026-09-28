/**
 * @file anchor_store_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Binding, resolution and every hook of the anchor store on a hand-built graph.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/anchors/anchor_store.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <map>
#include <optional>
#include <vector>

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

void ExpectPoseNear(const Eigen::Affine2d& expected, const Eigen::Affine2d& actual,
                    double tolerance = 1e-12) {
  EXPECT_LT((expected.translation() - actual.translation()).norm(), tolerance);
  EXPECT_LT(
      std::abs(transform::NormalizeAngle(transform::GetYaw(expected) - transform::GetYaw(actual))),
      tolerance);
}

SubmapId AddSubmap(PoseGraphData& graph, SessionId session, const Eigen::Affine2d& global_pose) {
  SubmapRecord record;
  record.id = graph.AllocateSubmapId(session);
  record.local_pose = global_pose;
  record.global_pose = global_pose;
  graph.AddSubmap(record);
  return record.id;
}

NodeId AddNode(PoseGraphData& graph, SessionId session, const Eigen::Affine2d& global_pose,
               const std::vector<SubmapId>& submaps, int time_index) {
  Node node;
  node.id = graph.AllocateNodeId(session);
  node.constant_data.time = TestTime(time_index);
  node.constant_data.local_pose = global_pose;
  node.constant_data.point_cloud.push_back(sensor::Point2d{Eigen::Vector2d(1.013, -0.507)});
  node.constant_data.point_cloud.push_back(sensor::Point2d{Eigen::Vector2d(2.271, 0.389)});
  node.global_pose = global_pose;
  graph.AddNode(node, submaps);
  return node.id;
}

// Two sessions: s0 holds submaps a, b and node n in both; s1 holds submap c and node m.
struct Fixture {
  PoseGraphData graph;
  SessionId s0;
  SessionId s1;
  SubmapId a;
  SubmapId b;
  SubmapId c;
  NodeId n;
  NodeId m;
  Eigen::Affine2d n_pose = transform::FromXYTheta(3.217, 1.083, 0.713);
  Eigen::Affine2d m_pose = transform::FromXYTheta(-2.419, 4.607, -1.171);

  Fixture() {
    s0 = graph.StartNewSession(TestTime(0));
    s1 = graph.StartNewSession(TestTime(50));
    a = AddSubmap(graph, s0, transform::FromXYTheta(0.513, 0.227, 0.3));
    b = AddSubmap(graph, s0, transform::FromXYTheta(2.911, 0.731, 0.6));
    c = AddSubmap(graph, s1, transform::FromXYTheta(-3.007, 3.991, -0.9));
    n = AddNode(graph, s0, n_pose, {b, a}, 7);
    m = AddNode(graph, s1, m_pose, {c}, 53);
  }

  // Deletes `deleted` through the graph, as the trimmer's report would describe it.
  TrimReport Trim(const SubmapId& deleted, std::optional<SubmapId> successor) {
    TrimRequest request;
    request.deleted_submap_ids.push_back(deleted);
    if (successor.has_value()) {
      request.successors.emplace(deleted, *successor);
    }
    return graph.ApplyTrim(request);
  }
};

TEST(AnchorStoreTest, SaveBindsTheOldestContainingSubmapAndResolvesToTheNodePose) {
  Fixture f;
  AnchorStore store;
  const std::optional<Anchor> anchor = store.Save(f.graph, f.n, /*keep_scan=*/true);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_EQ(anchor->id, 1u);
  EXPECT_EQ(anchor->submap_id, f.a) << "a was created first, so it finishes first";
  EXPECT_EQ(anchor->node_id, f.n);
  EXPECT_EQ(anchor->saved_at, TestTime(7));
  EXPECT_EQ(anchor->state, AnchorState::BOUND);
  ASSERT_TRUE(anchor->scan.has_value());
  EXPECT_EQ(anchor->scan->size(), 2u);
  ExpectPoseNear(f.graph.submap(f.a).global_pose.inverse() * f.n_pose, anchor->submap_from_anchor);
  EXPECT_EQ(store.size(), 1);
  EXPECT_EQ(store.revision(), 1);

  const std::optional<ResolvedAnchor> resolved = store.Resolve(f.graph, anchor->id);
  ASSERT_TRUE(resolved.has_value());
  EXPECT_EQ(resolved->state, AnchorState::BOUND);
  EXPECT_FALSE(resolved->frozen);
  ASSERT_TRUE(resolved->global_pose.has_value());
  ExpectPoseNear(f.n_pose, *resolved->global_pose);

  const std::optional<Anchor> without_scan = store.Save(f.graph, f.m, /*keep_scan=*/false);
  ASSERT_TRUE(without_scan.has_value());
  EXPECT_EQ(without_scan->id, 2u);
  EXPECT_FALSE(without_scan->scan.has_value());
  EXPECT_FALSE(store.Resolve(f.graph, 99).has_value());
}

TEST(AnchorStoreTest, SaveOfAMissingOrUnboundNodeIssuesNoId) {
  Fixture f;
  AnchorStore store;
  EXPECT_FALSE(store.Save(f.graph, NodeId{0, 42}, true).has_value());
  const NodeId loose = AddNode(f.graph, f.s1, f.m_pose, {}, 60);
  EXPECT_FALSE(store.Save(f.graph, loose, true).has_value());
  EXPECT_EQ(store.revision(), 0);
  EXPECT_EQ(store.Save(f.graph, f.n, true)->id, 1u);
}

TEST(AnchorStoreTest, ResolveFollowsTheSubmapAndDerivesFrozenFromTheSession) {
  Fixture f;
  AnchorStore store;
  const AnchorId id = store.Save(f.graph, f.n, false)->id;
  const Eigen::Affine2d correction = transform::FromXYTheta(0.37, -0.21, 0.05);
  f.graph.SetSubmapGlobalPose(f.a, correction * f.graph.submap(f.a).global_pose);
  ExpectPoseNear(correction * f.n_pose, *store.Resolve(f.graph, id)->global_pose, 1e-12);

  f.graph.FreezeSession(f.s0);
  const ResolvedAnchor resolved = *store.Resolve(f.graph, id);
  EXPECT_TRUE(resolved.frozen);
  EXPECT_EQ(resolved.state, AnchorState::BOUND) << "frozen is derived, never stored";
  EXPECT_EQ(store.Get(id)->state, AnchorState::BOUND);
}

TEST(AnchorStoreTest, TrimWithASuccessorRebindsWithoutMovingTheAnchor) {
  Fixture f;
  AnchorStore store;
  const AnchorId id = store.Save(f.graph, f.n, false)->id;
  const Eigen::Affine2d before = *store.Resolve(f.graph, id)->global_pose;
  const int64_t revision = store.revision();

  store.OnTrim(f.Trim(f.a, f.c));
  const Anchor anchor = *store.Get(id);
  EXPECT_EQ(anchor.state, AnchorState::REBOUND);
  EXPECT_EQ(anchor.submap_id, f.c) << "the successor, not another submap holding the node";
  EXPECT_GT(store.revision(), revision);
  const ResolvedAnchor resolved = *store.Resolve(f.graph, id);
  EXPECT_EQ(resolved.submap_id, f.c);
  ExpectPoseNear(before, *resolved.global_pose, 1e-9);
}

TEST(AnchorStoreTest, TrimWithoutASuccessorOrphansAndOrphanIsFinal) {
  Fixture f;
  AnchorStore store;
  const AnchorId id = store.Save(f.graph, f.n, false)->id;
  store.OnTrim(f.Trim(f.a, std::nullopt));
  const ResolvedAnchor resolved = *store.Resolve(f.graph, id);
  EXPECT_EQ(resolved.state, AnchorState::ORPHAN);
  EXPECT_EQ(resolved.orphan_reason, OrphanReason::TRIMMED_NO_SUCCESSOR);
  EXPECT_FALSE(resolved.frozen);
  EXPECT_FALSE(resolved.global_pose.has_value());

  const int64_t revision = store.revision();
  store.Reconcile(f.graph);
  store.OnSessionRemoved(f.s0);
  EXPECT_EQ(store.Get(id)->orphan_reason, OrphanReason::TRIMMED_NO_SUCCESSOR);
  EXPECT_EQ(store.revision(), revision);
}

TEST(AnchorStoreTest, HooksThatTouchNoAnchorLeaveTheRevisionAlone) {
  Fixture f;
  AnchorStore store;
  store.Save(f.graph, f.m, false);
  const int64_t revision = store.revision();
  store.OnTrim(f.Trim(f.b, f.a));
  store.OnSubmapsTransferred({{f.a, SubmapId{f.s0.session_index, 9}}});
  store.OnSessionRemoved(f.s0);
  store.Reconcile(f.graph);
  EXPECT_EQ(store.revision(), revision);
  EXPECT_EQ(store.Get(1)->state, AnchorState::BOUND);
}

TEST(AnchorStoreTest, TransferRenamesOnlyAndKeepsTheState) {
  Fixture f;
  AnchorStore store;
  const AnchorId bound = store.Save(f.graph, f.n, false)->id;
  const AnchorId rebound = store.Save(f.graph, f.m, false)->id;
  store.OnTrim(f.Trim(f.c, f.b));
  ASSERT_EQ(store.Get(rebound)->state, AnchorState::REBOUND);
  const Eigen::Affine2d bound_before = *store.Resolve(f.graph, bound)->global_pose;
  const Eigen::Affine2d rebound_before = *store.Resolve(f.graph, rebound)->global_pose;

  const SessionId next = f.graph.StartNewSession(TestTime(90));
  std::map<SubmapId, SubmapId> adoption;
  for (const SubmapId& old_id : {f.a, f.b}) {
    const SubmapId new_id = f.graph.AllocateSubmapId(next);
    f.graph.TransferSubmap(old_id, new_id);
    adoption.emplace(old_id, new_id);
  }
  store.OnSubmapsTransferred(adoption);

  EXPECT_EQ(store.Get(bound)->submap_id, adoption.at(f.a));
  EXPECT_EQ(store.Get(bound)->state, AnchorState::BOUND);
  EXPECT_EQ(store.Get(rebound)->submap_id, adoption.at(f.b));
  EXPECT_EQ(store.Get(rebound)->state, AnchorState::REBOUND);
  ExpectPoseNear(bound_before, *store.Resolve(f.graph, bound)->global_pose);
  ExpectPoseNear(rebound_before, *store.Resolve(f.graph, rebound)->global_pose);
}

TEST(AnchorStoreTest, SessionRemovalOrphansThatSessionsAnchorsOnly) {
  Fixture f;
  AnchorStore store;
  const AnchorId in_s0 = store.Save(f.graph, f.n, false)->id;
  const AnchorId in_s1 = store.Save(f.graph, f.m, false)->id;
  f.graph.RemoveSession(f.s1);
  store.OnSessionRemoved(f.s1);
  EXPECT_EQ(store.Get(in_s1)->state, AnchorState::ORPHAN);
  EXPECT_EQ(store.Get(in_s1)->orphan_reason, OrphanReason::SESSION_REMOVED);
  EXPECT_EQ(store.Get(in_s0)->state, AnchorState::BOUND);
  EXPECT_TRUE(store.Resolve(f.graph, in_s0)->global_pose.has_value());
  EXPECT_FALSE(store.Resolve(f.graph, in_s1)->global_pose.has_value());
}

TEST(AnchorStoreTest, ReconcileTellsARemovedSessionFromAMissingSubmap) {
  Fixture f;
  AnchorStore store;
  const AnchorId on_a = store.Save(f.graph, f.n, false)->id;
  const AnchorId on_c = store.Save(f.graph, f.m, false)->id;
  const AnchorTable table = store.Table();

  PoseGraphData reloaded;
  reloaded.StartNewSession(TestTime(0));
  AddSubmap(reloaded, SessionId{0}, transform::FromXYTheta(0.0, 0.0, 0.0));
  AnchorStore restored;
  restored.Restore(table);
  restored.Reconcile(reloaded);
  EXPECT_EQ(restored.Get(on_a)->orphan_reason, OrphanReason::NONE) << "submap 0.0 is there";
  EXPECT_EQ(restored.Get(on_c)->state, AnchorState::ORPHAN);
  EXPECT_EQ(restored.Get(on_c)->orphan_reason, OrphanReason::SESSION_REMOVED);

  PoseGraphData without_a;
  without_a.StartNewSession(TestTime(0));
  AnchorStore other;
  other.Restore(table);
  other.Reconcile(without_a);
  EXPECT_EQ(other.Get(on_a)->orphan_reason, OrphanReason::SUBMAP_MISSING);
}

TEST(AnchorStoreTest, RebindKeepsTheIdRevivesAnOrphanAndDropsTheOldScan) {
  Fixture f;
  AnchorStore store;
  const AnchorId id = store.Save(f.graph, f.n, true)->id;
  store.OnTrim(f.Trim(f.a, std::nullopt));
  ASSERT_EQ(store.Get(id)->state, AnchorState::ORPHAN);

  const std::optional<Anchor> rebound = store.Rebind(f.graph, id, f.m, /*keep_scan=*/false);
  ASSERT_TRUE(rebound.has_value());
  EXPECT_EQ(rebound->id, id);
  EXPECT_EQ(rebound->state, AnchorState::BOUND);
  EXPECT_EQ(rebound->orphan_reason, OrphanReason::NONE);
  EXPECT_EQ(rebound->submap_id, f.c);
  EXPECT_FALSE(rebound->scan.has_value());
  ExpectPoseNear(f.m_pose, *store.Resolve(f.graph, id)->global_pose);
  EXPECT_EQ(store.Table().next_id, 2u) << "a rebind issues no id";

  EXPECT_FALSE(store.Rebind(f.graph, 42, f.m, false).has_value());
  EXPECT_FALSE(store.Rebind(f.graph, id, NodeId{1, 99}, false).has_value());
  EXPECT_EQ(store.Get(id)->submap_id, f.c) << "a failed rebind changes nothing";
}

TEST(AnchorStoreTest, RestoreKeepsTheHighWaterMarkAndResetsTheRevision) {
  Fixture f;
  AnchorStore store;
  store.Save(f.graph, f.n, true);
  store.Save(f.graph, f.m, false);
  AnchorTable table = store.Table();
  table.next_id = 10;

  AnchorStore restored;
  restored.Restore(table);
  EXPECT_EQ(restored.revision(), 0);
  EXPECT_EQ(restored.size(), 2);
  const std::vector<Anchor> all = restored.All();
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0].id, 1u);
  EXPECT_EQ(all[1].id, 2u);
  EXPECT_TRUE(all[0].scan.has_value());
  EXPECT_EQ(restored.Save(f.graph, f.n, false)->id, 10u);
  const std::vector<ResolvedAnchor> resolved = restored.ResolveAll(f.graph);
  ASSERT_EQ(resolved.size(), 3u);
  EXPECT_EQ(resolved[2].id, 10u);
}

TEST(AnchorStoreTest, RollbackRestoresTheRowOrErasesItWithoutReissuingTheId) {
  Fixture f;
  AnchorStore store;
  const Anchor kept = *store.Save(f.graph, f.n, /*keep_scan=*/false);
  const Anchor moved = *store.Rebind(f.graph, kept.id, f.m, /*keep_scan=*/false);
  ASSERT_FALSE(moved.node_id == kept.node_id);
  int64_t revision = store.revision();
  store.Replace(kept);
  EXPECT_GT(store.revision(), revision);
  EXPECT_EQ(store.Get(kept.id)->node_id, kept.node_id);
  ExpectPoseNear(kept.submap_from_anchor, store.Get(kept.id)->submap_from_anchor);

  const AnchorId refused = store.Save(f.graph, f.m, /*keep_scan=*/false)->id;
  revision = store.revision();
  store.Erase(refused);
  EXPECT_GT(store.revision(), revision);
  EXPECT_FALSE(store.Get(refused).has_value());
  EXPECT_EQ(store.Table().next_id, refused + 1);
  EXPECT_GT(store.Save(f.graph, f.m, /*keep_scan=*/false)->id, refused);
}

TEST(AnchorStoreTest, ALiveAnchorWhoseSubmapVanishedResolvesAsAnOrphan) {
  Fixture f;
  AnchorStore store;
  const AnchorId id = store.Save(f.graph, f.n, false)->id;
  f.Trim(f.a, f.b);
  const ResolvedAnchor resolved = *store.Resolve(f.graph, id);
  EXPECT_EQ(resolved.state, AnchorState::ORPHAN);
  EXPECT_EQ(resolved.orphan_reason, OrphanReason::SUBMAP_MISSING);
  EXPECT_FALSE(resolved.global_pose.has_value());
  EXPECT_EQ(store.Get(id)->state, AnchorState::BOUND) << "Resolve never writes";
}

TEST(AnchorStoreDeathTest, RestoreRefusesANextIdThatWouldReissueAnId) {
  AnchorTable table;
  table.next_id = 3;
  Anchor anchor;
  anchor.id = 3;
  table.anchors.push_back(anchor);
  AnchorStore store;
  EXPECT_DEATH(store.Restore(table), "next_id");
}

}  // namespace
}  // namespace evergreenslam::lifelong

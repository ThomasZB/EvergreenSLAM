/**
 * @file pose_graph_data_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Id allocation, the shape of the structures the later phases bind to, membership
 *        bookkeeping and the mutation interface the trimmer drives.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_data.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <vector>

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

TEST(IdAllocator, SessionIndicesAreMonotone) {
  IdAllocator allocator;
  EXPECT_EQ(allocator.AllocateSession().session_index, 0);
  EXPECT_EQ(allocator.AllocateSession().session_index, 1);
  EXPECT_EQ(allocator.AllocateSession().session_index, 2);
}

TEST(IdAllocator, NodeIndicesAreMonotonePerSession) {
  IdAllocator allocator;
  const SessionId first = allocator.AllocateSession();
  const SessionId second = allocator.AllocateSession();

  EXPECT_EQ(allocator.AllocateNode(first).node_index, 0);
  EXPECT_EQ(allocator.AllocateNode(first).node_index, 1);
  EXPECT_EQ(allocator.AllocateNode(second).node_index, 0);
  EXPECT_EQ(allocator.AllocateNode(first).node_index, 2);

  const NodeId id = allocator.AllocateNode(second);
  EXPECT_EQ(id.session_id, 1);
  EXPECT_EQ(id.node_index, 1);
}

// The same {session, index} pair never comes back, so an id in a file always means one thing.
TEST(IdAllocator, ObservedIdsAreNeverHandedOutAgain) {
  IdAllocator allocator;
  allocator.Observe(NodeId{7, 42});
  allocator.Observe(SubmapId{7, 5});

  EXPECT_EQ(allocator.AllocateSession().session_index, 8);
  EXPECT_EQ(allocator.AllocateNode(SessionId{7}).node_index, 43);
  EXPECT_EQ(allocator.AllocateSubmap(SessionId{7}).submap_index, 6);

  // Out of order observation must not walk the counter backwards.
  allocator.Observe(NodeId{7, 3});
  EXPECT_EQ(allocator.AllocateNode(SessionId{7}).node_index, 44);
}

TEST(IdAllocator, SessionsDoNotShareIndexSpace) {
  IdAllocator allocator;
  const SessionId first = allocator.AllocateSession();
  for (int i = 0; i < 5; ++i) {
    allocator.AllocateNode(first);
  }
  const SessionId second = allocator.AllocateSession();
  const NodeId id = allocator.AllocateNode(second);
  EXPECT_EQ(id.node_index, 0);
  EXPECT_FALSE(id == (NodeId{first.session_index, 0}));
}

TEST(VariableId, KeepsNodesAndSubmapsApart) {
  const VariableId node = VariableId::Of(NodeId{1, 2});
  const VariableId submap = VariableId::Of(SubmapId{1, 2});
  EXPECT_FALSE(node == submap);
  EXPECT_TRUE(node < submap || submap < node);
  EXPECT_TRUE(node.node_id() == (NodeId{1, 2}));
  EXPECT_TRUE(submap.submap_id() == (SubmapId{1, 2}));
  EXPECT_EQ(node.session().session_index, 1);
}

TEST(Constraint, PriorIsTheAbsentSecondEndpoint) {
  Constraint constraint;
  constraint.type = Constraint::Type::PRIOR;
  constraint.from = VariableId::Of(NodeId{0, 3});
  constraint.relative_pose = transform::FromXYTheta(1.0, 2.0, 0.3);
  EXPECT_FALSE(constraint.to.has_value());

  Constraint loop;
  loop.type = Constraint::Type::INTER_SUBMAP;
  loop.from = VariableId::Of(SubmapId{0, 1});
  loop.to = VariableId::Of(NodeId{1, 9});
  EXPECT_TRUE(loop.to.has_value());
  EXPECT_TRUE(loop.sqrt_information.isApprox(Eigen::Matrix3d::Identity()));
}

TEST(TrimReport, CarriesSuccessionAndDeletions) {
  TrimReport report;
  TrimReport::SubmapSuccession succession;
  succession.deleted_submap_id = SubmapId{0, 4};
  succession.successor_submap_id = SubmapId{0, 6};
  succession.successor_from_deleted = transform::FromXYTheta(0.5, -0.25, 0.7);
  report.submap_successions.push_back(succession);
  report.deleted_node_ids.push_back(NodeId{0, 11});

  Constraint constraint;
  constraint.from = VariableId::Of(SubmapId{0, 6});
  constraint.to = VariableId::Of(SubmapId{0, 7});
  report.added_constraints.push_back(constraint);

  ASSERT_EQ(report.submap_successions.size(), 1u);
  EXPECT_TRUE(report.submap_successions.front().successor_submap_id.has_value());
  EXPECT_TRUE(report.submap_successions.front().deleted_submap_id == (SubmapId{0, 4}));
  EXPECT_EQ(report.deleted_node_ids.size(), 1u);
  EXPECT_EQ(report.added_constraints.size(), 1u);

  // A missing successor is a real outcome, not an error: nothing survived to carry it.
  TrimReport::SubmapSuccession orphan;
  orphan.deleted_submap_id = SubmapId{0, 5};
  EXPECT_FALSE(orphan.successor_submap_id.has_value());
}

constexpr double kEpsilon = 1e-12;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

SubmapRecord MakeSubmap(const SubmapId& id, const Eigen::Affine2d& local_pose,
                        const Eigen::Affine2d& global_pose) {
  SubmapRecord record;
  record.id = id;
  record.local_pose = local_pose;
  record.global_pose = global_pose;
  return record;
}

Node MakeNode(const NodeId& id, const Eigen::Affine2d& local_pose) {
  Node node;
  node.id = id;
  node.constant_data.time = TestTime(id.node_index);
  node.constant_data.local_pose = local_pose;
  node.global_pose = local_pose;
  return node;
}

Constraint MakeIntra(const SubmapId& submap_id, const NodeId& node_id) {
  Constraint constraint;
  constraint.type = Constraint::Type::INTRA_SUBMAP;
  constraint.from = VariableId::Of(submap_id);
  constraint.to = VariableId::Of(node_id);
  return constraint;
}

bool HasConstraintBetween(const PoseGraphData& graph, const SubmapId& submap_id,
                          const NodeId& node_id) {
  for (const Constraint& constraint : graph.constraints()) {
    if (constraint.from == VariableId::Of(submap_id) && constraint.to.has_value() &&
        *constraint.to == VariableId::Of(node_id)) {
      return true;
    }
  }
  return false;
}

TEST(PoseGraphData, SessionIdsAreMonotone) {
  PoseGraphData graph;
  const SessionId first = graph.StartNewSession(TestTime(0));
  const SessionId second = graph.StartNewSession(TestTime(1));
  EXPECT_EQ(first.session_index, 0);
  EXPECT_EQ(second.session_index, 1);
  EXPECT_EQ(graph.session(first).state, SessionState::ACTIVE);

  graph.FreezeSession(first);
  EXPECT_TRUE(graph.session(first).frozen());
  EXPECT_FALSE(graph.session(second).frozen());
}

TEST(PoseGraphData, SessionToGlobalFollowsTheLastSubmap) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  EXPECT_FALSE(graph.ComputeSessionToGlobal(session).has_value());

  const Eigen::Affine2d local_pose = transform::FromXYTheta(2.0, 1.0, 0.3);
  const Eigen::Affine2d global_pose = transform::FromXYTheta(12.0, -4.0, 1.1);
  graph.AddSubmap(MakeSubmap(SubmapId{session.session_index, 0}, local_pose, global_pose));

  const std::optional<Eigen::Affine2d> alignment = graph.ComputeSessionToGlobal(session);
  ASSERT_TRUE(alignment.has_value());
  const Eigen::Affine2d recovered = *alignment * local_pose;
  EXPECT_NEAR((recovered.translation() - global_pose.translation()).norm(), 0.0, kEpsilon);
  EXPECT_NEAR(transform::GetYaw(recovered), transform::GetYaw(global_pose), kEpsilon);
}

// Trimming can take the newest submap out from under the alignment, so the chain has to fall
// back to the newest survivor.
TEST(PoseGraphData, SessionToGlobalFallsBackWhenTheNewestSubmapIsTrimmed) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId older{session.session_index, 0};
  const SubmapId newer{session.session_index, 1};
  const Eigen::Affine2d older_local = transform::FromXYTheta(1.0, 0.0, 0.1);
  const Eigen::Affine2d older_global = transform::FromXYTheta(5.0, 2.0, 0.6);
  graph.AddSubmap(MakeSubmap(older, older_local, older_global));
  graph.AddSubmap(MakeSubmap(newer, transform::FromXYTheta(3.0, 0.0, 0.2),
                             transform::FromXYTheta(-7.0, 8.0, -1.2)));

  TrimRequest request;
  request.deleted_submap_ids = {newer};
  graph.ApplyTrim(request);

  const std::optional<Eigen::Affine2d> alignment = graph.ComputeSessionToGlobal(session);
  ASSERT_TRUE(alignment.has_value());
  const Eigen::Affine2d expected = older_global * older_local.inverse();
  EXPECT_NEAR((alignment->translation() - expected.translation()).norm(), 0.0, kEpsilon);
  EXPECT_NEAR(transform::GetYaw(*alignment), transform::GetYaw(expected), kEpsilon);
}

TEST(PoseGraphData, NodeSurvivesUntilItsLastSubmapIsTrimmed) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId older{session.session_index, 0};
  const SubmapId newer{session.session_index, 1};
  graph.AddSubmap(MakeSubmap(older, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(newer, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));

  const NodeId node_id{session.session_index, 0};
  graph.AddNode(MakeNode(node_id, transform::FromXYTheta(0.5, 0.25, 0.1)), {older, newer});
  graph.AddConstraint(MakeIntra(older, node_id));
  graph.AddConstraint(MakeIntra(newer, node_id));
  EXPECT_EQ(graph.ContainingSubmapIds(node_id).size(), 2u);

  TrimRequest first_request;
  first_request.deleted_submap_ids = {older};
  const TrimReport first_report = graph.ApplyTrim(first_request);
  EXPECT_TRUE(first_report.deleted_node_ids.empty());
  EXPECT_TRUE(graph.HasNode(node_id));
  EXPECT_EQ(graph.ContainingSubmapIds(node_id).size(), 1u);
  EXPECT_FALSE(HasConstraintBetween(graph, older, node_id));
  EXPECT_TRUE(HasConstraintBetween(graph, newer, node_id));

  TrimRequest second_request;
  second_request.deleted_submap_ids = {newer};
  const TrimReport second_report = graph.ApplyTrim(second_request);
  ASSERT_EQ(second_report.deleted_node_ids.size(), 1u);
  EXPECT_TRUE(second_report.deleted_node_ids.front() == node_id);
  EXPECT_FALSE(graph.HasNode(node_id));
  EXPECT_TRUE(graph.constraints().empty());
  EXPECT_TRUE(graph.session(session).node_ids.empty());
  EXPECT_TRUE(graph.session(session).submap_ids.empty());
}

TEST(PoseGraphData, TrimmingOneSubmapKeepsTheNodesOfAnother) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId older{session.session_index, 0};
  const SubmapId newer{session.session_index, 1};
  graph.AddSubmap(MakeSubmap(older, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(newer, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));

  const NodeId shared_node{session.session_index, 0};
  const NodeId older_only{session.session_index, 1};
  graph.AddNode(MakeNode(shared_node, Eigen::Affine2d::Identity()), {older, newer});
  graph.AddNode(MakeNode(older_only, Eigen::Affine2d::Identity()), {older});

  TrimRequest request;
  request.deleted_submap_ids = {older};
  const TrimReport report = graph.ApplyTrim(request);
  ASSERT_EQ(report.deleted_node_ids.size(), 1u);
  EXPECT_TRUE(report.deleted_node_ids.front() == older_only);
  EXPECT_TRUE(graph.HasNode(shared_node));
  EXPECT_FALSE(graph.HasNode(older_only));
  EXPECT_EQ(graph.submap(newer).node_ids.count(shared_node), 1u);
}

// successor_from_deleted maps a pose expressed in the deleted submap into the successor. Both a
// rotation and a translation, or the inverse would pass too.
TEST(PoseGraphData, SuccessionTransformIsExpressedInTheSuccessorFrame) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId deleted{session.session_index, 0};
  const SubmapId successor{session.session_index, 1};
  const Eigen::Affine2d deleted_global = transform::FromXYTheta(3.0, 1.0, 0.4);
  const Eigen::Affine2d successor_global = transform::FromXYTheta(-2.0, 5.0, -0.9);
  graph.AddSubmap(MakeSubmap(deleted, Eigen::Affine2d::Identity(), deleted_global));
  graph.AddSubmap(MakeSubmap(successor, Eigen::Affine2d::Identity(), successor_global));

  TrimRequest request;
  request.deleted_submap_ids = {deleted};
  request.successors = {{deleted, successor}};
  const TrimReport report = graph.ApplyTrim(request);

  ASSERT_EQ(report.submap_successions.size(), 1u);
  const TrimReport::SubmapSuccession& succession = report.submap_successions.front();
  ASSERT_TRUE(succession.successor_submap_id.has_value());
  EXPECT_TRUE(*succession.successor_submap_id == successor);

  // An anchor ahead of the deleted submap must land on the same global point after migration.
  const Eigen::Affine2d anchor_in_deleted = transform::FromXYTheta(1.5, -0.7, 0.25);
  const Eigen::Affine2d migrated = succession.successor_from_deleted * anchor_in_deleted;
  const Eigen::Affine2d before = deleted_global * anchor_in_deleted;
  const Eigen::Affine2d after = successor_global * migrated;
  EXPECT_NEAR((before.translation() - after.translation()).norm(), 0.0, kEpsilon);
  EXPECT_NEAR(transform::GetYaw(before), transform::GetYaw(after), kEpsilon);
}

TEST(PoseGraphData, TrimReportRoundTripsTheAddedConstraints) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId deleted{session.session_index, 0};
  const SubmapId kept{session.session_index, 1};
  graph.AddSubmap(MakeSubmap(deleted, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(kept, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));

  const NodeId survivor{session.session_index, 0};
  graph.AddNode(MakeNode(survivor, Eigen::Affine2d::Identity()), {kept});

  Constraint prior;
  prior.type = Constraint::Type::PRIOR;
  prior.from = VariableId::Of(survivor);
  prior.relative_pose = transform::FromXYTheta(1.0, 2.0, 0.5);

  Constraint pairwise;
  pairwise.type = Constraint::Type::INTER_SUBMAP;
  pairwise.from = VariableId::Of(kept);
  pairwise.to = VariableId::Of(survivor);

  TrimRequest request;
  request.deleted_submap_ids = {deleted};
  request.successors = {{deleted, kept}};
  request.added_constraints = {prior, pairwise};

  const TrimReport report = graph.ApplyTrim(request);
  ASSERT_EQ(report.added_constraints.size(), 2u);
  EXPECT_EQ(report.added_constraints[0].type, Constraint::Type::PRIOR);
  EXPECT_FALSE(report.added_constraints[0].to.has_value());
  EXPECT_EQ(graph.constraints().size(), 2u);
  EXPECT_TRUE(graph.constraints().front().from == VariableId::Of(survivor));
}

TEST(PoseGraphData, FrozenSubmapsAreNotTrimmable) {
  PoseGraphData graph;
  const SessionId frozen = graph.StartNewSession(TestTime(0));
  graph.AddSubmap(MakeSubmap(SubmapId{frozen.session_index, 0}, Eigen::Affine2d::Identity(),
                             Eigen::Affine2d::Identity()));
  graph.FreezeSession(frozen);

  const SessionId active = graph.StartNewSession(TestTime(1));
  const SubmapId active_submap{active.session_index, 0};
  graph.AddSubmap(
      MakeSubmap(active_submap, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));

  const std::vector<SubmapId> trimmable = graph.TrimmableSubmapIds();
  ASSERT_EQ(trimmable.size(), 1u);
  EXPECT_TRUE(trimmable.front() == active_submap);
}

TEST(PoseGraphDeathTest, TrimmingAFrozenSubmapIsRejected) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id{session.session_index, 0};
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  graph.FreezeSession(session);

  TrimRequest request;
  request.deleted_submap_ids = {submap_id};
  EXPECT_DEATH(graph.ApplyTrim(request), "frozen");
}

TEST(PoseGraphDeathTest, PriorAndBinaryConstraintsCannotBeMixedUp) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id{session.session_index, 0};
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  const NodeId node_id{session.session_index, 0};
  graph.AddNode(MakeNode(node_id, Eigen::Affine2d::Identity()), {submap_id});

  Constraint mislabelled = MakeIntra(submap_id, node_id);
  mislabelled.type = Constraint::Type::PRIOR;
  EXPECT_DEATH(graph.AddConstraint(mislabelled), "unary residual");
}

// Ids removed by trimming stay burned: anchors and serialized files can rely on them.
TEST(PoseGraphData, TrimmedIdsAreNeverHandedOutAgain) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id = graph.AllocateSubmapId(session);
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  const NodeId node_id = graph.AllocateNodeId(session);
  graph.AddNode(MakeNode(node_id, Eigen::Affine2d::Identity()), {submap_id});

  TrimRequest request;
  request.deleted_submap_ids = {submap_id};
  graph.ApplyTrim(request);
  EXPECT_FALSE(graph.HasSubmap(submap_id));
  EXPECT_FALSE(graph.HasNode(node_id));

  EXPECT_EQ(graph.AllocateSubmapId(session).submap_index, submap_id.submap_index + 1);
  EXPECT_EQ(graph.AllocateNodeId(session).node_index, node_id.node_index + 1);
  EXPECT_EQ(graph.StartNewSession(TestTime(1)).session_index, session.session_index + 1);
}

TEST(PoseGraphData, RemoveSessionCascadesAndLeavesOthersAlone) {
  PoseGraphData graph;
  const SessionId doomed = graph.StartNewSession(TestTime(0));
  const SessionId kept = graph.StartNewSession(TestTime(1));

  const SubmapId doomed_submap = graph.AllocateSubmapId(doomed);
  graph.AddSubmap(
      MakeSubmap(doomed_submap, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  const NodeId doomed_node = graph.AllocateNodeId(doomed);
  graph.AddNode(MakeNode(doomed_node, Eigen::Affine2d::Identity()), {doomed_submap});
  graph.AddConstraint(MakeIntra(doomed_submap, doomed_node));

  const SubmapId kept_submap = graph.AllocateSubmapId(kept);
  graph.AddSubmap(
      MakeSubmap(kept_submap, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  const NodeId kept_node = graph.AllocateNodeId(kept);
  graph.AddNode(MakeNode(kept_node, Eigen::Affine2d::Identity()), {kept_submap});
  graph.AddConstraint(MakeIntra(kept_submap, kept_node));

  // A loop closure into the removed session must go with its endpoint.
  Constraint cross;
  cross.type = Constraint::Type::INTER_SUBMAP;
  cross.from = VariableId::Of(kept_submap);
  cross.to = VariableId::Of(doomed_node);
  graph.AddConstraint(cross);

  graph.RemoveSession(doomed);

  EXPECT_FALSE(graph.HasSession(doomed));
  EXPECT_FALSE(graph.HasSubmap(doomed_submap));
  EXPECT_FALSE(graph.HasNode(doomed_node));
  EXPECT_TRUE(graph.ContainingSubmapIds(doomed_node).empty());

  // No surviving constraint may keep a dangling endpoint.
  ASSERT_EQ(graph.constraints().size(), 1u);
  for (const Constraint& constraint : graph.constraints()) {
    EXPECT_TRUE(graph.HasVariable(constraint.from));
    if (constraint.to.has_value()) {
      EXPECT_TRUE(graph.HasVariable(*constraint.to));
    }
  }

  EXPECT_TRUE(graph.HasSession(kept));
  EXPECT_TRUE(graph.HasSubmap(kept_submap));
  EXPECT_TRUE(graph.HasNode(kept_node));
  EXPECT_EQ(graph.ContainingSubmapIds(kept_node).size(), 1u);
  EXPECT_EQ(graph.submap(kept_submap).node_ids.count(kept_node), 1u);
  EXPECT_TRUE(HasConstraintBetween(graph, kept_submap, kept_node));
}

TEST(PoseGraphData, RemoveSessionDoesNotLowerTheIdWatermarks) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id = graph.AllocateSubmapId(session);
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  const NodeId node_id = graph.AllocateNodeId(session);
  graph.AddNode(MakeNode(node_id, Eigen::Affine2d::Identity()), {submap_id});

  graph.RemoveSession(session);

  EXPECT_EQ(graph.id_allocator().next_submap_index(session), submap_id.submap_index + 1);
  EXPECT_EQ(graph.id_allocator().next_node_index(session), node_id.node_index + 1);
  EXPECT_EQ(graph.StartNewSession(TestTime(1)).session_index, session.session_index + 1);
}

TEST(PoseGraphDeathTest, RemovingAFrozenSessionIsRejected) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  graph.FreezeSession(session);
  EXPECT_DEATH(graph.RemoveSession(session), "frozen");
}

// The freeze rotation's handover: every reference the graph keeps to the submap must move
// together -- record, both sessions' lists, memberships, constraint endpoints, watermark.
TEST(PoseGraphData, TransferSubmapRewritesEveryReference) {
  PoseGraphData graph;
  const SessionId from = graph.StartNewSession(TestTime(0));
  const SessionId to = graph.StartNewSession(TestTime(1));
  const SubmapId staying{from.session_index, 0};
  const SubmapId moving{from.session_index, 1};
  const Eigen::Affine2d local = transform::FromXYTheta(2.0, 1.0, 0.3);
  const Eigen::Affine2d global = transform::FromXYTheta(12.0, -4.0, 1.1);
  graph.AddSubmap(MakeSubmap(staying, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(moving, local, global));
  const NodeId node{from.session_index, 0};
  graph.AddNode(MakeNode(node, local), {staying, moving});
  graph.AddConstraint(MakeIntra(moving, node));

  const SubmapId new_id = graph.AllocateSubmapId(to);
  graph.TransferSubmap(moving, new_id);

  EXPECT_FALSE(graph.HasSubmap(moving));
  ASSERT_TRUE(graph.HasSubmap(new_id));
  EXPECT_TRUE(graph.submap(new_id).id == new_id);
  EXPECT_NEAR((graph.submap(new_id).global_pose.translation() - global.translation()).norm(), 0.0,
              kEpsilon);

  const std::vector<SubmapId> from_ids = graph.session(from).submap_ids;
  EXPECT_EQ(from_ids.size(), 1u);
  EXPECT_TRUE(from_ids.front() == staying);
  const std::vector<SubmapId> to_ids = graph.session(to).submap_ids;
  ASSERT_EQ(to_ids.size(), 1u);
  EXPECT_TRUE(to_ids.front() == new_id);

  const std::vector<SubmapId> containing = graph.ContainingSubmapIds(node);
  EXPECT_TRUE(std::count(containing.begin(), containing.end(), new_id) == 1);
  EXPECT_TRUE(std::count(containing.begin(), containing.end(), moving) == 0);
  EXPECT_EQ(graph.submap(new_id).node_ids.count(node), 1u);

  EXPECT_TRUE(HasConstraintBetween(graph, new_id, node));
  EXPECT_FALSE(HasConstraintBetween(graph, moving, node));

  // The successor's watermark moved past the transferred id; the source's stays retired.
  EXPECT_EQ(graph.id_allocator().next_submap_index(to), new_id.submap_index + 1);
  EXPECT_EQ(graph.id_allocator().next_submap_index(from), moving.submap_index + 1);

  // The old id is never reused, and the alignment now rides the transferred submap.
  EXPECT_EQ(graph.AllocateSubmapId(from).submap_index, moving.submap_index + 1);
  const std::optional<Eigen::Affine2d> alignment = graph.ComputeSessionToGlobal(to);
  ASSERT_TRUE(alignment.has_value());
  EXPECT_NEAR(((*alignment * local).translation() - global.translation()).norm(), 0.0, kEpsilon);
}

TEST(PoseGraphDeathTest, TransferSubmapRejectsFrozenEndpoints) {
  PoseGraphData graph;
  const SessionId from = graph.StartNewSession(TestTime(0));
  const SessionId to = graph.StartNewSession(TestTime(1));
  const SubmapId submap{from.session_index, 0};
  graph.AddSubmap(MakeSubmap(submap, Eigen::Affine2d::Identity(), Eigen::Affine2d::Identity()));
  graph.FreezeSession(from);
  EXPECT_DEATH(graph.TransferSubmap(submap, SubmapId{to.session_index, 0}), "frozen");
}

}  // namespace
}  // namespace evergreenslam::lifelong

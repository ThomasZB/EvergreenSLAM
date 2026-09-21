/**
 * @file pose_graph_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The backend entry class: deferred graph mutation, id minting through the translation
 *        table, the global initial value of a node, and the boot path that needs no simulation.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph.h"

#include <gtest/gtest.h>

#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph_option.h"
#include "lifelong/pose_graph_trimmer/pose_graph_trimmer.h"
#include "testing/explicit_ingest.h"
#include "testing/loop_scenario.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kEpsilon = 1e-12;
constexpr double kResolution = 0.05;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

std::shared_ptr<const mapping::Submap> MakeSubmap(int local_index,
                                                  const Eigen::Affine2d& local_pose) {
  return std::make_shared<const mapping::Submap>(local_index, local_pose, kResolution);
}

mapping::LocalTrajectoryBuilder::InsertionResult MakeInsertionResult(
    int node_index, const Eigen::Affine2d& local_pose,
    const std::vector<std::shared_ptr<const mapping::Submap>>& submaps) {
  mapping::LocalTrajectoryBuilder::InsertionResult result;
  result.node_index = node_index;
  result.node.time = TestTime(node_index);
  result.node.local_pose = local_pose;
  result.insertion_submaps = submaps;
  return result;
}

void ExpectPoseNear(const Eigen::Affine2d& actual, const Eigen::Affine2d& expected) {
  EXPECT_NEAR((actual.translation() - expected.translation()).norm(), 0.0, kEpsilon);
  EXPECT_NEAR(transform::GetYaw(actual), transform::GetYaw(expected), kEpsilon);
}

Constraint MakeLoopClosure(const SubmapId& submap_id, const NodeId& node_id) {
  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.from = VariableId::Of(submap_id);
  constraint.to = VariableId::Of(node_id);
  constraint.relative_pose = transform::FromXYTheta(0.1, 0.0, 0.0);
  constraint.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
  return constraint;
}

// The submap is never finished, so the constraint builder finds nothing and the only solves are
// the ones under test.
void FeedKeyframe(PoseGraph& pose_graph, int node_index,
                  const std::shared_ptr<const mapping::Submap>& submap) {
  pose_graph.AddInsertionResult(MakeInsertionResult(
      node_index, transform::FromXYTheta(0.1 * node_index, 0.0, 0.0), {submap}));
}

// The gate holds the consumer inside its first task, so the ingest behind it is provably still
// queued while the test reads the graph.
TEST(PoseGraphTest, NothingReachesTheGraphUntilTheQueueRunsTheIngest) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));

  std::promise<void> gate;
  std::future<void> gate_open = gate.get_future();
  pose_graph.Enqueue([&gate_open] { gate_open.wait(); });
  pose_graph.EnqueueInsertionResult(MakeInsertionResult(
      0, transform::FromXYTheta(0.4, 0.2, 0.1), {MakeSubmap(0, Eigen::Affine2d::Identity())}));

  EXPECT_TRUE(pose_graph.graph().nodes().empty());
  EXPECT_TRUE(pose_graph.graph().submaps().empty());
  EXPECT_EQ(pose_graph.task_queue().pending_count(), 2u);

  gate.set_value();
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.graph().nodes().size(), 1u);
  EXPECT_EQ(pose_graph.graph().submaps().size(), 1u);
  EXPECT_EQ(pose_graph.task_queue().pending_count(), 0u);
}

TEST(PoseGraphTest, FirstSessionAlignmentSeedsTheFirstSubmap) {
  const Eigen::Affine2d local_to_global = transform::FromXYTheta(10.0, -3.0, 0.75);
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0), local_to_global);
  const SessionId session = *pose_graph.session_manager().fed_session();

  const Eigen::Affine2d submap_local = transform::FromXYTheta(1.0, 0.5, 0.2);
  const Eigen::Affine2d node_local = transform::FromXYTheta(1.3, 0.6, 0.22);
  pose_graph.EnqueueInsertionResult(
      MakeInsertionResult(0, node_local, {MakeSubmap(0, submap_local)}));
  pose_graph.WaitUntilQuiescent();

  const SubmapId submap_id{session.session_index, 0};
  ExpectPoseNear(pose_graph.graph().submap(submap_id).global_pose, local_to_global * submap_local);
  ExpectPoseNear(pose_graph.graph().node(NodeId{session.session_index, 0}).global_pose,
                 local_to_global * node_local);
}

// The initial value rides the pose chain of the session's last submap, so a node added after
// optimization moved that submap comes in already corrected.
TEST(PoseGraphTest, NodeInitialValueFollowsTheOptimizedLastSubmap) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const Eigen::Affine2d submap_local = transform::FromXYTheta(1.0, 0.5, 0.2);
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, submap_local);

  pose_graph.EnqueueInsertionResult(
      MakeInsertionResult(0, transform::FromXYTheta(1.1, 0.5, 0.2), {submap}));
  pose_graph.WaitUntilQuiescent();

  const SubmapId submap_id{session.session_index, 0};
  const Eigen::Affine2d optimized = transform::FromXYTheta(4.0, -2.0, 1.4);
  pose_graph.mutable_graph().SetSubmapGlobalPose(submap_id, optimized);

  const Eigen::Affine2d node_local = transform::FromXYTheta(1.7, 0.9, 0.35);
  pose_graph.EnqueueInsertionResult(MakeInsertionResult(1, node_local, {submap}));
  pose_graph.WaitUntilQuiescent();

  const Eigen::Affine2d expected = optimized * submap_local.inverse() * node_local;
  ExpectPoseNear(pose_graph.graph().node(NodeId{session.session_index, 1}).global_pose, expected);
  // The stale session alignment would have put it somewhere else entirely.
  EXPECT_GT((expected.translation() - node_local.translation()).norm(), 1.0);
}

TEST(PoseGraphTest, IntraSubmapConstraintPerContainingSubmap) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const Eigen::Affine2d older_local = transform::FromXYTheta(0.0, 0.0, 0.0);
  const Eigen::Affine2d newer_local = transform::FromXYTheta(2.0, 1.0, 0.4);

  const Eigen::Affine2d node_local = transform::FromXYTheta(2.2, 1.1, 0.45);
  pose_graph.EnqueueInsertionResult(
      MakeInsertionResult(0, node_local, {MakeSubmap(0, older_local), MakeSubmap(1, newer_local)}));
  pose_graph.WaitUntilQuiescent();

  const NodeId node_id{session.session_index, 0};
  const SubmapId older{session.session_index, 0};
  const SubmapId newer{session.session_index, 1};
  const PoseGraphData& graph = pose_graph.graph();
  EXPECT_EQ(graph.ContainingSubmapIds(node_id).size(), 2u);
  ASSERT_EQ(graph.constraints().size(), 2u);
  for (const Constraint& constraint : graph.constraints()) {
    EXPECT_EQ(constraint.type, Constraint::Type::INTRA_SUBMAP);
    EXPECT_EQ(constraint.from.kind, VariableId::Kind::SUBMAP);
    ASSERT_TRUE(constraint.to.has_value());
    EXPECT_TRUE(constraint.to->node_id() == node_id);
    EXPECT_TRUE(
        constraint.sqrt_information.isApprox(OdometrySqrtInformation(ConstraintWeightOption())));
    ExpectPoseNear(constraint.relative_pose,
                   graph.submap(constraint.from.submap_id()).local_pose.inverse() * node_local);
  }
  EXPECT_EQ(graph.submap(older).node_ids.count(node_id), 1u);
  EXPECT_EQ(graph.submap(newer).node_ids.count(node_id), 1u);
}

// The weights are configuration, not a constant baked into the ingest path.
TEST(PoseGraphTest, IntraSubmapConstraintCarriesTheConfiguredWeight) {
  PoseGraphOption option;
  option.constraint_weight.odometry_translation_stddev = 0.02;
  option.constraint_weight.odometry_rotation_stddev = 0.005;
  PoseGraph pose_graph(option);
  pose_graph.Start(TestTime(0));

  pose_graph.EnqueueInsertionResult(MakeInsertionResult(
      0, transform::FromXYTheta(0.4, 0.2, 0.1), {MakeSubmap(0, Eigen::Affine2d::Identity())}));
  pose_graph.WaitUntilQuiescent();

  ASSERT_EQ(pose_graph.graph().constraints().size(), 1u);
  const Eigen::Matrix3d sqrt_information =
      pose_graph.graph().constraints().front().sqrt_information;
  EXPECT_EQ(sqrt_information(0, 0), 50.0);
  EXPECT_EQ(sqrt_information(1, 1), 50.0);
  EXPECT_EQ(sqrt_information(2, 2), 200.0);
}

// A submap fed again resolves to the id it was minted under instead of a duplicate, and node ids
// are minted in the fed session in feed order.
TEST(PoseGraphTest, IngestMintsIdsInTheFedSessionAndDedupsSubmapsThroughTheTable) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();

  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());
  pose_graph.EnqueueInsertionResult(
      MakeInsertionResult(0, transform::FromXYTheta(0.1, 0.0, 0.0), {submap}));
  pose_graph.EnqueueInsertionResult(
      MakeInsertionResult(1, transform::FromXYTheta(0.2, 0.0, 0.0), {submap}));
  pose_graph.WaitUntilQuiescent();

  EXPECT_EQ(pose_graph.graph().submaps().size(), 1u);
  EXPECT_EQ(pose_graph.graph().nodes().size(), 2u);
  EXPECT_TRUE(pose_graph.graph().HasSubmap(SubmapId{session.session_index, 0}));
  EXPECT_TRUE(pose_graph.graph().HasNode(NodeId{session.session_index, 0}));
  EXPECT_TRUE(pose_graph.graph().HasNode(NodeId{session.session_index, 1}));
  EXPECT_EQ(pose_graph.graph().ContainingSubmapIds(NodeId{session.session_index, 1}).size(), 1u);
}

PoseGraphOption CadenceOfThree() {
  PoseGraphOption option;
  option.optimization.optimize_every_n_nodes = 3;
  return option;
}

// The node cadence is the only solve trigger: a closure landing between batches waits for it
// like any other edge, and the keyframes that follow count towards the cadence alone.
TEST(PoseGraphTest, AClosureBetweenBatchesDoesNotSolve) {
  PoseGraph pose_graph(CadenceOfThree());
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  FeedKeyframe(pose_graph, 0, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);

  pose_graph.AddConstraint(
      MakeLoopClosure(SubmapId{session.session_index, 0}, NodeId{session.session_index, 0}));
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0) << "a closure is not a solve trigger";

  FeedKeyframe(pose_graph, 1, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);
  FeedKeyframe(pose_graph, 2, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 1) << "the third node is the cadence";
}

// The gate fixes the order: closures, then the batch. Only the cadence solves at the batch end,
// however many closures landed ahead of it.
TEST(PoseGraphTest, ClosuresAheadOfABatchLeaveTheBatchEndToTheCadence) {
  PoseGraph pose_graph(CadenceOfThree());
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  FeedKeyframe(pose_graph, 0, submap);
  pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(pose_graph.optimization().num_solves(), 0);

  std::promise<void> gate;
  std::future<void> gate_open = gate.get_future();
  pose_graph.Enqueue([&gate_open] { gate_open.wait(); });
  const SubmapId submap_id{session.session_index, 0};
  for (int i = 0; i < 3; ++i) {
    pose_graph.AddConstraint(MakeLoopClosure(submap_id, NodeId{session.session_index, 0}));
  }
  FeedKeyframe(pose_graph, 1, submap);
  gate.set_value();
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0) << "two nodes are short of the cadence";
  EXPECT_EQ(pose_graph.graph().constraints().size(), 5u);

  FeedKeyframe(pose_graph, 2, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 1);
}

TEST(PoseGraphTest, RecoveredSummariesDoNotSolveEither) {
  PoseGraph pose_graph(CadenceOfThree());
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  FeedKeyframe(pose_graph, 0, submap);
  Constraint summary =
      MakeLoopClosure(SubmapId{session.session_index, 0}, NodeId{session.session_index, 0});
  summary.recovered = true;
  pose_graph.AddConstraint(summary);
  FeedKeyframe(pose_graph, 1, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);
  EXPECT_EQ(pose_graph.graph().constraints().size(), 3u);
}

TEST(PoseGraphTest, NodeCadenceSolvesAtBatchEndNotInsideIngest) {
  PoseGraphOption option;
  option.optimization.optimize_every_n_nodes = 3;
  PoseGraph pose_graph(option);
  pose_graph.Start(TestTime(0));
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  const std::vector<int> expected_solves = {0, 0, 1, 1, 1, 2};
  for (int i = 0; i < static_cast<int>(expected_solves.size()); ++i) {
    FeedKeyframe(pose_graph, i, submap);
    pose_graph.WaitUntilQuiescent();
    EXPECT_EQ(pose_graph.optimization().num_solves(), expected_solves[i]) << "node " << i;
  }

  // Ingest alone only counts; without a batch end nothing solves.
  for (int i = 6; i < 9; ++i) {
    pose_graph.EnqueueInsertionResult(
        MakeInsertionResult(i, transform::FromXYTheta(0.1 * i, 0.0, 0.0), {submap}));
    pose_graph.WaitUntilQuiescent();
  }
  EXPECT_EQ(pose_graph.optimization().num_solves(), 2);
  FeedKeyframe(pose_graph, 9, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 3);
}

// Closures neither reset the node cadence nor run the trim and checkpoint chain: solves and
// checkpoints land on the cadence's nodes exactly, closure or not.
TEST(PoseGraphTest, ClosuresLeaveTheCadenceAndTheMaintenanceChainAlone) {
  PoseGraphOption option = CadenceOfThree();
  option.checkpoint_min_interval = common::Duration::zero();
  PoseGraph pose_graph(option, testing::MakeTempDir("evergreenslam_pose_graph_test"));
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  FeedKeyframe(pose_graph, 0, submap);
  pose_graph.AddConstraint(
      MakeLoopClosure(SubmapId{session.session_index, 0}, NodeId{session.session_index, 0}));
  FeedKeyframe(pose_graph, 1, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);
  EXPECT_EQ(pose_graph.map_manager()->num_checkpoints_written(), 0);

  FeedKeyframe(pose_graph, 2, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 1) << "the third node is the cadence";
  EXPECT_EQ(pose_graph.map_manager()->num_checkpoints_written(), 1);

  FeedKeyframe(pose_graph, 3, submap);
  pose_graph.AddConstraint(
      MakeLoopClosure(SubmapId{session.session_index, 0}, NodeId{session.session_index, 3}));
  FeedKeyframe(pose_graph, 4, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 1);
  EXPECT_EQ(pose_graph.map_manager()->num_checkpoints_written(), 1);
  FeedKeyframe(pose_graph, 5, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 2);
  EXPECT_EQ(pose_graph.map_manager()->num_checkpoints_written(), 2);
}

// The match-result funnel: results land through AddConstraintIfEndpointsLive on the backend
// task, between two batches, and solve no sooner than a closure sent from outside.
TEST(PoseGraphTest, MatchResultFunnelDoesNotSolveBetweenBatches) {
  PoseGraph pose_graph(CadenceOfThree());
  pose_graph.Start(TestTime(0));
  const SessionId session = *pose_graph.session_manager().fed_session();
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  FeedKeyframe(pose_graph, 0, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);

  const Constraint closure =
      MakeLoopClosure(SubmapId{session.session_index, 0}, NodeId{session.session_index, 0});
  pose_graph.Enqueue(
      [&pose_graph, closure] { EXPECT_TRUE(pose_graph.AddConstraintIfEndpointsLive(closure)); });
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);
  EXPECT_EQ(pose_graph.graph().constraints().size(), 2u);

  FeedKeyframe(pose_graph, 1, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);

  FeedKeyframe(pose_graph, 2, submap);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 1);
}

TEST(PoseGraphTest, FirstBootOpensASessionAndFeedsNothing) {
  PoseGraph backend;
  backend.Start(common::FromUnixSeconds(1785000000.0));
  const std::optional<SessionId> fed = backend.session_manager().fed_session();
  ASSERT_TRUE(fed.has_value());
  EXPECT_TRUE(backend.graph().HasSession(*fed));
  EXPECT_EQ(backend.graph().sessions().size(), 1u);
  EXPECT_TRUE(backend.graph().session(*fed).node_ids.empty());
}

// Manual relocalization comes in as the boot session's initial alignment; the frontend still
// starts from zero.
TEST(PoseGraphTest, StartHonoursAManualInitialGlobalPose) {
  PoseGraph backend;
  const Eigen::Affine2d relocated = transform::FromXYTheta(4.0, -1.5, 0.6);
  backend.Start(common::FromUnixSeconds(1785000000.0), relocated);
  const std::optional<SessionId> fed = backend.session_manager().fed_session();
  ASSERT_TRUE(fed.has_value());
  ExpectPoseNear(backend.graph().session(*fed).local_to_global, relocated);
}

TEST(PoseGraphTest, CheckpointsRideSolvesNoOftenerThanTheInterval) {
  PoseGraphOption option;
  option.optimization.optimize_every_n_nodes = 3;
  option.checkpoint_min_interval = common::FromSeconds(0.5);
  PoseGraph pose_graph(option, testing::MakeTempDir("evergreenslam_pose_graph_interval"));
  pose_graph.Start(TestTime(0));
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());

  // Keyframes 0.1 s apart, solves at nodes 2, 5, 8, 11: only the solves at least 0.5 s after
  // the previous write (Start counts as one) checkpoint.
  const std::vector<int> expected_checkpoints = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2};
  for (int i = 0; i < static_cast<int>(expected_checkpoints.size()); ++i) {
    FeedKeyframe(pose_graph, i, submap);
    pose_graph.WaitUntilQuiescent();
    EXPECT_EQ(pose_graph.map_manager()->num_checkpoints_written(), expected_checkpoints[i])
        << "node " << i;
  }
  EXPECT_EQ(pose_graph.optimization().num_solves(), 4);

  pose_graph.Finish();
  EXPECT_EQ(pose_graph.map_manager()->num_checkpoints_written(), 3) << "Finish always writes";
}

// Nothing of another session to match against: every request is accepted, searches nothing and
// moves nothing, before and after the first keyframe alike.
TEST(PoseGraphTest, RelocalizationRequestsAreNeverRejectedAndMoveNothingOnTheirOwn) {
  PoseGraph pose_graph;
  const Eigen::Affine2d seed = transform::FromXYTheta(50.0, 40.0, 1.0);
  pose_graph.Start(TestTime(0), seed);
  const SessionId session = *pose_graph.session_manager().fed_session();
  const Eigen::Affine2d hint = transform::FromXYTheta(4.0, -1.5, 0.6);
  pose_graph.SetInitialPose(hint);
  pose_graph.RelocalizeGlobally();
  pose_graph.WaitUntilQuiescent();
  ExpectPoseNear(pose_graph.graph().session(session).local_to_global, seed);

  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());
  const Eigen::Affine2d node_local = transform::FromXYTheta(1.1, 0.5, 0.2);
  pose_graph.AddInsertionResult(MakeInsertionResult(0, node_local, {submap}));
  pose_graph.WaitUntilQuiescent();
  for (int repeat = 0; repeat < 2; ++repeat) {
    pose_graph.SetInitialPose(hint);
    pose_graph.RelocalizeGlobally();
    pose_graph.WaitUntilQuiescent();
  }
  ExpectPoseNear(pose_graph.graph().session(session).local_to_global, seed);
  ExpectPoseNear(pose_graph.graph().node(NodeId{session.session_index, 0}).global_pose,
                 seed * node_local);
  EXPECT_EQ(pose_graph.constraint_builder().num_matches_attempted(), 0);
  EXPECT_EQ(pose_graph.constraint_builder().num_constraints_added(), 0);
}

// The frontend's window can still name a submap the trimmer already deleted; the stale
// translation entry must be retired, not handed to AddNode.
TEST(PoseGraphTest, AKeyframeNamingATrimmedSubmapSkipsIt) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId fed = *pose_graph.session_manager().fed_session();
  const auto old_submap =
      std::make_shared<mapping::Submap>(0, Eigen::Affine2d::Identity(), kResolution);
  FeedKeyframe(pose_graph, 0, old_submap);
  pose_graph.WaitUntilQuiescent();
  const SubmapId old_id{fed.session_index, 0};
  ASSERT_TRUE(pose_graph.graph().HasSubmap(old_id));

  old_submap->Finish();
  PoseGraphTrimmer trimmer(pose_graph);
  trimmer.EnqueueForTrim(old_id);
  trimmer.TrimOnce();
  pose_graph.WaitUntilQuiescent();
  ASSERT_FALSE(pose_graph.graph().HasSubmap(old_id));

  const Eigen::Affine2d local_pose = transform::FromXYTheta(0.1, 0.0, 0.0);
  pose_graph.AddInsertionResult(
      MakeInsertionResult(1, local_pose, {old_submap, MakeSubmap(1, local_pose)}));
  pose_graph.WaitUntilQuiescent();
  const NodeId node{fed.session_index, 1};
  ASSERT_TRUE(pose_graph.graph().HasNode(node));
  const std::vector<SubmapId> containing = pose_graph.graph().ContainingSubmapIds(node);
  ASSERT_EQ(containing.size(), 1u);
  EXPECT_EQ(containing.front(), (SubmapId{fed.session_index, 1}));
}

struct FrozenBase {
  SessionId session;
  SubmapId submap;
  Eigen::Affine2d submap_global = Eigen::Affine2d::Identity();
};

FrozenBase AddFrozenBase(PoseGraph& pose_graph, const Eigen::Affine2d& local_to_global,
                         int local_index, bool freeze = true) {
  FrozenBase base;
  base.session = pose_graph.StartNewSession(TestTime(0), local_to_global);
  base.submap = SubmapId{base.session.session_index, 0};
  base.submap_global = local_to_global;
  testing::ExplicitInsertion insertion;
  insertion.node_id = NodeId{base.session.session_index, 0};
  insertion.node.time = TestTime(0);
  insertion.node.local_pose = transform::FromXYTheta(0.1, 0.0, 0.0);
  insertion.insertion_submaps.emplace_back(base.submap,
                                           MakeSubmap(local_index, Eigen::Affine2d::Identity()));
  testing::EnqueueExplicitInsertion(pose_graph, std::move(insertion));
  if (freeze) {
    pose_graph.FreezeSession(base.session);
  }
  pose_graph.WaitUntilQuiescent();
  return base;
}

// The first closure onto frozen truth places the whole floating session rigidly where the edge
// says, before the solve; a floating other endpoint or a later closure moves the alignment only
// through the solve.
TEST(PoseGraphTest, FirstClosureOntoAFrozenSessionReseedsTheFedSessionBeforeSolving) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId fed = *pose_graph.session_manager().fed_session();
  const Eigen::Affine2d submap_local = transform::FromXYTheta(0.3, -0.2, 0.1);
  const Eigen::Affine2d node_local = transform::FromXYTheta(0.5, 0.1, 0.15);
  pose_graph.AddInsertionResult(MakeInsertionResult(0, node_local, {MakeSubmap(0, submap_local)}));
  pose_graph.WaitUntilQuiescent();
  const NodeId fed_node{fed.session_index, 0};
  const SubmapId fed_submap{fed.session_index, 0};

  const FrozenBase floating =
      AddFrozenBase(pose_graph, transform::FromXYTheta(-3.0, 8.0, -0.4), 1, /*freeze=*/false);
  pose_graph.AddConstraint(MakeLoopClosure(floating.submap, fed_node));
  pose_graph.WaitUntilQuiescent();
  ExpectPoseNear(pose_graph.graph().session(fed).local_to_global, Eigen::Affine2d::Identity());

  const FrozenBase base = AddFrozenBase(pose_graph, transform::FromXYTheta(20.0, -7.0, 1.3), 2);
  ASSERT_TRUE(pose_graph.graph().session(base.session).frozen());
  const Constraint closure = MakeLoopClosure(base.submap, fed_node);
  pose_graph.AddConstraint(closure);
  pose_graph.WaitUntilQuiescent();

  const Eigen::Affine2d node_global = Eigen::Affine2d(base.submap_global * closure.relative_pose);
  const Eigen::Affine2d alignment = Eigen::Affine2d(node_global * node_local.inverse());
  const PoseGraphData& graph = pose_graph.graph();
  ExpectPoseNear(graph.session(fed).local_to_global, alignment);
  // Reseeded onto the edge exactly, the solve has nothing left to move.
  EXPECT_LT((graph.node(fed_node).global_pose.matrix() - node_global.matrix()).norm(), 1e-6);
  EXPECT_LT(
      (graph.submap(fed_submap).global_pose.matrix() - (alignment * submap_local).matrix()).norm(),
      1e-6);
  ASSERT_TRUE(pose_graph.ActiveSessionToGlobal().has_value());
  EXPECT_LT(
      (pose_graph.ActiveSessionToGlobal()->matrix() - graph.ComputeSessionToGlobal(fed)->matrix())
          .norm(),
      1e-12);

  Constraint second = MakeLoopClosure(base.submap, fed_node);
  second.relative_pose = transform::FromXYTheta(0.4, 0.2, 0.05);
  pose_graph.AddConstraint(second);
  pose_graph.WaitUntilQuiescent();
  ExpectPoseNear(graph.session(fed).local_to_global, alignment);
}

TEST(PoseGraphTest, AClosureOntoAFrozenSessionDoesNotReseedAFedSessionCarryingAPrior) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId fed = *pose_graph.session_manager().fed_session();
  FeedKeyframe(pose_graph, 0, MakeSubmap(0, Eigen::Affine2d::Identity()));
  Constraint prior;
  prior.type = Constraint::Type::PRIOR;
  prior.from = VariableId::Of(SubmapId{fed.session_index, 0});
  prior.relative_pose = Eigen::Affine2d::Identity();
  prior.sqrt_information = OdometrySqrtInformation(ConstraintWeightOption());
  pose_graph.AddConstraint(prior);
  pose_graph.WaitUntilQuiescent();
  const FrozenBase base = AddFrozenBase(pose_graph, transform::FromXYTheta(20.0, -7.0, 1.3), 1);
  const int solves_before = pose_graph.optimization().num_solves();
  pose_graph.AddConstraint(MakeLoopClosure(base.submap, NodeId{fed.session_index, 0}));
  pose_graph.WaitUntilQuiescent();
  ExpectPoseNear(pose_graph.graph().session(fed).local_to_global, Eigen::Affine2d::Identity());
  EXPECT_EQ(pose_graph.optimization().num_solves(), solves_before) << "no reseed, no solve";
}

// Why the reseed asks the graph: the gauge set only knows the sessions of the last solve, so a
// session fed since then reads as anchored although nothing ties it to anything.
TEST(PoseGraphTest, IsSessionAnchoredToFrozenIsStaleForASessionFedSinceTheLastSolve) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId fed = *pose_graph.session_manager().fed_session();
  const FrozenBase base = AddFrozenBase(pose_graph, Eigen::Affine2d::Identity(), 1);
  ASSERT_TRUE(pose_graph.graph().session(base.session).frozen());
  FeedKeyframe(pose_graph, 0, MakeSubmap(0, Eigen::Affine2d::Identity()));
  pose_graph.WaitUntilQuiescent();
  EXPECT_TRUE(pose_graph.optimization().IsSessionAnchoredToFrozen(fed));
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  EXPECT_FALSE(pose_graph.optimization().IsSessionAnchoredToFrozen(fed));
}

// The batch end solves on the node count alone, and consumes it: a second batch end right
// behind finds the cadence spent.
TEST(PoseGraphTest, SolveAtBatchEndSolvesOnlyWhenTheCadenceIsDue) {
  PoseGraph pose_graph(CadenceOfThree());
  pose_graph.Start(TestTime(0));
  const std::shared_ptr<const mapping::Submap> submap = MakeSubmap(0, Eigen::Affine2d::Identity());
  const auto ingest = [&pose_graph, &submap](int node_index) {
    pose_graph.EnqueueInsertionResult(MakeInsertionResult(
        node_index, transform::FromXYTheta(0.1 * node_index, 0.0, 0.0), {submap}));
  };
  std::vector<bool> due;
  const auto batch_end = [&pose_graph, &due] {
    pose_graph.Enqueue([&pose_graph, &due] { due.push_back(pose_graph.SolveAtBatchEnd()); });
  };

  ingest(0);
  ingest(1);
  batch_end();
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 0);

  ingest(2);
  batch_end();
  batch_end();
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), 1);
  EXPECT_EQ(due, (std::vector<bool>{false, true, false}));
}

// The first closure onto frozen truth is the one closure that solves: the rigid reseed leaves
// the session's own drift uncorrected, and a stationary robot given a hint must see the
// published alignment move without a keyframe. A later closure onto the base adds an edge and
// nothing more.
TEST(PoseGraphTest, FirstAnchoringClosureSolvesOnceAndRepublishesTheAlignment) {
  PoseGraph pose_graph;
  pose_graph.Start(TestTime(0));
  const SessionId fed = *pose_graph.session_manager().fed_session();
  FeedKeyframe(pose_graph, 0, MakeSubmap(0, Eigen::Affine2d::Identity()));
  pose_graph.WaitUntilQuiescent();
  const FrozenBase base = AddFrozenBase(pose_graph, transform::FromXYTheta(20.0, -7.0, 1.3), 1);
  const int solves_before = pose_graph.optimization().num_solves();
  const NodeId fed_node{fed.session_index, 0};

  pose_graph.AddConstraint(MakeLoopClosure(base.submap, fed_node));
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), solves_before + 1);
  ASSERT_TRUE(pose_graph.ActiveSessionToGlobal().has_value());
  EXPECT_LT((pose_graph.ActiveSessionToGlobal()->matrix() -
             pose_graph.graph().ComputeSessionToGlobal(fed)->matrix())
                .norm(),
            1e-12);
  EXPECT_GT(pose_graph.ActiveSessionToGlobal()->translation().norm(), 10.0)
      << "the published alignment moved onto the base without a keyframe";

  Constraint second = MakeLoopClosure(base.submap, fed_node);
  second.relative_pose = transform::FromXYTheta(0.4, 0.2, 0.05);
  pose_graph.AddConstraint(second);
  pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(pose_graph.optimization().num_solves(), solves_before + 1) << "once per session";
}

}  // namespace
}  // namespace evergreenslam::lifelong

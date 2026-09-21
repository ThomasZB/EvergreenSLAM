/**
 * @file optimization_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Incremental maintenance of the resident Problem, freezing, gauge and weighting.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/optimization/optimization.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "common/time.h"
#include "lifelong/optimization/optimization_option.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

SubmapRecord MakeSubmap(const SubmapId& id, const Eigen::Affine2d& global_pose) {
  SubmapRecord record;
  record.id = id;
  record.local_pose = Eigen::Affine2d::Identity();
  record.global_pose = global_pose;
  return record;
}

Node MakeNode(const NodeId& id, const Eigen::Affine2d& global_pose) {
  Node node;
  node.id = id;
  node.constant_data.time = TestTime(id.node_index);
  node.constant_data.local_pose = global_pose;
  node.global_pose = global_pose;
  return node;
}

Constraint MakeRelative(const VariableId& from, const VariableId& to,
                        const Eigen::Affine2d& relative_pose, double weight) {
  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.from = from;
  constraint.to = to;
  constraint.relative_pose = relative_pose;
  constraint.sqrt_information = Eigen::Matrix3d::Identity() * weight;
  return constraint;
}

Constraint MakePrior(const VariableId& from, const Eigen::Affine2d& absolute_pose, double weight) {
  Constraint constraint;
  constraint.type = Constraint::Type::PRIOR;
  constraint.from = from;
  constraint.relative_pose = absolute_pose;
  constraint.sqrt_information = Eigen::Matrix3d::Identity() * weight;
  return constraint;
}

void ExpectPoseNear(const Eigen::Affine2d& actual, const Eigen::Affine2d& expected,
                    double tolerance) {
  EXPECT_NEAR((actual.translation() - expected.translation()).norm(), 0.0, tolerance);
  EXPECT_NEAR(transform::NormalizeAngle(transform::GetYaw(actual) - transform::GetYaw(expected)),
              0.0, tolerance);
}

// A chain of submaps whose stored global poses are deliberately wrong, so a solve has work.
struct ChainGraph {
  PoseGraphData graph;
  SessionId session;
  std::vector<SubmapId> submap_ids;
};

ChainGraph MakeChain(int count, double step) {
  ChainGraph chain;
  chain.session = chain.graph.StartNewSession(TestTime(0));
  for (int i = 0; i < count; ++i) {
    const SubmapId id{chain.session.session_index, i};
    chain.graph.AddSubmap(MakeSubmap(id, transform::FromXYTheta(0.0, 0.0, 0.0)));
    chain.submap_ids.push_back(id);
    if (i > 0) {
      chain.graph.AddConstraint(MakeRelative(VariableId::Of(chain.submap_ids[i - 1]),
                                             VariableId::Of(id),
                                             transform::FromXYTheta(step, 0.0, 0.0), 1e2));
    }
  }
  return chain;
}

TEST(Optimization, IncrementalAddKeepsTheEarlierResiduals) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id{session.session_index, 0};
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity()));

  Optimization optimization;
  optimization.AddVariable(VariableId::Of(submap_id), Eigen::Affine2d::Identity());

  const Eigen::Affine2d first_relative = transform::FromXYTheta(1.0, 0.5, 0.2);
  const NodeId first{session.session_index, 0};
  graph.AddNode(MakeNode(first, Eigen::Affine2d::Identity()), {submap_id});
  optimization.AddVariable(VariableId::Of(first), Eigen::Affine2d::Identity());
  optimization.AddConstraint(
      MakeRelative(VariableId::Of(submap_id), VariableId::Of(first), first_relative, 1e2));

  const Eigen::Affine2d second_relative = transform::FromXYTheta(-0.4, 2.0, -0.6);
  const NodeId second{session.session_index, 1};
  graph.AddNode(MakeNode(second, Eigen::Affine2d::Identity()), {submap_id});
  optimization.AddVariable(VariableId::Of(second), Eigen::Affine2d::Identity());
  optimization.AddConstraint(
      MakeRelative(VariableId::Of(submap_id), VariableId::Of(second), second_relative, 1e2));

  optimization.Optimize(graph);
  EXPECT_EQ(optimization.num_residual_blocks(), 2);
  ExpectPoseNear(graph.node(first).global_pose, first_relative, 1e-8);
  ExpectPoseNear(graph.node(second).global_pose, second_relative, 1e-8);

  // Adding a third variable must not rebuild anything: the first two residuals and their
  // solution have to survive.
  const Eigen::Affine2d third_relative = transform::FromXYTheta(3.0, -3.0, 1.4);
  const NodeId third{session.session_index, 2};
  graph.AddNode(MakeNode(third, Eigen::Affine2d::Identity()), {submap_id});
  optimization.AddVariable(VariableId::Of(third), Eigen::Affine2d::Identity());
  optimization.AddConstraint(
      MakeRelative(VariableId::Of(submap_id), VariableId::Of(third), third_relative, 1e2));
  EXPECT_EQ(optimization.num_residual_blocks(), 3);
  EXPECT_EQ(optimization.num_variables(), 4);

  optimization.Optimize(graph);
  ExpectPoseNear(graph.node(first).global_pose, first_relative, 1e-8);
  ExpectPoseNear(graph.node(second).global_pose, second_relative, 1e-8);
  ExpectPoseNear(graph.node(third).global_pose, third_relative, 1e-8);
}

// The residual is linear in sqrt_information, so two priors settle at the ratio of the squared
// weights.
TEST(Optimization, SqrtInformationDecidesTheCompromise) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id{session.session_index, 0};
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity()));

  Optimization optimization;
  optimization.AddVariable(VariableId::Of(submap_id), Eigen::Affine2d::Identity());
  optimization.AddConstraint(
      MakePrior(VariableId::Of(submap_id), transform::FromXYTheta(0.0, 0.0, 0.0), 1.0));
  optimization.AddConstraint(
      MakePrior(VariableId::Of(submap_id), transform::FromXYTheta(10.0, 0.0, 0.0), 3.0));
  optimization.Optimize(graph);

  EXPECT_FALSE(optimization.IsVariableConstant(VariableId::Of(submap_id)))
      << "a prior anchors the component, so no gauge is needed";
  EXPECT_NEAR(graph.submap(submap_id).global_pose.translation().x(), 10.0 * 9.0 / 10.0, 1e-6);
}

// Inter-submap constraints ride a Huber loss: one wrong closure that slipped the gates must not
// drag the trajectory, and recovered summaries go stale as the graph moves. Odometry stays exact.
TEST(Optimization, ARobustLossKeepsAWrongLoopClosureFromDraggingTheChain) {
  const auto run = [](bool recovered) {
    PoseGraphData graph;
    const SessionId session = graph.StartNewSession(TestTime(0));
    const SubmapId submap_id{session.session_index, 0};
    graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity()));
    const NodeId node_id{session.session_index, 0};
    graph.AddNode(MakeNode(node_id, transform::FromXYTheta(1.0, 0.0, 0.0)), {submap_id});

    Optimization optimization;
    optimization.AddVariable(VariableId::Of(submap_id), Eigen::Affine2d::Identity());
    optimization.AddVariable(VariableId::Of(node_id), transform::FromXYTheta(1.0, 0.0, 0.0));

    Constraint odometry = MakeRelative(VariableId::Of(submap_id), VariableId::Of(node_id),
                                       transform::FromXYTheta(1.0, 0.0, 0.0), 1e2);
    odometry.type = Constraint::Type::INTRA_SUBMAP;
    optimization.AddConstraint(odometry);

    // Same weight, claiming the node 5 m from where the odometry puts it.
    Constraint bogus = MakeRelative(VariableId::Of(submap_id), VariableId::Of(node_id),
                                    transform::FromXYTheta(6.0, 0.0, 0.0), 1e2);
    bogus.recovered = recovered;
    optimization.AddConstraint(bogus);

    optimization.Optimize(graph);
    return graph.node(node_id).global_pose.translation().x();
  };

  // Robust: the bogus closure's pull saturates and the odometry keeps the node near 1.
  EXPECT_LT(std::abs(run(false) - 1.0), 0.2);
  // A trimmer-recovered constraint is robust too: a stale summary must not drag the chain.
  EXPECT_LT(std::abs(run(true) - 1.0), 0.2);
}

TEST(Optimization, APriorPullsAFreeNodeToItsAbsolutePose) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId submap_id{session.session_index, 0};
  graph.AddSubmap(MakeSubmap(submap_id, Eigen::Affine2d::Identity()));
  const NodeId node_id{session.session_index, 0};
  graph.AddNode(MakeNode(node_id, Eigen::Affine2d::Identity()), {submap_id});

  Optimization optimization;
  optimization.AddVariable(VariableId::Of(node_id), transform::FromXYTheta(-5.0, 7.0, 2.0));
  const Eigen::Affine2d absolute_pose = transform::FromXYTheta(3.0, -2.0, 0.7);
  optimization.AddConstraint(MakePrior(VariableId::Of(node_id), absolute_pose, 1e2));
  optimization.Optimize(graph);

  ExpectPoseNear(graph.node(node_id).global_pose, absolute_pose, 1e-8);
}

TEST(Optimization, AnUnanchoredComponentGetsExactlyOneSubmapDatum) {
  ChainGraph chain = MakeChain(4, 1.0);
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.Optimize(chain.graph);

  EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(chain.submap_ids.front())));
  for (size_t i = 1; i < chain.submap_ids.size(); ++i) {
    EXPECT_FALSE(optimization.IsVariableConstant(VariableId::Of(chain.submap_ids[i])));
    ExpectPoseNear(chain.graph.submap(chain.submap_ids[i]).global_pose,
                   transform::FromXYTheta(static_cast<double>(i), 0.0, 0.0), 1e-8);
  }
}

// Two sessions with nothing joining them are two components: one datum each, or the second is
// rank deficient.
TEST(Optimization, EachDisconnectedComponentGetsItsOwnDatum) {
  PoseGraphData graph;
  const SessionId first = graph.StartNewSession(TestTime(0));
  const SessionId second = graph.StartNewSession(TestTime(1));
  const SubmapId first_submap{first.session_index, 0};
  const SubmapId second_submap{second.session_index, 0};
  graph.AddSubmap(MakeSubmap(first_submap, Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(second_submap, transform::FromXYTheta(20.0, 0.0, 0.0)));

  Optimization optimization;
  optimization.BuildFrom(graph);
  optimization.Optimize(graph);

  EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(first_submap)));
  EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(second_submap)));
}

TEST(Optimization, FrozenInternalConstraintsNeverEnterTheProblem) {
  PoseGraphData graph;
  const SessionId frozen = graph.StartNewSession(TestTime(0));
  const SubmapId frozen_a{frozen.session_index, 0};
  const SubmapId frozen_b{frozen.session_index, 1};
  graph.AddSubmap(MakeSubmap(frozen_a, Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(frozen_b, transform::FromXYTheta(1.0, 0.0, 0.0)));
  graph.AddConstraint(MakeRelative(VariableId::Of(frozen_a), VariableId::Of(frozen_b),
                                   transform::FromXYTheta(1.0, 0.0, 0.0), 1e2));

  const SessionId active = graph.StartNewSession(TestTime(1));
  const SubmapId active_submap{active.session_index, 0};
  graph.AddSubmap(MakeSubmap(active_submap, transform::FromXYTheta(5.0, 5.0, 0.0)));
  graph.AddConstraint(MakeRelative(VariableId::Of(frozen_b), VariableId::Of(active_submap),
                                   transform::FromXYTheta(2.0, 0.0, 0.0), 1e2));
  graph.FreezeSession(frozen);

  Optimization optimization;
  optimization.BuildFrom(graph);
  EXPECT_EQ(optimization.num_residual_blocks(), 1)
      << "only the constraint that crosses out of the frozen session";
  EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(frozen_a)));
  EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(frozen_b)));
  EXPECT_FALSE(optimization.IsVariableConstant(VariableId::Of(active_submap)));

  optimization.Optimize(graph);
  ExpectPoseNear(graph.submap(active_submap).global_pose, transform::FromXYTheta(3.0, 0.0, 0.0),
                 1e-8);
}

// Freezing has to take the session's own residuals back out of the Problem, not just pin the
// blocks.
TEST(Optimization, FreezingRemovesTheResidualsThatWereAlreadyThere) {
  ChainGraph chain = MakeChain(3, 1.0);
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  EXPECT_EQ(optimization.num_residual_blocks(), 2);

  optimization.Optimize(chain.graph);
  chain.graph.FreezeSession(chain.session);
  optimization.FreezeSession(chain.session);
  EXPECT_EQ(optimization.num_residual_blocks(), 0);
  EXPECT_TRUE(optimization.IsSessionFrozen(chain.session));
  for (const SubmapId& id : chain.submap_ids) {
    EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(id)));
  }
}

// The datum we picked is not truth, so it has to step aside once real truth shows up in the same
// component.
TEST(Optimization, TheDatumIsReleasedOnceTheComponentReachesFrozenTruth) {
  PoseGraphData graph;
  const SessionId frozen = graph.StartNewSession(TestTime(0));
  const SubmapId anchor{frozen.session_index, 0};
  graph.AddSubmap(MakeSubmap(anchor, transform::FromXYTheta(100.0, 0.0, 0.0)));
  graph.FreezeSession(frozen);

  const SessionId active = graph.StartNewSession(TestTime(1));
  const SubmapId drifted{active.session_index, 0};
  graph.AddSubmap(MakeSubmap(drifted, Eigen::Affine2d::Identity()));

  Optimization optimization;
  optimization.BuildFrom(graph);
  optimization.Optimize(graph);
  EXPECT_TRUE(optimization.IsVariableConstant(VariableId::Of(drifted)))
      << "on its own the active session is a component with no datum";

  const Constraint closure = MakeRelative(VariableId::Of(anchor), VariableId::Of(drifted),
                                          transform::FromXYTheta(1.0, 0.0, 0.0), 1e2);
  graph.AddConstraint(closure);
  optimization.AddConstraint(closure);
  optimization.Optimize(graph);

  EXPECT_FALSE(optimization.IsVariableConstant(VariableId::Of(drifted)));
  ExpectPoseNear(graph.submap(drifted).global_pose, transform::FromXYTheta(101.0, 0.0, 0.0), 1e-8);
}

// Four in a chain, not three, so one residual has to survive: with three, removing the middle
// removes everything and the test could not tell a correct purge from one that dropped the lot.
TEST(Optimization, RemovingAVariableTakesItsResidualsAndOnlyThose) {
  ChainGraph chain = MakeChain(4, 1.0);
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  EXPECT_EQ(optimization.num_residual_blocks(), 3);

  optimization.RemoveVariable(VariableId::Of(chain.submap_ids[1]));
  EXPECT_EQ(optimization.num_variables(), 3);
  EXPECT_EQ(optimization.num_residual_blocks(), 1) << "the edge between the survivors stays";
  EXPECT_FALSE(optimization.HasVariable(VariableId::Of(chain.submap_ids[1])));

  TrimRequest request;
  request.deleted_submap_ids = {chain.submap_ids[1]};
  chain.graph.ApplyTrim(request);
  optimization.Optimize(chain.graph);

  optimization.RemoveVariable(VariableId::Of(chain.submap_ids[0]));
  EXPECT_EQ(optimization.num_variables(), 2);
  EXPECT_EQ(optimization.num_residual_blocks(), 1);

  TrimRequest second_request;
  second_request.deleted_submap_ids = {chain.submap_ids[0]};
  chain.graph.ApplyTrim(second_request);
  optimization.Optimize(chain.graph);
  EXPECT_EQ(optimization.num_variables(), 2);
}

TEST(Optimization, BuildFromReproducesTheIncrementalProblem) {
  ChainGraph chain = MakeChain(5, 0.7);
  Optimization incremental;
  for (const SubmapId& id : chain.submap_ids) {
    incremental.AddVariable(VariableId::Of(id), chain.graph.submap(id).global_pose);
  }
  for (const Constraint& constraint : chain.graph.constraints()) {
    incremental.AddConstraint(constraint);
  }
  incremental.Optimize(chain.graph);

  PoseGraphData rebuilt_graph = chain.graph;
  Optimization rebuilt;
  rebuilt.BuildFrom(rebuilt_graph);
  rebuilt.Optimize(rebuilt_graph);

  EXPECT_EQ(incremental.num_residual_blocks(), rebuilt.num_residual_blocks());
  for (const SubmapId& id : chain.submap_ids) {
    ExpectPoseNear(rebuilt_graph.submap(id).global_pose, chain.graph.submap(id).global_pose, 1e-9);
  }
}

// The freeze rotation's handover: the parameter block must not move (Ceres holds its address for
// the variable's lifetime), the residuals must survive, and a solve must behave as before.
TEST(Optimization, RenameVariableKeepsTheBlockAddressAndItsResiduals) {
  ChainGraph chain = MakeChain(4, 1.0);
  chain.graph.AddConstraint(
      MakePrior(VariableId::Of(chain.submap_ids.front()), Eigen::Affine2d::Identity(), 1e2));

  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  const int residuals_before = optimization.num_residual_blocks();

  const VariableId old_id = VariableId::Of(chain.submap_ids.back());
  const double* address_before = optimization.mutable_parameter_block(old_id);
  const SessionId successor = chain.graph.StartNewSession(TestTime(10));
  const VariableId new_id = VariableId::Of(SubmapId{successor.session_index, 0});
  optimization.RenameVariable(old_id, new_id);

  EXPECT_FALSE(optimization.HasVariable(old_id));
  ASSERT_TRUE(optimization.HasVariable(new_id));
  EXPECT_EQ(optimization.mutable_parameter_block(new_id), address_before);
  EXPECT_EQ(optimization.num_residual_blocks(), residuals_before);

  // The graph must agree on the id before the write back of the solve.
  const SubmapId new_submap_id = new_id.submap_id();
  chain.graph.TransferSubmap(chain.submap_ids.back(), new_submap_id);
  optimization.Optimize(chain.graph);
  ExpectPoseNear(chain.graph.submap(new_submap_id).global_pose,
                 transform::FromXYTheta(3.0, 0.0, 0.0), 1e-8);
}

TEST(OptimizationDeathTest, RenameVariableRejectsConstantAndDuplicateIds) {
  ChainGraph chain = MakeChain(3, 1.0);
  chain.graph.AddConstraint(
      MakePrior(VariableId::Of(chain.submap_ids.front()), Eigen::Affine2d::Identity(), 1e2));
  Optimization optimization;
  optimization.BuildFrom(chain.graph);
  optimization.FreezeSession(chain.session);

  const VariableId frozen = VariableId::Of(chain.submap_ids.front());
  const VariableId free_id{VariableId::Kind::SUBMAP, chain.session.session_index + 1, 0};
  EXPECT_DEATH(optimization.RenameVariable(frozen, free_id), "constant");
  EXPECT_DEATH(optimization.RenameVariable(free_id, frozen), "already exists|never added");
}

// One outlier closure against the configured Huber: past the knee the pull stops growing, so
// where the solve lands reads the knee back out.
double SolveOutlierPull(double huber_distance) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const SubmapId anchor{session.session_index, 0};
  const SubmapId pulled{session.session_index, 1};
  graph.AddSubmap(MakeSubmap(anchor, Eigen::Affine2d::Identity()));
  graph.AddSubmap(MakeSubmap(pulled, Eigen::Affine2d::Identity()));

  ConstraintWeightOption weight;
  weight.loop_closure_huber_distance = huber_distance;
  Optimization optimization{OptimizationOption(), weight};
  optimization.AddVariable(VariableId::Of(anchor), Eigen::Affine2d::Identity());
  optimization.AddVariable(VariableId::Of(pulled), Eigen::Affine2d::Identity());
  optimization.AddConstraint(MakePrior(VariableId::Of(anchor), Eigen::Affine2d::Identity(), 1e6));
  optimization.AddConstraint(MakePrior(VariableId::Of(pulled), Eigen::Affine2d::Identity(), 1.0));
  optimization.AddConstraint(MakeRelative(VariableId::Of(anchor), VariableId::Of(pulled),
                                          transform::FromXYTheta(10.0, 0.0, 0.0), 1e2));
  optimization.Optimize(graph);
  return graph.submap(pulled).global_pose.translation().x();
}

TEST(Optimization, TheLoopClosureLossKneeIsConfigured) {
  EXPECT_NEAR(SolveOutlierPull(0.1), 10.0, 0.01);
  EXPECT_NEAR(SolveOutlierPull(0.0005), 5.0, 0.01);
}

}  // namespace
}  // namespace evergreenslam::lifelong

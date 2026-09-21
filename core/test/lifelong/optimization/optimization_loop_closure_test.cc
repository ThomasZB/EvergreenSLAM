/**
 * @file optimization_loop_closure_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief End to end: a simulated loop with odometry drift, closed by one handcrafted constraint.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "common/time.h"
#include "lifelong/optimization/optimization.h"
#include "lifelong/optimization/optimization_option.h"
#include "lifelong/pose_graph.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr int kNumNodes = 80;
constexpr int kNodesPerSubmap = 10;
constexpr double kRadius = 5.0;
constexpr double kYawBiasPerStep = 0.004;
constexpr double kResolution = 0.05;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// A closed circular loop starting at the origin heading along +x; nothing is rasterized here.
Eigen::Affine2d GroundTruth(int index) {
  const double phi = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(kNumNodes);
  return transform::FromXYTheta(kRadius * std::sin(phi), kRadius * (1.0 - std::cos(phi)), phi);
}

// A constant yaw bias per step, which is what an uncalibrated laser odometry does.
std::vector<Eigen::Affine2d> SimulateOdometry() {
  std::vector<Eigen::Affine2d> poses;
  poses.push_back(Eigen::Affine2d::Identity());
  for (int i = 1; i < kNumNodes; ++i) {
    const Eigen::Affine2d increment = GroundTruth(i - 1).inverse() * GroundTruth(i);
    poses.push_back(Eigen::Affine2d(poses.back() * increment *
                                    transform::FromXYTheta(0.0, 0.0, kYawBiasPerStep)));
  }
  return poses;
}

class SessionSimulator {
 public:
  SessionSimulator(PoseGraph& pose_graph, SessionId session)
      : pose_graph_(pose_graph), session_(session) {}

  void FeedNode(int index, const Eigen::Affine2d& local_pose) {
    const int newest = index / kNodesPerSubmap;
    for (int k = std::max(0, newest - 1); k <= newest; ++k) {
      if (static_cast<int>(submaps_.size()) <= k) {
        submaps_.push_back(std::make_shared<const mapping::Submap>(k, local_pose, kResolution));
      }
    }

    testing::ExplicitInsertion insertion;
    insertion.node_id = NodeId{session_.session_index, index};
    insertion.node.time = TestTime(index);
    insertion.node.local_pose = local_pose;
    // Every node lands in the two open submaps, which is the only thing that chains one submap
    // to the next: without the overlap the graph is a forest of stars.
    for (int k = std::max(0, newest - 1); k <= newest; ++k) {
      insertion.insertion_submaps.emplace_back(SubmapId{session_.session_index, k}, submaps_[k]);
    }
    testing::EnqueueExplicitInsertion(pose_graph_, std::move(insertion));
  }

 private:
  PoseGraph& pose_graph_;
  SessionId session_;
  std::vector<std::shared_ptr<const mapping::Submap>> submaps_;
};

double NodeError(const PoseGraphData& graph, const NodeId& id) {
  return (graph.node(id).global_pose.translation() - GroundTruth(id.node_index).translation())
      .norm();
}

double MeanNodeError(const PoseGraphData& graph, SessionId session) {
  double total = 0.0;
  int count = 0;
  for (const NodeId& id : graph.session(session).node_ids) {
    total += (graph.node(id).global_pose.translation() - GroundTruth(id.node_index).translation())
                 .norm();
    ++count;
  }
  return count == 0 ? 0.0 : total / static_cast<double>(count);
}

int CountConstantVariables(const PoseGraphData& graph, const Optimization& optimization) {
  int count = 0;
  for (const auto& [id, record] : graph.submaps()) {
    count += optimization.IsVariableConstant(VariableId::Of(id)) ? 1 : 0;
  }
  for (const auto& [id, node] : graph.nodes()) {
    count += optimization.IsVariableConstant(VariableId::Of(id)) ? 1 : 0;
  }
  return count;
}

std::vector<std::array<double, 6>> SnapshotSession(const PoseGraphData& graph, SessionId session) {
  std::vector<std::array<double, 6>> poses;
  const auto append = [&poses](const Eigen::Affine2d& pose) {
    poses.push_back({pose.linear()(0, 0), pose.linear()(0, 1), pose.linear()(1, 0),
                     pose.linear()(1, 1), pose.translation().x(), pose.translation().y()});
  };
  for (const SubmapId& id : graph.session(session).submap_ids) {
    append(graph.submap(id).global_pose);
  }
  for (const NodeId& id : graph.session(session).node_ids) {
    append(graph.node(id).global_pose);
  }
  return poses;
}

Constraint MakeLoopClosure(const VariableId& from, const VariableId& to,
                           const Eigen::Affine2d& relative_pose) {
  Constraint constraint;
  constraint.type = Constraint::Type::INTER_SUBMAP;
  constraint.from = from;
  constraint.to = to;
  constraint.relative_pose = relative_pose;
  constraint.sqrt_information = OdometrySqrtInformation(ConstraintWeightOption());
  return constraint;
}

TEST(OptimizationLoopClosure, OneCorrectClosureRemovesMostOfTheAccumulatedDrift) {
  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  SessionSimulator simulator(pose_graph, session);

  const std::vector<Eigen::Affine2d> odometry = SimulateOdometry();
  for (int i = 0; i < kNumNodes; ++i) {
    simulator.FeedNode(i, odometry[i]);
  }
  pose_graph.WaitUntilQuiescent();
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();

  const PoseGraphData& graph = pose_graph.graph();
  ASSERT_EQ(static_cast<int>(graph.session(session).node_ids.size()), kNumNodes);
  EXPECT_EQ(CountConstantVariables(graph, pose_graph.optimization()), 1)
      << "the odometry chain has to be one connected component with one datum";

  const double error_before = MeanNodeError(graph, session);
  const double last_error_before = NodeError(graph, NodeId{session.session_index, kNumNodes - 1});
  EXPECT_GT(error_before, 0.5) << "the simulated drift has to be worth correcting";

  // Submap 0 was created at node 0, so its ground truth is the identity.
  const NodeId last{session.session_index, kNumNodes - 1};
  pose_graph.AddConstraint(MakeLoopClosure(VariableId::Of(SubmapId{session.session_index, 0}),
                                           VariableId::Of(last), GroundTruth(kNumNodes - 1)));
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();

  const double error_after = MeanNodeError(graph, session);
  // A single closure cannot do better than halve a systematic drift: SPA spreads the correction
  // over every edge. Where the closure acts, the error collapses.
  EXPECT_LT(error_after, error_before / 2.0)
      << "before " << error_before << " after " << error_after;
  EXPECT_GT(last_error_before, 1.0);
  EXPECT_LT(NodeError(graph, last), 0.05);
}

// The second run registers into the frozen first one, and the frozen one does not move a bit.
TEST(OptimizationLoopClosure, AFrozenSessionStaysBitwiseWhatItWasWhenItFroze) {
  PoseGraph pose_graph;
  const SessionId first = pose_graph.StartNewSession(TestTime(0));
  SessionSimulator first_simulator(pose_graph, first);

  // error is measured against something exact.
  for (int i = 0; i < kNumNodes; ++i) {
    first_simulator.FeedNode(i, GroundTruth(i));
  }
  pose_graph.WaitUntilQuiescent();
  pose_graph.FreezeSession(first);
  pose_graph.WaitUntilQuiescent();

  const PoseGraphData& graph = pose_graph.graph();
  EXPECT_LT(MeanNodeError(graph, first), 1e-6);
  const std::vector<std::array<double, 6>> frozen_before = SnapshotSession(graph, first);
  ASSERT_FALSE(frozen_before.empty());

  const SessionId second = pose_graph.StartNewSession(TestTime(kNumNodes));
  SessionSimulator second_simulator(pose_graph, second);
  const std::vector<Eigen::Affine2d> odometry = SimulateOdometry();
  for (int i = 0; i < kNumNodes; ++i) {
    second_simulator.FeedNode(i, odometry[i]);
  }
  pose_graph.WaitUntilQuiescent();
  const double error_before = MeanNodeError(graph, second);
  EXPECT_GT(error_before, 0.5);

  // A revisit produces one closure per submap it walks back through, not just one at the end.
  for (int k = 0; k * kNodesPerSubmap < kNumNodes; ++k) {
    const SubmapId frozen_submap{first.session_index, k};
    const int matched_node = k * kNodesPerSubmap + kNodesPerSubmap / 2;
    pose_graph.AddConstraint(MakeLoopClosure(
        VariableId::Of(frozen_submap), VariableId::Of(NodeId{second.session_index, matched_node}),
        GroundTruth(k * kNodesPerSubmap).inverse() * GroundTruth(matched_node)));
  }
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();

  const std::vector<std::array<double, 6>> frozen_after = SnapshotSession(graph, first);
  ASSERT_EQ(frozen_before.size(), frozen_after.size());
  for (size_t i = 0; i < frozen_before.size(); ++i) {
    for (size_t j = 0; j < frozen_before[i].size(); ++j) {
      EXPECT_EQ(frozen_before[i][j], frozen_after[i][j])
          << "frozen pose " << i << " coefficient " << j << " moved";
    }
  }

  const double error_after = MeanNodeError(graph, second);
  EXPECT_LT(error_after, error_before / 5.0)
      << "before " << error_before << " after " << error_after;
  EXPECT_LT(error_after, 0.1);
}

}  // namespace
}  // namespace evergreenslam::lifelong

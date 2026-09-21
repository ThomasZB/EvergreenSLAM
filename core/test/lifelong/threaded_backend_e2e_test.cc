/**
 * @file threaded_backend_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Simulated input through the threaded backend: error metrics, bounded submap counts,
 *        zero false closures, frozen truth untouched, and a clean mid-flight shutdown.
 * @version 0.1
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "testing/loop_scenario.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;
using testing::DriftedOdometry;
using testing::Frontend;
using testing::kNodesPerLap;
using testing::kNodesPerSubmap;
using testing::LoopGroundTruth;
using testing::MeanNodeError;
using testing::TestTime;

// A submap's frame is the odometry pose at its creation step, a node's the pose at its feed step.
// Submaps go through the emulator's creation-step map keyed by local_index, which a freeze
// rotation's graph-id rename cannot disturb.
Eigen::Affine2d TruthPoseOf(const PoseGraphData& graph, const VariableId& variable,
                            const std::map<int, int>& session_starts,
                            const std::map<int, int>& submap_steps) {
  const int step = variable.kind == VariableId::Kind::NODE
                       ? session_starts.at(variable.session_id) + variable.index
                       : submap_steps.at(graph.submap(variable.submap_id()).submap->local_index());
  return LoopGroundTruth(step);
}

bool PoseClose(const Eigen::Affine2d& a, const Eigen::Affine2d& b, double max_translation,
               double max_rotation) {
  const double translation_error = (a.translation() - b.translation()).norm();
  const double rotation_error =
      std::abs(transform::NormalizeAngle(transform::GetYaw(a) - transform::GetYaw(b)));
  return translation_error < max_translation && rotation_error < max_rotation;
}

// The constraint builder's closures carry exactly the configured loop closure weight and always
// bind a submap to a node; the trimmer's recovered summaries carry marginalized information.
bool IsBuilderClosure(const Constraint& constraint) {
  return constraint.type == Constraint::Type::INTER_SUBMAP &&
         constraint.from.kind == VariableId::Kind::SUBMAP && constraint.to.has_value() &&
         constraint.to->kind == VariableId::Kind::NODE &&
         constraint.sqrt_information.isApprox(LoopClosureSqrtInformation(ConstraintWeightOption()));
}

// A wrong loop closure is metres off; a right one differs from truth only by the drift baked
// into one submap's grid (a fraction of these bounds).
bool ClosureConsistentWithTruth(const PoseGraphData& graph, const Constraint& constraint,
                                const std::map<int, int>& session_starts,
                                const std::map<int, int>& submap_steps) {
  const Eigen::Affine2d truth_rel =
      Eigen::Affine2d(TruthPoseOf(graph, constraint.from, session_starts, submap_steps).inverse() *
                      TruthPoseOf(graph, *constraint.to, session_starts, submap_steps));
  return PoseClose(constraint.relative_pose, truth_rel, 0.35, 0.2);
}

Eigen::Affine2d EstimatePoseOf(const PoseGraphData& graph, const VariableId& variable) {
  return variable.kind == VariableId::Kind::NODE ? graph.node(variable.node_id()).global_pose
                                                 : graph.submap(variable.submap_id()).global_pose;
}

// Recovered summaries and priors are linearized at the estimate, so absolute truth is the
// wrong yardstick for them; instead they must not contradict the graph they summarize.
bool SummaryConsistentWithEstimate(const Constraint& constraint, const PoseGraphData& graph) {
  if (!constraint.to.has_value()) {
    return PoseClose(constraint.relative_pose, EstimatePoseOf(graph, constraint.from), 0.5, 0.25);
  }
  const Eigen::Affine2d estimate_rel = Eigen::Affine2d(
      EstimatePoseOf(graph, constraint.from).inverse() * EstimatePoseOf(graph, *constraint.to));
  return PoseClose(constraint.relative_pose, estimate_rel, 0.5, 0.25);
}

struct ScenarioResult {
  int first_session_index = 0;
  int sessions_frozen = 0;
  double mean_second_session_error = 0.0;
  int constraints_added = 0;
  int constraints_dropped = 0;
  int submaps_created = 0;
  int submaps_surviving = 0;
  int submaps_trimmed = 0;
  int steps_fed = 0;
  int nodes_ingested = 0;
  int bad_closures = 0;
  int bad_summaries = 0;
  double max_frozen_move = 0.0;
};

// The boot session freezes at its first finished submap (bootstrap); the rest of the drifted lap,
// the closure on the revisit and two more laps over the same circle land in the successor, which
// never maps anything new and so stays open while trimming engages.
void RunScenario(ScenarioResult& result) {
  const std::string config_path = std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml";
  PoseGraphOption option = LoadPoseGraphOptionFromFile(config_path);
  option.constraint_builder.sampler_option.sampling_ratio = 0.25;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;

  const int max_steps = 4 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(max_steps);

  PoseGraph backend(option);

  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  const SessionId first_session = *backend.session_manager().fed_session();
  result.first_session_index = first_session.session_index;

  int step = 0;
  // Paced like a real frontend: the backend keeps up with the feed. Flat out, the emulator
  // floods the queue and matches queue up behind dozens of keyframes, so the trim after each
  // solve deletes what they name. The worker-vs-backend interleaving is still live within a step.
  const auto feed = [&] {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  };

  while (backend.session_manager().num_sessions_frozen() == 0 && step < kNodesPerLap) {
    feed();
  }
  while (step < kNodesPerLap + 30) {
    feed();
  }
  backend.WaitUntilQuiescent();
  std::ostringstream verdict_detail;
  for (const SubmapUncertainty& submap : backend.session_manager().last_verdict().submaps) {
    verdict_detail << " [" << submap.id.submap_index << (submap.passed ? " ok " : " BAD ")
                   << submap.stddev.transpose() << "]";
  }
  ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
      << "the run never froze; last verdict: "
      << ToString(backend.session_manager().last_verdict().rejection) << "; attempted "
      << backend.constraint_builder().num_matches_attempted() << ", added "
      << backend.constraint_builder().num_constraints_added() << ", at step " << step
      << verdict_detail.str();
  ASSERT_TRUE(backend.graph().session(first_session).frozen());
  // Quiescent, so the consumer-owned fed session is safe to read from here.
  const SessionId second_session = *backend.session_manager().fed_session();
  ASSERT_EQ(second_session.session_index, first_session.session_index + 1);

  std::vector<SubmapId> frozen_ids = backend.graph().session(first_session).submap_ids;
  std::vector<Eigen::Vector2d> frozen_translations;
  for (const SubmapId& id : frozen_ids) {
    frozen_translations.push_back(backend.graph().submap(id).global_pose.translation());
  }

  const int stage2_end = step + 2 * kNodesPerLap;
  while (step < stage2_end) {
    feed();
  }
  backend.Finish();

  const PoseGraphData& graph = backend.graph();
  // Which session each keyframe landed in can jitter with the rotation's task timing; the total
  // minted must equal the feeds either way.
  const std::map<int, int> session_starts = testing::SessionStartSteps(graph);
  result.sessions_frozen = backend.session_manager().num_sessions_frozen();
  // The drifted lap and its closure live in the successor; the frozen session is bitwise
  // constant and says nothing about recovery.
  result.mean_second_session_error =
      MeanNodeError(graph, second_session, session_starts.at(second_session.session_index));
  result.constraints_added = backend.constraint_builder().num_constraints_added();
  result.constraints_dropped = backend.constraint_builder().num_constraints_dropped();
  result.submaps_created = graph.id_allocator().next_submap_index(second_session);
  result.submaps_surviving = static_cast<int>(graph.session(second_session).submap_ids.size());
  result.submaps_trimmed = backend.trimmer().num_submaps_trimmed();
  result.steps_fed = step;
  for (const auto& [id, session] : graph.sessions()) {
    result.nodes_ingested += graph.id_allocator().next_node_index(id);
  }

  for (size_t i = 0; i < frozen_ids.size(); ++i) {
    result.max_frozen_move = std::max(
        result.max_frozen_move,
        (graph.submap(frozen_ids[i]).global_pose.translation() - frozen_translations[i]).norm());
  }
  for (const Constraint& constraint : graph.constraints()) {
    if (constraint.type == Constraint::Type::INTRA_SUBMAP) {
      continue;
    }
    if (IsBuilderClosure(constraint)) {
      if (!ClosureConsistentWithTruth(graph, constraint, session_starts,
                                      frontend.submap_creation_steps())) {
        ++result.bad_closures;
      }
    } else if (!SummaryConsistentWithEstimate(constraint, graph)) {
      ++result.bad_summaries;
    }
  }
}

TEST(ThreadedBackendE2eTest, RecoversTheTrajectoryAndBoundsTheGraph) {
  ScenarioResult run;
  RunScenario(run);
  if (::testing::Test::HasFatalFailure()) {
    return;
  }

  std::cout << "threaded: mean error " << run.mean_second_session_error << " m, constraints "
            << run.constraints_added << " (" << run.bad_closures << " bad closures, "
            << run.bad_summaries << " bad summaries, " << run.constraints_dropped
            << " dropped), submaps " << run.submaps_surviving << "/" << run.submaps_created
            << " (trimmed " << run.submaps_trimmed << "), frozen moved " << run.max_frozen_move
            << " m, nodes " << run.nodes_ingested << "/" << run.steps_fed << std::endl;

  EXPECT_EQ(run.sessions_frozen, 1);
  EXPECT_LT(run.mean_second_session_error, 0.2);

  EXPECT_GT(run.constraints_added, 0);
  EXPECT_EQ(run.constraints_dropped, 0) << "a trim deleted the endpoint of a match in flight";
  EXPECT_EQ(run.bad_closures, 0);
  // Recovered summaries ride a Huber loss, so a stale one gets suppressed instead of pulled into
  // agreement. Bounded, not zero.
  EXPECT_LE(run.bad_summaries, 2);

  EXPECT_GT(run.submaps_trimmed, 0);
  const PoseGraphOption defaults;
  // The two actives, the newest finished ones that never go, the ones behind them that lack
  // enough coverers yet, and up to one trim cadence of submaps minted since the last trim.
  const int submap_bound = 2 + defaults.trimmer.selector.keep_newest_submaps +
                           defaults.trimmer.selector.min_covering_newer_submaps +
                           defaults.optimization.optimize_every_n_nodes *
                               defaults.trim_every_n_optimizations / kNodesPerSubmap;
  EXPECT_GT(run.submaps_created, run.submaps_surviving);
  EXPECT_LE(run.submaps_surviving, submap_bound);

  EXPECT_LT(run.max_frozen_move, 1e-12) << "frozen means frozen, to the bit";

  // Every keyframe fed lands in the graph; only which session a rotation-boundary keyframe lands
  // in may differ between runs.
  EXPECT_EQ(run.nodes_ingested, run.steps_fed);
}

// Shutdown safety: a backend destroyed mid-flight must quiesce and join without touching
// destroyed members.
TEST(ThreadedBackendE2eTest, DestructsCleanlyWithoutFinish) {
  const std::string config_path = std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml";
  const PoseGraphOption option = LoadPoseGraphOptionFromFile(config_path);

  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(60);
  PoseGraph backend(option);
  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  for (int step = 0; step < 60; ++step) {
    frontend.Feed(step);
  }
  // Destructor runs with searches likely still queued.
}

}  // namespace
}  // namespace evergreenslam::lifelong

/**
 * @file rotation_continuity_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The freeze rotation must be seamless: the successor session inherits the unfinished
 *        submaps, the frontend keeps inserting into them, and the recovered trajectory shows no
 *        error step across the boundary (the young-map plateau this handover exists to remove).
 * @version 0.1
 * @date 2026-08-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "testing/loop_scenario.h"

namespace evergreenslam::lifelong {
namespace {

using testing::DriftedOdometry;
using testing::Frontend;
using testing::kNodesPerLap;
using testing::kNodesPerSubmap;
using testing::LoopGroundTruth;
using testing::TestTime;

double StepError(const PoseGraphData& graph, const NodeId& id, int step) {
  return (graph.node(id).global_pose.translation() - LoopGroundTruth(step).translation()).norm();
}

// One drifted lap, closure on the revisit, then the freeze rotation -- and the drive goes on.
// One match worker: the per-step error comparison needs the same closures every run.
TEST(RotationContinuityE2eTest, HandedOverSubmapsKeepAbsorbingAndTheErrorShowsNoStep) {
  const std::string config_path = std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml";
  PoseGraphOption option = LoadPoseGraphOptionFromFile(config_path);
  option.constraint_builder.num_match_workers = 1;
  // Full search budget: this test measures the rotation's effect on the recovered trajectory.
  option.constraint_builder.sampler_option.sampling_ratio = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 8;
  // Off on purpose: the crossing-window nodes must all survive for the per-step comparison.
  option.trim = false;

  const int max_steps = 3 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(max_steps);

  // The freeze is placed by hand right after the closure: the story is the rotation itself, and
  // the bootstrap rule would otherwise rotate at the first finished submap, before any drift.
  option.session_manager.auto_freeze = false;
  PoseGraph backend(option);

  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  const SessionId first_session = *backend.session_manager().fed_session();

  int step = 0;
  // Paced like a real frontend: flat out, where the between-batch solves fall against the feed
  // varies run to run, and so does the per-step error this test compares.
  for (; step < kNodesPerLap + 30; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0);

  // Quiescent before and after: the rotation has to complete before the next keyframe, or the
  // step-to-node mapping below no longer holds.
  backend.FreezeFedSession();
  backend.WaitUntilQuiescent();
  ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
  const PoseGraphData& graph = backend.graph();
  ASSERT_TRUE(graph.session(first_session).frozen());

  // The node fed at the rotation step still landed in the frozen session, so the successor's node
  // index n maps to ground-truth step rotation_step + 1 + n.
  const int rotation_step = step - 1;
  const SessionId second_session = *backend.session_manager().fed_session();
  ASSERT_EQ(second_session.session_index, first_session.session_index + 1);

  // The frozen session keeps only finished submaps; the successor owns the transferred actives.
  for (const SubmapId& id : graph.session(first_session).submap_ids) {
    EXPECT_TRUE(graph.submap(id).submap->finished());
  }
  const std::vector<SubmapId> transferred = graph.session(second_session).submap_ids;
  ASSERT_FALSE(transferred.empty()) << "the rotation transferred nothing";
  for (const SubmapId& id : transferred) {
    EXPECT_EQ(id.session_id, second_session.session_index);
    EXPECT_FALSE(graph.submap(id).submap->finished());
  }
  const SubmapId adopted_newest = transferred.back();
  const int scans_at_rotation = graph.submap(adopted_newest).submap->num_scans();

  const int crossing_window = 3 * kNodesPerSubmap;
  for (int i = 0; i < crossing_window; ++i) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  backend.Finish();

  // The adopted submaps finished in place, and the newest one kept absorbing the successor's
  // nodes. The older one may slide out of the window on the next feed, so only finishing is
  // guaranteed for it.
  for (const SubmapId& id : transferred) {
    EXPECT_TRUE(graph.submap(id).submap->finished()) << "adopted submap " << id.submap_index;
  }
  const SubmapRecord& newest_record = graph.submap(adopted_newest);
  EXPECT_GT(newest_record.submap->num_scans(), scans_at_rotation);
  const bool absorbed_successor_nodes =
      std::any_of(newest_record.node_ids.begin(), newest_record.node_ids.end(),
                  [&second_session](const NodeId& node_id) {
                    return node_id.session_id == second_session.session_index;
                  });
  EXPECT_TRUE(absorbed_successor_nodes)
      << "the newest adopted submap absorbed no node after the rotation";

  // No young-map plateau: the truth-relative error must be continuous across the boundary. The
  // error field varies along the ring, so the windows hug the boundary instead of averaging.
  const auto error_at = [&graph, rotation_step, &first_session, &second_session](int s) {
    const NodeId id = s <= rotation_step
                          ? NodeId{first_session.session_index, s}
                          : NodeId{second_session.session_index, s - rotation_step - 1};
    return StepError(graph, id, s);
  };
  constexpr int kErrorWindow = 10;
  double mean_before = 0.0;
  double max_before = 0.0;
  for (int s = rotation_step - kErrorWindow + 1; s <= rotation_step; ++s) {
    mean_before += error_at(s);
    max_before = std::max(max_before, error_at(s));
  }
  mean_before /= kErrorWindow;
  double mean_after = 0.0;
  double max_after = 0.0;
  for (int s = rotation_step + 1; s <= rotation_step + kErrorWindow; ++s) {
    mean_after += error_at(s);
    max_after = std::max(max_after, error_at(s));
  }
  mean_after /= kErrorWindow;

  // The single-step jump at the boundary: a cold-restart plateau shows up here first.
  EXPECT_LT(std::abs(error_at(rotation_step + 1) - error_at(rotation_step)), 0.03)
      << "error step at the boundary: " << error_at(rotation_step) << " -> "
      << error_at(rotation_step + 1);
  EXPECT_LT(mean_after, mean_before + 0.08) << "mean error stepped across the rotation: before "
                                            << mean_before << ", after " << mean_after;
  EXPECT_LT(max_after, max_before + 0.08)
      << "max error stepped across the rotation: before " << max_before << ", after " << max_after;
  // Absolute sanity: both windows sit at the closed-loop error level of this drift setting.
  EXPECT_LT(mean_before, 0.35);
  EXPECT_LT(mean_after, 0.35);
}

}  // namespace
}  // namespace evergreenslam::lifelong

/**
 * @file multi_boot_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Floating sessions across many boots: the GC sweep for the abandoned ones, the late
 *        freeze for the one a cross-session loop closure anchors, relocalization of a wrongly
 *        seeded boot, and the last-pose seed.
 * @version 0.1
 * @date 2026-08-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/map_manager/map_manager.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "testing/load_map.h"
#include "testing/loop_scenario.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using testing::Frontend;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::TestTime;

// Drift-free local frames: these stories are about session lifecycle across boots.
std::vector<Eigen::Affine2d> GroundTruthOdometry(int num_steps) {
  std::vector<Eigen::Affine2d> poses;
  poses.reserve(num_steps);
  for (int i = 0; i < num_steps; ++i) {
    poses.push_back(LoopGroundTruth(i));
  }
  return poses;
}

// Relaxed judge and a small solve window so judging and checkpointing get their cadence within a
// few dozen nodes; the slack cap is shrunk to keep loaded floating sessions' windows affordable.
// One match worker: these stories assert which closures exist, so the match order must not vary.
PoseGraphOption RelaxedOption() {
  PoseGraphOption option;
  option.constraint_builder.num_match_workers = 1;
  option.optimization.optimize_every_n_nodes = 10;
  option.checkpoint_min_interval = common::Duration::zero();
  option.session_manager.freeze_judge.max_translation_stddev = 1.0;
  option.session_manager.freeze_judge.max_rotation_stddev = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  option.constraint_builder.max_candidate_slack = 5.0;
  option.trim = false;
  return option;
}

// A boot whose session is under test as a floating one, so it must never freeze itself.
PoseGraphOption NoFreeze(PoseGraphOption option) {
  option.session_manager.auto_freeze = false;
  return option;
}

SessionId FreezeABaseSession(const std::string& directory, const PoseGraphOption& option,
                             const std::vector<Eigen::Affine2d>& odometry) {
  PoseGraph backend(option, directory);
  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  const SessionId frozen_session = *backend.session_manager().fed_session();
  int step = 0;
  while (backend.session_manager().num_sessions_frozen() == 0 &&
         step < static_cast<int>(odometry.size())) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  EXPECT_EQ(backend.session_manager().num_sessions_frozen(), 1)
      << "the first session never froze; last verdict: "
      << ToString(backend.session_manager().last_verdict().rejection);
  return frozen_session;
}

double TranslationError(const Eigen::Affine2d& pose, int step) {
  return (pose.translation() - LoopGroundTruth(step).translation()).norm();
}

double RotationError(const Eigen::Affine2d& pose, int step) {
  return std::abs(transform::NormalizeAngle(transform::GetYaw(pose) -
                                            transform::GetYaw(LoopGroundTruth(step))));
}

int CountCrossSessionConstraints(const PoseGraphData& graph, SessionId session) {
  int count = 0;
  for (const Constraint& constraint : graph.constraints()) {
    if (!constraint.to.has_value()) {
      continue;
    }
    const SessionId from = constraint.from.session();
    const SessionId to = constraint.to->session();
    if (from == to) {
      continue;
    }
    if (from == session || to == session) {
      ++count;
    }
  }
  return count;
}

// A floating session nothing will ever anchor must be swept once it lags the boot count by more
// than gc_unfrozen_after_n_boots, with graph and manifest kept in agreement.
TEST(MultiBootE2eTest, AbandonedFloatingSessionIsGarbageCollectedAfterEnoughBoots) {
  const std::string directory = testing::MakeTempDir("evergreenslam_multi_boot_gc");
  PoseGraphOption option = RelaxedOption();
  option.gc_unfrozen_after_n_boots = 3;
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = GroundTruthOdometry(total_steps);

  SessionId frozen_session;
  std::vector<Eigen::Vector2d> frozen_translations;
  SessionId floating_session;

  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    frozen_session = *backend.session_manager().fed_session();
    int step = 0;
    while (backend.session_manager().num_sessions_frozen() == 0 && step < total_steps) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
        << "the first session never froze; last verdict: "
        << ToString(backend.session_manager().last_verdict().rejection);
    ASSERT_TRUE(backend.graph().session(frozen_session).frozen());
    for (const SubmapId& id : backend.graph().session(frozen_session).submap_ids) {
      frozen_translations.push_back(backend.graph().submap(id).global_pose.translation());
    }
  }

  {
    // A wrong relocalization far outside the base, held unfrozen and killed.
    PoseGraph backend(NoFreeze(option), directory);
    backend.Start(TestTime(total_steps), transform::FromXYTheta(60.0, 45.0, 0.8));
    Frontend frontend(backend, odometry);
    floating_session = *backend.session_manager().fed_session();
    for (int step = 0; step < 30; ++step) {
      frontend.Feed(step);
    }
    backend.WaitUntilQuiescent();
    EXPECT_FALSE(backend.graph().session(floating_session).frozen());
  }

  // The floater was last fed at boot 2; with the threshold at 3 it survives through boot 5 and
  // must be swept at boot 6.
  std::optional<std::string> floating_file;
  for (int boot = 3; boot <= 5; ++boot) {
    PoseGraph backend(NoFreeze(option), directory);
    backend.Start(TestTime(total_steps + boot));
    EXPECT_EQ(backend.map_manager()->boot_count(), boot);
    EXPECT_TRUE(backend.graph().HasSession(floating_session))
        << "swept at boot " << boot << ", before the lag exceeded the threshold";
    floating_file = backend.map_manager()->FileNameOf(floating_session);
  }

  PoseGraph backend(NoFreeze(option), directory);
  backend.Start(TestTime(total_steps + 6));
  EXPECT_EQ(backend.map_manager()->boot_count(), 6);

  EXPECT_FALSE(backend.graph().HasSession(floating_session));
  ASSERT_TRUE(floating_file.has_value());
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(directory) / *floating_file))
      << "the swept session's file must go with it";
  const SessionId empty_replacement{frozen_session.session_index + 1};
  EXPECT_FALSE(backend.graph().HasSession(empty_replacement));

  // The frozen base is intact, bitwise up to the serialization round trip.
  ASSERT_TRUE(backend.graph().HasSession(frozen_session));
  EXPECT_TRUE(backend.graph().session(frozen_session).frozen());
  const std::vector<SubmapId>& frozen_ids = backend.graph().session(frozen_session).submap_ids;
  ASSERT_EQ(frozen_ids.size(), frozen_translations.size());
  for (size_t i = 0; i < frozen_ids.size(); ++i) {
    EXPECT_LT(
        (backend.graph().submap(frozen_ids[i]).global_pose.translation() - frozen_translations[i])
            .norm(),
        1e-12)
        << "frozen submap " << i << " moved";
  }

  // Graph and manifest agree: a fresh load returns exactly the sessions the live graph holds.
  PoseGraphData reloaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = testing::LoadMap(reader, reloaded);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_frozen_sessions, 1);
  EXPECT_EQ(reloaded.sessions().size(), backend.graph().sessions().size());
  for (const auto& [id, session] : backend.graph().sessions()) {
    ASSERT_TRUE(reloaded.HasSession(id)) << "session " << id.session_index << " not in manifest";
    EXPECT_EQ(reloaded.session(id).frozen(), session.frozen());
  }
}

// Once the fed session closes a loop against the floater's reloaded submaps, the floater is
// anchored, earns its freeze in place, and no rotation fires.
TEST(MultiBootE2eTest, FloatingSessionAnchoredByCrossSessionLoopClosureFreezesWithoutRotation) {
  const std::string directory = testing::MakeTempDir("evergreenslam_multi_boot_anchor");
  const PoseGraphOption option = RelaxedOption();
  const int total_steps = 3 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = GroundTruthOdometry(total_steps);

  SessionId first_session;
  {
    PoseGraph backend(NoFreeze(option), directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    first_session = *backend.session_manager().fed_session();
    for (int step = 0; step < 30; ++step) {
      frontend.Feed(step);
    }
    backend.WaitUntilQuiescent();
    EXPECT_EQ(backend.session_manager().num_sessions_frozen(), 0);
  }

  // Nothing is frozen, so the bootstrap rule applies: the closure joins both sessions into one
  // component whose datum sits on the older, floating one, and only the dirty constraint gets
  // that session judged at all.
  PoseGraph backend(option, directory);
  backend.Start(TestTime(30));
  const SessionId boot_session = *backend.session_manager().fed_session();
  ASSERT_TRUE(backend.graph().HasSession(first_session));
  EXPECT_FALSE(backend.graph().session(first_session).frozen());
  EXPECT_EQ(boot_session.session_index, first_session.session_index + 1);

  // The restarted frontend opens a fresh identity local frame where the checkpoint left the
  // robot, so the odometry stream is rebased onto the last restored keyframe.
  ASSERT_FALSE(backend.graph().session(first_session).node_ids.empty());
  const NodeId last_restored = backend.graph().session(first_session).node_ids.back();
  const int resume_step = last_restored.node_index;
  const Eigen::Affine2d resume_inverse = odometry[resume_step].inverse();
  std::vector<Eigen::Affine2d> rebased;
  rebased.reserve(odometry.size());
  for (const Eigen::Affine2d& pose : odometry) {
    rebased.push_back(Eigen::Affine2d(resume_inverse * pose));
  }

  Frontend frontend(backend, rebased);
  int step = resume_step + 1;
  const int anchor_deadline = step + 2 * kNodesPerLap;
  while (CountCrossSessionConstraints(backend.graph(), first_session) == 0 &&
         step < anchor_deadline) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_GT(CountCrossSessionConstraints(backend.graph(), first_session), 0)
      << "no cross-session loop closure anchored the floating session";

  const int freeze_deadline = step + kNodesPerLap;
  while (backend.session_manager().num_sessions_frozen() == 0 && step < freeze_deadline) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
      << "the anchored floating session never froze; last verdict: "
      << ToString(backend.session_manager().last_verdict().rejection);
  EXPECT_TRUE(backend.graph().session(first_session).frozen())
      << "the late freeze must land on the floating session";
  EXPECT_FALSE(backend.graph().session(boot_session).frozen());
  EXPECT_EQ(*backend.session_manager().fed_session(), boot_session)
      << "a floating freeze rotates nothing";
}

// A wrong manual seed leaves the session floating; an operator hint off by a few decimetres lets
// the targeted search close a loop that snaps the session onto the base and solves on its own.
// A further hint on the anchored session is searched like any other.
TEST(MultiBootE2eTest, InitialPoseHintTriggersATargetedClosureThatSnapsTheSessionIntoPlace) {
  const std::string directory = testing::MakeTempDir("evergreenslam_multi_boot_hint");
  const PoseGraphOption option = RelaxedOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = GroundTruthOdometry(total_steps);
  FreezeABaseSession(directory, option, odometry);

  PoseGraph backend(NoFreeze(option), directory);
  backend.Start(TestTime(total_steps), transform::FromXYTheta(60.0, 45.0, 0.8));
  const SessionId boot_session = *backend.session_manager().fed_session();
  Frontend frontend(backend, odometry);
  const int hint_step = 14;
  for (int step = 0; step <= hint_step; ++step) {
    frontend.Feed(step);
  }
  backend.WaitUntilQuiescent();
  const NodeId hinted_node = backend.graph().session(boot_session).node_ids.back();
  ASSERT_GT(TranslationError(backend.graph().node(hinted_node).global_pose, hint_step), 30.0);
  ASSERT_EQ(backend.constraint_builder().num_constraints_added(), 0)
      << "the wrong seed must keep the base out of the radius-gated search";

  const Eigen::Affine2d hint =
      Eigen::Affine2d(LoopGroundTruth(hint_step) * transform::FromXYTheta(0.3, -0.25, 0.1));
  const int solves_before = backend.optimization().num_solves();
  backend.SetInitialPose(hint);
  backend.WaitUntilQuiescent();
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0)
      << "the targeted search found no closure against the base";
  EXPECT_GT(backend.optimization().num_solves(), solves_before)
      << "the closure must solve without waiting for a keyframe";
  EXPECT_LT(TranslationError(backend.graph().node(hinted_node).global_pose, hint_step), 0.15);
  EXPECT_LT(RotationError(backend.graph().node(hinted_node).global_pose, hint_step), 0.05);

  frontend.Feed(hint_step + 1);
  backend.WaitUntilQuiescent();
  const NodeId newest = backend.graph().session(boot_session).node_ids.back();
  EXPECT_LT(TranslationError(backend.graph().node(newest).global_pose, hint_step + 1), 0.15);
  EXPECT_LT(RotationError(backend.graph().node(newest).global_pose, hint_step + 1), 0.05);

  const int added_before = backend.constraint_builder().num_constraints_added();
  const Eigen::Affine2d alignment_before = backend.graph().session(boot_session).local_to_global;
  backend.SetInitialPose(LoopGroundTruth(hint_step + 1));
  backend.WaitUntilQuiescent();
  EXPECT_GT(backend.constraint_builder().num_constraints_added(), added_before)
      << "a hint on an anchored session is still searched";
  EXPECT_LT(
      (backend.graph().session(boot_session).local_to_global.matrix() - alignment_before.matrix())
          .norm(),
      1e-12)
      << "an anchored session is never reseeded";
}

// The same wrong seed, no hint at all: the whole-map, all-heading search finds the base.
TEST(MultiBootE2eTest, GlobalRelocalizationRecoversAWronglySeededBootWithoutAHint) {
  const std::string directory = testing::MakeTempDir("evergreenslam_multi_boot_global");
  const PoseGraphOption option = RelaxedOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = GroundTruthOdometry(total_steps);
  FreezeABaseSession(directory, option, odometry);

  PoseGraph backend(NoFreeze(option), directory);
  backend.Start(TestTime(total_steps), transform::FromXYTheta(60.0, 45.0, 0.8));
  const SessionId boot_session = *backend.session_manager().fed_session();
  Frontend frontend(backend, odometry);
  const int relocalize_step = 14;
  for (int step = 0; step <= relocalize_step; ++step) {
    frontend.Feed(step);
  }
  backend.WaitUntilQuiescent();
  ASSERT_EQ(backend.constraint_builder().num_constraints_added(), 0);

  const NodeId relocalized_node = backend.graph().session(boot_session).node_ids.back();
  backend.RelocalizeGlobally();
  backend.WaitUntilQuiescent();
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0)
      << "the global search found no closure against the base";
  EXPECT_LT(TranslationError(backend.graph().node(relocalized_node).global_pose, relocalize_step),
            0.15);

  frontend.Feed(relocalize_step + 1);
  backend.WaitUntilQuiescent();
  const NodeId newest = backend.graph().session(boot_session).node_ids.back();
  EXPECT_LT(TranslationError(backend.graph().node(newest).global_pose, relocalize_step + 1), 0.15);
  EXPECT_LT(RotationError(backend.graph().node(newest).global_pose, relocalize_step + 1), 0.05);
}

// Checkpoints stop at the solve cadence, the last-pose file follows every keyframe; a reboot
// seeds from the newer of the two.
TEST(MultiBootE2eTest, LastPoseFileSeedsTheRebootWhenNewerThanTheCheckpoint) {
  const std::string directory = testing::MakeTempDir("evergreenslam_multi_boot_last_pose");
  const PoseGraphOption option = RelaxedOption();
  const int total_steps = kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = GroundTruthOdometry(total_steps);
  const int last_fed_step = 24;
  const int last_checkpoint_step = 19;

  SessionId first_session;
  {
    PoseGraph backend(NoFreeze(option), directory);
    backend.Start(TestTime(0));
    first_session = *backend.session_manager().fed_session();
    Frontend frontend(backend, odometry);
    for (int step = 0; step <= last_fed_step; ++step) {
      frontend.Feed(step);
    }
    backend.WaitUntilQuiescent();
    EXPECT_EQ(backend.map_manager()->num_checkpoints_written(), 2);
  }

  {
    PoseGraph backend(NoFreeze(option), directory);
    backend.Start(TestTime(last_fed_step + 1));
    const PoseGraphData& graph = backend.graph();
    ASSERT_TRUE(graph.HasNode(NodeId{first_session.session_index, last_checkpoint_step}));
    ASSERT_FALSE(graph.HasNode(NodeId{first_session.session_index, last_fed_step}));
    const std::optional<MapManager::LastPose> last_pose = backend.map_manager()->ReadLastPose();
    ASSERT_TRUE(last_pose.has_value());
    EXPECT_EQ(last_pose->node_id, (NodeId{first_session.session_index, last_fed_step}));
    // Written at ingest, before the closures of that lap nudged the poses by a centimetre.
    const Eigen::Affine2d& seed =
        graph.session(*backend.session_manager().fed_session()).local_to_global;
    EXPECT_LT(TranslationError(seed, last_fed_step), 0.05);
    EXPECT_LT(RotationError(seed, last_fed_step), 0.01);
    EXPECT_GT(TranslationError(seed, last_checkpoint_step), 0.5);
  }

  // A stale last pose from a session the map no longer holds falls back to the checkpoint.
  {
    MapManager writer(directory);
    Node stray;
    stray.id = NodeId{99, 0};
    stray.constant_data.time = TestTime(1000);
    stray.global_pose = transform::FromXYTheta(7.0, 7.0, 0.7);
    writer.WriteLastPose(stray);
  }
  PoseGraph backend(NoFreeze(option), directory);
  backend.Start(TestTime(last_fed_step + 2));
  const Eigen::Affine2d& seed =
      backend.graph().session(*backend.session_manager().fed_session()).local_to_global;
  EXPECT_LT(TranslationError(seed, last_checkpoint_step), 0.05);
  EXPECT_GT(TranslationError(seed, last_fed_step), 0.5);
}

}  // namespace
}  // namespace evergreenslam::lifelong

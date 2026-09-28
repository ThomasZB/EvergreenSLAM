/**
 * @file anchor_graph_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Anchors through the graph changes that move or rename what they hang off: a closure
 *        that corrects the whole session, trim by succession, and the freeze rotation.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "anchor_scenario.h"
#include "lifelong/map_manager/map_manager.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using testing::ExpectNearTruth;
using testing::FreezeABase;
using testing::Frontend;
using testing::GetAnchor;
using testing::kAnchorTranslationTolerance;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::ReadAnchorsFromDisk;
using testing::ResolveAnchor;
using testing::RunOnTask;
using testing::SaveAnchor;
using testing::TestTime;
using testing::TranslationError;

// The loop fixture's room is small enough that the matcher closes against the session's own
// submaps from the first lap, so odometry drift is corrected as it accrues and never builds up
// into a correction worth measuring. The error here is the session's whole frame instead: a boot
// seeded 60 m off, which only a closure can remove.
TEST(AnchorGraphE2eTest, AnchorSurvivesLoopClosure) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_loop_closure");
  const PoseGraphOption option = testing::AnchorTestOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    FreezeABase(backend, frontend, total_steps);
  }

  PoseGraph backend(testing::NoFreeze(option), directory);
  backend.Start(TestTime(total_steps), transform::FromXYTheta(60.0, 45.0, 0.8));
  const SessionId boot_session = *backend.session_manager().fed_session();
  Frontend frontend(backend, odometry);
  const int save_step = 10;
  const int relocalize_step = 14;
  int step = 0;
  for (; step <= save_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  const std::optional<Anchor> anchor = SaveAnchor(backend);
  ASSERT_TRUE(anchor.has_value());
  EXPECT_EQ(anchor->submap_id.session_id, boot_session.session_index);
  const ResolvedAnchor before = ResolveAnchor(backend, anchor->id);
  ASSERT_TRUE(before.global_pose.has_value());
  const double error_before = TranslationError(*before.global_pose, LoopGroundTruth(save_step));
  ASSERT_GT(error_before, 30.0) << "the wrong seed must put the anchor far from the truth";

  for (; step <= relocalize_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  ASSERT_EQ(backend.constraint_builder().num_constraints_added(), 0)
      << "the wrong seed must keep the base out of the radius-gated search";
  backend.RelocalizeGlobally();
  backend.WaitUntilQuiescent();
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0)
      << "the global search found no closure against the base";

  const ResolvedAnchor after = ResolveAnchor(backend, anchor->id);
  EXPECT_EQ(after.state, AnchorState::BOUND);
  EXPECT_FALSE(after.frozen);
  ASSERT_TRUE(after.global_pose.has_value());
  EXPECT_GT(TranslationError(*after.global_pose, *before.global_pose),
            error_before - kAnchorTranslationTolerance)
      << "the anchor must travel with its submap, the whole correction";
  ExpectNearTruth(after, save_step);

  // More than a lap of closures and solves later it is still where the robot stood.
  for (; step < total_steps; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  ExpectNearTruth(ResolveAnchor(backend, anchor->id), save_step);
}

// A localizing session over known area sheds its older submaps; an anchor on one of them moves
// to the successor the trimmer names and still resolves where the robot stood.
TEST(AnchorGraphE2eTest, AnchorSurvivesTrimBySuccession) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_trim");
  PoseGraphOption option = testing::AnchorTestOption();
  option.trim = true;
  const int max_steps = 7 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(max_steps);

  PoseGraph backend(option, directory);
  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  int step = FreezeABase(backend, frontend, kNodesPerLap);
  const SessionId successor = *backend.session_manager().fed_session();

  const int save_step = step + kNodesPerLap / 2;
  for (; step <= save_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  const std::optional<Anchor> anchor = SaveAnchor(backend);
  ASSERT_TRUE(anchor.has_value());
  ASSERT_EQ(anchor->submap_id.session_id, successor.session_index);
  const SubmapId bound_to = anchor->submap_id;

  while (GetAnchor(backend, anchor->id)->state == AnchorState::BOUND && step < max_steps) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_GT(backend.trimmer().num_submaps_trimmed(), 0) << "trimming never engaged";
  const Anchor rebound = *GetAnchor(backend, anchor->id);
  ASSERT_EQ(rebound.state, AnchorState::REBOUND)
      << "the anchor's submap was never trimmed within " << max_steps << " steps";
  EXPECT_FALSE(rebound.submap_id == bound_to);
  EXPECT_FALSE(RunOnTask(backend, [&] { return backend.graph().HasSubmap(bound_to); }));
  EXPECT_TRUE(RunOnTask(backend, [&] { return backend.graph().HasSubmap(rebound.submap_id); }));
  const ResolvedAnchor resolved = ResolveAnchor(backend, anchor->id);
  EXPECT_EQ(resolved.state, AnchorState::REBOUND);
  ExpectNearTruth(resolved, save_step);

  // The trim that rebound it came with a checkpoint, and the checkpoint with the anchors file.
  const std::optional<AnchorTable> on_disk = ReadAnchorsFromDisk(directory);
  ASSERT_TRUE(on_disk.has_value());
  ASSERT_EQ(on_disk->anchors.size(), 1u);
  EXPECT_EQ(on_disk->anchors[0].state, AnchorState::REBOUND);
  EXPECT_EQ(on_disk->anchors[0].submap_id, rebound.submap_id);
}

// Two anchors across a manual freeze: one on a submap that finished before it, which stays with
// the frozen session and never moves again, one on an active submap the successor adopts.
TEST(AnchorGraphE2eTest, AnchorSurvivesFreezeRotation) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_rotation");
  const PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  const int total_steps = kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const int finished_step = 5;
  const int active_step = 25;

  AnchorId finished_id = 0;
  AnchorId active_id = 0;
  SessionId first_session;
  SessionId next_session;
  Eigen::Affine2d frozen_pose = Eigen::Affine2d::Identity();
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    first_session = *backend.session_manager().fed_session();
    Frontend frontend(backend, odometry);
    int step = 0;
    for (; step <= finished_step; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    const Anchor finished = *SaveAnchor(backend);
    finished_id = finished.id;
    for (; step <= active_step; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    // The window holds the submaps born at steps 10 and 20, both unfinished.
    const Anchor active = *SaveAnchor(backend);
    active_id = active.id;
    const Eigen::Affine2d active_before = *ResolveAnchor(backend, active_id).global_pose;

    backend.FreezeFedSession(first_session);
    backend.WaitUntilQuiescent();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
    next_session = *backend.session_manager().fed_session();
    ASSERT_NE(next_session, first_session);

    const Anchor finished_now = *GetAnchor(backend, finished_id);
    EXPECT_EQ(finished_now.submap_id, finished.submap_id) << "a finished submap stays behind";
    const ResolvedAnchor finished_resolved = ResolveAnchor(backend, finished_id);
    EXPECT_TRUE(finished_resolved.frozen);
    EXPECT_EQ(finished_resolved.state, AnchorState::BOUND) << "frozen is derived, not stored";
    ExpectNearTruth(finished_resolved, finished_step);
    frozen_pose = *finished_resolved.global_pose;

    const Anchor active_now = *GetAnchor(backend, active_id);
    EXPECT_EQ(active.submap_id.session_id, first_session.session_index);
    EXPECT_EQ(active_now.submap_id.session_id, next_session.session_index)
        << "the adopted submap's new id must follow it";
    EXPECT_EQ(active_now.state, AnchorState::BOUND) << "a transfer is a rename, not a rebind";
    const ResolvedAnchor active_resolved = ResolveAnchor(backend, active_id);
    EXPECT_FALSE(active_resolved.frozen);
    ExpectNearTruth(active_resolved, active_step);
    EXPECT_LT(TranslationError(*active_resolved.global_pose, active_before), 0.02)
        << "the rotation's solve moved the anchor";

    // Persisted with the rotation, before any later checkpoint.
    const std::optional<AnchorTable> on_disk = ReadAnchorsFromDisk(directory);
    ASSERT_TRUE(on_disk.has_value());
    ASSERT_EQ(on_disk->anchors.size(), 2u);
    EXPECT_EQ(on_disk->anchors[1].submap_id, active_now.submap_id);

    // More solves: the frozen one never moves again, not by a bit.
    for (; step < active_step + 3 * testing::kNodesPerSubmap; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    const Eigen::Affine2d frozen_later = *ResolveAnchor(backend, finished_id).global_pose;
    EXPECT_TRUE(frozen_later.matrix() == frozen_pose.matrix());
    ExpectNearTruth(ResolveAnchor(backend, active_id), active_step);
  }

  // Killed without Finish: both resolve from what the rotation wrote.
  PoseGraph backend(option, directory);
  backend.Start(TestTime(total_steps));
  const ResolvedAnchor finished_reloaded = ResolveAnchor(backend, finished_id);
  EXPECT_TRUE(finished_reloaded.frozen);
  ASSERT_TRUE(finished_reloaded.global_pose.has_value());
  EXPECT_LT(TranslationError(*finished_reloaded.global_pose, frozen_pose), 1e-9);
  const ResolvedAnchor active_reloaded = ResolveAnchor(backend, active_id);
  EXPECT_EQ(active_reloaded.submap_id.session_id, next_session.session_index);
  ExpectNearTruth(active_reloaded, active_step);
}

}  // namespace
}  // namespace evergreenslam::lifelong

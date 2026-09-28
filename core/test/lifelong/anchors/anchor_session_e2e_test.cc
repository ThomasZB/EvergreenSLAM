/**
 * @file anchor_session_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Anchors as the agent layer drives sessions: a saved place as the relocalization hint of a
 *        wrongly seeded boot, and deleting a floating session under anchors of three kinds.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "../testing/load_map.h"
#include "anchor_scenario.h"
#include "lifelong/map_manager/map_manager.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using testing::ExpectNearTruth;
using testing::Frontend;
using testing::GetAnchor;
using testing::kAnchorRotationTolerance;
using testing::kAnchorTranslationTolerance;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::ResolveAnchor;
using testing::RotationError;
using testing::RunOnTask;
using testing::SaveAnchor;
using testing::TestTime;
using testing::TranslationError;

void FeedTo(PoseGraph& backend, Frontend& frontend, int& step, int last_step) {
  for (; step <= last_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
}

void FeedUntilFrozen(PoseGraph& backend, Frontend& frontend, int& step, int max_steps) {
  while (backend.session_manager().num_sessions_frozen() == 0 && step < max_steps) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
      << "the boot session never froze; last verdict: "
      << ToString(backend.session_manager().last_verdict().rejection);
}

Eigen::Affine2d NewestNodePose(PoseGraph& backend, SessionId session) {
  return RunOnTask(backend, [&] {
    return backend.graph().node(backend.graph().session(session).node_ids.back()).global_pose;
  });
}

// `egs init-pose --place dock` on a boot whose seed is 60 m off: the place resolves on the frozen
// base, and as the hint it lets the targeted search snap the session onto the base.
TEST(AnchorSessionE2eTest, SetInitialPoseByPlaceThenRelocalize) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_init_pose");
  const PoseGraphOption option = testing::AnchorTestOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const int place_step = 14;

  AnchorId place = 0;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, place_step);
    place = SaveAnchor(backend)->id;
    FeedUntilFrozen(backend, frontend, step, total_steps);
    const ResolvedAnchor dock = ResolveAnchor(backend, place);
    EXPECT_TRUE(dock.frozen);
    ExpectNearTruth(dock, place_step);
  }

  PoseGraph backend(testing::NoFreeze(option), directory);
  backend.Start(TestTime(total_steps), transform::FromXYTheta(60.0, 45.0, 0.8));
  const SessionId boot_session = *backend.session_manager().fed_session();
  Frontend frontend(backend, odometry);
  int step = 0;
  FeedTo(backend, frontend, step, place_step);
  ASSERT_EQ(backend.constraint_builder().num_constraints_added(), 0);
  ASSERT_GT(TranslationError(NewestNodePose(backend, boot_session), LoopGroundTruth(place_step)),
            30.0);

  const ResolvedAnchor dock = ResolveAnchor(backend, place);
  ASSERT_TRUE(dock.global_pose.has_value());
  EXPECT_TRUE(dock.frozen);
  backend.SetInitialPose(*dock.global_pose);
  backend.WaitUntilQuiescent();
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0)
      << "the place hint produced no closure against the base";
  const Eigen::Affine2d hinted = NewestNodePose(backend, boot_session);
  EXPECT_LT(TranslationError(hinted, LoopGroundTruth(place_step)), kAnchorTranslationTolerance);
  EXPECT_LT(RotationError(hinted, LoopGroundTruth(place_step)), kAnchorRotationTolerance);

  FeedTo(backend, frontend, step, place_step + 1);
  const Eigen::Affine2d next = NewestNodePose(backend, boot_session);
  EXPECT_LT(TranslationError(next, LoopGroundTruth(place_step + 1)), kAnchorTranslationTolerance);
  EXPECT_LT(RotationError(next, LoopGroundTruth(place_step + 1)), kAnchorRotationTolerance);
}

// Anchors on the frozen base, the floating session and the fed one; the floating session goes
// through PoseGraph::RemoveSession on the backend task, as the agent's rm/apply does.
TEST(AnchorSessionE2eTest, DeleteFloatingSessionKeepsOthers) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_delete");
  const PoseGraphOption option = testing::AnchorTestOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const int frozen_step = 5;
  const int floating_step = 45;
  const int fed_step = 60;

  AnchorId on_frozen = 0;
  AnchorId on_floating = 0;
  SessionId floating;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, frozen_step);
    on_frozen = SaveAnchor(backend)->id;
    FeedUntilFrozen(backend, frontend, step, total_steps);
    floating = *backend.session_manager().fed_session();
    FeedTo(backend, frontend, step, floating_step);
    const Anchor anchor = *SaveAnchor(backend);
    on_floating = anchor.id;
    ASSERT_EQ(anchor.submap_id.session_id, floating.session_index);
    backend.Finish();
  }

  PoseGraph backend(testing::NoFreeze(option), directory);
  backend.Start(TestTime(total_steps));
  const SessionId fed = *backend.session_manager().fed_session();
  ASSERT_TRUE(RunOnTask(backend, [&] {
    return backend.graph().HasSession(floating) && !backend.graph().session(floating).frozen();
  }));
  // The restarted frontend's local frame starts at identity where the checkpoint left the robot.
  const Eigen::Affine2d resume_inverse = odometry[floating_step].inverse();
  std::vector<Eigen::Affine2d> rebased;
  for (const Eigen::Affine2d& pose : odometry) {
    rebased.push_back(Eigen::Affine2d(resume_inverse * pose));
  }
  Frontend frontend(backend, rebased);
  int step = floating_step + 1;
  FeedTo(backend, frontend, step, fed_step);
  const Anchor fed_anchor = *SaveAnchor(backend);
  ASSERT_EQ(fed_anchor.submap_id.session_id, fed.session_index);
  const AnchorId on_fed = fed_anchor.id;

  const ResolvedAnchor frozen_before = ResolveAnchor(backend, on_frozen);
  const ResolvedAnchor fed_before = ResolveAnchor(backend, on_fed);
  ExpectNearTruth(frozen_before, frozen_step);
  ExpectNearTruth(fed_before, fed_step);
  ExpectNearTruth(ResolveAnchor(backend, on_floating), floating_step);

  std::vector<VariableId> removed_variables;
  std::optional<std::string> floating_file;
  RunOnTask(backend, [&] {
    floating_file = backend.map_manager()->FileNameOf(floating);
    const SessionData& session = backend.graph().session(floating);
    for (const SubmapId& id : session.submap_ids) {
      removed_variables.push_back(VariableId::Of(id));
    }
    for (const NodeId& id : session.node_ids) {
      removed_variables.push_back(VariableId::Of(id));
    }
    backend.RemoveSession(floating);
  });
  ASSERT_FALSE(removed_variables.empty());

  const Anchor orphaned = *GetAnchor(backend, on_floating);
  EXPECT_EQ(orphaned.state, AnchorState::ORPHAN);
  EXPECT_EQ(orphaned.orphan_reason, OrphanReason::SESSION_REMOVED);
  EXPECT_FALSE(ResolveAnchor(backend, on_floating).global_pose.has_value());
  const ResolvedAnchor frozen_after = ResolveAnchor(backend, on_frozen);
  const ResolvedAnchor fed_after = ResolveAnchor(backend, on_fed);
  EXPECT_EQ(frozen_after.state, AnchorState::BOUND);
  EXPECT_EQ(fed_after.state, AnchorState::BOUND);
  EXPECT_TRUE(frozen_after.global_pose->matrix() == frozen_before.global_pose->matrix());
  EXPECT_TRUE(fed_after.global_pose->matrix() == fed_before.global_pose->matrix());

  // Graph, Problem and files agree.
  RunOnTask(backend, [&] {
    EXPECT_FALSE(backend.graph().HasSession(floating));
    EXPECT_TRUE(backend.graph().HasSession(fed));
    for (const VariableId& variable : removed_variables) {
      EXPECT_FALSE(backend.optimization().HasVariable(variable));
    }
  });
  ASSERT_TRUE(floating_file.has_value());
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(directory) / *floating_file));
  PoseGraphData reloaded;
  MapManager reader(directory);
  ASSERT_TRUE(testing::LoadMap(reader, reloaded).has_value());
  EXPECT_FALSE(reloaded.HasSession(floating));
  const std::optional<AnchorTable> on_disk = testing::ReadAnchorsFromDisk(directory);
  ASSERT_TRUE(on_disk.has_value());
  ASSERT_EQ(on_disk->anchors.size(), 3u);
  EXPECT_EQ(on_disk->anchors[1].id, on_floating);
  EXPECT_EQ(on_disk->anchors[1].orphan_reason, OrphanReason::SESSION_REMOVED);

  // The graph still solves and closes around what is left.
  FeedTo(backend, frontend, step, fed_step + 2 * testing::kNodesPerSubmap);
  ExpectNearTruth(ResolveAnchor(backend, on_fed), fed_step);
  EXPECT_TRUE(ResolveAnchor(backend, on_frozen).global_pose->matrix() ==
              frozen_before.global_pose->matrix());
}

// --ignore_last_pose: neither the checkpoint node nor last_pose.pb seeds the boot session.
TEST(AnchorSessionE2eTest, IgnoringThePreviousBootSeedsAtIdentity) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_no_seed");
  const PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(kNodesPerLap);
  const int last_step = 30;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, last_step);
    backend.Finish();
  }
  const auto boot_seed = [&](bool seed_from_previous_boot) {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(last_step + 1), std::nullopt, seed_from_previous_boot);
    EXPECT_EQ(backend.boot_first_session(), *backend.session_manager().fed_session());
    return RunOnTask(backend, [&] {
      return backend.graph().session(backend.boot_first_session()).local_to_global;
    });
  };
  EXPECT_TRUE(boot_seed(false).matrix() == Eigen::Affine2d::Identity().matrix());
  EXPECT_LT(TranslationError(boot_seed(true), LoopGroundTruth(last_step)),
            kAnchorTranslationTolerance);
}

}  // namespace
}  // namespace evergreenslam::lifelong

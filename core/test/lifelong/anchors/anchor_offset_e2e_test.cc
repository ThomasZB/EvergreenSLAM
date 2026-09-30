/**
 * @file anchor_offset_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Anchors saved at an offset from the robot: where they resolve after a closure, and the
 *        free-space check that refuses an offset into a wall or unknown space.
 * @version 0.1
 * @date 2026-09-29
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

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using Refusal = PoseGraph::SaveAnchorResult::Refusal;
using testing::FreezeABase;
using testing::Frontend;
using testing::kAnchorRotationTolerance;
using testing::kAnchorTranslationTolerance;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::ResolveAnchor;
using testing::RotationError;
using testing::RunOnTask;
using testing::TestTime;
using testing::TranslationError;

PoseGraph::SaveAnchorResult SaveAt(PoseGraph& backend, const Eigen::Affine2d& node_from_anchor,
                                   std::optional<AnchorId> rebind = std::nullopt) {
  return RunOnTask(backend, [&] {
    return backend.SaveAnchorOnTask(/*keep_scan=*/true, rebind, node_from_anchor);
  });
}

// The robot-frame offset from the robot's true pose at `step` to `target`, a world point.
Eigen::Affine2d OffsetTo(int step, const Eigen::Vector2d& target) {
  const Eigen::Vector2d local = LoopGroundTruth(step).inverse() * target;
  return transform::FromXYTheta(local.x(), local.y(), 0.0);
}

// The offset anchor rides the closure that moves its whole session, like a plain one does.
TEST(AnchorOffsetE2eTest, OffsetAnchorSurvivesLoopClosure) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_offset_closure");
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
  Frontend frontend(backend, odometry);
  const int save_step = 10;
  const int relocalize_step = 14;
  int step = 0;
  for (; step <= save_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  const Eigen::Affine2d offset = transform::FromXYTheta(1.0, 0.2, 0.5);
  const PoseGraph::SaveAnchorResult saved = SaveAt(backend, offset);
  ASSERT_EQ(saved.refusal, Refusal::NONE);
  const AnchorId id = saved.anchor->id;
  const Eigen::Affine2d truth = LoopGroundTruth(save_step) * offset;
  ASSERT_GT(TranslationError(*ResolveAnchor(backend, id).global_pose, truth), 30.0);

  for (; step <= relocalize_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  backend.RelocalizeGlobally();
  backend.WaitUntilQuiescent();
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0);

  const ResolvedAnchor after = ResolveAnchor(backend, id);
  ASSERT_TRUE(after.global_pose.has_value());
  EXPECT_LT(TranslationError(*after.global_pose, truth), kAnchorTranslationTolerance);
  EXPECT_LT(RotationError(*after.global_pose, truth), kAnchorRotationTolerance);

  // Rebinding the same id from a later keyframe with another offset keeps the id.
  const int rebind_step = step - 1;
  const Eigen::Affine2d rebind_offset = transform::FromXYTheta(0.8, -0.3, -0.2);
  const PoseGraph::SaveAnchorResult rebound = SaveAt(backend, rebind_offset, id);
  ASSERT_EQ(rebound.refusal, Refusal::NONE);
  EXPECT_EQ(rebound.anchor->id, id);
  EXPECT_LT(TranslationError(*ResolveAnchor(backend, id).global_pose,
                             LoopGroundTruth(rebind_step) * rebind_offset),
            kAnchorTranslationTolerance);
}

TEST(AnchorOffsetE2eTest, OffsetIntoAWallOrUnknownSpaceIsRefusedAndChangesNothing) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_offset_free");
  const int save_step = 15;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(save_step + 1);
  PoseGraph backend(testing::NoFreeze(testing::AnchorTestOption()), directory);
  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  for (int step = 0; step <= save_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  const Eigen::Vector2d robot = LoopGroundTruth(save_step).translation();
  const double wall_x = testing::LoopRoom().max_x;

  const PoseGraph::SaveAnchorResult wall =
      SaveAt(backend, OffsetTo(save_step, Eigen::Vector2d(wall_x, robot.y())));
  EXPECT_EQ(wall.refusal, Refusal::OFFSET_NOT_FREE);
  EXPECT_FALSE(wall.anchor.has_value());
  const PoseGraph::SaveAnchorResult beyond =
      SaveAt(backend, OffsetTo(save_step, Eigen::Vector2d(wall_x + 0.5, robot.y())));
  EXPECT_EQ(beyond.refusal, Refusal::OFFSET_NOT_FREE);
  EXPECT_EQ(RunOnTask(backend, [&] { return backend.anchors().size(); }), 0);

  const PoseGraph::SaveAnchorResult free =
      SaveAt(backend, OffsetTo(save_step, Eigen::Vector2d(wall_x - 1.0, robot.y())));
  ASSERT_EQ(free.refusal, Refusal::NONE);
  EXPECT_EQ(free.anchor->id, 1u) << "a refusal issues no id";

  // A pure rotation checks nothing: the target is the robot's own cell.
  EXPECT_EQ(SaveAt(backend, transform::FromXYTheta(0.0, 0.0, 1.0)).refusal, Refusal::NONE);
}

}  // namespace
}  // namespace evergreenslam::lifelong

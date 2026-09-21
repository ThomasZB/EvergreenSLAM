/**
 * @file relocalization_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Operator relocalization as an asynchronous loop-closure job: a session booted far from
 *        truth is snapped onto the frozen base by a hinted match, with no keyframe in between.
 * @version 0.1
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <optional>
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

using testing::Frontend;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::TestTime;

// The rebooted frontend's local frame sits this far from the global one, so a session booted at
// identity starts more than 20 m and about a right angle from truth.
const Eigen::Affine2d& LocalToTruth() {
  static const Eigen::Affine2d offset = transform::FromXYTheta(47.65, -33.8, 1.6);
  return offset;
}

double TranslationError(const Eigen::Affine2d& pose, int step) {
  return (pose.translation() - LoopGroundTruth(step).translation()).norm();
}

double RotationError(const Eigen::Affine2d& pose, int step) {
  return std::abs(transform::NormalizeAngle(transform::GetYaw(pose) -
                                            transform::GetYaw(LoopGroundTruth(step))));
}

double PoseDistance(const Eigen::Affine2d& a, const Eigen::Affine2d& b) {
  return (a.matrix() - b.matrix()).norm();
}

// Closures within the booted session itself are ordinary and say nothing about relocalization.
int CountClosuresOntoOtherSessions(const PoseGraphData& graph, SessionId session) {
  int count = 0;
  for (const Constraint& constraint : graph.constraints()) {
    if (constraint.type != Constraint::Type::INTER_SUBMAP || !constraint.to.has_value()) {
      continue;
    }
    const SessionId from = constraint.from.session();
    const SessionId to = constraint.to->session();
    if (from != to && (from == session || to == session)) {
      ++count;
    }
  }
  return count;
}

// Relaxed judge so the base freezes within two laps. A frozen base adds no drift slack, so the
// ordinary radius-gated searches of a session seeded 20 m away cannot reach it; only the hint can.
PoseGraphOption RelocalizationOption() {
  PoseGraphOption option;
  option.optimization.optimize_every_n_nodes = 10;
  option.checkpoint_min_interval = common::Duration::zero();
  option.session_manager.freeze_judge.max_translation_stddev = 1.0;
  option.session_manager.freeze_judge.max_rotation_stddev = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  option.trim = false;
  return option;
}

class RelocalizationE2eTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    directory_ = testing::MakeTempDir("evergreenslam_relocalization_" + name);
    option_ = RelocalizationOption();
    truth_odometry_.reserve(kTotalSteps);
    local_odometry_.reserve(kTotalSteps);
    for (int i = 0; i < kTotalSteps; ++i) {
      truth_odometry_.push_back(LoopGroundTruth(i));
      local_odometry_.push_back(Eigen::Affine2d(LocalToTruth().inverse() * LoopGroundTruth(i)));
    }
    FreezeTheBase();
  }

  void FreezeTheBase() {
    PoseGraph backend(option_, directory_);
    backend.Start(TestTime(0));
    Frontend frontend(backend, truth_odometry_);
    int step = 0;
    while (backend.session_manager().num_sessions_frozen() == 0 && step < kTotalSteps) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    backend.Finish();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
        << "the base never froze; last verdict: "
        << ToString(backend.session_manager().last_verdict().rejection);
  }

  // Identity alignment on purpose: the boot has no idea where it is. Freezing is held back:
  // the boot session is under test as a floating one.
  void Boot() {
    PoseGraphOption boot_option = option_;
    boot_option.session_manager.auto_freeze = false;
    backend_ = std::make_unique<PoseGraph>(boot_option, directory_);
    backend_->Start(TestTime(kTotalSteps), Eigen::Affine2d::Identity());
    session_ = *backend_->session_manager().fed_session();
    frontend_ = std::make_unique<Frontend>(*backend_, local_odometry_);
  }

  void FeedThrough(int last_step) {
    for (int step = next_step_; step <= last_step; ++step) {
      frontend_->Feed(step);
      backend_->WaitUntilQuiescent();
    }
    next_step_ = last_step + 1;
  }

  NodeId NewestNode() const { return backend_->graph().session(session_).node_ids.back(); }
  const Eigen::Affine2d& NewestPose() const {
    return backend_->graph().node(NewestNode()).global_pose;
  }

  int NumClosuresOntoOtherSessions() const {
    return CountClosuresOntoOtherSessions(backend_->graph(), session_);
  }

  void ExpectLost(int step) {
    EXPECT_GT(TranslationError(NewestPose(), step), 20.0);
    EXPECT_EQ(NumClosuresOntoOtherSessions(), 0);
    EXPECT_LT(PoseDistance(backend_->graph().session(session_).local_to_global,
                           Eigen::Affine2d::Identity()),
              1e-12);
  }

  void ExpectSnappedOntoTruth(int step) {
    ASSERT_GT(NumClosuresOntoOtherSessions(), 0) << "no closure against the base";
    EXPECT_LT(TranslationError(NewestPose(), step), 0.15);
    EXPECT_LT(RotationError(NewestPose(), step), 0.05);
    const std::optional<Eigen::Affine2d> alignment =
        backend_->graph().ComputeSessionToGlobal(session_);
    ASSERT_TRUE(alignment.has_value());
    EXPECT_LT((alignment->translation() - LocalToTruth().translation()).norm(), 0.15);
    EXPECT_LT(std::abs(transform::NormalizeAngle(transform::GetYaw(*alignment) -
                                                 transform::GetYaw(LocalToTruth()))),
              0.05);
    // The reseed wrote the session alignment itself, not only the poses hanging off it.
    EXPECT_LT((backend_->graph().session(session_).local_to_global.translation() -
               LocalToTruth().translation())
                  .norm(),
              0.15);
    const std::optional<Eigen::Affine2d> published = backend_->ActiveSessionToGlobal();
    ASSERT_TRUE(published.has_value());
    EXPECT_LT(PoseDistance(*published, *alignment), 1e-12)
        << "the frontend-facing alignment must follow the solve without a keyframe";
  }

  static constexpr int kTotalSteps = 2 * kNodesPerLap;

  std::string directory_;
  PoseGraphOption option_;
  std::vector<Eigen::Affine2d> truth_odometry_;
  std::vector<Eigen::Affine2d> local_odometry_;
  std::unique_ptr<PoseGraph> backend_;
  std::unique_ptr<Frontend> frontend_;
  SessionId session_;
  int next_step_ = 0;
};

TEST_F(RelocalizationE2eTest, ACorrectHintSnapsTheSessionOntoTheBaseWithoutAKeyframe) {
  Boot();
  const int hint_step = 14;
  FeedThrough(hint_step);
  ExpectLost(hint_step);

  const int solves_before = backend_->optimization().num_solves();
  backend_->SetInitialPose(LoopGroundTruth(hint_step));
  backend_->WaitUntilQuiescent();
  EXPECT_GT(backend_->optimization().num_solves(), solves_before);
  ExpectSnappedOntoTruth(hint_step);
}

// The operator's first guess is wrong; nothing moves, and the pose resent from an easier spot is
// not refused.
TEST_F(RelocalizationE2eTest, AWrongHintMovesNothingAndTheResentHintSucceeds) {
  Boot();
  FeedThrough(14);
  ExpectLost(14);

  const Eigen::Affine2d wrong =
      Eigen::Affine2d(LoopGroundTruth(14) * transform::FromXYTheta(-3.5, 2.5, 1.3));
  backend_->SetInitialPose(wrong);
  backend_->WaitUntilQuiescent();
  EXPECT_GT(backend_->constraint_builder().num_matches_attempted(), 0)
      << "the wrong hint still lies inside the room, so candidates exist";
  ExpectLost(14);

  FeedThrough(20);
  ExpectLost(20);
  backend_->SetInitialPose(LoopGroundTruth(20));
  backend_->WaitUntilQuiescent();
  ExpectSnappedOntoTruth(20);
}

TEST_F(RelocalizationE2eTest, AHintSentBeforeTheFirstKeyframeIsAppliedToIt) {
  Boot();
  backend_->SetInitialPose(
      Eigen::Affine2d(LoopGroundTruth(0) * transform::FromXYTheta(-3.5, 2.5, 1.3)));
  backend_->SetInitialPose(LoopGroundTruth(0));
  backend_->WaitUntilQuiescent();
  EXPECT_EQ(backend_->constraint_builder().num_matches_attempted(), 0);

  FeedThrough(0);
  ExpectSnappedOntoTruth(0);
}

// Once anchored, a hint is only a match prior: the search runs and its closure is a normal edge
// that neither reseeds nor solves; it waits for the node cadence like any other closure.
TEST_F(RelocalizationE2eTest, AHintOnAnAnchoredSessionAddsAClosureWithoutReseeding) {
  Boot();
  FeedThrough(14);
  backend_->SetInitialPose(LoopGroundTruth(14));
  backend_->WaitUntilQuiescent();
  ExpectSnappedOntoTruth(14);

  FeedThrough(20);
  backend_->WaitUntilQuiescent();
  const int closures_before = NumClosuresOntoOtherSessions();
  const int solves_before = backend_->optimization().num_solves();
  const Eigen::Affine2d alignment_before = backend_->graph().session(session_).local_to_global;
  backend_->SetInitialPose(
      Eigen::Affine2d(LoopGroundTruth(20) * transform::FromXYTheta(0.2, -0.1, 0.05)));
  backend_->WaitUntilQuiescent();
  EXPECT_GT(NumClosuresOntoOtherSessions(), closures_before);
  EXPECT_EQ(backend_->optimization().num_solves(), solves_before)
      << "a closure on an anchored session is an edge, not a solve";
  EXPECT_LT(PoseDistance(backend_->graph().session(session_).local_to_global, alignment_before),
            1e-12);
  ExpectSnappedOntoTruth(20);
}

}  // namespace
}  // namespace evergreenslam::lifelong

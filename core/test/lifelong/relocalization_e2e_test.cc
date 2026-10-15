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
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/optimization/optimization_option.h"
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

std::vector<PoseGraphEvent> EventsOfKind(const PoseGraph& backend, PoseGraphEvent::Kind kind) {
  std::vector<PoseGraphEvent> events = backend.EventsSince(0).events;
  events.erase(std::remove_if(events.begin(), events.end(),
                              [kind](const PoseGraphEvent& event) { return event.kind != kind; }),
               events.end());
  return events;
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

  RelocalizationStatus StatusOf(RelocalizationId id) const {
    const std::optional<RelocalizationStatus> status = backend_->relocalization(id);
    CHECK(status.has_value()) << "request " << id << " is not tracked";
    return *status;
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
  const Eigen::Affine2d lost_pose = NewestPose();
  const NodeId hinted_node = NewestNode();
  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(hint_step));
  EXPECT_EQ(id, 1);
  EXPECT_EQ(StatusOf(id).state, RelocalizationState::PENDING);
  backend_->WaitUntilQuiescent();
  EXPECT_GT(backend_->optimization().num_solves(), solves_before);
  ExpectSnappedOntoTruth(hint_step);

  const RelocalizationStatus status = StatusOf(id);
  EXPECT_EQ(status.state, RelocalizationState::MATCHED);
  EXPECT_TRUE(status.has_prior);
  EXPECT_GT(status.candidates, 0);
  EXPECT_GT(status.constraints_added, 0);
  EXPECT_LE(status.constraints_added, status.candidates);
  ASSERT_TRUE(status.requested_at.has_value());
  EXPECT_EQ(*status.requested_at, backend_->graph().node(hinted_node).constant_data.time);
  ASSERT_TRUE(status.resolved_at.has_value());
  ASSERT_TRUE(status.correction.has_value());
  EXPECT_NEAR(status.correction->translation,
              (NewestPose().translation() - lost_pose.translation()).norm(), 0.05);
  EXPECT_GT(status.correction->translation, 20.0);
  EXPECT_NEAR(status.correction->rotation, RotationError(lost_pose, hint_step), 0.05);
  EXPECT_EQ(backend_->latest_relocalization()->id, id);

  const std::vector<PoseGraphEvent> outcomes =
      EventsOfKind(*backend_, PoseGraphEvent::Kind::RELOCALIZATION);
  ASSERT_EQ(outcomes.size(), 1u);
  EXPECT_EQ(outcomes[0].relocalization_id, id);
  EXPECT_EQ(outcomes[0].outcome, RelocalizationState::MATCHED);
  ASSERT_TRUE(outcomes[0].correction.has_value());
  EXPECT_GT(outcomes[0].correction->translation, 20.0);
  const std::vector<PoseGraphEvent> jumps =
      EventsOfKind(*backend_, PoseGraphEvent::Kind::POSE_CORRECTION);
  ASSERT_FALSE(jumps.empty());
  EXPECT_EQ(jumps.front().session, session_);
  EXPECT_GT(jumps.front().correction->translation, 20.0);
  EXPECT_LT(jumps.front().seq, outcomes[0].seq);
}

// The operator's first guess is wrong; nothing moves, and the pose resent from an easier spot is
// not refused.
TEST_F(RelocalizationE2eTest, AWrongHintMovesNothingAndTheResentHintSucceeds) {
  Boot();
  FeedThrough(14);
  ExpectLost(14);

  const Eigen::Affine2d wrong =
      Eigen::Affine2d(LoopGroundTruth(14) * transform::FromXYTheta(-3.5, 2.5, 1.3));
  const RelocalizationId wrong_id = backend_->SetInitialPose(wrong);
  backend_->WaitUntilQuiescent();
  EXPECT_GT(backend_->constraint_builder().num_matches_attempted(), 0)
      << "the wrong hint still lies inside the room, so candidates exist";
  ExpectLost(14);
  const RelocalizationStatus rejected = StatusOf(wrong_id);
  EXPECT_EQ(rejected.state, RelocalizationState::NO_MATCH);
  EXPECT_GT(rejected.candidates, 0);
  EXPECT_EQ(rejected.constraints_added, 0);
  EXPECT_FALSE(rejected.correction.has_value());

  FeedThrough(20);
  ExpectLost(20);
  const RelocalizationId resent_id = backend_->SetInitialPose(LoopGroundTruth(20));
  EXPECT_EQ(resent_id, wrong_id + 1);
  backend_->WaitUntilQuiescent();
  ExpectSnappedOntoTruth(20);
  EXPECT_EQ(StatusOf(resent_id).state, RelocalizationState::MATCHED);
  EXPECT_EQ(StatusOf(wrong_id).state, RelocalizationState::NO_MATCH);
}

TEST_F(RelocalizationE2eTest, AHintOutsideEveryFinishedSubmapIsNoMatchWithZeroCandidates) {
  Boot();
  FeedThrough(14);
  const int attempted_before = backend_->constraint_builder().num_matches_attempted();
  const RelocalizationId id = backend_->SetInitialPose(transform::FromXYTheta(400.0, -250.0, 0.3));
  backend_->WaitUntilQuiescent();
  EXPECT_EQ(backend_->constraint_builder().num_matches_attempted(), attempted_before);
  const RelocalizationStatus status = StatusOf(id);
  EXPECT_EQ(status.state, RelocalizationState::NO_MATCH);
  EXPECT_EQ(status.candidates, 0);
  EXPECT_TRUE(status.resolved_at.has_value());
  ExpectLost(14);
}

TEST_F(RelocalizationE2eTest, AGlobalSearchWithoutAPriorResolvesToo) {
  Boot();
  FeedThrough(14);
  const RelocalizationId id = backend_->RelocalizeGlobally();
  backend_->WaitUntilQuiescent();
  const RelocalizationStatus status = StatusOf(id);
  EXPECT_FALSE(status.has_prior);
  EXPECT_NE(status.state, RelocalizationState::PENDING);
  EXPECT_GT(status.candidates, 0);
  if (status.state == RelocalizationState::MATCHED) {
    ExpectSnappedOntoTruth(14);
  }
}

// The second request's task runs before any worker result of the first is applied.
TEST_F(RelocalizationE2eTest, ASecondRequestBeforeTheFirstResolvesSupersedesIt) {
  Boot();
  FeedThrough(14);
  const RelocalizationId first = backend_->SetInitialPose(LoopGroundTruth(14));
  const RelocalizationId second = backend_->SetInitialPose(LoopGroundTruth(14));
  backend_->WaitUntilQuiescent();

  const RelocalizationStatus superseded = StatusOf(first);
  EXPECT_EQ(superseded.state, RelocalizationState::SUPERSEDED);
  EXPECT_EQ(superseded.superseded_by, second);
  EXPECT_GT(superseded.candidates, 0);
  EXPECT_EQ(StatusOf(second).state, RelocalizationState::MATCHED);
  ExpectSnappedOntoTruth(14);

  const std::vector<PoseGraphEvent> outcomes =
      EventsOfKind(*backend_, PoseGraphEvent::Kind::RELOCALIZATION);
  ASSERT_EQ(outcomes.size(), 2u);
  EXPECT_EQ(outcomes[0].relocalization_id, first);
  EXPECT_EQ(outcomes[0].outcome, RelocalizationState::SUPERSEDED);
  EXPECT_EQ(outcomes[1].relocalization_id, second);
  EXPECT_EQ(outcomes[1].outcome, RelocalizationState::MATCHED);
}

TEST_F(RelocalizationE2eTest, AHintSentBeforeTheFirstKeyframeIsAppliedToIt) {
  Boot();
  const RelocalizationId stale = backend_->SetInitialPose(
      Eigen::Affine2d(LoopGroundTruth(0) * transform::FromXYTheta(-3.5, 2.5, 1.3)));
  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(0));
  backend_->WaitUntilQuiescent();
  EXPECT_EQ(backend_->constraint_builder().num_matches_attempted(), 0);
  EXPECT_EQ(StatusOf(stale).state, RelocalizationState::SUPERSEDED);
  EXPECT_EQ(StatusOf(stale).superseded_by, id);
  EXPECT_EQ(StatusOf(id).state, RelocalizationState::PENDING);
  EXPECT_FALSE(StatusOf(id).requested_at.has_value());

  FeedThrough(0);
  ExpectSnappedOntoTruth(0);
  EXPECT_EQ(StatusOf(id).state, RelocalizationState::MATCHED);
  EXPECT_EQ(StatusOf(id).requested_at, TestTime(0));
  ASSERT_TRUE(StatusOf(id).correction.has_value());
  EXPECT_GT(StatusOf(id).correction->translation, 20.0);
}

TEST_F(RelocalizationE2eTest, DroppingTheFedSessionSupersedesAWaitingRequest) {
  Boot();
  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(0));
  backend_->WaitUntilQuiescent();
  const PoseGraph::DropFedSessionResult dropped = backend_->DropFedSession(session_);
  ASSERT_TRUE(dropped.fed_now.has_value());
  const RelocalizationStatus status = StatusOf(id);
  EXPECT_EQ(status.state, RelocalizationState::SUPERSEDED);
  EXPECT_FALSE(status.superseded_by.has_value());

  const std::vector<PoseGraphEvent> events = backend_->EventsSince(0).events;
  ASSERT_GE(events.size(), 3u);
  const PoseGraphEvent& removed = events[events.size() - 3];
  EXPECT_EQ(removed.kind, PoseGraphEvent::Kind::SESSION_REMOVED);
  EXPECT_EQ(removed.session, session_);
  const PoseGraphEvent& drop = events[events.size() - 2];
  EXPECT_EQ(drop.kind, PoseGraphEvent::Kind::FED_SESSION_DROPPED);
  EXPECT_EQ(drop.session, session_);
  EXPECT_EQ(drop.fed_now, dropped.fed_now);
  EXPECT_EQ(events.back().kind, PoseGraphEvent::Kind::RELOCALIZATION);
  EXPECT_EQ(events.back().outcome, RelocalizationState::SUPERSEDED);
}

// The drop task is queued behind the request's, so its worker matches land on a removed node.
TEST_F(RelocalizationE2eTest, DroppingTheFedSessionMidRoundDiscardsTheRoundsMatches) {
  Boot();
  FeedThrough(14);
  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(14));
  const PoseGraph::DropFedSessionResult dropped = backend_->DropFedSession(session_);
  ASSERT_TRUE(dropped.fed_now.has_value());
  backend_->WaitUntilQuiescent();
  const RelocalizationStatus status = StatusOf(id);
  EXPECT_EQ(status.state, RelocalizationState::SUPERSEDED);
  EXPECT_GT(status.candidates, 0);
  EXPECT_FALSE(status.superseded_by.has_value());
  EXPECT_EQ(CountClosuresOntoOtherSessions(backend_->graph(), *dropped.fed_now), 0);
}

// Once anchored, a hint is only a match prior: its closure is a normal edge that does not reseed;
// the request's one solve settles it.
TEST_F(RelocalizationE2eTest, AHintOnAnAnchoredSessionAddsAClosureAndOneSolveWithoutReseeding) {
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
  const RelocalizationId id = backend_->SetInitialPose(
      Eigen::Affine2d(LoopGroundTruth(20) * transform::FromXYTheta(0.2, -0.1, 0.05)));
  backend_->WaitUntilQuiescent();
  EXPECT_GT(NumClosuresOntoOtherSessions(), closures_before);
  EXPECT_EQ(backend_->optimization().num_solves(), solves_before + 1);
  EXPECT_LT(PoseDistance(backend_->graph().session(session_).local_to_global, alignment_before),
            1e-12);
  ExpectSnappedOntoTruth(20);
  const RelocalizationStatus status = StatusOf(id);
  EXPECT_EQ(status.state, RelocalizationState::MATCHED);
  ASSERT_TRUE(status.correction.has_value());
  EXPECT_LT(status.correction->translation, 0.1);
}

// An anchored session pinned to a wrong pose by `count` closures onto the base, newest nodes first.
void PinWithWrongClosures(PoseGraph& backend, SessionId session, int count,
                          const ConstraintWeightOption& weight) {
  SessionId base;
  for (const auto& [id, data] : backend.graph().sessions()) {
    if (data.frozen()) {
      base = id;
    }
  }
  const SubmapId base_submap = backend.graph().session(base).submap_ids.front();
  const Eigen::Affine2d submap_pose = backend.graph().submap(base_submap).global_pose;
  const Eigen::Affine2d shift = transform::FromXYTheta(1.5, -1.2, 0.35);
  const std::vector<NodeId> node_ids = backend.graph().session(session).node_ids;
  for (int i = 0; i < count; ++i) {
    const NodeId node_id = node_ids[node_ids.size() - 1 - i];
    const int step = testing::StepAt(backend.graph().node(node_id).constant_data.time);
    Constraint wrong;
    wrong.type = Constraint::Type::INTER_SUBMAP;
    wrong.from = VariableId::Of(base_submap);
    wrong.to = VariableId::Of(node_id);
    wrong.relative_pose = Eigen::Affine2d(submap_pose.inverse() * shift * LoopGroundTruth(step));
    wrong.sqrt_information = LeverArmSqrtInformation(LoopClosureSqrtInformation(weight),
                                                     wrong.relative_pose.translation().norm());
    backend.AddConstraint(wrong);
  }
  backend.Optimize();
  backend.WaitUntilQuiescent();
}

// The hint's round adds a handful of Huber-lossed edges, one solve, no reseed: they win only when
// they outnumber the wrong ones.
TEST_F(RelocalizationE2eTest, AHintOnASessionPinnedByOneWrongClosureOutvotesIt) {
  Boot();
  FeedThrough(14);
  PinWithWrongClosures(*backend_, session_, 1, option_.constraint_weight);
  ASSERT_GT(TranslationError(NewestPose(), 14), 1.5);

  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(14));
  backend_->WaitUntilQuiescent();
  EXPECT_EQ(StatusOf(id).state, RelocalizationState::MATCHED);
  EXPECT_GT(StatusOf(id).constraints_added, 1);
  EXPECT_LT(TranslationError(NewestPose(), 14), 0.15);
}

// What `session rm <fed>` then `init-pose` is for: MATCHED, yet the robot stays where it was.
TEST_F(RelocalizationE2eTest, AHintOnASessionPinnedBySixWrongClosuresMatchesButLeavesItWrong) {
  Boot();
  FeedThrough(14);
  PinWithWrongClosures(*backend_, session_, 6, option_.constraint_weight);
  const double error_before = TranslationError(NewestPose(), 14);
  ASSERT_GT(error_before, 1.5);

  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(14));
  backend_->WaitUntilQuiescent();
  const RelocalizationStatus status = StatusOf(id);
  EXPECT_EQ(status.state, RelocalizationState::MATCHED);
  EXPECT_GT(status.constraints_added, 0);
  EXPECT_LT(status.constraints_added, 6);
  EXPECT_GT(TranslationError(NewestPose(), 14), 1.0);
  ASSERT_TRUE(status.correction.has_value());
  EXPECT_LT(status.correction->translation, error_before - 1.0);
}

}  // namespace
}  // namespace evergreenslam::lifelong

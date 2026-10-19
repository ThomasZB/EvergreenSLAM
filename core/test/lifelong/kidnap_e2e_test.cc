/**
 * @file kidnap_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief A robot put down somewhere else on the frozen base, found again without an operator.
 * @version 0.1
 * @date 2026-10-19
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
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

// The rebooted frontend's local frame, as in the relocalization tests.
const Eigen::Affine2d& LocalToTruth() {
  static const Eigen::Affine2d offset = transform::FromXYTheta(47.65, -33.8, 1.6);
  return offset;
}

// A wrong guess that still lands the loop inside the room, so the base can disagree with it;
// one outside the room would read as unmapped ground.
Eigen::Affine2d WrongSeed() {
  const Eigen::Affine2d center = transform::FromXYTheta(6.5, 5.0, 0.0);
  const Eigen::Affine2d displaced = Eigen::Affine2d(transform::FromXYTheta(1.5, -0.5, 0.0) * center *
                                                    transform::FromXYTheta(0.0, 0.0, 0.6) *
                                                    center.inverse());
  return Eigen::Affine2d(displaced * LocalToTruth());
}

// The simulated room's base reads differently from a real one: free cells seen from one loop
// stay near 0.4, and a displaced scan keeps about half its points on known cells.
PoseGraphOption KidnapOption() {
  PoseGraphOption option;
  option.optimization.optimize_every_n_nodes = 10;
  option.checkpoint_min_interval = common::Duration::zero();
  option.session_manager.freeze_judge.max_translation_stddev = 1.0;
  option.session_manager.freeze_judge.max_rotation_stddev = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  option.trim = false;
  option.kidnap_detector.max_base_score = 0.5;
  option.kidnap_detector.min_base_coverage = 0.4;
  return option;
}

std::vector<PoseGraphEvent> EventsOfKind(const PoseGraph& backend, PoseGraphEvent::Kind kind) {
  std::vector<PoseGraphEvent> events = backend.EventsSince(0).events;
  events.erase(std::remove_if(events.begin(), events.end(),
                              [kind](const PoseGraphEvent& event) { return event.kind != kind; }),
               events.end());
  return events;
}

class KidnapE2eTest : public ::testing::Test {
 protected:
  static constexpr int kBootStep = 2 * kNodesPerLap;
  static constexpr int kSteps = 2 * kNodesPerLap;
  static constexpr double kBrokenMatch = 0.1;

  void SetUp() override {
    const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    directory_ = testing::MakeTempDir("evergreenslam_kidnap_" + name);
    option_ = KidnapOption();
    PoseGraph backend(option_, directory_);
    backend.Start(TestTime(0));
    Frontend frontend(backend, testing::LoopTruth(kBootStep));
    int step = 0;
    while (backend.session_manager().num_sessions_frozen() == 0 && step < kBootStep) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    backend.Finish();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
  }

  // Freezing held back: the boot session is under test. From `carried_at` steps after the boot
  // the robot is `carried_by` steps further along the loop than its odometry says, and that
  // keyframe's frontend match breaks.
  void Boot(const Eigen::Affine2d& seed, std::optional<int> carried_at = std::nullopt,
            int carried_by = 0) {
    PoseGraphOption option = option_;
    option.session_manager.auto_freeze = false;
    backend_ = std::make_unique<PoseGraph>(option, directory_);
    backend_->Start(TestTime(kBootStep), seed);
    session_ = *backend_->session_manager().fed_session();
    std::vector<Eigen::Affine2d> odometry;
    for (int step = 0; step < kBootStep + kSteps; ++step) {
      const bool carried = carried_at.has_value() && step >= kBootStep + *carried_at;
      truth_.push_back(LoopGroundTruth(carried ? step + carried_by : step));
      odometry.push_back(Eigen::Affine2d(LocalToTruth().inverse() * LoopGroundTruth(step)));
    }
    frontend_ = std::make_unique<Frontend>(*backend_, testing::LoopRoom(), truth_, odometry);
    const std::optional<int> broken_step =
        carried_at.has_value() ? std::optional<int>(kBootStep + *carried_at) : std::nullopt;
    frontend_->set_match_score([broken_step](int step) -> std::optional<double> {
      return step == broken_step ? kBrokenMatch : 0.7;
    });
    next_step_ = kBootStep;
  }

  void FeedThrough(int last) {
    for (; next_step_ <= kBootStep + last; ++next_step_) {
      frontend_->Feed(next_step_);
      backend_->WaitUntilQuiescent();
    }
  }

  // Feeds until the detector starts a search; the boot-relative step it did, or -1.
  int FeedUntilKidnap(int last) {
    const size_t before = Kidnaps().size();
    while (next_step_ <= kBootStep + last) {
      frontend_->Feed(next_step_++);
      backend_->WaitUntilQuiescent();
      if (Kidnaps().size() > before) {
        return next_step_ - 1 - kBootStep;
      }
    }
    return -1;
  }

  std::vector<PoseGraphEvent> Kidnaps() const {
    return EventsOfKind(*backend_, PoseGraphEvent::Kind::KIDNAP_SUSPECTED);
  }

  double NewestError() const {
    const Node& node = backend_->graph().node(backend_->graph().session(session_).node_ids.back());
    return (node.global_pose.translation() -
            truth_.at(testing::StepAt(node.constant_data.time)).translation())
        .norm();
  }

  std::string directory_;
  PoseGraphOption option_;
  std::unique_ptr<PoseGraph> backend_;
  std::unique_ptr<Frontend> frontend_;
  std::vector<Eigen::Affine2d> truth_;
  SessionId session_;
  int next_step_ = 0;
};

// Seeded from a stale last pose: the guess lands inside the room but 3 m and 0.6 rad off, and
// the ordinary searches around it find nothing before the detector does.
TEST_F(KidnapE2eTest, ABootSeededOntoTheWrongSpotIsFoundWithoutAnOperator) {
  Boot(WrongSeed());
  const int fired = FeedUntilKidnap(40);
  ASSERT_GT(fired, 0) << "the detector never fired";

  const std::vector<PoseGraphEvent> kidnaps = Kidnaps();
  ASSERT_EQ(kidnaps.size(), 1u);
  const PoseGraphEvent& kidnap = kidnaps.front();
  EXPECT_EQ(kidnap.session, session_);
  ASSERT_TRUE(kidnap.relocalization_id.has_value());
  ASSERT_TRUE(kidnap.kidnap.has_value());
  EXPECT_EQ(kidnap.kidnap->keyframes, option_.kidnap_detector.min_suspect_keyframes);
  EXPECT_FALSE(kidnap.kidnap->lowest_match_score.has_value())
      << "the session never matched the base; the boot seed alone broke continuity";
  EXPECT_LT(kidnap.kidnap->base_score, option_.kidnap_detector.max_base_score);
  EXPECT_GE(kidnap.kidnap->base_coverage, option_.kidnap_detector.min_base_coverage);
  EXPECT_GE(kidnap.kidnap->distance_without_base_match,
            option_.kidnap_detector.min_distance_without_base_match);
  ASSERT_TRUE(backend_->latest_kidnap().has_value());
  EXPECT_EQ(backend_->latest_kidnap()->seq, kidnap.seq);

  const std::optional<RelocalizationStatus> status =
      backend_->relocalization(*kidnap.relocalization_id);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->trigger, RelocalizationTrigger::KIDNAP);
  EXPECT_FALSE(status->has_prior);
  EXPECT_EQ(status->state, RelocalizationState::MATCHED);
  ASSERT_TRUE(status->correction.has_value());
  EXPECT_GT(status->correction->translation, 1.0);
  EXPECT_LT(NewestError(), 0.15);

  const std::vector<PoseGraphEvent> outcomes =
      EventsOfKind(*backend_, PoseGraphEvent::Kind::RELOCALIZATION);
  ASSERT_EQ(outcomes.size(), 1u);
  EXPECT_EQ(outcomes[0].relocalization_id, kidnap.relocalization_id);
  EXPECT_LT(kidnap.seq, outcomes[0].seq);

  // Found and confirmed: a lap later nothing else fired and the pose held.
  FeedThrough(fired + kNodesPerLap);
  EXPECT_EQ(Kidnaps().size(), 1u);
  EXPECT_FALSE(backend_->kidnap_detector().status().continuity_broken);
  EXPECT_LT(NewestError(), 0.15);
}

// Carried a quarter lap mid-run while tracking resumed at once. The session is already pinned to
// the base by its earlier closures, so the search's few edges are outvoted (API.md, Relocalization
// outcome): the detector keeps saying so and backs off instead of searching on every keyframe.
TEST_F(KidnapE2eTest, ACarriedRobotIsDetectedAndAnOutvotedSearchBacksOff) {
  const int carried_at = 30;
  Boot(LocalToTruth(), carried_at, kNodesPerLap / 4);
  FeedThrough(carried_at - 1);
  EXPECT_LT(NewestError(), 0.1);
  EXPECT_TRUE(Kidnaps().empty());
  EXPECT_FALSE(backend_->kidnap_detector().status().continuity_broken);

  const int first = FeedUntilKidnap(carried_at + 60);
  ASSERT_GT(first, carried_at);
  const PoseGraphEvent kidnap = Kidnaps().front();
  ASSERT_TRUE(kidnap.kidnap->lowest_match_score.has_value());
  EXPECT_DOUBLE_EQ(*kidnap.kidnap->lowest_match_score, kBrokenMatch);
  const std::optional<RelocalizationStatus> status =
      backend_->relocalization(*kidnap.relocalization_id);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->trigger, RelocalizationTrigger::KIDNAP);
  EXPECT_NE(status->state, RelocalizationState::PENDING);
  EXPECT_TRUE(backend_->kidnap_detector().status().continuity_broken);

  const int cooldown = option_.kidnap_detector.cooldown_keyframes;
  const int second = FeedUntilKidnap(first + 4 * cooldown);
  ASSERT_GT(second, 0);
  EXPECT_GE(second - first, cooldown);
  const int third = FeedUntilKidnap(kSteps - 1);
  if (third > 0) {
    EXPECT_GE(third - second, 2 * cooldown);
  }
}

TEST_F(KidnapE2eTest, ARobotLeftWhereItWasRaisesNothing) {
  Boot(LocalToTruth());
  FeedThrough(kSteps - 1);
  EXPECT_TRUE(Kidnaps().empty());
  EXPECT_FALSE(backend_->latest_relocalization().has_value());
  EXPECT_LT(NewestError(), 0.1);
}

TEST_F(KidnapE2eTest, DisabledNothingIsSearched) {
  option_.kidnap_detector.enabled = false;
  Boot(WrongSeed());
  FeedThrough(40);
  EXPECT_TRUE(Kidnaps().empty());
  EXPECT_FALSE(backend_->latest_relocalization().has_value());
}

// An operator's request in flight owns the correction; the detector waits for it.
TEST_F(KidnapE2eTest, AnOperatorRequestInFlightHoldsTheDetector) {
  Boot(WrongSeed());
  const RelocalizationId id = backend_->SetInitialPose(LoopGroundTruth(kBootStep));
  FeedThrough(40);
  EXPECT_EQ(backend_->relocalization(id)->state, RelocalizationState::MATCHED);
  EXPECT_TRUE(Kidnaps().empty());
  EXPECT_LT(NewestError(), 0.15);
}

}  // namespace
}  // namespace evergreenslam::lifelong

/**
 * @file drift_tracker_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The distance since the last match to the frozen base, through the real wiring: absent
 * without a base, carried across the freeze rotation, gone with a dropped session.
 * @version 0.1
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "../testing/loop_scenario.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"

namespace evergreenslam::lifelong {
namespace {

using testing::DriftedOdometry;
using testing::Frontend;
using testing::kNodesPerLap;
using testing::TestTime;

TEST(DriftTrackerE2eTest, TheBaseMatchDistanceCarriesAcrossRotationAndDiesWithADrop) {
  PoseGraphOption option =
      LoadPoseGraphOptionFromFile(std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml");
  option.constraint_builder.num_match_workers = 1;
  option.constraint_builder.sampler_option.sampling_ratio = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 8;
  option.session_manager.auto_freeze = false;
  option.trim = false;
  PoseGraph backend(option);
  backend.Start(TestTime(0));
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(3 * kNodesPerLap);
  Frontend frontend(backend, odometry);
  const DriftTracker& tracker = backend.constraint_builder().drift_tracker();
  const auto fed = [&backend] { return *backend.session_manager().fed_session(); };

  int step = 0;
  for (; step < kNodesPerLap; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0);
  EXPECT_FALSE(tracker.SinceBaseMatch(fed()).has_value());

  backend.FreezeFedSession();
  backend.WaitUntilQuiescent();
  const SessionId second = fed();
  EXPECT_FALSE(tracker.SinceBaseMatch(second).has_value())
      << "the predecessor had no base to match";

  for (; step < 2 * kNodesPerLap && !tracker.SinceBaseMatch(second).has_value(); ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  ASSERT_TRUE(tracker.SinceBaseMatch(second).has_value());
  for (const int end = step + 10; step < end; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  const double before = *tracker.SinceBaseMatch(second);

  backend.FreezeFedSession();
  backend.WaitUntilQuiescent();
  const SessionId third = fed();
  ASSERT_NE(third, second);
  ASSERT_TRUE(tracker.SinceBaseMatch(third).has_value());
  EXPECT_DOUBLE_EQ(*tracker.SinceBaseMatch(third), before);

  frontend.Feed(step);
  backend.WaitUntilQuiescent();
  const double driven = (odometry[step].translation() - odometry[step - 1].translation()).norm();
  ASSERT_TRUE(tracker.SinceBaseMatch(third).has_value());
  EXPECT_LE(*tracker.SinceBaseMatch(third), before + driven + 1e-9);
  ++step;

  const PoseGraph::DropFedSessionResult dropped = backend.DropFedSession(third);
  ASSERT_TRUE(dropped.fed_now.has_value());
  EXPECT_FALSE(tracker.SinceBaseMatch(*dropped.fed_now).has_value());
  EXPECT_FALSE(tracker.SinceBaseMatch(third).has_value());
}

}  // namespace
}  // namespace evergreenslam::lifelong

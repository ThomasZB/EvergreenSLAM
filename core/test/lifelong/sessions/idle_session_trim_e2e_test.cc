/**
 * @file idle_session_trim_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief An idle session the fed one re-maps is trimmed away whole, and stays gone on disk.
 * @version 0.1
 * @date 2026-10-13
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

#include "../anchors/anchor_scenario.h"
#include "lifelong/map_manager/map_manager.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"

namespace evergreenslam::lifelong {
namespace {

using testing::AnchorTestOption;
using testing::ExpectNearTruth;
using testing::Frontend;
using testing::GroundTruthOdometry;
using testing::kNodesPerLap;
using testing::NoFreeze;
using testing::ResolveAnchor;
using testing::SaveAnchor;
using testing::TestTime;

TEST(IdleSessionTrimE2eTest, AnIdleSessionTheFedOneReMapsIsRemovedAndStaysGone) {
  const std::string directory = testing::MakeTempDir("evergreenslam_idle_session_trim");
  PoseGraphOption option = NoFreeze(AnchorTestOption());
  option.trim = true;
  const int total_steps = 4 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = GroundTruthOdometry(total_steps);

  SessionId frozen_session;
  SessionId idle_session;
  std::optional<std::string> idle_file;
  AnchorId anchor_id = 0;
  int anchor_step = 0;
  int step = 0;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    frozen_session = *backend.session_manager().fed_session();
    for (; step < 25; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    backend.FreezeFedSession();
    backend.WaitUntilQuiescent();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
    idle_session = *backend.session_manager().fed_session();

    for (; step < kNodesPerLap + 10; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    anchor_step = step - 1;
    const std::optional<Anchor> anchor = SaveAnchor(backend);
    ASSERT_TRUE(anchor.has_value());
    ASSERT_EQ(SessionOf(anchor->submap_id), idle_session);
    anchor_id = anchor->id;
    EXPECT_GE(backend.graph().session(idle_session).submap_ids.size(), 4u);
    idle_file = backend.map_manager()->FileNameOf(idle_session);
    ASSERT_TRUE(idle_file.has_value());
  }

  const int resume_step = step - 1;
  const Eigen::Affine2d resume_inverse = odometry[resume_step].inverse();
  std::vector<Eigen::Affine2d> rebased;
  rebased.reserve(odometry.size());
  for (const Eigen::Affine2d& pose : odometry) {
    rebased.push_back(Eigen::Affine2d(resume_inverse * pose));
  }

  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(step));
    const SessionId fed_session = *backend.session_manager().fed_session();
    ASSERT_TRUE(backend.graph().HasSession(idle_session));
    ASSERT_FALSE(backend.graph().session(idle_session).frozen());

    Frontend frontend(backend, rebased);
    const int deadline = step + 2 * kNodesPerLap;
    while (backend.graph().HasSession(idle_session) && step < deadline) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    EXPECT_FALSE(backend.graph().HasSession(idle_session))
        << "the fed session re-mapped the idle one's room, yet "
        << backend.graph().session(idle_session).submap_ids.size() << " of its submaps survive";
    EXPECT_TRUE(backend.graph().session(frozen_session).frozen());
    EXPECT_EQ(*backend.session_manager().fed_session(), fed_session);
    ExpectNearTruth(ResolveAnchor(backend, anchor_id), anchor_step);
    backend.Finish();
    EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(directory) / *idle_file))
        << "the removed session's file must go with it";
  }

  PoseGraphData reloaded;
  MapManager reader(directory);
  ASSERT_TRUE(testing::LoadMap(reader, reloaded).has_value());
  EXPECT_FALSE(reloaded.HasSession(idle_session));
  EXPECT_TRUE(reloaded.HasSession(frozen_session));

  PoseGraph backend(option, directory);
  backend.Start(TestTime(step));
  EXPECT_FALSE(backend.graph().HasSession(idle_session)) << "the removed session came back";
  ExpectNearTruth(ResolveAnchor(backend, anchor_id), anchor_step);
}

}  // namespace
}  // namespace evergreenslam::lifelong

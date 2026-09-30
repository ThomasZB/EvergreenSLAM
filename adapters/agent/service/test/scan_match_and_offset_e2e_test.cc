/**
 * @file scan_match_and_offset_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The scan-match score in /here and /status, and /place/save at an offset, over real HTTP.
 * @version 0.1
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "service/agent_service.h"
#include "service_test_client.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

using lifelong::PoseGraph;
using lifelong::testing::Frontend;
using lifelong::testing::kNodesPerLap;
using lifelong::testing::LoopGroundTruth;
using lifelong::testing::LoopRoom;
using lifelong::testing::SimulateScan;
using lifelong::testing::TestTime;
using testing::JsonNumber;
using testing::JsonRaw;
using testing::Reply;
using testing::SimulatedHost;
using testing::TestClient;

constexpr double kPlaceTolerance = 0.05;

struct Rig {
  std::string map_dir = lifelong::testing::MakeTempDir("evergreenslam_scan_match_offset");
  std::vector<Eigen::Affine2d> odometry = lifelong::testing::GroundTruthOdometry(kNodesPerLap);
  PoseGraph backend{lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()), map_dir};
  SimulatedHost host;
  AgentService service{backend, host.Hooks(), AgentServiceOption{}};
  Frontend frontend{backend, odometry};

  void FeedTo(int last_step, double match_score) {
    for (int step = 0; step <= last_step; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
      host.Update(TestTime(step), odometry[step], SimulateScan(LoopRoom(), LoopGroundTruth(step)),
                  match_score);
    }
  }
};

// The robot-frame offset from the robot's true pose at `step` to `target`, a world point.
std::string OffsetTo(int step, const Eigen::Vector2d& target) {
  const Eigen::Vector2d local = LoopGroundTruth(step).inverse() * target;
  return std::to_string(local.x()) + "," + std::to_string(local.y()) + ",0";
}

TEST(ScanMatchE2eTest, HereAndStatusReportTheHostScoreAndItsAverage) {
  Rig rig;
  ASSERT_GT(rig.service.port(), 0);
  TestClient client(rig.service.port());
  rig.backend.Start(TestTime(0));
  rig.service.OnBackendStarted();

  EXPECT_EQ(JsonRaw(client.Get("/here").body, "scan_match"), "null");
  EXPECT_EQ(JsonRaw(client.Get("/status").body, "scan_match"), "null");

  const int last_step = 5;
  rig.FeedTo(last_step, 0.75);
  const Reply here = client.Get("/here");
  ASSERT_EQ(JsonRaw(here.body, "ok"), "true") << here.body;
  EXPECT_DOUBLE_EQ(JsonNumber(here.body, "scan_match.score"), 0.75);
  EXPECT_DOUBLE_EQ(JsonNumber(here.body, "scan_match.avg"), 0.75);

  // One bad scan moves the score at once and the 2 s average by 1 - exp(-0.1 / 2) of the gap.
  rig.host.Update(TestTime(last_step + 1), rig.odometry[last_step + 1],
                  SimulateScan(LoopRoom(), LoopGroundTruth(last_step + 1)), 0.25);
  const double expected_avg = 0.75 - (1.0 - std::exp(-0.1 / 2.0)) * 0.5;
  const Reply status = client.Get("/status");
  ASSERT_EQ(JsonRaw(status.body, "ok"), "true") << status.body;
  EXPECT_DOUBLE_EQ(JsonNumber(status.body, "scan_match.score"), 0.25);
  EXPECT_NEAR(JsonNumber(status.body, "scan_match.avg"), expected_avg, 1e-9);
  EXPECT_NEAR(JsonNumber(client.Get("/here").body, "scan_match.avg"), expected_avg, 1e-9);

  rig.service.Stop();
  rig.backend.Finish();
}

TEST(PlaceOffsetE2eTest, SavesAtAnOffsetAndRefusesFarOrNonFreeTargets) {
  Rig rig;
  ASSERT_GT(rig.service.port(), 0);
  TestClient client(rig.service.port());
  rig.backend.Start(TestTime(0));
  rig.service.OnBackendStarted();
  const int step = 5;
  rig.FeedTo(step, 0.8);

  const Reply saved =
      client.Post("/place/save", {{"path", "places/shelf"}, {"offset", "1,0.25,0.5"}});
  ASSERT_EQ(JsonRaw(saved.body, "ok"), "true") << saved.body;
  EXPECT_NE(saved.body.find("\"offset\":[1,0.25,0.5]"), std::string::npos) << saved.body;
  const std::string anchor = JsonRaw(saved.body, "anchor");
  const Reply resolved = client.Get("/anchors/" + anchor + "?robot=1");
  ASSERT_EQ(JsonRaw(resolved.body, "ok"), "true") << resolved.body;
  const Eigen::Affine2d robot = utils::transform::FromXYTheta(
      JsonNumber(resolved.body, "robot.x"), JsonNumber(resolved.body, "robot.y"),
      JsonNumber(resolved.body, "robot.theta"));
  const Eigen::Affine2d expected = robot * utils::transform::FromXYTheta(1.0, 0.25, 0.5);
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.x"), expected.translation().x(), kPlaceTolerance);
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.y"), expected.translation().y(), kPlaceTolerance);
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.theta"), utils::transform::GetYaw(expected),
              kPlaceTolerance);

  // Without an offset the receipt says so.
  EXPECT_EQ(JsonRaw(client.Post("/place/save", {{"path", "places/dock"}}).body, "offset"), "null");

  const Reply far = client.Post("/place/save", {{"path", "places/far"}, {"offset", "2.9,1,0"}});
  EXPECT_EQ(far.status, 200);
  EXPECT_EQ(JsonRaw(far.body, "reason"), "offset_too_far") << far.body;
  EXPECT_EQ(JsonRaw(far.body, "anchor"), "null");

  // The wall below the robot, then the unknown space behind it.
  const Eigen::Vector2d robot_xy = LoopGroundTruth(step).translation();
  const double wall_y = LoopRoom().min_y;
  const Reply wall = client.Post(
      "/place/save",
      {{"path", "places/wall"}, {"offset", OffsetTo(step, Eigen::Vector2d(robot_xy.x(), wall_y))}});
  EXPECT_EQ(JsonRaw(wall.body, "reason"), "offset_not_free") << wall.body;
  const Reply behind = client.Post(
      "/place/save", {{"path", "places/behind"},
                      {"offset", OffsetTo(step, Eigen::Vector2d(robot_xy.x(), wall_y - 0.3))}});
  EXPECT_EQ(JsonRaw(behind.body, "reason"), "offset_not_free") << behind.body;
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(rig.map_dir) / "memory/places/wall"));

  EXPECT_EQ(client.Post("/place/save", {{"path", "places/x"}, {"offset", "1,0"}}).status, 400);
  EXPECT_EQ(client.Post("/place/save", {{"path", "places/x"}, {"offset", "1,a,0"}}).status, 400);

  rig.service.Stop();
  rig.backend.Finish();
}

TEST(PlaceOffsetE2eTest, OffsetIsFromTheRobotNotFromTheLastKeyframe) {
  Rig rig;
  ASSERT_GT(rig.service.port(), 0);
  TestClient client(rig.service.port());
  rig.backend.Start(TestTime(0));
  rig.service.OnBackendStarted();
  const int step = 5;
  rig.FeedTo(step, 0.8);
  // The host moves on past the last keyframe; the graph has not seen it yet.
  const Eigen::Affine2d ahead = utils::transform::FromXYTheta(0.15, 0.03, 0.05);
  rig.host.Update(TestTime(step), rig.odometry[step] * ahead,
                  SimulateScan(LoopRoom(), LoopGroundTruth(step) * ahead), 0.8);

  const Reply saved = client.Post("/place/save", {{"path", "places/ahead"}, {"offset", "1,0,0"}});
  ASSERT_EQ(JsonRaw(saved.body, "ok"), "true") << saved.body;
  const Reply resolved = client.Get("/anchors/" + JsonRaw(saved.body, "anchor") + "?robot=1");
  ASSERT_EQ(JsonRaw(resolved.body, "ok"), "true") << resolved.body;
  const Eigen::Affine2d robot = utils::transform::FromXYTheta(
      JsonNumber(resolved.body, "robot.x"), JsonNumber(resolved.body, "robot.y"),
      JsonNumber(resolved.body, "robot.theta"));
  const Eigen::Affine2d expected = robot * utils::transform::FromXYTheta(1.0, 0.0, 0.0);
  constexpr double kTight = 1e-3;
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.x"), expected.translation().x(), kTight);
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.y"), expected.translation().y(), kTight);
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.theta"), utils::transform::GetYaw(expected), kTight);

  // dtheta is echoed and stored wrapped to (-pi, pi].
  const Reply wrapped =
      client.Post("/place/save", {{"path", "places/turned"}, {"offset", "0,0,7"}});
  ASSERT_EQ(JsonRaw(wrapped.body, "ok"), "true") << wrapped.body;
  const size_t echoed = wrapped.body.find("\"offset\":[0,0,");
  ASSERT_NE(echoed, std::string::npos) << wrapped.body;
  EXPECT_NEAR(std::strtod(wrapped.body.c_str() + echoed + 14, nullptr), 7.0 - 2.0 * M_PI, 1e-6);

  rig.service.Stop();
  rig.backend.Finish();
}

}  // namespace
}  // namespace evergreenslam::agent

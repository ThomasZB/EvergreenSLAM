/**
 * @file pose_timeline_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief GET /pose, /anchors/{id}?at= and the serial executor over real HTTP, in front of a
 * PoseGraph fed by the simulated laser loop.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "service/agent_service.h"
#include "service_test_client.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

using lifelong::PoseGraph;
using lifelong::testing::DriftedOdometry;
using lifelong::testing::Frontend;
using lifelong::testing::LoopGroundTruth;
using lifelong::testing::LoopRoom;
using lifelong::testing::SimulateScan;
using lifelong::testing::TestTime;
using testing::JsonNumber;
using testing::JsonRaw;
using testing::Reply;
using testing::SimulatedHost;
using testing::TestClient;

constexpr int kSteps = 30;
constexpr double kExact = 1e-6;

std::string Reason(const Reply& reply) { return JsonRaw(reply.body, "reason"); }
bool IsObject(const Reply& reply, const std::string& key) {
  return JsonRaw(reply.body, key).rfind("{", 0) == 0;
}

struct Fixture {
  explicit Fixture(const std::string& name)
      : backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()),
                lifelong::testing::MakeTempDir(name)),
        odometry(DriftedOdometry(kSteps)) {}

  void Start() { backend.Start(TestTime(0), utils::transform::FromXYTheta(3.1, -1.7, 0.4)); }

  // Each step's local velocity is (step, 0.5, 0.1 * step), so a reply names its sample.
  void Feed(Frontend& frontend, int first, int last) {
    for (int step = first; step <= last; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
      host.Update(TestTime(step), odometry[step], SimulateScan(LoopRoom(), LoopGroundTruth(step)),
                  0.0, Eigen::Vector3d(static_cast<double>(step), 0.5, 0.1 * step));
    }
  }

  PoseGraph backend;
  SimulatedHost host;
  const std::vector<Eigen::Affine2d> odometry;
};

void ExpectPose(const Reply& reply, const std::string& key, const Eigen::Affine2d& expected) {
  ASSERT_TRUE(IsObject(reply, key)) << reply.body;
  EXPECT_NEAR(JsonNumber(reply.body, key + ".x"), expected.translation().x(), kExact);
  EXPECT_NEAR(JsonNumber(reply.body, key + ".y"), expected.translation().y(), kExact);
  EXPECT_NEAR(utils::transform::NormalizeAngle(JsonNumber(reply.body, key + ".theta") -
                                               utils::transform::GetYaw(expected)),
              0.0, kExact);
}

TEST(PoseTimelineE2eTest, PoseIsTheNewestSampleInTheGlobalFrameAndNullAcrossADrop) {
  Fixture fixture("evergreenslam_agent_pose");
  AgentServiceHooks hooks = fixture.host.Hooks();
  std::string pose_during_drop;
  int port = 0;
  hooks.drop_session = [&](const SessionDropRequest& request) {
    // The host still holds the old frontend's sample and the old alignment is published.
    TestClient side(port);
    pose_during_drop = side.Get("/pose").body;
    const PoseGraph::DropFedSessionResult result = fixture.backend.DropFedSession(request.id);
    fixture.host.OnFedSessionDropped();
    return result;
  };
  AgentService service(fixture.backend, hooks, AgentServiceOption{});
  ASSERT_GT(service.port(), 0);
  port = service.port();
  TestClient client(port);

  // No task: answers before the backend starts.
  Reply pose = client.Get("/pose");
  ASSERT_EQ(pose.status, 200);
  EXPECT_EQ(JsonRaw(pose.body, "ok"), "true");
  EXPECT_EQ(JsonRaw(pose.body, "pose"), "null");
  EXPECT_EQ(JsonRaw(pose.body, "seq"), "null");
  EXPECT_EQ(JsonRaw(pose.body, "velocity"), "null");

  fixture.Start();
  service.OnBackendStarted();
  EXPECT_EQ(JsonRaw(client.Get("/pose").body, "pose"), "null");

  Frontend frontend(fixture.backend, fixture.odometry);
  fixture.Feed(frontend, 0, kSteps - 1);
  const int last = kSteps - 1;
  pose = client.Get("/pose");
  ASSERT_EQ(JsonRaw(pose.body, "ok"), "true") << pose.body;
  EXPECT_EQ(JsonRaw(pose.body, "seq"), std::to_string(kSteps));
  EXPECT_EQ(JsonRaw(pose.body, "stamp_ns"), std::to_string(common::ToUnixNanos(TestTime(last))));
  EXPECT_GE(JsonNumber(pose.body, "age_s"), 0.0);
  EXPECT_LT(JsonNumber(pose.body, "age_s"), 10.0);
  const std::optional<Eigen::Affine2d> session_to_global = fixture.backend.ActiveSessionToGlobal();
  ASSERT_TRUE(session_to_global.has_value());
  ExpectPose(pose, "pose", Eigen::Affine2d(*session_to_global * fixture.odometry[last]));
  // The local velocity rotated into the robot frame.
  const Eigen::Vector2d body =
      fixture.odometry[last].linear().transpose() * Eigen::Vector2d(static_cast<double>(last), 0.5);
  EXPECT_NEAR(JsonNumber(pose.body, "velocity.vx"), body.x(), kExact);
  EXPECT_NEAR(JsonNumber(pose.body, "velocity.vy"), body.y(), kExact);
  EXPECT_NEAR(JsonNumber(pose.body, "velocity.omega"), 0.1 * last, kExact);
  // The same robot pose /here derives in a task.
  const Reply here = client.Get("/here");
  EXPECT_NEAR(JsonNumber(here.body, "robot.x"), JsonNumber(pose.body, "pose.x"), 1e-3);
  EXPECT_NEAR(JsonNumber(here.body, "robot.y"), JsonNumber(pose.body, "pose.y"), 1e-3);

  const std::string fed = JsonRaw(client.Get("/status").body, "fed_session");
  const Reply plan = client.Post("/sessions/rm/plan", {{"id", fed}});
  const Reply applied = client.Post(
      "/sessions/rm/apply", {{"id", fed}, {"plan_token", JsonRaw(plan.body, "plan_token")}});
  ASSERT_EQ(JsonRaw(applied.body, "ok"), "true") << applied.body;
  // During the drop the sample and the alignment were both there, but not paired.
  EXPECT_EQ(JsonRaw(pose_during_drop, "seq"), std::to_string(kSteps)) << pose_during_drop;
  EXPECT_EQ(JsonRaw(pose_during_drop, "pose"), "null") << pose_during_drop;

  pose = client.Get("/pose");
  EXPECT_EQ(JsonRaw(pose.body, "pose"), "null") << pose.body;
  EXPECT_EQ(JsonRaw(pose.body, "seq"), "null") << pose.body;

  // A new frontend: its sample shows at once, the pose once its session is aligned.
  const std::vector<Eigen::Affine2d> fresh = DriftedOdometry(5);
  fixture.host.Update(TestTime(0), fresh[0], SimulateScan(LoopRoom(), LoopGroundTruth(0)));
  pose = client.Get("/pose");
  EXPECT_EQ(JsonRaw(pose.body, "seq"), std::to_string(kSteps + 1)) << pose.body;
  EXPECT_EQ(JsonRaw(pose.body, "pose"), "null") << pose.body;
  Frontend fresh_frontend(fixture.backend, fresh);
  for (int step = 0; step < 5; ++step) {
    fresh_frontend.Feed(step);
  }
  fixture.backend.WaitUntilQuiescent();
  fixture.host.Update(TestTime(4), fresh[4], SimulateScan(LoopRoom(), LoopGroundTruth(4)));
  const std::optional<Eigen::Affine2d> fresh_to_global = fixture.backend.ActiveSessionToGlobal();
  ASSERT_TRUE(fresh_to_global.has_value());
  pose = client.Get("/pose");
  EXPECT_EQ(JsonRaw(pose.body, "seq"), std::to_string(kSteps + 2)) << pose.body;
  ExpectPose(pose, "pose", Eigen::Affine2d(*fresh_to_global * fresh[4]));
  service.Stop();
  fixture.backend.Finish();
}

// Holds every current_frame call (inside a backend task) until released.
class Gate {
 public:
  void Wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    ++num_entered_;
    changed_.notify_all();
    changed_.wait(lock, [this] { return open_; });
  }
  void AwaitEntered() {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this] { return num_entered_ > 0; });
  }
  void Open() {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    changed_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  int num_entered_ = 0;
  bool open_ = false;
};

double SecondsFor(const std::function<void()>& function) {
  const auto start = std::chrono::steady_clock::now();
  function();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

TEST(PoseTimelineE2eTest, ASlowHandlerDelaysNeitherPoseNorTheBusyRefusal) {
  Fixture fixture("evergreenslam_agent_pool");
  AgentServiceHooks hooks = fixture.host.Hooks();
  Gate gate;
  std::atomic<bool> hold{false};
  hooks.current_frame = [&, frame = hooks.current_frame]() {
    if (hold.load()) {
      gate.Wait();
    }
    return frame();
  };
  AgentService service(fixture.backend, hooks, AgentServiceOption{});
  ASSERT_GT(service.port(), 0);
  const int port = service.port();
  fixture.Start();
  service.OnBackendStarted();
  Frontend frontend(fixture.backend, fixture.odometry);
  fixture.Feed(frontend, 0, 10);

  hold = true;
  std::future<Reply> slow =
      std::async(std::launch::async, [port] { return TestClient(port).Get("/here"); });
  gate.AwaitEntered();

  TestClient client(port);
  Reply pose;
  EXPECT_LT(SecondsFor([&] { pose = client.Get("/pose"); }), 0.5);
  EXPECT_TRUE(IsObject(pose, "pose")) << pose.body;
  EXPECT_EQ(client.Get("/root").status, 200);

  // Four more wait behind the held one; the next is refused at once.
  std::vector<std::future<Reply>> waiting;
  for (int i = 0; i < 4; ++i) {
    waiting.push_back(
        std::async(std::launch::async, [port] { return TestClient(port).Get("/status"); }));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  Reply busy;
  EXPECT_LT(SecondsFor([&] { busy = client.Get("/sessions"); }), 0.5);
  EXPECT_EQ(busy.status, 503);
  EXPECT_EQ(Reason(busy), "busy");
  EXPECT_EQ(client.Get("/pose").status, 200);

  gate.Open();
  EXPECT_EQ(slow.get().status, 200);
  for (std::future<Reply>& reply : waiting) {
    EXPECT_EQ(JsonRaw(reply.get().body, "ok"), "true");
  }
  EXPECT_EQ(JsonRaw(client.Get("/sessions").body, "ok"), "true");
  service.Stop();
  fixture.backend.Finish();
}

TEST(PoseTimelineE2eTest, AnchorAtInterpolatesTheRobotPoseAndRefusesOutsideTheHistory) {
  Fixture fixture("evergreenslam_agent_pose_at");
  AgentService service(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
  ASSERT_GT(service.port(), 0);
  TestClient client(service.port());
  fixture.Start();
  service.OnBackendStarted();
  Frontend frontend(fixture.backend, fixture.odometry);
  fixture.Feed(frontend, 0, kSteps - 1);
  const int last = kSteps - 1;
  const std::string anchor = std::to_string(lifelong::testing::SaveAnchor(fixture.backend)->id);
  const std::string path = "/anchors/" + anchor;

  // The robot now, from the same alignment the at= reads use.
  const Reply now = client.Get(path + "?robot=1");
  ASSERT_TRUE(IsObject(now, "robot")) << now.body;
  const Eigen::Affine2d robot_now = utils::transform::FromXYTheta(
      JsonNumber(now.body, "robot.x"), JsonNumber(now.body, "robot.y"),
      JsonNumber(now.body, "robot.theta"));
  const Eigen::Affine2d session_to_global(robot_now * fixture.odometry[last].inverse());
  const auto at = [&](common::Time time) {
    return client.Get(path + "?at=" + std::to_string(common::ToUnixNanos(time)));
  };

  const common::Time between = TestTime(last - 3) + common::FromSeconds(0.05);
  const Reply interpolated = at(between);
  ASSERT_EQ(JsonRaw(interpolated.body, "ok"), "true") << interpolated.body;
  EXPECT_EQ(JsonRaw(interpolated.body, "at_ns"), std::to_string(common::ToUnixNanos(between)));
  EXPECT_EQ(JsonRaw(interpolated.body, "aligned_to_base"), "false");
  EXPECT_TRUE(IsObject(interpolated, "pose")) << interpolated.body;
  ExpectPose(interpolated, "robot",
             Eigen::Affine2d(session_to_global *
                             utils::transform::Interpolate(fixture.odometry[last - 3],
                                                           fixture.odometry[last - 2], 0.5)));
  ExpectPose(at(TestTime(last - 3)), "robot",
             Eigen::Affine2d(session_to_global * fixture.odometry[last - 3]));
  // Up to 0.2 s past the newest sample answers the newest.
  ExpectPose(at(TestTime(last) + common::FromSeconds(0.15)), "robot", robot_now);

  Reply refused = at(TestTime(last) + common::FromSeconds(0.3));
  EXPECT_EQ(Reason(refused), "in_future");
  EXPECT_EQ(JsonRaw(refused.body, "robot"), "null");
  EXPECT_TRUE(IsObject(refused, "pose")) << refused.body;
  EXPECT_EQ(Reason(at(TestTime(0) - common::FromSeconds(1.0))), "too_old");
  fixture.host.Update(TestTime(last) + common::FromSeconds(1.0), fixture.odometry[last],
                      SimulateScan(LoopRoom(), LoopGroundTruth(last)));
  EXPECT_EQ(Reason(at(TestTime(last) + common::FromSeconds(0.5))), "gap");
  EXPECT_EQ(client.Get(path + "?at=soon").status, 400);
  fixture.host.OnFedSessionDropped();
  EXPECT_EQ(Reason(at(TestTime(last))), "no_robot_pose");
  EXPECT_EQ(
      Reason(client.Get("/anchors/999?at=" + std::to_string(common::ToUnixNanos(TestTime(last))))),
      "unknown_anchor");
  service.Stop();
  fixture.backend.Finish();
}

}  // namespace
}  // namespace evergreenslam::agent

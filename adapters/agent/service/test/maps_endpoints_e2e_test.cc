/**
 * @file maps_endpoints_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief /maps, /maps/new, /maps/open and the fed session's rm over real HTTP, with a recording
 * switch hook and a drop hook that drops on the test's backend.
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "lifelong/map_manager/map_root.h"
#include "service/agent_service.h"
#include "service_test_client.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

using lifelong::MapRoot;
using lifelong::PoseGraph;
using lifelong::testing::DriftedOdometry;
using lifelong::testing::Frontend;
using lifelong::testing::TestTime;
using testing::JsonRaw;
using testing::Reply;
using testing::SimulatedHost;
using testing::TestClient;

std::string Reason(const Reply& reply) { return JsonRaw(reply.body, "reason"); }

struct MapsFixture {
  explicit MapsFixture(const std::string& name)
      : root(lifelong::testing::MakeTempDir(name)),
        backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()),
                MakeMapDir(root)) {}

  static std::string MakeMapDir(const MapRoot& root) {
    fs::create_directories(root.Resolve("default"));
    return root.Resolve("default");
  }

  AgentServiceOption Option() const {
    AgentServiceOption option;
    option.map_root = root.root();
    option.map_name = "default";
    return option;
  }

  MapRoot root;
  PoseGraph backend;
  SimulatedHost host;
};

TEST(MapsEndpointsE2eTest, ListsAndRequestsSwitches) {
  MapsFixture fixture("evergreenslam_agent_maps");
  const MapRoot& root = fixture.root;
  std::vector<MapSwitchRequest> requests;
  AgentServiceHooks hooks = fixture.host.Hooks();
  hooks.switch_map = [&requests](const MapSwitchRequest& request) {
    requests.push_back(request);
    return true;
  };
  AgentService service(fixture.backend, hooks, fixture.Option());
  ASSERT_GT(service.port(), 0);
  TestClient client(service.port());

  const Reply maps = client.Get("/maps");
  ASSERT_EQ(JsonRaw(maps.body, "ok"), "true") << maps.body;
  EXPECT_EQ(JsonRaw(maps.body, "current"), "default");
  EXPECT_EQ(JsonRaw(maps.body, "maps.name"), "default");
  EXPECT_EQ(JsonRaw(maps.body, "maps.current"), "true");
  EXPECT_EQ(JsonRaw(client.Get("/root").body, "map"), "default");

  EXPECT_EQ(Reason(client.Post("/maps/new", {{"name", "default"}})), "exists");
  const Reply bad = client.Post("/maps/new", {{"name", "bad/name"}});
  EXPECT_EQ(bad.status, 400);
  EXPECT_EQ(Reason(bad), "not_slug");
  EXPECT_EQ(Reason(client.Post("/maps/open", {{"name", "missing"}})), "unknown_map");
  const Reply same = client.Post("/maps/open", {{"name", "default"}});
  EXPECT_EQ(JsonRaw(same.body, "ok"), "true") << same.body;
  EXPECT_EQ(JsonRaw(same.body, "pending"), "false");
  EXPECT_TRUE(requests.empty());

  const Reply fresh = client.Post("/maps/new", {{"name", "fresh"}});
  ASSERT_EQ(JsonRaw(fresh.body, "ok"), "true") << fresh.body;
  EXPECT_EQ(JsonRaw(fresh.body, "map"), "fresh");
  EXPECT_EQ(JsonRaw(fresh.body, "pending"), "true");
  EXPECT_TRUE(fs::is_directory(root.Resolve("fresh")));
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].name, "fresh");
  EXPECT_TRUE(requests[0].create);

  // Pending: every further switch is refused, nothing is created, the hook is not called.
  EXPECT_EQ(Reason(client.Post("/maps/new", {{"name", "other"}})), "switching");
  EXPECT_FALSE(fs::exists(root.Resolve("other")));
  EXPECT_EQ(Reason(client.Post("/maps/open", {{"name", "default"}})), "switching");
  EXPECT_EQ(requests.size(), 1u);
  service.Stop();
}

// Started and fed a few keyframes, one anchor saved in the fed session.
struct FedFixture : MapsFixture {
  explicit FedFixture(const std::string& name) : MapsFixture(name) {}

  void Boot(AgentService& service) {
    backend.Start(TestTime(0));
    service.OnBackendStarted();
    const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(kSteps);
    Frontend frontend(backend, odometry);
    for (int step = 0; step < kSteps; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    anchor = std::to_string(lifelong::testing::SaveAnchor(backend)->id);
  }

  static constexpr int kSteps = 30;
  std::string anchor;
};

TEST(MapsEndpointsE2eTest, FedSessionRmDropsItInProcess) {
  FedFixture fixture("evergreenslam_agent_drop");
  std::vector<SessionDropRequest> drops;
  AgentServiceHooks hooks = fixture.host.Hooks();
  hooks.drop_session = [&drops, &fixture](const SessionDropRequest& request) {
    drops.push_back(request);
    return fixture.backend.DropFedSession(request.id);
  };
  int switches = 0;
  hooks.switch_map = [&switches](const MapSwitchRequest&) { return ++switches > 0; };
  AgentService service(fixture.backend, hooks, fixture.Option());
  ASSERT_GT(service.port(), 0);
  fixture.Boot(service);
  TestClient client(service.port());
  const std::string fed = JsonRaw(client.Get("/status").body, "fed_session");
  const std::string next = std::to_string(std::stoi(fed) + 1);

  const Reply plan = client.Post("/sessions/rm/plan", {{"id", fed}});
  ASSERT_EQ(JsonRaw(plan.body, "ok"), "true") << plan.body;
  EXPECT_EQ(JsonRaw(plan.body, "drops_fed"), "true");
  EXPECT_NE(JsonRaw(plan.body, "note").find("replaced by a fresh one"), std::string::npos);
  EXPECT_NE(plan.body.find("\"would_delete\":[[" + fed + ","), std::string::npos) << plan.body;
  EXPECT_NE(plan.body.find("\"anchors_orphaned\":[" + fixture.anchor + "]"), std::string::npos)
      << plan.body;
  const std::string token = JsonRaw(plan.body, "plan_token");

  EXPECT_EQ(Reason(client.Post("/sessions/rm/apply", {{"id", fed}, {"plan_token", "0"}})),
            "plan_changed");
  EXPECT_TRUE(drops.empty());
  const Reply applied = client.Post("/sessions/rm/apply", {{"id", fed}, {"plan_token", token}});
  ASSERT_EQ(JsonRaw(applied.body, "ok"), "true") << applied.body;
  EXPECT_EQ(JsonRaw(applied.body, "removed"), fed);
  EXPECT_EQ(JsonRaw(applied.body, "fed_session"), next);
  EXPECT_EQ(JsonRaw(applied.body, "pose_lost"), "true");
  EXPECT_EQ(applied.body.find("\"pending\""), std::string::npos) << applied.body;
  EXPECT_NE(applied.body.find("\"anchors_orphaned\":[" + fixture.anchor + "]"), std::string::npos)
      << applied.body;
  ASSERT_EQ(drops.size(), 1u);
  EXPECT_EQ(std::to_string(drops[0].id.session_index), fed);

  const Reply sessions = client.Get("/sessions");
  EXPECT_EQ(JsonRaw(sessions.body, "fed"), next);
  EXPECT_EQ(sessions.body.find("\"id\":" + fed + ","), std::string::npos) << sessions.body;
  EXPECT_NE(sessions.body.find("\"id\":" + next + ","), std::string::npos) << sessions.body;
  EXPECT_EQ(JsonRaw(client.Get("/status").body, "fed_session"), next);
  EXPECT_EQ(JsonRaw(client.Get("/anchors/" + fixture.anchor).body, "state"), "orphan");
  EXPECT_EQ(Reason(client.Post("/sessions/rm/apply", {{"id", fed}, {"plan_token", token}})),
            "plan_changed");
  // Nothing is pending: the new session takes plans and switches at once.
  EXPECT_EQ(Reason(client.Post("/place/save", {{"path", "places/dock"}})), "no_keyframe");
  EXPECT_NE(JsonRaw(client.Post("/sessions/freeze/plan", {{"force", "true"}}).body, "rejection"),
            "switching");
  EXPECT_EQ(drops.size(), 1u);
  EXPECT_EQ(switches, 0);

  // A fresh frontend feeds the new session from node index 0.
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(5);
  Frontend frontend(fixture.backend, odometry);
  for (int step = 0; step < 5; ++step) {
    frontend.Feed(step);
  }
  fixture.backend.WaitUntilQuiescent();
  EXPECT_EQ(JsonRaw(client.Get("/sessions").body, "nodes"), "5");
  service.Stop();
  fixture.backend.Finish();
}

TEST(MapsEndpointsE2eTest, FedSessionRmThatNoLongerNamesTheFedSessionIsSessionChanged) {
  FedFixture fixture("evergreenslam_agent_drop_changed");
  AgentServiceHooks hooks = fixture.host.Hooks();
  hooks.drop_session = [](const SessionDropRequest&) {
    PoseGraph::DropFedSessionResult result;
    result.refusal = PoseGraph::DropFedSessionResult::Refusal::NOT_FED;
    return result;
  };
  AgentService service(fixture.backend, hooks, fixture.Option());
  ASSERT_GT(service.port(), 0);
  fixture.Boot(service);
  TestClient client(service.port());
  const std::string fed = JsonRaw(client.Get("/status").body, "fed_session");

  const Reply plan = client.Post("/sessions/rm/plan", {{"id", fed}});
  ASSERT_EQ(JsonRaw(plan.body, "ok"), "true") << plan.body;
  const Reply applied = client.Post(
      "/sessions/rm/apply", {{"id", fed}, {"plan_token", JsonRaw(plan.body, "plan_token")}});
  EXPECT_EQ(Reason(applied), "session_changed");
  EXPECT_EQ(JsonRaw(applied.body, "removed"), "null");
  EXPECT_EQ(JsonRaw(client.Get("/status").body, "fed_session"), fed);
  EXPECT_EQ(JsonRaw(client.Get("/anchors/" + fixture.anchor).body, "state"), "pending");
  service.Stop();
  fixture.backend.Finish();
}

TEST(MapsEndpointsE2eTest, FedSessionRmWhileAMapSwitchIsPendingIsSwitching) {
  FedFixture fixture("evergreenslam_agent_drop_after_switch");
  int drops = 0;
  AgentServiceHooks hooks = fixture.host.Hooks();
  hooks.drop_session = [&drops](const SessionDropRequest&) {
    ++drops;
    PoseGraph::DropFedSessionResult result;
    result.refusal = PoseGraph::DropFedSessionResult::Refusal::NOT_FED;
    return result;
  };
  hooks.switch_map = [](const MapSwitchRequest&) { return true; };
  AgentService service(fixture.backend, hooks, fixture.Option());
  ASSERT_GT(service.port(), 0);
  fixture.Boot(service);
  TestClient client(service.port());
  const std::string fed = JsonRaw(client.Get("/status").body, "fed_session");

  const Reply plan = client.Post("/sessions/rm/plan", {{"id", fed}});
  ASSERT_EQ(JsonRaw(plan.body, "ok"), "true") << plan.body;
  ASSERT_EQ(JsonRaw(client.Post("/maps/new", {{"name", "fresh"}}).body, "ok"), "true");
  EXPECT_EQ(JsonRaw(client.Post("/sessions/rm/plan", {{"id", fed}}).body, "rejection"),
            "switching");
  EXPECT_EQ(Reason(client.Post("/sessions/rm/apply",
                               {{"id", fed}, {"plan_token", JsonRaw(plan.body, "plan_token")}})),
            "switching");
  EXPECT_EQ(drops, 0);
  service.Stop();
  fixture.backend.Finish();
}

TEST(MapsEndpointsE2eTest, FedSessionRmWithoutADropHookIsNotSupported) {
  FedFixture fixture("evergreenslam_agent_drop_unsupported");
  AgentService service(fixture.backend, fixture.host.Hooks(), fixture.Option());
  ASSERT_GT(service.port(), 0);
  fixture.Boot(service);
  TestClient client(service.port());
  const std::string fed = JsonRaw(client.Get("/status").body, "fed_session");

  const Reply plan = client.Post("/sessions/rm/plan", {{"id", fed}});
  EXPECT_EQ(JsonRaw(plan.body, "ok"), "false");
  EXPECT_EQ(JsonRaw(plan.body, "rejection"), "not_supported");
  EXPECT_EQ(JsonRaw(plan.body, "drops_fed"), "false");
  EXPECT_EQ(Reason(client.Post("/sessions/rm/apply",
                               {{"id", fed}, {"plan_token", JsonRaw(plan.body, "plan_token")}})),
            "plan_changed");
  EXPECT_EQ(JsonRaw(client.Get("/sessions").body, "role"), "fed");
  service.Stop();
  fixture.backend.Finish();
}

TEST(MapsEndpointsE2eTest, RefusedHookLeavesTheDirectory) {
  MapsFixture fixture("evergreenslam_agent_maps_refused");
  AgentServiceHooks hooks = fixture.host.Hooks();
  hooks.switch_map = [](const MapSwitchRequest&) { return false; };
  AgentService service(fixture.backend, hooks, fixture.Option());
  ASSERT_GT(service.port(), 0);
  TestClient client(service.port());

  EXPECT_EQ(Reason(client.Post("/maps/new", {{"name", "fresh"}})), "not_supported");
  EXPECT_TRUE(fs::is_directory(fixture.root.Resolve("fresh")));
  EXPECT_EQ(Reason(client.Post("/maps/open", {{"name", "fresh"}})), "not_supported");
  service.Stop();
}

TEST(MapsEndpointsE2eTest, HostWithoutRootIsNotSupported) {
  MapsFixture fixture("evergreenslam_agent_maps_rootless");
  AgentService rootless(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
  ASSERT_GT(rootless.port(), 0);
  TestClient client(rootless.port());
  EXPECT_EQ(Reason(client.Get("/maps")), "not_supported");
  EXPECT_EQ(Reason(client.Post("/maps/new", {{"name", "other"}})), "not_supported");
  EXPECT_EQ(Reason(client.Post("/maps/open", {{"name", "default"}})), "not_supported");
  EXPECT_EQ(JsonRaw(client.Get("/root").body, "map"), "null");
  rootless.Stop();
}

}  // namespace
}  // namespace evergreenslam::agent

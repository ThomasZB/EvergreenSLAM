/**
 * @file maps_endpoints_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief /maps, /maps/new and /maps/open over real HTTP, with a recording switch hook.
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

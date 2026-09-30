/**
 * @file zones_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief GET /zones and the zones view layer over real HTTP, on the simulated laser loop.
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "common/file.h"
#include "service/agent_service.h"
#include "service_test_client.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

using lifelong::PoseGraph;
using lifelong::testing::Frontend;
using lifelong::testing::kNodesPerLap;
using lifelong::testing::LoopGroundTruth;
using lifelong::testing::LoopRoom;
using lifelong::testing::SimulateScan;
using lifelong::testing::TestTime;
using testing::JsonNumber;
using testing::JsonRaw;
using testing::MatchingClose;
using testing::Reply;
using testing::SimulatedHost;
using testing::TestClient;

constexpr char kSquare[] = "polygon: [[1.0, -1.0], [3.0, -1.0], [3.0, 1.0], [1.0, 1.0]]\n";

void WriteText(const fs::path& path, const std::string& text) {
  fs::create_directories(path.parent_path());
  ASSERT_TRUE(common::WriteFileAtomically(path.string(), text)) << path;
}

std::string Keepout(const std::string& frame) {
  return "kind: keepout\nframe: " + frame + "\n" + kSquare;
}

// The zones[] element for `path`; empty when absent.
std::string ZoneObject(const std::string& body, const std::string& path) {
  const size_t begin = body.find("{\"path\":\"" + path + "\"");
  if (begin == std::string::npos) {
    return "";
  }
  return body.substr(begin, MatchingClose(body, begin) - begin + 1);
}

std::vector<Eigen::Vector2d> PolygonXy(const std::string& zone) {
  std::vector<Eigen::Vector2d> vertices;
  size_t at = zone.find("\"polygon_xy\":[");
  if (at == std::string::npos) {
    return vertices;
  }
  at += std::string("\"polygon_xy\":[").size();
  double x = 0.0;
  double y = 0.0;
  while (zone[at] == '[' && std::sscanf(zone.c_str() + at, "[%lf,%lf]", &x, &y) == 2) {
    vertices.emplace_back(x, y);
    at = zone.find(']', at) + 1;
    if (zone[at] == ',') {
      ++at;
    }
  }
  return vertices;
}

void FeedTo(PoseGraph& backend, Frontend& frontend, SimulatedHost& host,
            const std::vector<Eigen::Affine2d>& odometry, int& step, int last_step) {
  for (; step <= last_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
    host.Update(TestTime(step), odometry[step], SimulateScan(LoopRoom(), LoopGroundTruth(step)));
  }
}

TEST(ZonesE2eTest, ResolvesInTheAnchorFrameAndReportsWhyNot) {
  const std::string map_dir = lifelong::testing::MakeTempDir("evergreenslam_agent_zones");
  const fs::path places = fs::path(map_dir) / "memory" / "places";
  const std::vector<Eigen::Affine2d> odometry =
      lifelong::testing::GroundTruthOdometry(kNodesPerLap);
  PoseGraph backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()), map_dir);
  SimulatedHost host;
  AgentService service(backend, host.Hooks(), AgentServiceOption{});
  ASSERT_GT(service.port(), 0);
  backend.Start(TestTime(0), utils::transform::FromXYTheta(3.1, -1.7, 0.4));
  service.OnBackendStarted();
  TestClient client(service.port());
  Frontend frontend(backend, odometry);
  int step = 0;
  FeedTo(backend, frontend, host, odometry, step, 14);
  const Reply saved = client.Post("/place/save", {{"path", "places/kitchen"}});
  ASSERT_EQ(JsonRaw(saved.body, "ok"), "true") << saved.body;
  const std::string anchor = JsonRaw(saved.body, "anchor");

  EXPECT_NE(client.Get("/zones").body.find("\"zones\":[]"), std::string::npos);

  WriteText(places / "kitchen/slope/zone.yaml", Keepout("places/kitchen"));
  WriteText(places / "kitchen/moved/zone.yaml", Keepout("places/dock"));
  WriteText(places / "kitchen/ramp/zone.yaml", std::string("kind: slowdown\n") + kSquare);
  WriteText(places / "kitchen/bad/zone.yaml", "kind: keepout\npolygon: [[0, 0], [1, 0]]\n");
  WriteText(places / "kitchen/skills/x/zone.yaml", Keepout("places/kitchen"));
  WriteText(places / "kitchen/fridge/place.yaml", "anchor: " + anchor + "\n");
  WriteText(places / "kitchen/fridge/zone.yaml", Keepout("places/kitchen/fridge"));
  WriteText(places / "nowhere/zone.yaml", Keepout("places/kitchen"));
  WriteText(places / "ghost/place.yaml", "anchor: 999\n");
  WriteText(places / "ghost/puddle/zone.yaml", Keepout("places/ghost"));
  WriteText(places / "kitchen/table/spill/zone.yaml", Keepout("places/kitchen"));
  WriteText(places / "cellar/place.yaml", "anchor: [oops\n");
  WriteText(places / "cellar/stairs/zone.yaml", Keepout("places/cellar"));
  WriteText(places / "kitchen/typo/zone.yaml",
            "kind: keepout\nframe: places/kitchen\npolygon: [[0, 0], [1000, 0], [0, 1]]\n");

  const Reply zones = client.Get("/zones");
  ASSERT_EQ(JsonRaw(zones.body, "ok"), "true") << zones.body;
  EXPECT_GE(JsonNumber(zones.body, "at_num_solves"), 0);
  EXPECT_EQ(zones.body.find("skills"), std::string::npos) << zones.body;

  const std::string slope = ZoneObject(zones.body, "places/kitchen/slope");
  ASSERT_FALSE(slope.empty()) << zones.body;
  EXPECT_EQ(JsonRaw(slope, "kind"), "keepout");
  EXPECT_EQ(JsonRaw(slope, "frame_path"), "places/kitchen");
  EXPECT_EQ(JsonRaw(slope, "anchor"), anchor);
  EXPECT_EQ(JsonRaw(slope, "state"), "pending");
  EXPECT_EQ(JsonRaw(slope, "reason"), "null");
  const Reply resolved = client.Get("/anchors/" + anchor);
  const Eigen::Affine2d anchor_pose = utils::transform::FromXYTheta(
      JsonNumber(resolved.body, "pose.x"), JsonNumber(resolved.body, "pose.y"),
      JsonNumber(resolved.body, "pose.theta"));
  const std::vector<Eigen::Vector2d> xy = PolygonXy(slope);
  ASSERT_EQ(xy.size(), 4u) << slope;
  const Eigen::Vector2d local[] = {{1.0, -1.0}, {3.0, -1.0}, {3.0, 1.0}, {1.0, 1.0}};
  for (size_t i = 0; i < xy.size(); ++i) {
    EXPECT_NEAR((xy[i] - anchor_pose * local[i]).norm(), 0.0, 1e-3) << i << ": " << slope;
  }
  // The anchor frame is not the map frame: the seed rotates it.
  EXPECT_GT((xy[0] - local[0]).norm(), 1.0);

  // Two levels up, through a directory without place.yaml.
  const std::string spill = ZoneObject(zones.body, "places/kitchen/table/spill");
  EXPECT_EQ(JsonRaw(spill, "frame_path"), "places/kitchen") << spill;
  EXPECT_EQ(JsonRaw(spill, "reason"), "null") << spill;
  EXPECT_EQ(PolygonXy(spill).size(), 4u) << spill;

  const std::string moved = ZoneObject(zones.body, "places/kitchen/moved");
  EXPECT_EQ(JsonRaw(moved, "reason"), "frame_mismatch");
  EXPECT_EQ(JsonRaw(moved, "frame"), "places/dock");
  EXPECT_EQ(PolygonXy(moved).size(), 4u) << moved;

  const auto reason_of = [&zones](const std::string& path) {
    const std::string zone = ZoneObject(zones.body, path);
    EXPECT_FALSE(zone.empty()) << path << " missing in " << zones.body;
    EXPECT_EQ(JsonRaw(zone, "polygon_xy"), "null") << zone;
    return JsonRaw(zone, "reason");
  };
  EXPECT_EQ(reason_of("places/kitchen/ramp"), "unknown_kind");
  EXPECT_EQ(reason_of("places/kitchen/bad"), "bad_zone_file");
  EXPECT_EQ(reason_of("places/kitchen/fridge"), "holds_place");
  EXPECT_EQ(reason_of("places/nowhere"), "no_place");
  EXPECT_EQ(reason_of("places/ghost/puddle"), "unknown_anchor");
  EXPECT_EQ(JsonRaw(ZoneObject(zones.body, "places/ghost/puddle"), "state"), "orphan");
  EXPECT_EQ(reason_of("places/cellar/stairs"), "unreadable_place");
  EXPECT_EQ(reason_of("places/kitchen/typo"), "too_large");

  const Reply map = client.Get("/view?preset=map");
  ASSERT_EQ(map.status, 200) << map.body;
  EXPECT_EQ(map.Header("X-EGS-Layers"), "map,places,zones");
  EXPECT_NE(map.Header("X-EGS-Legend").find("zones: 3 keepout, 6 unresolvable"), std::string::npos)
      << map.Header("X-EGS-Legend");
  const Reply here = client.Get("/view?preset=here");
  EXPECT_EQ(here.Header("X-EGS-Layers"), "map,robot,scan,zones");
  EXPECT_EQ(client.Get("/view?preset=custom&layers=map,zones").status, 200);

  // A directory the walk cannot enter makes the reading partial: refused, never a shorter list.
  if (::geteuid() != 0) {
    const fs::path locked = places / "locked";
    fs::create_directories(locked / "inner");
    fs::permissions(locked, fs::perms::none);
    const Reply partial = client.Get("/zones");
    const Reply view = client.Get("/view?preset=map");
    fs::permissions(locked, fs::perms::owner_all);
    EXPECT_NE(view.Header("X-EGS-Legend").find(", incomplete"), std::string::npos)
        << view.Header("X-EGS-Legend");
    EXPECT_EQ(JsonRaw(partial.body, "reason"), "fs_error") << partial.body;
    EXPECT_EQ(partial.body.find("\"zones\""), std::string::npos) << partial.body;
  }

  service.Stop();
  backend.Finish();
}

// A zone under a place on a floating session loses its polygon when that session goes.
TEST(ZonesE2eTest, OrphanAnchorLeavesNoPolygon) {
  const std::string map_dir = lifelong::testing::MakeTempDir("evergreenslam_agent_zones_orphan");
  const lifelong::PoseGraphOption option = lifelong::testing::AnchorTestOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = lifelong::testing::GroundTruthOdometry(total_steps);

  lifelong::SessionId floating;
  lifelong::AnchorId on_floating = 0;
  {
    PoseGraph backend(option, map_dir);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    while (backend.session_manager().num_sessions_frozen() == 0 && step < total_steps) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
    floating = lifelong::testing::RunOnTask(
        backend, [&backend] { return *backend.session_manager().fed_session(); });
    for (; step <= 45; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    on_floating = lifelong::testing::SaveAnchor(backend)->id;
    backend.Finish();
  }

  const fs::path places = fs::path(map_dir) / "memory" / "places";
  PoseGraph backend(lifelong::testing::NoFreeze(option), map_dir);
  SimulatedHost host;
  AgentService service(backend, host.Hooks(), AgentServiceOption{});
  backend.Start(TestTime(total_steps));
  service.OnBackendStarted();
  TestClient client(service.port());
  WriteText(places / "hall/place.yaml", "anchor: " + std::to_string(on_floating) + "\n");
  WriteText(places / "hall/step/zone.yaml", Keepout("places/hall"));

  const std::string before = ZoneObject(client.Get("/zones").body, "places/hall/step");
  EXPECT_EQ(PolygonXy(before).size(), 4u) << before;

  const std::string id = std::to_string(floating.session_index);
  const Reply plan = client.Post("/sessions/rm/plan", {{"id", id}});
  ASSERT_EQ(JsonRaw(plan.body, "ok"), "true") << plan.body;
  const Reply applied = client.Post("/sessions/rm/apply",
                                    {{"id", id}, {"plan_token", JsonRaw(plan.body, "plan_token")}});
  ASSERT_EQ(JsonRaw(applied.body, "ok"), "true") << applied.body;

  const std::string after = ZoneObject(client.Get("/zones").body, "places/hall/step");
  EXPECT_EQ(JsonRaw(after, "state"), "orphan") << after;
  EXPECT_EQ(JsonRaw(after, "reason"), "orphan") << after;
  EXPECT_EQ(JsonRaw(after, "polygon_xy"), "null") << after;
  EXPECT_NE(after.find("\"polygon\":[[1,-1],"), std::string::npos) << after;

  service.Stop();
  backend.Finish();
}

}  // namespace
}  // namespace evergreenslam::agent

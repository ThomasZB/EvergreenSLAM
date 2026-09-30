/**
 * @file agent_service_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The agent service over real HTTP in front of a PoseGraph fed by the simulated laser loop.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
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
using lifelong::testing::kNodesPerSubmap;
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

void FeedTo(PoseGraph& backend, Frontend& frontend, SimulatedHost& host,
            const std::vector<Eigen::Affine2d>& odometry, int& step, int last_step) {
  for (; step <= last_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
    host.Update(TestTime(step), odometry[step], SimulateScan(LoopRoom(), LoopGroundTruth(step)));
  }
}

std::string ReadText(const fs::path& path) { return common::ReadFile(path.string()).value_or(""); }

// The pixel bytes after the three-line binary PGM header.
std::vector<uint8_t> PgmPixels(const std::string& pgm, int& width, int& height) {
  int max_value = 0;
  int consumed = 0;
  if (std::sscanf(pgm.c_str(), "P5\n%d %d\n%d\n%n", &width, &height, &max_value, &consumed) != 3) {
    return {};
  }
  return std::vector<uint8_t>(pgm.begin() + consumed, pgm.end());
}

TEST(AgentServiceE2eTest, PlacesSnapshotSessionsAndFs) {
  const std::string map_dir = lifelong::testing::MakeTempDir("evergreenslam_agent_service");
  const fs::path memory = fs::path(map_dir) / "memory";
  const std::vector<Eigen::Affine2d> odometry =
      lifelong::testing::GroundTruthOdometry(kNodesPerLap);
  PoseGraph backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()), map_dir);
  SimulatedHost host;
  AgentService service(backend, host.Hooks(), AgentServiceOption{});
  ASSERT_GT(service.port(), 0);
  TestClient client(service.port());

  // Installed and listening before the backend starts; task endpoints wait for it.
  EXPECT_NE(ReadText(memory / "README.md").find("# memory/"), std::string::npos);
  EXPECT_TRUE(fs::is_directory(memory / "places"));
  const Reply root = client.Get("/root");
  EXPECT_EQ(root.status, 200);
  EXPECT_EQ(JsonRaw(root.body, "memory_dir"), fs::canonical(memory).string());
  EXPECT_EQ(client.Get("/status").status, 503);
  EXPECT_EQ(JsonRaw(client.Get("/status").body, "reason"), "not_started");

  // Seeded off identity, so a robot pose that skips the local-to-global composition shows.
  const Eigen::Affine2d seed = utils::transform::FromXYTheta(3.1, -1.7, 0.4);
  backend.Start(TestTime(0), seed);
  service.OnBackendStarted();
  EXPECT_EQ(JsonRaw(client.Post("/place/save", {{"path", "places/dock"}}).body, "reason"),
            "no_keyframe");

  Frontend frontend(backend, odometry);
  int step = 0;
  const int place_step = 14;
  FeedTo(backend, frontend, host, odometry, step, place_step);

  // place save writes place.yaml; a second save rebinds the same id in place.
  const Reply saved = client.Post("/place/save", {{"path", "places/dock"}});
  ASSERT_EQ(saved.status, 200) << saved.body;
  ASSERT_EQ(JsonRaw(saved.body, "ok"), "true") << saved.body;
  EXPECT_EQ(JsonRaw(saved.body, "rebound_existing"), "false");
  EXPECT_EQ(JsonRaw(saved.body, "state"), "pending");
  const std::string anchor = JsonRaw(saved.body, "anchor");
  EXPECT_EQ(ReadText(memory / "places/dock/place.yaml"), "anchor: " + anchor + "\n");
  const Reply resaved = client.Post("/place/save", {{"path", "places/dock"}, {"scan", "0"}});
  EXPECT_EQ(JsonRaw(resaved.body, "rebound_existing"), "true") << resaved.body;
  EXPECT_EQ(JsonRaw(resaved.body, "anchor"), anchor);
  EXPECT_EQ(ReadText(memory / "places/dock/place.yaml"), "anchor: " + anchor + "\n");
  EXPECT_EQ(client.Post("/place/save", {{"path", "places/skills"}}).status, 400);
  EXPECT_EQ(
      JsonRaw(client.Post("/place/save", {{"path", "places/dock/attachments"}}).body, "reason"),
      "reserved_name");
  EXPECT_EQ(JsonRaw(client.Post("/place/save", {{"path", "places/Dock"}}).body, "reason"),
            "not_slug");
  EXPECT_EQ(JsonRaw(client.Post("/place/save", {{"path", "dock"}}).body, "reason"), "bad_param");

  // The anchor resolves where the robot stood, and the robot pose is global like it.
  const Reply resolved = client.Get("/anchors/" + anchor + "?robot=1");
  ASSERT_EQ(JsonRaw(resolved.body, "ok"), "true") << resolved.body;
  const Eigen::Vector2d truth = seed * LoopGroundTruth(place_step).translation();
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.x"), truth.x(), kPlaceTolerance);
  EXPECT_NEAR(JsonNumber(resolved.body, "pose.y"), truth.y(), kPlaceTolerance);
  EXPECT_NEAR(JsonNumber(resolved.body, "robot.x"), JsonNumber(resolved.body, "pose.x"),
              kPlaceTolerance);
  EXPECT_NEAR(JsonNumber(resolved.body, "robot.y"), JsonNumber(resolved.body, "pose.y"),
              kPlaceTolerance);
  EXPECT_EQ(JsonRaw(client.Get("/anchors/999").body, "reason"), "unknown_anchor");
  EXPECT_EQ(JsonRaw(client.Get("/anchors/zero").body, "reason"), "bad_param");
  EXPECT_NE(client.Get("/anchors").body.find("\"anchor\":" + anchor), std::string::npos);

  // Standing there: current; walking off keeps it until the leave radius.
  const Reply here = client.Get("/here");
  ASSERT_EQ(JsonRaw(here.body, "ok"), "true") << here.body;
  ASSERT_EQ(JsonRaw(here.body, "current").rfind("{", 0), 0u) << here.body;
  EXPECT_EQ(JsonRaw(here.body, "current.path"), "places/dock");
  EXPECT_LT(JsonNumber(here.body, "current.dist_m"), kPlaceTolerance);
  EXPECT_EQ(JsonRaw(here.body, "closest.state"), "pending");
  EXPECT_EQ(JsonRaw(here.body, "has_frozen_base"), "false");
  EXPECT_EQ(JsonRaw(here.body, "aligned_to_base"), "false");
  FeedTo(backend, frontend, host, odometry, step, 4 * kNodesPerSubmap);
  const double away = (seed * LoopGroundTruth(step - 1).translation() - truth).norm();
  ASSERT_GT(away, 3.75);
  EXPECT_EQ(JsonRaw(client.Get("/here").body, "current"), "null");

  // Snapshot: files, a trinary PGM, index.tsv.
  const Reply snapshot = client.Post("/snapshot");
  ASSERT_EQ(JsonRaw(snapshot.body, "ok"), "true") << snapshot.body;
  const fs::path snapshot_dir = JsonRaw(snapshot.body, "dir");
  EXPECT_EQ(snapshot_dir, fs::path(map_dir) / "snapshots" / "000001");
  for (const char* name : {"map.pgm", "map.yaml", "map.png", "map.json", "trajectory.csv",
                           "places.json", "summary.txt"}) {
    EXPECT_TRUE(fs::is_regular_file(snapshot_dir / name)) << name;
    EXPECT_NE(snapshot.body.find(std::string("\"") + name + "\""), std::string::npos) << name;
  }
  int width = 0;
  int height = 0;
  const std::vector<uint8_t> pixels = PgmPixels(ReadText(snapshot_dir / "map.pgm"), width, height);
  ASSERT_GT(width, 0);
  ASSERT_EQ(pixels.size(), static_cast<size_t>(width) * height);
  std::set<uint8_t> values(pixels.begin(), pixels.end());
  EXPECT_EQ(values, (std::set<uint8_t>{0, 205, 254}));
  const std::string map_json = ReadText(snapshot_dir / "map.json");
  EXPECT_EQ(static_cast<int>(JsonNumber(map_json, "width")), width);
  EXPECT_NE(ReadText(snapshot_dir / "map.yaml").find("mode: trinary"), std::string::npos);
  EXPECT_EQ(ReadText(snapshot_dir / "trajectory.csv").rfind("stamp_ns,x,y,theta,session\n", 0), 0u);
  EXPECT_NE(ReadText(snapshot_dir / "places.json").find("places/dock"), std::string::npos);
  EXPECT_NE(ReadText(memory / "index.tsv").find("places/dock\t" + anchor + "\tpending\t"),
            std::string::npos);
  EXPECT_EQ(JsonNumber(client.Post("/snapshot").body, "seq"), 2);

  // Status and sessions.
  const Reply status = client.Get("/status");
  ASSERT_EQ(JsonRaw(status.body, "ok"), "true") << status.body;
  EXPECT_EQ(JsonNumber(status.body, "num_place_files"), 1);
  EXPECT_EQ(JsonNumber(status.body, "num_anchors"), 1);
  EXPECT_EQ(JsonRaw(status.body, "has_frozen_base"), "false");
  const std::string fed = JsonRaw(status.body, "fed_session");
  const Reply sessions = client.Get("/sessions");
  EXPECT_EQ(JsonRaw(sessions.body, "fed"), fed);
  EXPECT_EQ(JsonRaw(sessions.body, "role"), "fed");
  EXPECT_EQ(JsonNumber(sessions.body, "anchors"), 1);

  // rm refuses the fed session on a host without a drop hook, and unknown ids; apply without a
  // matching plan does nothing.
  const Reply rm_fed = client.Post("/sessions/rm/plan", {{"id", fed}});
  EXPECT_EQ(JsonRaw(rm_fed.body, "ok"), "false");
  EXPECT_EQ(JsonRaw(rm_fed.body, "rejection"), "not_supported");
  const Reply rm_fed_apply = client.Post(
      "/sessions/rm/apply", {{"id", fed}, {"plan_token", JsonRaw(rm_fed.body, "plan_token")}});
  EXPECT_EQ(JsonRaw(rm_fed_apply.body, "reason"), "plan_changed");
  EXPECT_EQ(JsonRaw(client.Post("/sessions/rm/plan", {{"id", "42"}}).body, "rejection"),
            "unknown_session");
  EXPECT_EQ(client.Post("/sessions/rm/plan").status, 400);
  EXPECT_EQ(JsonRaw(client.Get("/sessions").body, "role"), "fed");

  // Mutating one-liners.
  EXPECT_EQ(JsonRaw(client.Post("/checkpoint").body, "ok"), "true");
  EXPECT_EQ(JsonRaw(client.Post("/relocalize").body, "ok"), "true");
  const Reply init = client.Post("/init-pose", {{"anchor", anchor}});
  EXPECT_EQ(JsonRaw(init.body, "ok"), "true") << init.body;
  EXPECT_NEAR(JsonNumber(init.body, "pose.x"), truth.x(), kPlaceTolerance);
  EXPECT_EQ(JsonRaw(client.Post("/init-pose", {{"x", "1"}}).body, "reason"), "bad_param");
  backend.WaitUntilQuiescent();

  // fs passthrough and its sandbox.
  EXPECT_EQ(JsonRaw(client.PostBody("/fs/write?path=places/dock/notes.md", "hi\n").body, "ok"),
            "true");
  EXPECT_EQ(JsonRaw(client.PostBody("/fs/append?path=places/dock/notes.md", "more\n").body, "ok"),
            "true");
  const Reply cat = client.Get("/fs/cat?path=places/dock/notes.md");
  EXPECT_EQ(cat.body, "hi\nmore\n");
  const Reply ls = client.Get("/fs/ls?path=places/dock");
  EXPECT_NE(ls.body.find("{\"name\":\"notes.md\",\"type\":\"file\",\"size\":8}"), std::string::npos)
      << ls.body;
  const std::string tree = client.Get("/fs/tree?path=.").body;
  EXPECT_EQ(tree, client.Get("/fs/tree?path=.&depth=3").body);
  EXPECT_NE(tree.find("└── place.yaml"), std::string::npos) << tree;
  EXPECT_EQ(tree.find("…"), std::string::npos) << tree;
  // places/dock is cut off at depth 2: one marker line under it, not counted.
  const std::string shallow = client.Get("/fs/tree?path=.&depth=2").body;
  EXPECT_NE(shallow.find("    └── dock\n        └── …\n"), std::string::npos) << shallow;
  EXPECT_EQ(shallow.find("place.yaml"), std::string::npos) << shallow;
  EXPECT_EQ(JsonRaw(client.Post("/fs/mkdir", {{"path", "places/kitchen/table"}}).body, "ok"),
            "true");
  EXPECT_TRUE(fs::is_directory(memory / "places/kitchen/table"));

  const auto reason = [](const Reply& reply) { return JsonRaw(reply.body, "reason"); };
  EXPECT_EQ(reason(client.Get("/fs/cat?path=../manifest.pb")), "path_escape");
  EXPECT_EQ(reason(client.Get("/fs/cat?path=places/../../manifest.pb")), "path_escape");
  EXPECT_EQ(reason(client.Get("/fs/ls?path=/etc")), "path_escape");
  EXPECT_EQ(reason(client.Get("/fs/ls?path=")), "bad_param");
  EXPECT_EQ(client.Get("/fs/cat?path=places/dock/missing.md").status, 404);
  EXPECT_EQ(reason(client.Get("/fs/cat?path=places/dock/missing.md")), "not_found");
  EXPECT_EQ(reason(client.Get("/fs/cat?path=places/dock")), "fs_error");
  EXPECT_EQ(reason(client.Post("/fs/rm", {{"path", "places/dock/none"}, {"recursive", "true"}})),
            "fs_error");
  fs::create_directory_symlink(map_dir, memory / "places/escape");
  EXPECT_EQ(reason(client.Get("/fs/ls?path=places/escape")), "symlink");
  EXPECT_EQ(reason(client.PostBody("/fs/write?path=places/escape/x.md", "x")), "symlink");
  EXPECT_EQ(client.Get("/fs/ls?path=places/escape").status, 400);
  EXPECT_EQ(reason(client.Post("/place/save", {{"path", "places/escape/spot"}})), "symlink");
  fs::remove(memory / "places/escape");
  for (const char* owned : {"places/dock/place.yaml", "index.tsv", "README.md"}) {
    EXPECT_EQ(reason(client.PostBody(std::string("/fs/write?path=") + owned, "x")),
              "owned_by_process")
        << owned;
    EXPECT_EQ(reason(client.PostBody(std::string("/fs/append?path=") + owned, "x")),
              "owned_by_process")
        << owned;
    EXPECT_EQ(reason(client.Post("/fs/rm", {{"path", owned}})), "owned_by_process") << owned;
  }
  EXPECT_EQ(reason(client.Post(
                "/fs/mv", {{"from", "places/dock/notes.md"}, {"to", "places/kitchen/place.yaml"}})),
            "owned_by_process");
  EXPECT_EQ(reason(client.Post(
                "/fs/mv", {{"from", "places/dock/place.yaml"}, {"to", "places/kitchen/x.yaml"}})),
            "owned_by_process");
  EXPECT_EQ(reason(client.Post("/fs/mkdir", {{"path", "places/kitchen/Table"}})), "not_slug");
  EXPECT_EQ(reason(client.Post("/fs/rm", {{"path", "places/kitchen"}})), "not_empty");
  EXPECT_EQ(reason(client.Post("/fs/mv", {{"from", "places/dock"}, {"to", "places/kitchen"}})),
            "exists");
  // The scan skips skills/: a place moved there would vanish. Skills themselves go there.
  EXPECT_EQ(
      reason(client.Post("/fs/mv", {{"from", "places/dock"}, {"to", "places/kitchen/skills"}})),
      "reserved_name");
  EXPECT_EQ(reason(client.Post("/fs/mv", {{"from", "places/dock"}, {"to", "places/attachments"}})),
            "reserved_name");
  EXPECT_EQ(reason(client.Post("/fs/mkdir", {{"path", "places/kitchen/skills/clean"}})), "null");
  EXPECT_EQ(reason(client.Post("/fs/mv", {{"from", "places/kitchen/skills/clean"},
                                          {"to", "places/kitchen/skills/wipe"}})),
            "null");
  ASSERT_TRUE(fs::is_directory(memory / "places/dock"));
  EXPECT_EQ(client.Get("/fs/nope").status, 404);
  EXPECT_EQ(reason(client.Get("/fs/nope")), "not_found");

  // A place moves with its directory, binding and all; rm -r deletes it.
  EXPECT_EQ(reason(client.Post("/fs/mv", {{"from", "places/dock"}, {"to", "places/home"}})),
            "null");
  EXPECT_EQ(ReadText(memory / "places/home/place.yaml"), "anchor: " + anchor + "\n");
  EXPECT_NE(client.Get("/anchors").body.find(anchor), std::string::npos);
  EXPECT_EQ(reason(client.Post("/fs/rm", {{"path", "places/kitchen"}, {"recursive", "1"}})),
            "null");
  EXPECT_FALSE(fs::exists(memory / "places/kitchen"));

  // A `cp -r` copy saved again gets an anchor of its own; the original keeps its id and pose.
  fs::copy(memory / "places/home", memory / "places/home2", fs::copy_options::recursive);
  EXPECT_NE(client.Get("/status").body.find("\"places/home2\""), std::string::npos);
  const Reply copy_saved = client.Post("/place/save", {{"path", "places/home2"}});
  ASSERT_EQ(JsonRaw(copy_saved.body, "ok"), "true") << copy_saved.body;
  EXPECT_EQ(JsonRaw(copy_saved.body, "rebound_existing"), "false");
  const std::string copy_anchor = JsonRaw(copy_saved.body, "anchor");
  EXPECT_NE(copy_anchor, anchor);
  EXPECT_EQ(ReadText(memory / "places/home2/place.yaml"), "anchor: " + copy_anchor + "\n");
  EXPECT_EQ(ReadText(memory / "places/home/place.yaml"), "anchor: " + anchor + "\n");
  const Reply original = client.Get("/anchors/" + anchor);
  EXPECT_NEAR(JsonNumber(original.body, "pose.x"), truth.x(), kPlaceTolerance) << original.body;
  EXPECT_NE(client.Get("/status").body.find("\"duplicate_anchor_paths\":[]"), std::string::npos);

  // Freeze through plan and apply: the fed session turns frozen and a fresh one is fed.
  const Reply freeze_plan = client.Post("/sessions/freeze/plan", {{"force", "1"}});
  ASSERT_EQ(JsonRaw(freeze_plan.body, "ok"), "true") << freeze_plan.body;
  const Reply frozen =
      client.Post("/sessions/freeze/apply",
                  {{"force", "1"}, {"plan_token", JsonRaw(freeze_plan.body, "plan_token")}});
  ASSERT_EQ(JsonRaw(frozen.body, "ok"), "true") << frozen.body;
  EXPECT_EQ(JsonRaw(frozen.body, "frozen_session"), fed);
  EXPECT_EQ(JsonRaw(frozen.body, "pending"), "true");
  backend.WaitUntilQuiescent();
  const Reply after_freeze = client.Get("/sessions");
  EXPECT_NE(after_freeze.body.find("\"id\":" + fed + ",\"role\":\"frozen\""), std::string::npos)
      << after_freeze.body;
  EXPECT_NE(JsonRaw(after_freeze.body, "fed"), fed) << after_freeze.body;

  service.Stop();
  EXPECT_EQ(client.Get("/root").status, 0);
  backend.Finish();
}

// Two boots: a frozen base, a floating session with a place on it, and a fed session.
TEST(AgentServiceE2eTest, RemovesOnlyFloatingSessionsThroughPlanAndApply) {
  const std::string map_dir = lifelong::testing::MakeTempDir("evergreenslam_agent_sessions");
  const lifelong::PoseGraphOption option = lifelong::testing::AnchorTestOption();
  const int total_steps = 2 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = lifelong::testing::GroundTruthOdometry(total_steps);
  const int floating_step = 45;

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
    for (; step <= floating_step; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    on_floating = lifelong::testing::SaveAnchor(backend)->id;
    backend.Finish();
  }

  PoseGraph backend(lifelong::testing::NoFreeze(option), map_dir);
  SimulatedHost host;
  AgentService service(backend, host.Hooks(), AgentServiceOption{});
  backend.Start(TestTime(total_steps));
  service.OnBackendStarted();
  TestClient client(service.port());

  const Reply sessions = client.Get("/sessions");
  ASSERT_EQ(JsonRaw(sessions.body, "ok"), "true") << sessions.body;
  EXPECT_NE(sessions.body.find("\"role\":\"frozen\""), std::string::npos) << sessions.body;
  EXPECT_NE(sessions.body.find("\"id\":" + std::to_string(floating.session_index) +
                               ",\"role\":\"floating\""),
            std::string::npos)
      << sessions.body;
  const Reply status = client.Get("/status");
  EXPECT_EQ(JsonRaw(status.body, "has_frozen_base"), "true");
  const Reply here = client.Get("/here");
  EXPECT_EQ(JsonRaw(here.body, "has_frozen_base"), "true");
  EXPECT_EQ(JsonRaw(here.body, "aligned_to_base"), JsonRaw(status.body, "aligned_to_base"));

  EXPECT_EQ(JsonRaw(client.Post("/sessions/rm/plan", {{"id", "0"}}).body, "rejection"),
            "frozen_session");

  const std::string id = std::to_string(floating.session_index);
  const Reply plan = client.Post("/sessions/rm/plan", {{"id", id}});
  ASSERT_EQ(JsonRaw(plan.body, "ok"), "true") << plan.body;
  EXPECT_NE(plan.body.find("\"anchors_orphaned\":[" + std::to_string(on_floating) + "]"),
            std::string::npos)
      << plan.body;
  EXPECT_NE(plan.body.find("\"would_delete\":[[" + id + ","), std::string::npos) << plan.body;
  const std::string token = JsonRaw(plan.body, "plan_token");
  EXPECT_EQ(token.size(), 16u);

  EXPECT_EQ(
      JsonRaw(client.Post("/sessions/rm/apply", {{"id", id}, {"plan_token", "0"}}).body, "reason"),
      "plan_changed");
  const Reply applied = client.Post("/sessions/rm/apply", {{"id", id}, {"plan_token", token}});
  ASSERT_EQ(JsonRaw(applied.body, "ok"), "true") << applied.body;
  EXPECT_EQ(JsonRaw(applied.body, "removed"), id);
  EXPECT_EQ(client.Get("/sessions").body.find("\"role\":\"floating\""), std::string::npos);
  const Reply orphan = client.Get("/anchors/" + std::to_string(on_floating));
  EXPECT_EQ(JsonRaw(orphan.body, "state"), "orphan");
  EXPECT_EQ(JsonRaw(orphan.body, "orphan_reason"), "session_removed");
  EXPECT_EQ(JsonRaw(orphan.body, "pose"), "null");
  EXPECT_EQ(JsonRaw(client.Post("/sessions/rm/apply", {{"id", id}, {"plan_token", token}}).body,
                    "reason"),
            "plan_changed");

  // Freeze plan answers with the judge's verdict; a stale token applies nothing.
  const Reply freeze = client.Post("/sessions/freeze/plan");
  ASSERT_EQ(freeze.status, 200) << freeze.body;
  EXPECT_NE(freeze.body.find("\"verdict\":{\"eligible\":"), std::string::npos) << freeze.body;
  const Reply freeze_apply =
      client.Post("/sessions/freeze/apply", {{"plan_token", "ffffffffffffffff"}});
  EXPECT_EQ(JsonRaw(freeze_apply.body, "reason"), "plan_changed");

  service.Stop();
  backend.Finish();
}

#if __has_include("service/render/renderer.h")
TEST(AgentServiceE2eTest, ViewsRenderPresetsAndRefuse) {
  const std::string map_dir = lifelong::testing::MakeTempDir("evergreenslam_agent_view");
  const std::vector<Eigen::Affine2d> odometry =
      lifelong::testing::GroundTruthOdometry(kNodesPerLap);
  PoseGraph backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()), map_dir);
  SimulatedHost host;
  AgentService service(backend, host.Hooks(), AgentServiceOption{});
  backend.Start(TestTime(0));
  service.OnBackendStarted();
  TestClient client(service.port());
  Frontend frontend(backend, odometry);
  int step = 0;
  FeedTo(backend, frontend, host, odometry, step, 10);
  ASSERT_EQ(JsonRaw(client.Post("/place/save", {{"path", "places/dock"}}).body, "ok"), "true");
  FeedTo(backend, frontend, host, odometry, step, 4 * kNodesPerSubmap);

  const std::string png_signature = "\x89PNG\r\n\x1a\n";
  for (const char* preset : {"map", "here", "trail", "session"}) {
    const Reply view = client.Get(std::string("/view?preset=") + preset);
    ASSERT_EQ(view.status, 200) << preset << ": " << view.body;
    EXPECT_EQ(view.content_type, "image/png") << preset;
    EXPECT_EQ(view.body.rfind(png_signature, 0), 0u) << preset;
    EXPECT_FALSE(view.Header("X-EGS-Layers").empty()) << preset;
    EXPECT_FALSE(view.Header("X-EGS-Legend").empty()) << preset;
    EXPECT_TRUE(fs::is_regular_file(fs::path(map_dir) / view.Header("X-EGS-View"))) << preset;
  }
  const Reply map = client.Get("/view?preset=map");
  EXPECT_EQ(map.Header("X-EGS-Layers"), "map,places,zones");
  EXPECT_NE(map.Header("X-EGS-Legend").find("places/dock"), std::string::npos)
      << map.Header("X-EGS-Legend");
  EXPECT_EQ(map.Header("X-EGS-View"), "views/000005_map.png");

  const Reply route = client.Get("/view?preset=route&target=places/dock&ego=7.5");
  ASSERT_EQ(route.status, 200) << route.body;
  EXPECT_NE(route.Header("X-EGS-Layers").find("target=places/dock"), std::string::npos);
  EXPECT_NE(route.Header("X-EGS-Legend").find("(target)"), std::string::npos)
      << route.Header("X-EGS-Legend");
  // The target is drawn once, highlighted, not also as a numbered place.
  const std::string legend = route.Header("X-EGS-Legend");
  EXPECT_EQ(legend.find("places/dock"), legend.rfind("places/dock")) << legend;

  const auto reason = [](const Reply& reply) { return JsonRaw(reply.body, "reason"); };
  EXPECT_EQ(reason(client.Get("/view?preset=route&target=places/nowhere")), "no_binding");
  EXPECT_EQ(reason(client.Get("/view?preset=route")), "bad_param");
  EXPECT_EQ(reason(client.Get("/view?preset=custom&layers=map,robot,scan,trail,submaps")),
            "too_many_layers");
  EXPECT_EQ(reason(client.Get("/view?preset=custom&layers=map,clouds")), "unknown_layer");
  EXPECT_EQ(reason(client.Get("/view?preset=custom&layers=session=42")), "unknown_session");
  const Reply custom = client.Get("/view?preset=custom&layers=map,submaps,robot");
  EXPECT_EQ(custom.status, 200) << custom.body;

  service.Stop();
  backend.Finish();
}
#endif

}  // namespace
}  // namespace evergreenslam::agent

/**
 * @file service_parts_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The service's pure parts: JSON writer, plan token, place yaml, here hysteresis, snapshot
 *        pixels and trajectory decimation, sandbox rules.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "mapping/grid_mapping/probability_values.h"
#include "service/fs_sandbox.h"
#include "service/here_tracker.h"
#include "service/json_writer.h"
#include "service/match_score_average.h"
#include "service/place_store.h"
#include "service/plan_report.h"
#include "service/request_error.h"
#include "service/snapshot_exporter.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

std::string RejectionOf(const FsSandbox& sandbox, const std::string& path) {
  try {
    sandbox.Resolve(path);
  } catch (const RequestError& error) {
    return error.reason;
  }
  return "";
}

TEST(JsonWriterTest, CommasNestingAndEscapes) {
  JsonWriter writer;
  writer.BeginObject()
      .Field("ok", true)
      .NullField("reason")
      .Key("list")
      .BeginArray()
      .Int(1)
      .BeginArray()
      .EndArray()
      .String("a\"b\n")
      .EndArray()
      .Field("x", 0.5)
      .OptionalField("gone", std::optional<double>())
      .EndObject();
  EXPECT_EQ(writer.str(),
            "{\"ok\":true,\"reason\":null,\"list\":[1,[],\"a\\\"b\\n\"],\"x\":0.5,\"gone\":null}");
}

TEST(PlanReportTest, TokenIgnoresOrderAndSolveCount) {
  PlanReport a;
  a.op = "rm";
  a.sessions_affected = {lifelong::SessionId{3}};
  a.would_delete = {{3, 1}, {3, 0}};
  a.anchors_orphaned = {9, 4};
  a.at_num_solves = 5;
  PlanReport b = a;
  b.would_delete = {{3, 0}, {3, 1}};
  b.anchors_orphaned = {4, 9};
  b.at_num_solves = 6;
  EXPECT_EQ(a.Token(), b.Token());
  EXPECT_EQ(a.Token().size(), 16u);
  b.anchors_orphaned.push_back(11);
  EXPECT_NE(a.Token(), b.Token());
  b = a;
  b.op = "freeze";
  EXPECT_NE(a.Token(), b.Token());
}

TEST(PlaceStoreTest, PlaceYamlAndNodePaths) {
  EXPECT_EQ(PlaceStore::ParsePlaceYaml("anchor: 7\n"), std::optional<lifelong::AnchorId>(7));
  EXPECT_EQ(PlaceStore::ParsePlaceYaml("{anchor: 12}"), std::optional<lifelong::AnchorId>(12));
  EXPECT_FALSE(PlaceStore::ParsePlaceYaml("anchor: 0").has_value());
  EXPECT_FALSE(PlaceStore::ParsePlaceYaml("anchor: [").has_value());
  EXPECT_FALSE(PlaceStore::ParsePlaceYaml("kind: fridge").has_value());

  EXPECT_NO_THROW(PlaceStore::CheckNodePath("places/kitchen/fridge-2"));
  const auto reason = [](const std::string& path) {
    try {
      PlaceStore::CheckNodePath(path);
    } catch (const RequestError& error) {
      return error.reason;
    }
    return std::string();
  };
  EXPECT_EQ(reason("kitchen"), "bad_param");
  EXPECT_EQ(reason("places/"), "bad_param");
  EXPECT_EQ(reason("places//a"), "bad_param");
  EXPECT_EQ(reason("places/a/skills/b"), "reserved_name");
  EXPECT_EQ(reason("places/Kitchen"), "not_slug");
  EXPECT_EQ(reason("places/_a"), "not_slug");
  EXPECT_EQ(reason("places/../x"), "path_escape");
}

TEST(PlaceStoreTest, ScanSkipsSkillsSymlinksAndFlagsCopies) {
  const fs::path map_dir =
      fs::temp_directory_path() / ("evergreenslam_place_store_" + std::to_string(::getpid()));
  fs::remove_all(map_dir);
  const PlaceStore store(map_dir.string());
  ASSERT_TRUE(store.Install());
  ASSERT_TRUE(store.WriteBinding("places/dock", 3));
  ASSERT_TRUE(store.WriteBinding("places/kitchen", 5));
  ASSERT_TRUE(store.WriteBinding("places/kitchen/copy", 5));
  ASSERT_TRUE(store.WriteBinding("places/kitchen/skills/x", 8));
  fs::create_directory_symlink(map_dir / "memory/places/dock", map_dir / "memory/places/link");

  const PlaceScan scan = store.Scan();
  ASSERT_EQ(scan.places.size(), 3u);
  EXPECT_EQ(scan.places[0].path, "places/dock");
  EXPECT_EQ(scan.num_place_files, 3);
  EXPECT_EQ(scan.duplicate_paths,
            (std::vector<std::string>{"places/kitchen", "places/kitchen/copy"}));
  ASSERT_EQ(scan.Unique().size(), 1u);
  EXPECT_EQ(store.ReadBinding("places/kitchen"), std::optional<lifelong::AnchorId>(5));
  EXPECT_FALSE(store.ReadBinding("places/none").has_value());
  fs::remove_all(map_dir);
}

TEST(HereTrackerTest, EntersAtTwoPointFiveLeavesAtThreePointSevenFive) {
  HereTracker tracker;
  EXPECT_FALSE(tracker.Update({{"places/a", 3.0}}).has_value());
  EXPECT_EQ(tracker.Update({{"places/a", 2.4}}), std::optional<size_t>(0));
  EXPECT_EQ(tracker.Update({{"places/b", 1.0}, {"places/a", 3.5}}), std::optional<size_t>(1));
  EXPECT_EQ(tracker.Update({{"places/b", 1.0}, {"places/a", 3.8}}), std::optional<size_t>(0));
  EXPECT_FALSE(tracker.Update({}).has_value());
}

TEST(MatchScoreAverageTest, SeedsWithTheFirstSampleAndForgetsOverSecondsNotScans) {
  const common::Time start = common::FromUnixSeconds(1785000000.0);
  MatchScoreAverage fast;
  MatchScoreAverage slow;
  EXPECT_DOUBLE_EQ(fast.Add(start, 0.8), 0.8);
  EXPECT_DOUBLE_EQ(slow.Add(start, 0.8), 0.8);
  // One time constant of zeros moves the average 1 - 1/e of the way, at 40 Hz or at 10 Hz.
  double fast_average = 0.0;
  for (int i = 1; i <= 80; ++i) {
    fast_average = fast.Add(start + common::FromSeconds(0.025 * i), 0.0);
  }
  double slow_average = 0.0;
  for (int i = 1; i <= 20; ++i) {
    slow_average = slow.Add(start + common::FromSeconds(0.1 * i), 0.0);
  }
  EXPECT_NEAR(fast_average, 0.8 * std::exp(-1.0), 1e-9);
  EXPECT_NEAR(slow_average, 0.8 * std::exp(-1.0), 1e-9);
  // A stamp that goes backwards changes nothing rather than extrapolating.
  EXPECT_DOUBLE_EQ(slow.Add(start, 1.0), slow_average);
}

TEST(SnapshotExporterTest, TrinaryPixels) {
  EXPECT_EQ(TrinaryValue(mapping::kUnknownValue), 205);
  EXPECT_EQ(TrinaryValue(mapping::kUpdateMarker), 205);
  EXPECT_EQ(TrinaryValue(mapping::ProbabilityToValue(0.8)), 0);
  EXPECT_EQ(TrinaryValue(mapping::ProbabilityToValue(0.2)), 254);
  EXPECT_EQ(TrinaryValue(mapping::ProbabilityToValue(0.2) + mapping::kUpdateMarker), 254);
}

TEST(SnapshotExporterTest, DecimationKeepsEndsPerSession) {
  std::vector<SnapshotNode> nodes;
  for (int i = 0; i < 31; ++i) {
    SnapshotNode node;
    node.time = common::FromUnixSeconds(1000.0 + i);
    node.global_pose = utils::transform::FromXYTheta(0.25 * i, 0.0, 0.0);
    node.session = i < 21 ? 0 : 1;
    nodes.push_back(node);
  }
  const std::vector<SnapshotNode> kept = DecimateTrajectory(nodes, 1.0);
  std::vector<double> xs;
  for (const SnapshotNode& node : kept) {
    xs.push_back(node.global_pose.translation().x());
  }
  EXPECT_EQ(xs, (std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.25, 6.25, 7.25, 7.5}));
}

TEST(FsSandboxTest, EscapesSymlinksAndOwnedFiles) {
  const fs::path base =
      fs::temp_directory_path() / ("evergreenslam_sandbox_" + std::to_string(::getpid()));
  fs::remove_all(base);
  fs::create_directories(base / "memory/places/dock");
  fs::create_directories(base / "outside");
  fs::create_directory_symlink(base / "outside", base / "memory/places/out");
  const FsSandbox sandbox((base / "memory").string());

  const SandboxPath dock = sandbox.Resolve("places/dock/./notes.md");
  EXPECT_EQ(dock.relative, "places/dock/notes.md");
  EXPECT_EQ(dock.existing_components, 2u);
  EXPECT_FALSE(dock.exists());
  EXPECT_TRUE(sandbox.Resolve(".").is_root());
  EXPECT_EQ(RejectionOf(sandbox, ""), "bad_param");
  EXPECT_EQ(RejectionOf(sandbox, "/etc/passwd"), "path_escape");
  EXPECT_EQ(RejectionOf(sandbox, "places/../../outside"), "path_escape");
  EXPECT_EQ(RejectionOf(sandbox, std::string("places\0x", 8)), "path_escape");
  EXPECT_EQ(RejectionOf(sandbox, "places/out"), "symlink");
  EXPECT_EQ(RejectionOf(sandbox, "places/out/new/file"), "symlink");

  EXPECT_TRUE(FsSandbox::IsProcessOwned(sandbox.Resolve("places/dock/place.yaml")));
  EXPECT_TRUE(FsSandbox::IsProcessOwned(sandbox.Resolve("index.tsv")));
  EXPECT_TRUE(FsSandbox::IsProcessOwned(sandbox.Resolve("README.md")));
  EXPECT_TRUE(FsSandbox::IsProcessOwned(sandbox.Resolve("places/dock/Place.YAML")));
  EXPECT_FALSE(FsSandbox::IsProcessOwned(sandbox.Resolve("places/README.md")));
  EXPECT_FALSE(FsSandbox::IsProcessOwned(sandbox.Resolve("places/dock")));
  EXPECT_THROW(FsSandbox::CheckDirectoryNames(sandbox.Resolve("places/New"), 1), RequestError);
  EXPECT_NO_THROW(FsSandbox::CheckDirectoryNames(sandbox.Resolve("places/dock/skills/x_1"), 2));
  fs::remove_all(base);
}

}  // namespace
}  // namespace evergreenslam::agent

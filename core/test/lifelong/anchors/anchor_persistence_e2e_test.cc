/**
 * @file anchor_persistence_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The anchors file across boots: a clean restart that changes nothing, a kill right after a
 * save, and the garbage collection of the session an anchor hangs off.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "anchor_scenario.h"
#include "common/file.h"
#include "lifelong/map_manager/map_manager.h"

namespace evergreenslam::lifelong {
namespace {

using testing::AnchorsPath;
using testing::ExpectNearTruth;
using testing::FreezeABase;
using testing::Frontend;
using testing::GetAnchor;
using testing::kNodesPerLap;
using testing::ReadAnchorsFromDisk;
using testing::ResolveAnchor;
using testing::RunOnTask;
using testing::SaveAnchor;
using testing::TestTime;
using testing::TranslationError;

void FeedTo(PoseGraph& backend, Frontend& frontend, int& step, int last_step) {
  for (; step <= last_step; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
}

// Three anchors, one on the frozen bootstrap submap and two in its floating successor, through
// Finish and a restart that changes nothing: the file is left byte for byte as it was.
TEST(AnchorPersistenceE2eTest, SaveLoadRestartIsByteIdentical) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_restart");
  const PoseGraphOption option = testing::AnchorTestOption();
  const int total_steps = kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const std::vector<int> save_steps = {5, 45, 70};

  std::vector<ResolvedAnchor> before;
  AnchorTable table_before;
  std::string bytes_before;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    for (const int save_step : save_steps) {
      FeedTo(backend, frontend, step, save_step);
      ASSERT_TRUE(SaveAnchor(backend).has_value());
    }
    backend.Finish();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
    before = RunOnTask(backend, [&] { return backend.anchors().ResolveAll(backend.graph()); });
    table_before = RunOnTask(backend, [&] { return backend.anchors().Table(); });
    const std::optional<std::string> bytes = common::ReadFile(AnchorsPath(*backend.map_manager()));
    ASSERT_TRUE(bytes.has_value());
    bytes_before = *bytes;
  }
  ASSERT_EQ(before.size(), 3u);
  EXPECT_TRUE(before[0].frozen) << "the bootstrap submap froze with the first session";
  EXPECT_FALSE(before[1].frozen);
  EXPECT_FALSE(before[2].frozen);
  for (size_t i = 0; i < before.size(); ++i) {
    ExpectNearTruth(before[i], save_steps[i]);
  }

  // No freeze in boot 2: a floating session's freeze trims, and a trim rebinds.
  PoseGraph backend(testing::NoFreeze(option), directory);
  backend.Start(TestTime(total_steps));
  EXPECT_EQ(common::ReadFile(AnchorsPath(*backend.map_manager())).value_or(""), bytes_before)
      << "a restart that changes nothing must not rewrite the anchors file";
  // Bytes alone cannot tell: the same table always serializes the same.
  EXPECT_EQ(RunOnTask(backend, [&] { return backend.map_manager()->num_anchor_writes(); }), 0);
  const AnchorTable table_after = RunOnTask(backend, [&] { return backend.anchors().Table(); });
  EXPECT_EQ(table_after.next_id, table_before.next_id);
  ASSERT_EQ(table_after.anchors.size(), table_before.anchors.size());
  for (size_t i = 0; i < table_before.anchors.size(); ++i) {
    const Anchor& expected = table_before.anchors[i];
    const Anchor& actual = table_after.anchors[i];
    EXPECT_EQ(actual.id, expected.id);
    EXPECT_EQ(actual.submap_id, expected.submap_id);
    EXPECT_EQ(actual.node_id, expected.node_id);
    EXPECT_EQ(actual.saved_at, expected.saved_at);
    EXPECT_EQ(actual.state, expected.state);
    EXPECT_LT(TranslationError(actual.submap_from_anchor, expected.submap_from_anchor), 1e-12);
    ASSERT_TRUE(actual.scan.has_value());
    EXPECT_EQ(actual.scan->size(), expected.scan->size());
  }

  const std::vector<ResolvedAnchor> after =
      RunOnTask(backend, [&] { return backend.anchors().ResolveAll(backend.graph()); });
  ASSERT_EQ(after.size(), 3u);
  EXPECT_TRUE(after[0].frozen);
  EXPECT_LT(TranslationError(*after[0].global_pose, *before[0].global_pose), 1e-9)
      << "a frozen anchor resolves identically across the reload";
  for (size_t i = 1; i < after.size(); ++i) {
    EXPECT_FALSE(after[i].frozen);
    ASSERT_TRUE(after[i].global_pose.has_value());
    EXPECT_LT(TranslationError(*after[i].global_pose, *before[i].global_pose),
              testing::kAnchorTranslationTolerance)
        << "anchor " << after[i].id << ": the reload re-solves the floating session";
    ExpectNearTruth(after[i], save_steps[i]);
  }

  backend.Finish();
  EXPECT_EQ(common::ReadFile(AnchorsPath(*backend.map_manager())).value_or(""), bytes_before)
      << "a checkpoint with an unchanged table must not rewrite it";
  EXPECT_EQ(RunOnTask(backend, [&] { return backend.map_manager()->num_anchor_writes(); }), 0);
}

// A save is a checkpoint: killed right after SaveAnchorOnTask returns, with no cadence checkpoint
// before or after it, the anchor and the submap it hangs off are both on disk.
TEST(AnchorPersistenceE2eTest, SaveKillRestart) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_kill");
  PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  option.checkpoint_min_interval = common::FromSeconds(1e6);
  const int total_steps = kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const int save_step = 33;

  AnchorId id = 0;
  SubmapId bound_to;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, save_step);
    ASSERT_EQ(backend.map_manager()->num_checkpoints_written(), 0)
        << "a cadence checkpoint would mask the save's own";
    const std::optional<Anchor> anchor = SaveAnchor(backend);
    ASSERT_TRUE(anchor.has_value());
    id = anchor->id;
    bound_to = anchor->submap_id;
    EXPECT_EQ(backend.map_manager()->num_checkpoints_written(), 1);
  }

  PoseGraph backend(option, directory);
  backend.Start(TestTime(total_steps));
  const std::optional<Anchor> reloaded = GetAnchor(backend, id);
  ASSERT_TRUE(reloaded.has_value()) << "the anchor did not survive the kill";
  EXPECT_EQ(reloaded->state, AnchorState::BOUND)
      << "orphaned as " << ToString(reloaded->orphan_reason);
  EXPECT_EQ(reloaded->submap_id, bound_to);
  EXPECT_TRUE(RunOnTask(backend, [&] { return backend.graph().HasSubmap(bound_to); }));
  ExpectNearTruth(ResolveAnchor(backend, id), save_step);
}

// An anchor on an abandoned floating session is orphaned when the session is swept, persisted as
// such, and stays exactly that on every later boot.
TEST(AnchorPersistenceE2eTest, GcOfFloatingSessionFlagsOrphan) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_gc");
  PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  option.gc_unfrozen_after_n_boots = 1;
  const int total_steps = kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const int save_step = 20;

  AnchorId id = 0;
  SessionId floating;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    floating = *backend.session_manager().fed_session();
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, save_step);
    id = SaveAnchor(backend)->id;
    backend.Finish();
  }
  {
    // Last fed one boot ago: kept.
    PoseGraph backend(option, directory);
    backend.Start(TestTime(total_steps));
    ASSERT_TRUE(RunOnTask(backend, [&] { return backend.graph().HasSession(floating); }));
    ExpectNearTruth(ResolveAnchor(backend, id), save_step);
  }
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(total_steps + 1));
    ASSERT_FALSE(RunOnTask(backend, [&] { return backend.graph().HasSession(floating); }))
        << "the floating session outlived the sweep";
    const std::optional<Anchor> swept = GetAnchor(backend, id);
    ASSERT_TRUE(swept.has_value()) << "an orphan is kept, never dropped";
    EXPECT_EQ(swept->state, AnchorState::ORPHAN);
    EXPECT_EQ(swept->orphan_reason, OrphanReason::SESSION_REMOVED);
    const ResolvedAnchor resolved = ResolveAnchor(backend, id);
    EXPECT_EQ(resolved.state, AnchorState::ORPHAN);
    EXPECT_FALSE(resolved.global_pose.has_value()) << "an orphan has no old position to offer";

    const std::optional<AnchorTable> on_disk = ReadAnchorsFromDisk(directory);
    ASSERT_TRUE(on_disk.has_value());
    ASSERT_EQ(on_disk->anchors.size(), 1u);
    EXPECT_EQ(on_disk->anchors[0].state, AnchorState::ORPHAN);
    EXPECT_EQ(on_disk->anchors[0].orphan_reason, OrphanReason::SESSION_REMOVED);
  }
  PoseGraph backend(option, directory);
  backend.Start(TestTime(total_steps + 2));
  const std::optional<Anchor> later = GetAnchor(backend, id);
  ASSERT_TRUE(later.has_value());
  EXPECT_EQ(later->orphan_reason, OrphanReason::SESSION_REMOVED) << "orphan is final";
}

// The load-time net: an anchors file naming a submap the session files no longer hold (a commit
// makes this unreachable by a kill; a stale file copied in by hand still reaches it). The orphan
// is persisted and final, and no id is ever issued twice. A map that fails to load refuses to
// boot instead: MapManagerTest.DamagedManifestRefusesToBoot.
TEST(AnchorPersistenceE2eTest, StartOrphansWhatTheFilesLost) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_reconcile");
  const PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  const int total_steps = kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(total_steps);
  const int kept_step = 5;
  const int adopted_step = 25;

  AnchorId kept = 0;
  AnchorId adopted = 0;
  std::string stale;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, kept_step);
    kept = SaveAnchor(backend)->id;
    FeedTo(backend, frontend, step, adopted_step);
    adopted = SaveAnchor(backend)->id;
    stale = common::ReadFile(AnchorsPath(*backend.map_manager())).value_or("");
    ASSERT_FALSE(stale.empty());
    backend.FreezeFedSession();
    backend.WaitUntilQuiescent();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
    ASSERT_NE(GetAnchor(backend, adopted)->submap_id.session_id, 0) << "not adopted";
  }
  {
    MapManager reader(directory);
    PoseGraphData graph;
    ASSERT_TRUE(testing::LoadMap(reader, graph).has_value());
    ASSERT_TRUE(common::WriteFileAtomically(AnchorsPath(reader), stale));
  }
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(total_steps));
    const Anchor lost = *GetAnchor(backend, adopted);
    EXPECT_EQ(lost.state, AnchorState::ORPHAN);
    EXPECT_EQ(lost.orphan_reason, OrphanReason::SUBMAP_MISSING);
    const ResolvedAnchor intact = ResolveAnchor(backend, kept);
    EXPECT_EQ(intact.state, AnchorState::BOUND);
    EXPECT_TRUE(intact.frozen);
    ExpectNearTruth(intact, kept_step);
    const std::optional<AnchorTable> on_disk = ReadAnchorsFromDisk(directory);
    ASSERT_TRUE(on_disk.has_value());
    EXPECT_EQ(on_disk->anchors[1].orphan_reason, OrphanReason::SUBMAP_MISSING)
        << "Start persists what Reconcile found";
  }

  PoseGraph backend(option, directory);
  backend.Start(TestTime(total_steps + 1));
  const AnchorTable table = RunOnTask(backend, [&] { return backend.anchors().Table(); });
  EXPECT_EQ(table.next_id, 3u);
  ASSERT_EQ(table.anchors.size(), 2u);
  EXPECT_EQ(table.anchors[0].state, AnchorState::BOUND);
  EXPECT_EQ(table.anchors[1].orphan_reason, OrphanReason::SUBMAP_MISSING) << "orphan is final";
  Frontend frontend(backend, odometry);
  int step = 0;
  FeedTo(backend, frontend, step, 0);
  EXPECT_EQ(SaveAnchor(backend)->id, 3u) << "an id was reissued";
}

// The trim that rebinds an anchor and the session file that drops its submap land in one commit.
// A kill inside that commit, before the manifest, must restart on the previous generation whole:
// the anchor still bound to the submap its session file still holds, never SUBMAP_MISSING.
TEST(AnchorPersistenceE2eTest, TrimThenKillNoLongerOrphans) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_trim_kill");
  const std::string killed = testing::MakeTempDir("evergreenslam_anchor_trim_kill_copy");
  PoseGraphOption option = testing::AnchorTestOption();
  option.trim = true;
  const int max_steps = 7 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(max_steps);

  PoseGraph backend(option, directory);
  backend.Start(TestTime(0));
  Frontend frontend(backend, odometry);
  int step = FreezeABase(backend, frontend, kNodesPerLap);
  const int save_step = step + kNodesPerLap / 2;
  FeedTo(backend, frontend, step, save_step);
  const std::optional<Anchor> anchor = SaveAnchor(backend);
  ASSERT_TRUE(anchor.has_value());
  const AnchorId id = anchor->id;
  const SubmapId bound_to = anchor->submap_id;

  std::atomic<bool> copied{false};
  RunOnTask(backend, [&] {
    backend.map_manager()->set_before_manifest_write_hook([&] {
      if (copied || backend.anchors().Get(id)->state != AnchorState::REBOUND) {
        return;
      }
      std::filesystem::remove_all(killed);
      std::filesystem::copy(directory, killed, std::filesystem::copy_options::recursive);
      copied = true;
    });
  });
  while (!copied && step < max_steps) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_TRUE(copied) << "the anchor's submap was never trimmed within " << max_steps << " steps";
  RunOnTask(backend, [&] { backend.map_manager()->set_before_manifest_write_hook(nullptr); });

  const Anchor rebound = *GetAnchor(backend, id);
  ASSERT_EQ(rebound.state, AnchorState::REBOUND);
  const std::optional<AnchorTable> committed = ReadAnchorsFromDisk(directory);
  ASSERT_TRUE(committed.has_value());
  ASSERT_EQ(committed->anchors.size(), 1u);
  EXPECT_EQ(committed->anchors[0].state, AnchorState::REBOUND);
  EXPECT_EQ(committed->anchors[0].submap_id, rebound.submap_id);

  {
    MapManager reader(killed);
    PoseGraphData graph;
    ASSERT_TRUE(testing::LoadMap(reader, graph).has_value());
    EXPECT_TRUE(graph.HasSubmap(bound_to)) << "the old session file lost the submap";
    const std::optional<AnchorTable> previous = reader.ReadAnchors();
    ASSERT_TRUE(previous.has_value());
    ASSERT_EQ(previous->anchors.size(), 1u);
    EXPECT_EQ(previous->anchors[0].state, AnchorState::BOUND);
    EXPECT_EQ(previous->anchors[0].submap_id, bound_to);
  }
  PoseGraph restarted(testing::NoFreeze(option), killed);
  restarted.Start(TestTime(max_steps));
  const std::optional<Anchor> reloaded = GetAnchor(restarted, id);
  ASSERT_TRUE(reloaded.has_value());
  EXPECT_EQ(reloaded->state, AnchorState::BOUND)
      << "orphaned as " << ToString(reloaded->orphan_reason);
  EXPECT_EQ(reloaded->submap_id, bound_to);
  ExpectNearTruth(ResolveAnchor(restarted, id), save_step);
}

// A save whose commit fails hands out no id and changes nothing: the agent would write the id
// into place.yaml and a kill would reissue it, and a refused rebind that stayed in memory would
// move the place at the next checkpoint anyway.
TEST(AnchorPersistenceE2eTest, UnpersistedSaveIsRefused) {
  using Refusal = PoseGraph::SaveAnchorResult::Refusal;
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_not_persisted");
  PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  option.checkpoint_min_interval = common::FromSeconds(1e6);
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(kNodesPerLap);
  const int kept_step = 8;
  const int save_step = 20;

  AnchorId kept = 0;
  AnchorId burned = 0;
  AnchorId saved = 0;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, kept_step);
    kept = SaveAnchor(backend)->id;
    const Anchor kept_before = *GetAnchor(backend, kept);
    FeedTo(backend, frontend, step, save_step);

    const std::filesystem::path blocker = RunOnTask(backend, [&] {
      char name[64];
      std::snprintf(name, sizeof(name), "session_%06d.g%lld.pb",
                    backend.session_manager().fed_session()->session_index,
                    static_cast<long long>(backend.map_manager()->generation() + 1));
      return std::filesystem::path(directory) / name;
    });
    ASSERT_TRUE(std::filesystem::create_directories(blocker / "occupied"));
    burned = RunOnTask(backend, [&] { return backend.anchors().Table().next_id; });
    const PoseGraph::SaveAnchorResult refused =
        RunOnTask(backend, [&] { return backend.SaveAnchorOnTask(true); });
    EXPECT_EQ(refused.refusal, Refusal::NOT_PERSISTED);
    EXPECT_FALSE(refused.anchor.has_value());
    EXPECT_FALSE(GetAnchor(backend, burned).has_value()) << "the refused row leaked";
    const PoseGraph::SaveAnchorResult refused_rebind =
        RunOnTask(backend, [&] { return backend.SaveAnchorOnTask(true, kept); });
    EXPECT_EQ(refused_rebind.refusal, Refusal::NOT_PERSISTED);
    const Anchor kept_after = *GetAnchor(backend, kept);
    EXPECT_EQ(kept_after.node_id, kept_before.node_id) << "the refused rebind moved the place";
    EXPECT_EQ(kept_after.submap_id, kept_before.submap_id);
    EXPECT_FALSE(RunOnTask(backend, [&] { return backend.CheckpointOnTask(); }));

    std::filesystem::remove_all(blocker);
    ASSERT_TRUE(RunOnTask(backend, [&] { return backend.CheckpointOnTask(); }));
    const PoseGraph::SaveAnchorResult accepted =
        RunOnTask(backend, [&] { return backend.SaveAnchorOnTask(true); });
    ASSERT_EQ(accepted.refusal, Refusal::NONE);
    ASSERT_TRUE(accepted.anchor.has_value());
    saved = accepted.anchor->id;
    EXPECT_GT(saved, burned) << "a refused id was issued again";
  }

  PoseGraph backend(option, directory);
  backend.Start(TestTime(kNodesPerLap));
  ExpectNearTruth(ResolveAnchor(backend, kept), kept_step);
  ExpectNearTruth(ResolveAnchor(backend, saved), save_step);
  EXPECT_FALSE(GetAnchor(backend, burned).has_value()) << "the refused row reached disk";
  const AnchorTable table = RunOnTask(backend, [&] { return backend.anchors().Table(); });
  EXPECT_EQ(table.anchors.size(), 2u);
  EXPECT_GT(table.next_id, saved) << "an id handed out was reissued";
}

// A kill inside a commit leaves its new-generation files next to the previous manifest. The next
// Start loads that previous generation and collects the leftovers before it writes anything.
TEST(AnchorPersistenceE2eTest, StartCollectsAnInterruptedCommit) {
  const std::string directory = testing::MakeTempDir("evergreenslam_anchor_start_gc");
  const std::string killed = testing::MakeTempDir("evergreenslam_anchor_start_gc_copy");
  PoseGraphOption option = testing::NoFreeze(testing::AnchorTestOption());
  option.checkpoint_min_interval = common::FromSeconds(1e6);
  const std::vector<Eigen::Affine2d> odometry = testing::GroundTruthOdometry(kNodesPerLap);
  const int kept_step = 8;

  AnchorId kept = 0;
  AnchorId lost = 0;
  int nodes_before = 0;
  std::vector<std::string> leftovers;
  {
    PoseGraph backend(option, directory);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    int step = 0;
    FeedTo(backend, frontend, step, kept_step);
    kept = SaveAnchor(backend)->id;
    nodes_before =
        RunOnTask(backend, [&] { return static_cast<int>(backend.graph().nodes().size()); });
    FeedTo(backend, frontend, step, kept_step + 10);
    RunOnTask(backend, [&] {
      backend.map_manager()->set_before_manifest_write_hook([&] {
        const std::string suffix =
            ".g" + std::to_string(backend.map_manager()->generation() + 1) + ".pb";
        for (const auto& item : std::filesystem::directory_iterator(directory)) {
          const std::string name = item.path().filename().string();
          if (name.size() > suffix.size() &&
              name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
            leftovers.push_back(name);
          }
        }
        std::filesystem::remove_all(killed);
        std::filesystem::copy(directory, killed, std::filesystem::copy_options::recursive);
      });
    });
    lost = SaveAnchor(backend)->id;
    RunOnTask(backend, [&] { backend.map_manager()->set_before_manifest_write_hook(nullptr); });
  }
  ASSERT_GE(leftovers.size(), 2u) << "the interrupted commit wrote a session and the anchors";
  for (const std::string& name : leftovers) {
    ASSERT_TRUE(std::filesystem::exists(std::filesystem::path(killed) / name)) << name;
  }

  PoseGraph backend(option, killed);
  backend.Start(TestTime(kNodesPerLap));
  for (const std::string& name : leftovers) {
    EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(killed) / name))
        << name << " outlived Start";
  }
  EXPECT_EQ(RunOnTask(backend, [&] { return static_cast<int>(backend.graph().nodes().size()); }),
            nodes_before)
      << "Start did not load the previous generation";
  ExpectNearTruth(ResolveAnchor(backend, kept), kept_step);
  EXPECT_FALSE(GetAnchor(backend, lost).has_value());
}

}  // namespace
}  // namespace evergreenslam::lifelong

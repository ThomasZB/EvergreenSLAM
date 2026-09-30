/**
 * @file drop_fed_session_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Dropping the fed session in process: a real frontend maps a run over a frozen base, the
 *        run is dropped, a fresh frontend starts elsewhere, relocalizes onto the base and
 *        persists; the dropped run is gone from graph, anchors and disk.
 * @version 0.1
 * @date 2026-09-30
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "anchors/anchor_scenario.h"
#include "common/time.h"
#include "lifelong/map_manager/map_manager.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "mapping/local_trajectory_builder.h"
#include "sensor/timed_point_cloud.h"
#include "testing/load_map.h"
#include "testing/loop_scenario.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using testing::Frontend;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::RunOnTask;
using testing::TestTime;
using Refusal = PoseGraph::DropFedSessionResult::Refusal;

constexpr int kScansPerStep = 4;

// Same relaxed backend as the relocalization story, so the base freezes within two laps. One
// match worker: which closures land must not vary run to run.
PoseGraphOption DropOption() {
  PoseGraphOption option;
  option.constraint_builder.num_match_workers = 1;
  option.optimization.optimize_every_n_nodes = 10;
  option.checkpoint_min_interval = common::Duration::zero();
  option.session_manager.freeze_judge.max_translation_stddev = 1.0;
  option.session_manager.freeze_judge.max_rotation_stddev = 1.0;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  option.trim = false;
  return option;
}

PoseGraphOption NoFreeze(PoseGraphOption option) {
  option.session_manager.auto_freeze = false;
  return option;
}

Eigen::Affine2d LoopPoseAt(double step) {
  const double phi = 2.0 * M_PI * step / static_cast<double>(kNodesPerLap);
  return transform::FromXYTheta(6.4 + testing::kLoopRadius * std::sin(phi),
                                1.9 + testing::kLoopRadius * (1.0 - std::cos(phi)), phi);
}

common::Time ScanTime(int scan_index) {
  return common::FromUnixSeconds(1785000100.0) + common::FromSeconds(0.05 * scan_index);
}

double TranslationError(const Eigen::Affine2d& actual, const Eigen::Affine2d& expected) {
  return (actual.translation() - expected.translation()).norm();
}

double RotationError(const Eigen::Affine2d& actual, const Eigen::Affine2d& expected) {
  return std::abs(
      transform::NormalizeAngle(transform::GetYaw(actual) - transform::GetYaw(expected)));
}

bool Touches(const Constraint& constraint, SessionId session) {
  return constraint.from.session() == session ||
         (constraint.to.has_value() && constraint.to->session() == session);
}

int ClosuresOntoOtherSessions(const PoseGraphData& graph, SessionId session) {
  int count = 0;
  for (const Constraint& constraint : graph.constraints()) {
    if (constraint.type == Constraint::Type::INTER_SUBMAP && constraint.to.has_value() &&
        constraint.from.session() != constraint.to->session() && Touches(constraint, session)) {
      ++count;
    }
  }
  return count;
}

// A real LocalTrajectoryBuilder driven along the loop, a scan every quarter of a loop step. Its
// local frame is identity at the first scan, so it maps to global through LoopPoseAt(first).
class BuilderRun {
 public:
  BuilderRun(PoseGraph& backend, double first_step, int first_scan)
      : backend_(backend), first_step_(first_step), scan_index_(first_scan) {}

  void Drive(int num_scans) {
    for (int i = 0; i < num_scans; ++i) {
      const double step = first_step_ + static_cast<double>(num_scans_) / kScansPerStep;
      const Eigen::Affine2d truth = LoopPoseAt(step);
      sensor::TimedPointCloud cloud;
      for (const sensor::Point2d& point : testing::SimulateScan(testing::LoopRoom(), truth)) {
        cloud.push_back(sensor::TimedPoint2d{point.point, common::Duration::zero()});
      }
      const auto matching = builder_.AddScan(ScanTime(scan_index_++), cloud);
      ++num_scans_;
      if (matching != nullptr && matching->insertion_result != nullptr) {
        backend_.AddInsertionResult(*matching->insertion_result);
        keyframe_truths_.push_back(truth);
        if (wait_) {
          backend_.WaitUntilQuiescent();
        }
      }
    }
  }

  const std::vector<Eigen::Affine2d>& keyframe_truths() const { return keyframe_truths_; }
  int scan_index() const { return scan_index_; }
  void set_wait(bool wait) { wait_ = wait; }

 private:
  PoseGraph& backend_;
  double first_step_;
  int scan_index_;
  int num_scans_ = 0;
  bool wait_ = true;
  mapping::LocalTrajectoryBuilder builder_;
  std::vector<Eigen::Affine2d> keyframe_truths_;
};

std::set<std::string> CommitFiles(const std::string& directory) {
  std::set<std::string> files;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    const std::string name = entry.path().filename().string();
    if (entry.is_regular_file() &&
        (name.rfind("session_", 0) == 0 || name.rfind("anchors", 0) == 0)) {
      files.insert(name);
    }
  }
  return files;
}

class DropFedSessionE2eTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    directory_ = testing::MakeTempDir("evergreenslam_drop_fed_" + name);
    option_ = DropOption();
  }

  void FreezeTheBase() {
    const int max_steps = 2 * kNodesPerLap;
    std::vector<Eigen::Affine2d> odometry;
    for (int i = 0; i < max_steps; ++i) {
      odometry.push_back(LoopGroundTruth(i));
    }
    PoseGraph backend(option_, directory_);
    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    base_ = *backend.session_manager().fed_session();
    int step = 0;
    while (backend.session_manager().num_sessions_frozen() == 0 && step < max_steps) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    backend.Finish();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
        << "the base never froze; last verdict: "
        << ToString(backend.session_manager().last_verdict().rejection);
  }

  std::string directory_;
  PoseGraphOption option_;
  SessionId base_;
};

TEST_F(DropFedSessionE2eTest, TheDroppedRunIsGoneAndAFreshFrontendRelocalizesOntoTheBase) {
  ASSERT_NO_FATAL_FAILURE(FreezeTheBase());

  constexpr double kFirstRunStep = 6.3;
  constexpr double kSecondRunStep = 47.7;
  auto backend = std::make_unique<PoseGraph>(NoFreeze(option_), directory_);
  backend->Start(ScanTime(0), LoopPoseAt(kFirstRunStep));
  const SessionId old = *backend->session_manager().fed_session();
  ASSERT_EQ(backend->boot_first_session(), old);

  auto first_run = std::make_unique<BuilderRun>(*backend, kFirstRunStep, 0);
  first_run->Drive(30 * kScansPerStep);
  ASSERT_GT(backend->graph().session(old).node_ids.size(), 5u);
  const std::optional<Anchor> anchor = testing::SaveAnchor(*backend, false);
  ASSERT_TRUE(anchor.has_value());
  ASSERT_EQ(SessionOf(anchor->submap_id), old);
  backend->WaitUntilQuiescent();
  const std::optional<std::string> old_file = backend->map_manager()->FileNameOf(old);
  ASSERT_TRUE(old_file.has_value()) << "the first run never checkpointed";
  const int next_scan = first_run->scan_index();

  const PoseGraph::DropFedSessionResult result = backend->DropFedSession(old);
  ASSERT_EQ(result.refusal, Refusal::NONE);
  ASSERT_TRUE(result.fed_now.has_value());
  const SessionId next = *result.fed_now;
  EXPECT_EQ(next.session_index, old.session_index + 1);
  EXPECT_EQ(*backend->session_manager().fed_session(), next);
  EXPECT_EQ(backend->boot_first_session(), old);
  EXPECT_FALSE(backend->ActiveSessionToGlobal().has_value()) << "the pose is lost until anchored";
  EXPECT_EQ(backend->DropFedSession(old).refusal, Refusal::NOT_FED);
  EXPECT_EQ(backend->DropFedSession(base_).refusal, Refusal::NOT_FED);

  // Any keyframe of the old builder from here on would be a caller bug; it is gone.
  first_run.reset();
  backend->SetInitialPose(LoopPoseAt(kSecondRunStep));
  BuilderRun second_run(*backend, kSecondRunStep, next_scan);
  second_run.Drive(25 * kScansPerStep);
  backend->WaitUntilQuiescent();

  const PoseGraphData& graph = backend->graph();
  EXPECT_FALSE(graph.HasSession(old));
  for (const Constraint& constraint : graph.constraints()) {
    EXPECT_FALSE(Touches(constraint, old));
  }
  EXPECT_EQ(backend->optimization().num_variables(),
            static_cast<int>(graph.submaps().size() + graph.nodes().size()));
  const std::optional<Anchor> orphaned = backend->anchors().Get(anchor->id);
  ASSERT_TRUE(orphaned.has_value());
  EXPECT_EQ(orphaned->state, AnchorState::ORPHAN);
  EXPECT_EQ(orphaned->orphan_reason, OrphanReason::SESSION_REMOVED);

  const SessionData& session = graph.session(next);
  const std::vector<Eigen::Affine2d>& truths = second_run.keyframe_truths();
  ASSERT_EQ(session.node_ids.size(), truths.size());
  ASSERT_GT(truths.size(), 5u);
  for (size_t k = 0; k < session.node_ids.size(); ++k) {
    EXPECT_EQ(session.node_ids[k], (NodeId{next.session_index, static_cast<int>(k)}));
  }
  ASSERT_GT(ClosuresOntoOtherSessions(graph, next), 0) << "no closure against the base";
  double mean_error = 0.0;
  for (size_t k = 0; k < truths.size(); ++k) {
    mean_error += TranslationError(graph.node(session.node_ids[k]).global_pose, truths[k]);
  }
  mean_error /= static_cast<double>(truths.size());
  EXPECT_LT(mean_error, 0.35);
  const Eigen::Affine2d& newest = graph.node(session.node_ids.back()).global_pose;
  EXPECT_LT(TranslationError(newest, truths.back()), 0.15);
  EXPECT_LT(RotationError(newest, truths.back()), 0.05);
  const std::optional<Eigen::Affine2d> alignment = graph.ComputeSessionToGlobal(next);
  ASSERT_TRUE(alignment.has_value());
  EXPECT_LT(TranslationError(*alignment, LoopPoseAt(kSecondRunStep)), 0.15);
  EXPECT_LT(RotationError(*alignment, LoopPoseAt(kSecondRunStep)), 0.05);
  const std::optional<Eigen::Affine2d> published = backend->ActiveSessionToGlobal();
  ASSERT_TRUE(published.has_value());
  EXPECT_LT((published->matrix() - alignment->matrix()).norm(), 1e-12);

  const size_t num_nodes = session.node_ids.size();
  backend->Finish();
  backend.reset();
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(directory_) / *old_file));

  {
    MapManager reader(directory_);
    PoseGraphData loaded;
    const std::optional<MapManager::LoadResult> loaded_result = testing::LoadMap(reader, loaded);
    ASSERT_TRUE(loaded_result.has_value());
    EXPECT_FALSE(loaded.HasSession(old));
    ASSERT_TRUE(loaded.HasSession(next));
    EXPECT_EQ(loaded.session(next).node_ids.size(), num_nodes);
    const auto entry = std::find_if(
        loaded_result->unfrozen_sessions.begin(), loaded_result->unfrozen_sessions.end(),
        [&next](const MapManager::LoadResult::UnfrozenSession& unfrozen) {
          return unfrozen.id == next;
        });
    ASSERT_NE(entry, loaded_result->unfrozen_sessions.end());
    EXPECT_EQ(entry->last_fed_boot, reader.boot_count()) << "the removal commit unmarked it fed";
    reader.RemoveUnreferencedFiles();
    std::set<std::string> referenced;
    for (const auto& [id, data] : loaded.sessions()) {
      referenced.insert(*reader.FileNameOf(id));
    }
    referenced.insert(*reader.anchors_file_name());
    EXPECT_EQ(CommitFiles(directory_), referenced);
  }

  PoseGraph reloaded(NoFreeze(option_), directory_);
  reloaded.Start(ScanTime(second_run.scan_index() + 1));
  reloaded.WaitUntilQuiescent();
  EXPECT_FALSE(reloaded.graph().HasSession(old));
  ASSERT_TRUE(reloaded.graph().HasSession(next));
  EXPECT_EQ(reloaded.graph().session(next).node_ids.size(), num_nodes);
  EXPECT_TRUE(reloaded.graph().session(base_).frozen());
  const std::optional<Anchor> reloaded_anchor = reloaded.anchors().Get(anchor->id);
  ASSERT_TRUE(reloaded_anchor.has_value());
  EXPECT_EQ(reloaded_anchor->orphan_reason, OrphanReason::SESSION_REMOVED);
}

// The drop lands behind the old run's last keyframes, so every match those keyframes start on the
// workers resolves after it and must be discarded, not added against a removed session.
TEST_F(DropFedSessionE2eTest, MatchesInFlightAtTheDropAreDiscarded) {
  ASSERT_NO_FATAL_FAILURE(FreezeTheBase());

  constexpr double kFirstRunStep = 6.3;
  constexpr double kSecondRunStep = 47.7;
  PoseGraphOption option = NoFreeze(option_);
  option.constraint_builder.num_match_workers = 3;
  PoseGraph backend(option, directory_);
  backend.Start(ScanTime(0), LoopPoseAt(kFirstRunStep));
  const SessionId old = *backend.session_manager().fed_session();

  auto first_run = std::make_unique<BuilderRun>(backend, kFirstRunStep, 0);
  first_run->Drive(15 * kScansPerStep);
  ASSERT_GT(ClosuresOntoOtherSessions(backend.graph(), old), 0);
  const int attempted_before = backend.constraint_builder().num_matches_attempted();
  const int dropped_before = backend.constraint_builder().num_constraints_dropped();

  std::promise<void> gate;
  std::shared_future<void> gate_open = gate.get_future().share();
  backend.Enqueue([gate_open] { gate_open.wait(); });
  first_run->set_wait(false);
  first_run->Drive(15 * kScansPerStep);
  const int next_scan = first_run->scan_index();
  std::promise<PoseGraph::DropFedSessionResult> dropped;
  std::future<PoseGraph::DropFedSessionResult> drop_result = dropped.get_future();
  backend.Enqueue(
      [&backend, &dropped, old] { dropped.set_value(backend.DropFedSessionOnTask(old)); });
  gate.set_value();
  const PoseGraph::DropFedSessionResult result = drop_result.get();
  ASSERT_EQ(result.refusal, Refusal::NONE);
  const SessionId next = *result.fed_now;
  first_run.reset();
  backend.WaitUntilQuiescent();
  EXPECT_GT(backend.constraint_builder().num_matches_attempted(), attempted_before);
  EXPECT_GT(backend.constraint_builder().num_constraints_dropped(), dropped_before)
      << "no match of the dropped run was still in flight";

  backend.SetInitialPose(LoopPoseAt(kSecondRunStep));
  BuilderRun second_run(backend, kSecondRunStep, next_scan);
  second_run.Drive(15 * kScansPerStep);
  backend.WaitUntilQuiescent();

  const PoseGraphData& graph = backend.graph();
  EXPECT_FALSE(graph.HasSession(old));
  for (const Constraint& constraint : graph.constraints()) {
    EXPECT_FALSE(Touches(constraint, old));
  }
  EXPECT_EQ(backend.optimization().num_variables(),
            static_cast<int>(graph.submaps().size() + graph.nodes().size()));
  const SessionData& session = graph.session(next);
  ASSERT_EQ(session.node_ids.size(), second_run.keyframe_truths().size());
  ASSERT_GT(session.node_ids.size(), 5u);
  EXPECT_EQ(session.node_ids.front(), (NodeId{next.session_index, 0}));
  EXPECT_GT(ClosuresOntoOtherSessions(graph, next), 0);
  backend.Finish();
}

TEST_F(DropFedSessionE2eTest, AFedSessionWithNothingIngestedDropsToo) {
  PoseGraph backend(option_, directory_);
  backend.Start(TestTime(0));
  const SessionId old = *backend.session_manager().fed_session();
  backend.WaitUntilQuiescent();
  const std::optional<std::string> old_file = backend.map_manager()->FileNameOf(old);
  ASSERT_TRUE(old_file.has_value()) << "Start commits the fed session's file";

  const PoseGraph::DropFedSessionResult result = backend.DropFedSession(old);
  ASSERT_EQ(result.refusal, Refusal::NONE);
  ASSERT_TRUE(result.fed_now.has_value());
  EXPECT_EQ(result.fed_now->session_index, old.session_index + 1);
  backend.WaitUntilQuiescent();
  EXPECT_FALSE(backend.graph().HasSession(old));
  EXPECT_FALSE(backend.map_manager()->FileNameOf(old).has_value());
  EXPECT_TRUE(backend.map_manager()->FileNameOf(*result.fed_now).has_value());
  EXPECT_FALSE(std::filesystem::exists(std::filesystem::path(directory_) / *old_file));

  // A fresh frontend feeds the replacement from node index 0.
  std::vector<Eigen::Affine2d> odometry;
  for (int i = 0; i < 25; ++i) {
    odometry.push_back(LoopGroundTruth(i));
  }
  Frontend frontend(backend, odometry);
  for (int step = 0; step < 25; ++step) {
    frontend.Feed(step);
  }
  backend.WaitUntilQuiescent();
  EXPECT_EQ(backend.graph().session(*result.fed_now).node_ids.size(), 25u);
}

TEST_F(DropFedSessionE2eTest, AFreezeInFlightRefusesTheDrop) {
  PoseGraph backend(NoFreeze(option_));
  backend.Start(TestTime(0));
  const SessionId fed = *backend.session_manager().fed_session();
  std::vector<Eigen::Affine2d> odometry;
  for (int i = 0; i < 30; ++i) {
    odometry.push_back(LoopGroundTruth(i));
  }
  Frontend frontend(backend, odometry);
  for (int step = 0; step < 30; ++step) {
    frontend.Feed(step);
  }
  backend.WaitUntilQuiescent();

  // The gate holds the consumer so the drop lands between the freeze and its queued rotation.
  std::promise<void> gate;
  std::shared_future<void> gate_open = gate.get_future().share();
  backend.Enqueue([gate_open] { gate_open.wait(); });
  backend.FreezeFedSession(fed);
  std::promise<PoseGraph::DropFedSessionResult> dropped;
  std::future<PoseGraph::DropFedSessionResult> drop_result = dropped.get_future();
  backend.Enqueue(
      [&backend, &dropped, fed] { dropped.set_value(backend.DropFedSessionOnTask(fed)); });
  gate.set_value();

  const PoseGraph::DropFedSessionResult result = drop_result.get();
  EXPECT_EQ(result.refusal, Refusal::FREEZING);
  EXPECT_FALSE(result.fed_now.has_value());
  backend.WaitUntilQuiescent();
  EXPECT_TRUE(backend.graph().session(fed).frozen()) << "the refused drop changed nothing";
}

// Two commits: the successor's, then the removal. A kill between them keeps the old session.
TEST_F(DropFedSessionE2eTest, AKillBetweenTheTwoCommitsKeepsTheOldSession) {
  PoseGraph backend(NoFreeze(option_), directory_);
  backend.Start(TestTime(0));
  const SessionId old = *backend.session_manager().fed_session();
  std::vector<Eigen::Affine2d> odometry;
  for (int i = 0; i < 25; ++i) {
    odometry.push_back(LoopGroundTruth(i));
  }
  Frontend frontend(backend, odometry);
  for (int step = 0; step < 25; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  ASSERT_TRUE(RunOnTask(backend, [&backend] { return backend.CheckpointOnTask(); }));
  const size_t old_nodes = backend.graph().session(old).node_ids.size();
  ASSERT_GT(old_nodes, 0u);

  const std::string killed = testing::MakeTempDir("evergreenslam_drop_fed_killed");
  int manifest_writes = 0;
  const std::string directory = directory_;
  backend.map_manager()->set_before_manifest_write_hook([&manifest_writes, &killed, directory] {
    if (++manifest_writes == 2) {
      std::filesystem::copy(directory, killed,
                            std::filesystem::copy_options::recursive |
                                std::filesystem::copy_options::overwrite_existing);
    }
  });
  const PoseGraph::DropFedSessionResult result = backend.DropFedSession(old);
  ASSERT_EQ(result.refusal, Refusal::NONE);
  ASSERT_EQ(manifest_writes, 2);

  MapManager reader(killed);
  PoseGraphData loaded;
  ASSERT_TRUE(testing::LoadMap(reader, loaded).has_value());
  ASSERT_TRUE(loaded.HasSession(old));
  EXPECT_EQ(loaded.session(old).node_ids.size(), old_nodes);
  EXPECT_TRUE(loaded.HasSession(*result.fed_now));
}

}  // namespace
}  // namespace evergreenslam::lifelong

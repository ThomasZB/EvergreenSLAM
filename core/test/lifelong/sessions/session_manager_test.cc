/**
 * @file session_manager_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Freeze sequencing: what fires, in what order, and exactly how many times.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/session_manager.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "common/time.h"
#include "lifelong/coarse_footprint.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "lifelong/sessions/frozen_coverage.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr int kNodesPerSubmap = 10;
constexpr double kRingRadius = 2.013;
// Session-local ring centres, off the coarse lattice; no two share a coarse cell.
const Eigen::Vector2d kHome(1.017, 0.503);
const Eigen::Vector2d kAway(40.281, 30.119);
const Eigen::Vector2d kFar(80.437, 60.223);
const Eigen::Vector2d kFarther(-50.311, 70.107);

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// Sequencing is what these tests are about: a small growth threshold, a short settling window
// and a numeric gate opened wide. One ring is ~13 m^2, so the shipped 20 m^2 tolerance would
// swallow a whole ring; 1 m^2 still hides the lattice jitter of a solved pose. The feeder walks
// 0.31 m a node, so a submap stands in ~8 fresh 0.4 m cells (~1.2 m^2) unless a test pins it.
SessionManagerOption TestOption() {
  SessionManagerOption option;
  option.min_new_area = 5.0;
  option.min_new_area_fraction = 0.0;
  option.growth_tolerance = 1.0;
  option.track_growth_tolerance = 0.5;
  option.submaps_without_growth = 2;
  option.freeze_judge.max_translation_stddev = 10.0;
  option.freeze_judge.max_rotation_stddev = 10.0;
  return option;
}

// A gate no submap passes, so a JUDGING session stays and its count stays observable.
SessionManagerOption ShutGateOption() {
  SessionManagerOption option = TestOption();
  option.freeze_judge.max_translation_stddev = 1e-9;
  return option;
}

using Phase = SessionManager::Phase;

PoseGraphOption ManualBackendOption() {
  PoseGraphOption option;
  // Solving is triggered by the test, so the freeze cadence is unambiguous.
  option.optimization.optimize_every_n_nodes = 100000;
  return option;
}

sensor::PointCloud RingScan() {
  sensor::PointCloud cloud;
  for (int i = 0; i < 72; ++i) {
    const double angle = 2.0 * M_PI * i / 72.0;
    cloud.push_back(
        sensor::Point2d{kRingRadius * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  return cloud;
}

// Every grid holds one ring around a session-local point, so a submap's footprint is either the
// frozen layer's own cells (kHome) or entirely new ones (kAway). Notifications mirror the
// production wiring, for a manager that is not the graph's own.
class Feeder {
 public:
  Feeder(PoseGraph& pose_graph, SessionManager* manager, SessionId session)
      : pose_graph_(pose_graph), manager_(manager) {
    Adopt(session);
  }

  // What the rotation does to the ingest table: the window's submaps now carry successor ids.
  void Adopt(SessionId session) {
    session_ = session;
    const IdAllocator& allocator = pose_graph_.graph().id_allocator();
    next_index_ = allocator.next_node_index(session);
    next_submap_index_ = allocator.next_submap_index(session);
    for (size_t k = 0; k < submaps_.size(); ++k) {
      submaps_[k].first = SubmapId{session.session_index, static_cast<int>(k)};
    }
  }

  void set_ring_center(const Eigen::Vector2d& center) { ring_center_ = center; }
  // 0 pins the robot: the rings still land wherever they are told, the nodes stand still.
  void set_step(double step) { step_ = step; }

  void FeedNodes(int count) {
    for (int i = 0; i < count; ++i) {
      const Eigen::Affine2d local_pose = transform::FromXYTheta(x_, 0.0, 0.0);
      x_ += step_;
      if (submaps_.empty() || fed_into_newest_ >= kNodesPerSubmap) {
        const SubmapId id{session_.session_index, next_submap_index_++};
        auto submap = std::make_shared<mapping::Submap>(id.submap_index, local_pose, 0.05);
        submap->InsertScan(transform::FromXYTheta(ring_center_.x(), ring_center_.y(), 0.0),
                           RingScan(), inserter_);
        submaps_.emplace_back(id, submap);
        fed_into_newest_ = 0;
        if (submaps_.size() > 2) {
          Finish(submaps_.front());
          submaps_.erase(submaps_.begin());
        }
      }
      ++fed_into_newest_;
      testing::ExplicitInsertion insertion;
      insertion.node_id = NodeId{session_.session_index, next_index_};
      insertion.node.time = TestTime(next_index_);
      insertion.node.local_pose = local_pose;
      for (const auto& [id, submap] : submaps_) {
        insertion.insertion_submaps.emplace_back(id, submap);
      }
      const std::vector<std::pair<SubmapId, std::shared_ptr<mapping::Submap>>> window = submaps_;
      testing::EnqueueExplicitInsertion(pose_graph_, std::move(insertion));
      if (manager_ != nullptr) {
        for (const auto& entry : window) {
          pose_graph_.Enqueue([manager = manager_, id = entry.first,
                               node = NodeId{session_.session_index, next_index_}] {
            Constraint constraint;
            constraint.type = Constraint::Type::INTRA_SUBMAP;
            constraint.from = VariableId::Of(id);
            constraint.to = VariableId::Of(node);
            manager->OnConstraintAddedOnTask(constraint);
          });
        }
      }
      ++next_index_;
    }
  }

  // Mints one submap ringed at `center`; the submap minted two calls earlier finishes.
  void FeedSubmap(const Eigen::Vector2d& center) {
    set_ring_center(center);
    FeedNodes(kNodesPerSubmap);
  }

  void FinishAll() {
    for (auto& entry : submaps_) {
      if (!entry.second->finished()) {
        Finish(entry);
      }
    }
  }

 private:
  void Finish(const std::pair<SubmapId, std::shared_ptr<mapping::Submap>>& entry) {
    entry.second->Finish();
    if (manager_ != nullptr) {
      pose_graph_.Enqueue(
          [manager = manager_, id = entry.first] { manager->OnSubmapFinishedOnTask(id); });
    }
  }

  PoseGraph& pose_graph_;
  SessionManager* manager_;
  SessionId session_;
  Eigen::Vector2d ring_center_ = kHome;
  mapping::CastRaysMapping inserter_;
  double x_ = 0.0;
  double step_ = 0.31;
  int next_index_ = 0;
  int next_submap_index_ = 0;
  int fed_into_newest_ = 0;
  std::vector<std::pair<SubmapId, std::shared_ptr<mapping::Submap>>> submaps_;
};

class RecordingObserver : public SessionObserver {
 public:
  void OnSessionFrozen(const PoseGraphData& graph, SessionId id) override {
    frozen.push_back(id);
    // By the time this fires the state has already flipped.
    frozen_when_notified.push_back(graph.session(id).frozen());
    events.push_back("frozen " + std::to_string(id.session_index));
  }

  void OnSessionStarted(const PoseGraphData& graph, SessionId id) override {
    started.push_back(id);
    int unfinished = 0;
    for (const SubmapId& submap_id : graph.session(id).submap_ids) {
      const auto& submap = graph.submap(submap_id).submap;
      if (submap != nullptr && !submap->finished()) {
        ++unfinished;
      }
    }
    submaps_when_notified.push_back(static_cast<int>(graph.session(id).submap_ids.size()));
    unfinished_when_notified.push_back(unfinished);
    events.push_back("started " + std::to_string(id.session_index));
  }

  std::vector<SessionId> frozen;
  std::vector<SessionId> started;
  std::vector<bool> frozen_when_notified;
  std::vector<int> submaps_when_notified;
  std::vector<int> unfinished_when_notified;
  std::vector<std::string> events;
};

struct Harness {
  explicit Harness(const SessionManagerOption& option = TestOption())
      : pose_graph(ManualBackendOption()), manager(pose_graph, option) {
    manager.AddObserver(observer);
  }

  // The production order: the cadence solve, then the manager's one entry.
  void Solve() {
    pose_graph.WaitUntilQuiescent();
    pose_graph.Optimize();
    pose_graph.Enqueue([this] { manager.OnOptimizedOnTask(); });
    pose_graph.WaitUntilQuiescent();
  }

  bool Has(SessionId session, const SubmapId& id) const {
    const std::vector<SubmapId>& ids = pose_graph.graph().session(session).submap_ids;
    return std::find(ids.begin(), ids.end(), id) != ids.end();
  }

  const SessionManager::ExpansionState& state(SessionId id) const {
    return manager.expansion().at(id);
  }

  double FootprintArea(const SubmapId& id) const {
    const double resolution = manager.coverage().resolution();
    return static_cast<double>(
               ComputeCoarseFootprint(pose_graph.graph().submap(id), resolution).size()) *
           resolution * resolution;
  }

  // Closure or not, the constraint only reaches the manager: the graph is not touched.
  void LandOnManager(SessionId session, int submap_index, int node_index,
                     Constraint::Type type = Constraint::Type::INTER_SUBMAP) {
    pose_graph.Enqueue([this, session, submap_index, node_index, type] {
      Constraint constraint;
      constraint.type = type;
      constraint.from = VariableId::Of(SubmapId{session.session_index, submap_index});
      constraint.to = VariableId::Of(NodeId{session.session_index, node_index});
      manager.OnConstraintAddedOnTask(constraint);
    });
  }

  // A closure at the graph's current relative pose, landed and notified in one task.
  void LandClosure(const SubmapId& submap, const NodeId& node) {
    pose_graph.Enqueue([this, submap, node] {
      const PoseGraphData& graph = pose_graph.graph();
      Constraint constraint;
      constraint.type = Constraint::Type::INTER_SUBMAP;
      constraint.from = VariableId::Of(submap);
      constraint.to = VariableId::Of(node);
      constraint.relative_pose =
          graph.submap(submap).global_pose.inverse() * graph.node(node).global_pose;
      constraint.sqrt_information = LoopClosureSqrtInformation(ConstraintWeightOption());
      ASSERT_TRUE(pose_graph.AddConstraintIfEndpointsLive(constraint));
      manager.OnConstraintAddedOnTask(constraint);
    });
  }

  PoseGraph pose_graph;
  SessionManager manager;
  std::shared_ptr<RecordingObserver> observer = std::make_shared<RecordingObserver>();
};

// Submaps of a frozen `session` lying inside what the other frozen sessions cover.
int CountCoveredSubmaps(const Harness& h, SessionId session) {
  const PoseGraphData& graph = h.pose_graph.graph();
  FrozenCoverage coverage(h.manager.coverage().resolution(),
                          h.manager.coverage().track_resolution());
  for (const auto& [id, data] : graph.sessions()) {
    if (data.frozen() && id != session) {
      coverage.AddSession(graph, id);
    }
  }
  int covered = 0;
  for (const SubmapId& id : graph.session(session).submap_ids) {
    if (coverage.Query(graph.submap(id)).covered_fraction() >=
        TestOption().frozen_covered_fraction) {
      ++covered;
    }
  }
  return covered;
}

// Three submaps into the boot session, the first of them finished, and the solve that judges it.
SessionId Bootstrap(Harness& h, Feeder& feeder) {
  feeder.FeedNodes(30);
  h.Solve();
  EXPECT_EQ(h.observer->frozen.size(), 1u) << ToString(h.manager.last_verdict().rejection);
  EXPECT_EQ(h.observer->started.size(), 2u);
  const SessionId second = *h.manager.fed_session();
  feeder.Adopt(second);
  return second;
}

TEST(SessionManagerTest, StartOpensTheFirstSessionAndTellsTheObserver) {
  Harness h;
  // The gate pins the consumer, so "not notified yet" is observable rather than a race.
  std::promise<void> gate;
  std::future<void> gate_open = gate.get_future();
  h.pose_graph.Enqueue([&gate_open] { gate_open.wait(); });
  const SessionId first = h.manager.Start(TestTime(0));
  ASSERT_TRUE(h.manager.fed_session().has_value());
  EXPECT_EQ(*h.manager.fed_session(), first);
  EXPECT_TRUE(h.pose_graph.graph().HasSession(first));
  EXPECT_EQ(h.pose_graph.task_queue().pending_count(), 2u)
      << "notifications arrive on the backend task";
  gate.set_value();

  h.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(h.observer->started.size(), 1u);
  EXPECT_TRUE(h.observer->frozen.empty());
}

TEST(SessionManagerTest, BootstrapFreezesTheFirstSessionAtItsFirstFinishedSubmapAndRotates) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);

  feeder.FeedNodes(15);
  h.Solve();
  EXPECT_TRUE(h.observer->frozen.empty());
  EXPECT_EQ(h.manager.num_judgements(), 0) << "nothing finished, so nothing asked for a verdict";
  ASSERT_EQ(h.manager.expansion().count(first), 1u);
  EXPECT_EQ(h.state(first).phase, Phase::BOOTSTRAP);

  feeder.FeedNodes(15);
  h.pose_graph.WaitUntilQuiescent();
  EXPECT_TRUE(h.observer->frozen.empty()) << "no solve has happened yet, so nothing was judged";
  h.Solve();

  const PoseGraphData& graph = h.pose_graph.graph();
  EXPECT_EQ(h.manager.num_judgements(), 1) << "the finished submap alone earns the judgement";
  ASSERT_EQ(h.observer->frozen.size(), 1u) << ToString(h.manager.last_verdict().rejection);
  EXPECT_EQ(h.observer->frozen.front(), first);
  EXPECT_TRUE(h.observer->frozen_when_notified.front());
  EXPECT_TRUE(h.manager.last_verdict().bootstrap);
  EXPECT_TRUE(graph.session(first).frozen());
  EXPECT_EQ(h.manager.num_sessions_frozen(), 1);
  EXPECT_GT(h.manager.coverage().size(), 0u);
  EXPECT_EQ(h.manager.expansion().count(first), 0u);

  ASSERT_EQ(h.observer->started.size(), 2u);
  const SessionId second = h.observer->started.back();
  EXPECT_EQ(second.session_index, first.session_index + 1);
  // The rotation handed the frozen session's two unfinished actives to the replacement.
  EXPECT_EQ(h.observer->submaps_when_notified.back(), 2);
  EXPECT_EQ(h.observer->unfinished_when_notified.back(), 2);
  ASSERT_TRUE(h.manager.fed_session().has_value());
  EXPECT_EQ(*h.manager.fed_session(), second);
  EXPECT_FALSE(graph.session(second).frozen());
  ASSERT_EQ(graph.session(first).submap_ids.size(), 1u);
  EXPECT_TRUE(graph.submap(graph.session(first).submap_ids.front()).submap->finished());

  const std::vector<std::string> expected{"started 0", "frozen 0", "started 1"};
  EXPECT_EQ(h.observer->events, expected);

  // The replacement inherits the alignment the frozen session ended with.
  const std::optional<Eigen::Affine2d> alignment = graph.ComputeSessionToGlobal(second);
  ASSERT_TRUE(alignment.has_value());
  EXPECT_NEAR(
      (graph.session(second).local_to_global.translation() - alignment->translation()).norm(), 0.0,
      1e-12);
}

TEST(SessionManagerTest, ASessionWithoutGrowthStaysLocalizingHoweverManySubmapsFinish) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();

  for (int round = 0; round < 10; ++round) {
    feeder.FeedSubmap(kHome);
    h.Solve();
  }

  EXPECT_EQ(h.manager.num_judgements(), judged);
  EXPECT_EQ(h.observer->frozen.size(), 1u);
  EXPECT_EQ(*h.manager.fed_session(), second);
  ASSERT_EQ(h.manager.expansion().count(second), 1u);
  const SessionManager::ExpansionState& state = h.state(second);
  EXPECT_TRUE(state.anchored) << "the handed-over submaps tie it to the frozen nodes";
  EXPECT_EQ(state.phase, Phase::LOCALIZING);
  EXPECT_DOUBLE_EQ(state.novel_area, 0.0);
  EXPECT_EQ(state.submaps_without_growth, 0) << "the count only runs from NO_GROWTH";
  EXPECT_GE(state.accounted_submap_index, 8);
}

TEST(SessionManagerTest, IntraEdgesAloneNeitherDirtyNorJudge) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  feeder.FeedNodes(kNodesPerSubmap + 5);
  h.Solve();
  const int judged = h.manager.num_judgements();
  const SessionManager::ExpansionState before = h.state(second);
  ASSERT_FALSE(before.dirty);

  // Four more nodes into the open submaps: INTRA edges and nothing else.
  feeder.FeedNodes(4);
  h.pose_graph.WaitUntilQuiescent();
  EXPECT_FALSE(h.state(second).dirty);
  h.Solve();
  EXPECT_EQ(h.manager.num_judgements(), judged);
  EXPECT_EQ(h.state(second).accounted_submap_index, before.accounted_submap_index);
}

// The area account is read from the solved graph at the entry: a closure landing changes
// nothing until then, and a solve without events re-judges nothing.
TEST(SessionManagerTest, AClosureOnlyMarksTheSessionDirtyUntilTheEntryRuns) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  feeder.FeedNodes(kNodesPerSubmap);
  h.Solve();
  const int judged = h.manager.num_judgements();
  const SessionManager::ExpansionState before = h.state(second);
  ASSERT_FALSE(before.dirty);
  ASSERT_TRUE(before.anchored);

  h.LandOnManager(second, 2, 0);
  h.pose_graph.WaitUntilQuiescent();
  const SessionManager::ExpansionState& state = h.state(second);
  EXPECT_TRUE(state.dirty);
  EXPECT_EQ(state.accounted_submap_index, before.accounted_submap_index);
  EXPECT_EQ(state.phase, before.phase);
  EXPECT_DOUBLE_EQ(state.novel_area, before.novel_area);
  EXPECT_EQ(h.manager.num_judgements(), judged);

  h.Solve();
  EXPECT_FALSE(state.dirty);
  EXPECT_EQ(state.phase, Phase::LOCALIZING);
  EXPECT_EQ(h.manager.num_judgements(), judged) << "refreshed, still localizing, not judged";
}

// Growth, then a flat submap opens the count, then the count runs out: the gate is shut here so
// the JUDGING session stays and the closure-only re-judge is observable.
TEST(SessionManagerTest, GrowthThenFlatSubmapsReachJudgingAndAClosureReJudges) {
  Harness h(ShutGateOption());
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();

  // A submap's ring is fixed when it is minted, and it finishes two submaps later.
  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kHome);
  feeder.FeedSubmap(kHome);
  h.Solve();
  // Finished: 0 and 1 in known area, 2 over the away ring.
  const SessionManager::ExpansionState& state = h.state(second);
  EXPECT_EQ(state.phase, Phase::EXPANDING);
  EXPECT_GT(state.novel_area, 5.0);
  EXPECT_DOUBLE_EQ(state.last_area, state.novel_area);
  EXPECT_EQ(state.accounted_submap_index, 2);
  EXPECT_EQ(h.manager.num_judgements(), judged) << "growing, not judged";

  // 3 finishes in known area: the union stands still, so the count opens.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_DOUBLE_EQ(state.baseline_area, state.novel_area);
  EXPECT_EQ(state.submaps_without_growth, 0);
  EXPECT_EQ(h.manager.num_judgements(), judged);

  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_EQ(state.submaps_without_growth, 1);
  EXPECT_EQ(h.manager.num_judgements(), judged);

  // The submap that fills the count is the event that asks for the verdict.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::JUDGING);
  EXPECT_EQ(state.submaps_without_growth, 2);
  EXPECT_EQ(state.accounted_submap_index, 5);
  EXPECT_EQ(h.manager.num_judgements(), judged + 1);
  EXPECT_EQ(h.manager.last_verdict().rejection, FreezeRejection::COVARIANCE_TOO_LARGE);
  EXPECT_GT(h.manager.last_verdict().frozen_links.at(SubmapId{second.session_index, 0}), 0);
  EXPECT_EQ(h.observer->frozen.size(), 1u);

  h.Solve();
  EXPECT_EQ(h.manager.num_judgements(), judged + 1) << "nothing landed, nothing to re-judge";

  // A closure lands: it may have tightened what the judge refused, and it is not a submap.
  h.LandOnManager(second, 4, 0);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::JUDGING);
  EXPECT_EQ(state.submaps_without_growth, 2) << "a closure-only entry advances no count";
  EXPECT_EQ(state.accounted_submap_index, 5);
  EXPECT_EQ(h.manager.num_judgements(), judged + 2);

  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::JUDGING);
  EXPECT_EQ(state.submaps_without_growth, 3);
  EXPECT_EQ(h.manager.num_judgements(), judged + 3);
}

// The same walk with the gate open: JUDGING is the covariance gate, and the anchored fed session
// passes it, freezes and rotates.
TEST(SessionManagerTest, AJudgingSessionThatPassesTheGateFreezesAndRotates) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();

  feeder.FeedSubmap(kAway);
  for (int round = 0; round < 4; ++round) {
    feeder.FeedSubmap(kHome);
    h.Solve();
  }
  ASSERT_EQ(h.state(second).phase, Phase::NO_GROWTH);
  ASSERT_EQ(h.state(second).submaps_without_growth, 1);
  ASSERT_EQ(h.manager.num_judgements(), judged);
  ASSERT_EQ(h.observer->frozen.size(), 1u);

  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(h.manager.num_judgements(), judged + 1);
  EXPECT_TRUE(h.manager.last_verdict().eligible) << ToString(h.manager.last_verdict().rejection);
  EXPECT_FALSE(h.manager.last_verdict().bootstrap);
  ASSERT_EQ(h.observer->frozen.size(), 2u);
  EXPECT_EQ(h.observer->frozen.back(), second);
  EXPECT_TRUE(h.pose_graph.graph().session(second).frozen());
  EXPECT_EQ(h.manager.num_sessions_frozen(), 2);
  ASSERT_EQ(h.observer->started.size(), 3u);
  EXPECT_EQ(*h.manager.fed_session(), h.observer->started.back());
  EXPECT_EQ(h.manager.expansion().count(second), 0u);
}

// The baseline is fixed when NO_GROWTH is entered: a rise above it by more than the tolerance,
// from NO_GROWTH or from JUDGING, is growth again, and the next flat submap restarts the count.
TEST(SessionManagerTest, ARiseAboveTheBaselineReturnsToExpandingAndRestartsTheCount) {
  Harness h(ShutGateOption());
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();

  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kHome);
  feeder.FeedSubmap(kHome);
  h.Solve();
  const SessionManager::ExpansionState& state = h.state(second);
  ASSERT_EQ(state.phase, Phase::EXPANDING);
  const double one_ring = state.novel_area;

  // 5 is minted over a second fresh ring; 3 and 4 finish in known area first.
  feeder.FeedSubmap(kFar);
  h.Solve();
  ASSERT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_NEAR(state.baseline_area, one_ring, 0.1 * one_ring);
  feeder.FeedSubmap(kHome);
  h.Solve();
  ASSERT_EQ(state.phase, Phase::NO_GROWTH);
  ASSERT_EQ(state.submaps_without_growth, 1);

  // 5 finishes: the union gains a ring, well past baseline + tolerance.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::EXPANDING);
  EXPECT_GT(state.novel_area, 1.8 * one_ring);
  EXPECT_DOUBLE_EQ(state.last_area, state.novel_area);
  EXPECT_EQ(h.manager.num_judgements(), judged);

  // 6 finishes flat: a new baseline, the count restarted.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_DOUBLE_EQ(state.baseline_area, state.novel_area);
  EXPECT_EQ(state.submaps_without_growth, 0);
  const double two_rings = state.baseline_area;

  // 10 is minted over a third ring; 7 and 8 finish flat first and fill the count.
  feeder.FeedSubmap(kHome);
  h.Solve();
  feeder.FeedSubmap(kFarther);
  h.Solve();
  ASSERT_EQ(state.phase, Phase::JUDGING);
  EXPECT_EQ(state.submaps_without_growth, 2);
  EXPECT_EQ(h.manager.num_judgements(), judged + 1);
  feeder.FeedSubmap(kHome);
  h.Solve();
  ASSERT_EQ(state.phase, Phase::JUDGING);
  EXPECT_EQ(state.submaps_without_growth, 3);
  EXPECT_EQ(h.manager.num_judgements(), judged + 2);

  // 10 finishes over the third ring: out of JUDGING without a verdict.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::EXPANDING);
  EXPECT_GT(state.novel_area, two_rings + TestOption().growth_tolerance);
  EXPECT_EQ(h.manager.num_judgements(), judged + 2) << "growing again, not judged";

  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_EQ(state.submaps_without_growth, 0) << "the count restarts";
  EXPECT_DOUBLE_EQ(state.baseline_area, state.novel_area);
  EXPECT_GT(state.baseline_area, two_rings);
}

// Growth is the union's own difference: a second pass over the session's own fresh ring adds
// nothing, so it is the flat submap that opens the count, not growth against the frozen layer.
TEST(SessionManagerTest, ReturningToItsOwnFreshAreaIsNotGrowth) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);

  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kHome);
  h.Solve();
  // Finished: 0 and 1 in known area, 2 over the away ring; 3, the second pass, is still open.
  const SessionManager::ExpansionState& state = h.state(second);
  ASSERT_EQ(state.accounted_submap_index, 2);
  ASSERT_EQ(state.phase, Phase::EXPANDING);
  const double one_ring = h.FootprintArea(SubmapId{second.session_index, 2});
  EXPECT_GE(state.novel_area, one_ring);

  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.accounted_submap_index, 3);
  EXPECT_EQ(state.phase, Phase::NO_GROWTH) << "two passes over one ring are one ring";
  EXPECT_LT(state.novel_area, 1.2 * one_ring);
  EXPECT_DOUBLE_EQ(state.baseline_area, state.novel_area);
}

// Growth is area AND track. A robot standing still while fresh rings land around it grows the
// map but not the track: the machine reads that as no growth, and rings past the baseline do
// not reset the count.
TEST(SessionManagerTest, AreaWithoutTrackIsNotGrowth) {
  Harness h(ShutGateOption());
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();
  feeder.set_step(0.0);

  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kFar);
  feeder.FeedSubmap(kHome);
  h.Solve();
  // Finished: 0 and 1 in known area, 2 over the away ring; the entry is area alone.
  const SessionManager::ExpansionState& state = h.state(second);
  ASSERT_EQ(state.phase, Phase::EXPANDING);
  const double one_ring = state.novel_area;
  const double one_cell = std::pow(h.manager.coverage().track_resolution(), 2);
  EXPECT_LE(state.novel_track, one_cell) << "every node stands in the one cell past the frozen";
  EXPECT_DOUBLE_EQ(state.last_track, state.novel_track);

  // 3 finishes over the far ring: a second ring of area, not a step of track.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_GT(state.novel_area, 1.8 * one_ring);
  EXPECT_DOUBLE_EQ(state.baseline_area, state.novel_area);
  EXPECT_DOUBLE_EQ(state.baseline_track, state.novel_track);
  EXPECT_EQ(state.submaps_without_growth, 0);

  // 6 is minted over a third ring; 4 and 5 finish flat and fill the count.
  feeder.FeedSubmap(kFarther);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::NO_GROWTH);
  EXPECT_EQ(state.submaps_without_growth, 1);
  feeder.FeedSubmap(kHome);
  h.Solve();
  ASSERT_EQ(state.phase, Phase::JUDGING);
  EXPECT_EQ(state.submaps_without_growth, 2);
  EXPECT_EQ(h.manager.num_judgements(), judged + 1);

  // 6 finishes over the third ring: area past the baseline, track still, no way out of JUDGING.
  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.phase, Phase::JUDGING) << "a ring of area past the baseline resets nothing";
  EXPECT_GT(state.novel_area, state.baseline_area + TestOption().growth_tolerance);
  EXPECT_LE(state.novel_track, one_cell);
  EXPECT_EQ(state.submaps_without_growth, 3);
  EXPECT_EQ(h.manager.num_judgements(), judged + 2);
}

// The entry threshold is a fraction of the frozen area when that is the smaller: one frozen
// ring makes one fresh ring worth expanding, under the shipped 200 m^2.
TEST(SessionManagerTest, AFractionOfASmallFrozenAreaOpensExpanding) {
  SessionManagerOption option = TestOption();
  option.min_new_area = SessionManagerOption().min_new_area;
  option.min_new_area_fraction = 0.5;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const double frozen_area = h.manager.coverage().area();
  ASSERT_GT(frozen_area, 5.0);
  ASSERT_LT(option.min_new_area_fraction * frozen_area, option.min_new_area);

  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kHome);
  feeder.FeedSubmap(kHome);
  h.Solve();
  const SessionManager::ExpansionState& state = h.state(second);
  EXPECT_LT(state.novel_area, option.min_new_area) << "below the absolute threshold";
  EXPECT_GE(state.novel_area, option.min_new_area_fraction * frozen_area);
  EXPECT_EQ(state.phase, Phase::EXPANDING);
}

// The owner's scenario: a robot circling inside one small novel patch. Summed per submap the
// account would grow without bound; recomputed from the union it stays the patch, below the
// shipped threshold, and the session localizes.
TEST(SessionManagerTest, LoopingInsideASmallNovelPatchKeepsTheNovelAreaFlat) {
  SessionManagerOption option = TestOption();
  option.min_new_area = SessionManagerOption().min_new_area;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();

  feeder.set_ring_center(kAway);
  feeder.FeedNodes(3 * kNodesPerSubmap);
  h.Solve();
  const SessionManager::ExpansionState& state = h.state(second);
  const double first_pass = state.novel_area;
  ASSERT_GT(first_pass, 5.0);
  ASSERT_LT(first_pass, option.min_new_area) << "the patch has to be below the threshold";

  // Each lap wanders a little, so the union genuinely grows by a rim and not just by repetition.
  const int rounds = 8;
  for (int round = 0; round < rounds; ++round) {
    feeder.FeedSubmap(kAway + 0.02 * (round + 1) * Eigen::Vector2d::Ones());
    h.Solve();
    EXPECT_NEAR(state.novel_area, first_pass, 0.1 * first_pass) << "round " << round;
  }
  EXPECT_GT(state.novel_area, first_pass) << "the rim is there";
  EXPECT_LT(state.novel_area, 2.0 * first_pass);
  EXPECT_EQ(state.phase, Phase::LOCALIZING);
  EXPECT_EQ(h.manager.num_judgements(), judged);
  EXPECT_EQ(*h.manager.fed_session(), second);
}

// The regular trimmer deletes submaps behind the robot, covered ones included: the count is a
// watermark, so a trim shrinks neither it nor the phase, while the area follows what still
// holds it.
TEST(SessionManagerTest, TrimmedSubmapsNeitherShrinkTheCountNorLeaveJudging) {
  Harness h(ShutGateOption());
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId second = Bootstrap(h, feeder);
  const int judged = h.manager.num_judgements();

  feeder.FeedSubmap(kAway);
  feeder.FeedSubmap(kHome);
  feeder.FeedSubmap(kHome);
  h.Solve();
  for (int round = 0; round < 3; ++round) {
    feeder.FeedSubmap(kHome);
    h.Solve();
  }
  // Finished: 0 and 1 in known area, 2 the growth, 3 to 5 back in known area.
  const SessionManager::ExpansionState& state = h.state(second);
  ASSERT_EQ(state.accounted_submap_index, 5);
  ASSERT_EQ(state.phase, Phase::JUDGING);
  ASSERT_EQ(state.submaps_without_growth, 2);
  ASSERT_GT(state.novel_area, 5.0);
  ASSERT_EQ(h.manager.num_judgements(), judged + 1) << "judged, and refused by the shut gate";

  const std::vector<SubmapId> trimmed{SubmapId{second.session_index, 2},
                                      SubmapId{second.session_index, 3}};
  const int trimmed_before = h.pose_graph.trimmer().num_submaps_trimmed();
  h.pose_graph.Enqueue([&h, trimmed] { h.pose_graph.TrimSubmapsOnTask(trimmed); });
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(h.pose_graph.trimmer().num_submaps_trimmed(), trimmed_before + 2);
  ASSERT_FALSE(h.Has(second, trimmed.front()));
  ASSERT_FALSE(h.Has(second, trimmed.back()));

  feeder.FeedSubmap(kHome);
  h.Solve();
  EXPECT_EQ(state.accounted_submap_index, 6);
  EXPECT_EQ(state.submaps_without_growth, 3) << "one more, not recounted over the survivors";
  EXPECT_EQ(state.phase, Phase::JUDGING);
  EXPECT_DOUBLE_EQ(state.novel_area, 0.0) << "nothing left holds the ring";
  EXPECT_EQ(h.manager.num_judgements(), judged + 2);
}

// The boot session's growth before anything is frozen is accounted the moment a freeze anchors
// it: here the floating datum holder freezes in place under the closure that landed on it, and
// the next entry moves the fed session out of BOOTSTRAP and reads its finished submap against
// the new frozen layer.
TEST(SessionManagerTest, GrowthFinishedBeforeTheBootstrapFreezeIsAccountedOnceTheSessionAnchors) {
  Harness h;
  const SessionId floating = h.pose_graph.StartNewSession(TestTime(0));
  Feeder floating_feeder(h.pose_graph, nullptr, floating);
  floating_feeder.FeedNodes(30);
  floating_feeder.FinishAll();
  h.pose_graph.WaitUntilQuiescent();

  const SessionId fed = h.manager.Start(TestTime(30));
  Feeder fed_feeder(h.pose_graph, &h.manager, fed);
  fed_feeder.set_ring_center(kAway);
  // Submap 0 finishes here; no solve yet, or the boot session would bootstrap-freeze itself.
  fed_feeder.FeedNodes(30);
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(h.manager.expansion().count(fed), 1u);
  EXPECT_TRUE(h.state(fed).dirty);
  EXPECT_EQ(h.state(fed).phase, Phase::BOOTSTRAP);
  EXPECT_FALSE(h.state(fed).anchored);
  EXPECT_DOUBLE_EQ(h.state(fed).novel_area, 0.0) << "nothing is read before the entry";

  h.LandClosure(SubmapId{floating.session_index, 0}, NodeId{fed.session_index, 0});
  h.Solve();
  ASSERT_EQ(h.observer->frozen.size(), 1u) << ToString(h.manager.last_verdict().rejection);
  EXPECT_EQ(h.observer->frozen.front(), floating);
  EXPECT_EQ(*h.manager.fed_session(), fed);
  const SessionManager::ExpansionState& state = h.state(fed);
  EXPECT_TRUE(state.dirty) << "the neighbour's freeze re-dirties it";
  EXPECT_EQ(state.phase, Phase::BOOTSTRAP) << "the datum holder's freeze ends the entry";
  EXPECT_FALSE(state.anchored) << "unanchored global poses are not read until the next entry";

  h.Solve();
  EXPECT_TRUE(state.anchored);
  EXPECT_EQ(state.accounted_submap_index, 0);
  EXPECT_GT(state.novel_area, 5.0);
  EXPECT_EQ(state.phase, Phase::EXPANDING) << "BOOTSTRAP -> LOCALIZING -> EXPANDING in one entry";
  EXPECT_DOUBLE_EQ(state.last_area, state.novel_area);
  EXPECT_FALSE(state.dirty);
}

// A floating session tied only to the fed one is unanchored until the fed one freezes: no
// account and no judgement before. After, it skips the growth phases: nothing feeds it, so its
// area is final and it is judged at the first entry that finds it worth freezing.
TEST(SessionManagerTest, AFloatingSessionLinkedToTheFedOneIsJudgedOnceTheFedOneFreezes) {
  Harness h;
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  const SessionId fed = Bootstrap(h, feeder);

  const SessionId floating = h.pose_graph.StartNewSession(TestTime(100));
  Feeder floating_feeder(h.pose_graph, nullptr, floating);
  floating_feeder.set_ring_center(kAway);
  floating_feeder.FeedNodes(3 * kNodesPerSubmap);
  floating_feeder.set_ring_center(kHome);
  floating_feeder.FeedNodes(2 * kNodesPerSubmap);
  floating_feeder.FinishAll();
  h.pose_graph.WaitUntilQuiescent();

  // Closures from the fed session's head submap onto both ends of the floater.
  for (const int node_index : {0, 5 * kNodesPerSubmap - 1}) {
    h.LandClosure(SubmapId{fed.session_index, 0}, NodeId{floating.session_index, node_index});
  }
  h.pose_graph.WaitUntilQuiescent();
  const int judged = h.manager.num_judgements();
  ASSERT_EQ(h.manager.expansion().count(floating), 1u);
  EXPECT_TRUE(h.state(floating).dirty);
  EXPECT_EQ(h.state(floating).phase, Phase::BOOTSTRAP);
  EXPECT_EQ(h.state(floating).accounted_submap_index, -1);
  h.Solve();
  const SessionManager::ExpansionState& state = h.state(floating);
  EXPECT_FALSE(state.anchored) << "the fed session is not truth";
  EXPECT_EQ(state.phase, Phase::LOCALIZING) << "out of BOOTSTRAP, but unanchored: not stepped";
  EXPECT_EQ(state.accounted_submap_index, -1) << "unanchored: not accounted";
  EXPECT_EQ(h.manager.num_judgements(), judged);

  feeder.FeedNodes(kNodesPerSubmap);
  h.pose_graph.WaitUntilQuiescent();
  h.manager.FreezeFedSession();
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(h.manager.num_sessions_frozen(), 2);
  EXPECT_EQ(h.manager.num_judgements(), judged) << "the manual freeze asks no verdict";
  EXPECT_TRUE(state.dirty);
  EXPECT_FALSE(state.anchored) << "the freeze only marks; the entry reads";

  // A floater gets no more submaps, so its area is final: worth freezing means judged.
  const SessionId fed_after_rotation = *h.manager.fed_session();
  h.Solve();
  EXPECT_EQ(h.manager.num_judgements(), judged + 1);
  EXPECT_EQ(h.manager.num_sessions_frozen(), 3) << ToString(h.manager.last_verdict().rejection);
  EXPECT_TRUE(h.pose_graph.graph().session(floating).frozen());
  EXPECT_EQ(*h.manager.fed_session(), fed_after_rotation) << "a floating freeze does not rotate";
}

// What a boot finds on disk: a frozen layer and unfrozen sessions whose account is rebuilt from
// the graph at the first entry.
TEST(SessionManagerTest, AfterLoadUnfrozenSessionsAreDirtyAndRefreshAtTheFirstEntry) {
  Harness h;
  const SessionId frozen = h.pose_graph.StartNewSession(TestTime(0));
  Feeder frozen_feeder(h.pose_graph, nullptr, frozen);
  frozen_feeder.FeedNodes(30);
  frozen_feeder.FinishAll();
  h.pose_graph.WaitUntilQuiescent();
  const SessionId other = h.pose_graph.StartNewSession(TestTime(30));
  Feeder other_feeder(h.pose_graph, nullptr, other);
  other_feeder.set_ring_center(kAway);
  other_feeder.FeedNodes(30);
  other_feeder.FinishAll();
  h.LandClosure(SubmapId{frozen.session_index, 0}, NodeId{other.session_index, 0});
  h.pose_graph.FreezeSession(frozen);
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_TRUE(h.pose_graph.graph().session(frozen).frozen());

  const SessionId fed = h.manager.Start(TestTime(60));
  h.pose_graph.WaitUntilQuiescent();
  EXPECT_GT(h.manager.coverage().size(), 0u);
  EXPECT_EQ(h.manager.expansion().count(frozen), 0u);
  ASSERT_EQ(h.manager.expansion().count(other), 1u);
  ASSERT_EQ(h.manager.expansion().count(fed), 1u);
  const SessionManager::ExpansionState& state = h.state(other);
  EXPECT_TRUE(state.dirty);
  EXPECT_TRUE(h.state(fed).dirty);
  EXPECT_EQ(state.phase, Phase::BOOTSTRAP) << "phase is not persisted";
  EXPECT_FALSE(state.anchored);
  EXPECT_EQ(state.accounted_submap_index, -1);
  EXPECT_DOUBLE_EQ(state.novel_area, 0.0);

  // Anchored and floating with fresh area on the first refresh: judged, and frozen in place.
  h.Solve();
  EXPECT_EQ(h.manager.num_judgements(), 1);
  EXPECT_TRUE(h.pose_graph.graph().session(other).frozen());
  EXPECT_EQ(h.manager.expansion().count(other), 0u);
  EXPECT_EQ(*h.manager.fed_session(), fed);
  // The freeze ends the entry; the fed session's turn comes at the next one.
  EXPECT_EQ(h.state(fed).phase, Phase::BOOTSTRAP);
  h.Solve();
  EXPECT_EQ(h.state(fed).phase, Phase::LOCALIZING);
  EXPECT_FALSE(h.state(fed).anchored) << "an empty fed session has nothing to anchor with";
}

TEST(SessionManagerTest, AutoFreezeOffNeverAccountsJudgesOrFreezes) {
  SessionManagerOption option = TestOption();
  option.auto_freeze = false;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  feeder.FeedNodes(30);
  h.Solve();
  h.Solve();
  h.LandOnManager(first, 2, 0);
  h.Solve();
  EXPECT_TRUE(h.manager.expansion().empty());
  EXPECT_EQ(h.manager.num_judgements(), 0);
  EXPECT_TRUE(h.observer->frozen.empty());
  EXPECT_EQ(*h.manager.fed_session(), first);
}

TEST(SessionManagerTest, FreezeFedSessionRunsTheSequenceWithoutAVerdict) {
  SessionManagerOption option = TestOption();
  option.auto_freeze = false;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);

  feeder.FeedNodes(15);
  h.pose_graph.WaitUntilQuiescent();
  h.manager.FreezeFedSession();
  h.pose_graph.WaitUntilQuiescent();
  EXPECT_TRUE(h.observer->frozen.empty()) << "nothing finished, nothing to freeze";

  feeder.FeedNodes(15);
  h.pose_graph.WaitUntilQuiescent();
  // The gate pins the consumer, so the sequence behind it is provably queued and not inline.
  std::promise<void> gate;
  std::future<void> gate_open = gate.get_future();
  h.pose_graph.Enqueue([&gate_open] { gate_open.wait(); });
  h.manager.FreezeFedSession();
  EXPECT_EQ(h.pose_graph.task_queue().pending_count(), 2u)
      << "the graph is not read on this thread";
  gate.set_value();
  h.pose_graph.WaitUntilQuiescent();

  EXPECT_EQ(h.manager.num_judgements(), 0);
  ASSERT_EQ(h.observer->frozen.size(), 1u);
  EXPECT_EQ(h.observer->frozen.front(), first);
  ASSERT_EQ(h.observer->started.size(), 2u);
  EXPECT_EQ(h.observer->submaps_when_notified.back(), 2);
  EXPECT_EQ(*h.manager.fed_session(), h.observer->started.back());
  EXPECT_EQ(h.manager.num_sessions_frozen(), 1);
}

// The agent plans against one fed session and applies later: a freeze meant for a session that
// has rotated away since must not land on its successor.
TEST(SessionManagerTest, FreezeFedSessionForASessionNoLongerFedDoesNothing) {
  SessionManagerOption option = TestOption();
  option.auto_freeze = false;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  feeder.FeedNodes(30);
  h.pose_graph.WaitUntilQuiescent();

  h.manager.FreezeFedSession(SessionId{first.session_index + 5});
  h.pose_graph.WaitUntilQuiescent();
  EXPECT_TRUE(h.observer->frozen.empty());
  EXPECT_EQ(*h.manager.fed_session(), first);

  h.manager.FreezeFedSession(first);
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(h.observer->frozen.size(), 1u);
  EXPECT_EQ(h.observer->frozen.front(), first);
  const SessionId successor = *h.manager.fed_session();
  ASSERT_NE(successor, first);

  h.manager.FreezeFedSession(first);
  h.pose_graph.WaitUntilQuiescent();
  EXPECT_EQ(h.observer->frozen.size(), 1u) << "the stale request froze the successor";
  EXPECT_EQ(*h.manager.fed_session(), successor);
}

// A floating session is not fed, so only a constraint landing on it can get it judged; in the
// bootstrap it carries the merged component's datum and freezes in place.
TEST(SessionManagerTest, AFloatingSessionIsFrozenInPlaceWhenAConstraintLandsOnIt) {
  Harness h;
  const SessionId floating = h.pose_graph.StartNewSession(TestTime(0));
  Feeder floating_feeder(h.pose_graph, nullptr, floating);
  floating_feeder.FeedNodes(30);
  floating_feeder.FinishAll();
  h.pose_graph.WaitUntilQuiescent();

  const SessionId fed = h.manager.Start(TestTime(30));
  Feeder fed_feeder(h.pose_graph, &h.manager, fed);
  fed_feeder.FeedNodes(5);
  h.Solve();
  EXPECT_TRUE(h.observer->frozen.empty());
  EXPECT_EQ(h.manager.num_judgements(), 0) << "INTRA edges alone never ask for a verdict";

  // Landed and notified in one task; a closure solves nothing, the cadence solve judges it.
  h.LandClosure(SubmapId{floating.session_index, 0}, NodeId{fed.session_index, 0});
  h.Solve();

  ASSERT_EQ(h.observer->frozen.size(), 1u) << ToString(h.manager.last_verdict().rejection);
  EXPECT_EQ(h.observer->frozen.front(), floating);
  EXPECT_TRUE(h.manager.last_verdict().bootstrap);
  EXPECT_TRUE(h.pose_graph.graph().session(floating).frozen());
  EXPECT_EQ(h.manager.num_sessions_frozen(), 1);
  EXPECT_EQ(*h.manager.fed_session(), fed);
  EXPECT_EQ(h.observer->started.size(), 1u) << "a floating freeze opens no replacement";
  EXPECT_FALSE(h.pose_graph.graph().session(fed).frozen());
}

// Finished submaps lying inside frozen area are trimmed on the way into the freeze; the one that
// grew the map stays, and the last finished one stays whatever it covers.
TEST(SessionManagerTest, TheOverlapTrimDropsCoveredSubmapsBeforeTheFreezeAndKeepsOne) {
  SessionManagerOption option = TestOption();
  option.auto_freeze = false;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  feeder.FeedNodes(30);
  h.pose_graph.WaitUntilQuiescent();
  h.manager.FreezeFedSession();
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(h.manager.num_sessions_frozen(), 1);
  const SessionId second = *h.manager.fed_session();
  feeder.Adopt(second);

  feeder.FeedNodes(kNodesPerSubmap);
  feeder.set_ring_center(kAway);
  feeder.FeedNodes(kNodesPerSubmap);
  feeder.set_ring_center(kHome);
  feeder.FeedNodes(2 * kNodesPerSubmap);
  h.pose_graph.WaitUntilQuiescent();
  const int trimmed_before = h.pose_graph.trimmer().num_submaps_trimmed();
  // Finished by now: 0, 1, 2 inside frozen area, 3 the excursion; 4 and 5 still open.
  const SubmapId excursion{second.session_index, 3};
  ASSERT_TRUE(h.pose_graph.graph().submap(excursion).submap->finished());

  h.manager.FreezeFedSession();
  h.pose_graph.WaitUntilQuiescent();

  ASSERT_EQ(h.manager.num_sessions_frozen(), 2);
  EXPECT_TRUE(h.pose_graph.graph().session(second).frozen());
  EXPECT_TRUE(h.Has(second, excursion));
  // At most the handover's junction submap may still lie in known area: as the sole holder of
  // frozen nodes it is the trimmer's constant guard's to keep.
  EXPECT_LE(CountCoveredSubmaps(h, second), 1);
  EXPECT_GE(h.pose_graph.trimmer().num_submaps_trimmed(), trimmed_before + 2);
  EXPECT_EQ(h.observer->submaps_when_notified.back(), 2) << "the open ones were handed over";

  // A third session entirely inside known area keeps its last finished submap whatever it
  // covers, plus at most the junction submap.
  const SessionId third = *h.manager.fed_session();
  feeder.Adopt(third);
  feeder.FeedNodes(3 * kNodesPerSubmap);
  h.pose_graph.WaitUntilQuiescent();
  h.manager.FreezeFedSession();
  h.pose_graph.WaitUntilQuiescent();
  ASSERT_EQ(h.manager.num_sessions_frozen(), 3);
  const std::vector<SubmapId>& kept = h.pose_graph.graph().session(third).submap_ids;
  ASSERT_FALSE(kept.empty());
  EXPECT_EQ(kept.back(), (SubmapId{third.session_index, 2}));
  EXPECT_LE(kept.size(), 2u);
  EXPECT_GE(h.pose_graph.trimmer().num_submaps_trimmed(), trimmed_before + 3);
}

TEST(SessionManagerTest, SessionIdsStayMonotoneAcrossRepeatedFreezes) {
  SessionManagerOption option = TestOption();
  option.auto_freeze = false;
  Harness h(option);
  const SessionId first = h.manager.Start(TestTime(0));
  Feeder feeder(h.pose_graph, &h.manager, first);
  for (int round = 0; round < 3; ++round) {
    feeder.Adopt(*h.manager.fed_session());
    feeder.FeedNodes(30);
    h.pose_graph.WaitUntilQuiescent();
    h.manager.FreezeFedSession();
    h.pose_graph.WaitUntilQuiescent();
  }

  ASSERT_EQ(h.observer->frozen.size(), 3u);
  ASSERT_EQ(h.observer->started.size(), 4u);
  for (size_t i = 0; i < h.observer->started.size(); ++i) {
    EXPECT_EQ(h.observer->started[i].session_index, static_cast<int>(i));
  }
  for (size_t i = 0; i < h.observer->frozen.size(); ++i) {
    EXPECT_EQ(h.observer->frozen[i].session_index, static_cast<int>(i));
    EXPECT_TRUE(h.pose_graph.graph().session(h.observer->frozen[i]).frozen());
  }
  EXPECT_EQ(h.manager.num_sessions_frozen(), 3);
}

TEST(SessionManagerTest, OptionRoundTripsThroughYaml) {
  const YAML::Node node = YAML::Load(
      "auto_freeze: false\nmin_new_area: 42.5\ngrowth_tolerance: 3.5\n"
      "submaps_without_growth: 4\ncoverage_resolution: 0.25\nfrozen_covered_fraction: 0.8\n"
      "freeze_judge:\n  max_rotation_stddev: 0.03\n");
  const SessionManagerOption option = LoadSessionManagerOption(node);
  EXPECT_FALSE(option.auto_freeze);
  EXPECT_DOUBLE_EQ(option.min_new_area, 42.5);
  EXPECT_DOUBLE_EQ(option.growth_tolerance, 3.5);
  EXPECT_EQ(option.submaps_without_growth, 4);
  EXPECT_DOUBLE_EQ(option.coverage_resolution, 0.25);
  EXPECT_DOUBLE_EQ(option.frozen_covered_fraction, 0.8);
  EXPECT_DOUBLE_EQ(option.freeze_judge.max_rotation_stddev, 0.03);
  EXPECT_DOUBLE_EQ(option.freeze_judge.max_translation_stddev,
                   FreezeJudgeOption().max_translation_stddev);

  const SessionManagerOption defaults = LoadSessionManagerOption(YAML::Load("{}"));
  EXPECT_TRUE(defaults.auto_freeze);
  EXPECT_DOUBLE_EQ(defaults.growth_tolerance, SessionManagerOption().growth_tolerance);
  EXPECT_EQ(defaults.submaps_without_growth, SessionManagerOption().submaps_without_growth);
}

}  // namespace
}  // namespace evergreenslam::lifelong

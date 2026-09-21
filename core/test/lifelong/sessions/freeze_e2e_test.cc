/**
 * @file freeze_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Freeze timing through the real wiring on a two-room world: the bootstrap, the excursion
 *        that settles inside the wing, freezes there and sheds its pass through known area, the
 *        localizing session that never freezes, and the manual freeze with auto_freeze off.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "../testing/loop_scenario.h"
#include "common/time.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "lifelong/sessions/frozen_coverage.h"
#include "lifelong/sessions/frozen_links.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using testing::Frontend;
using testing::kLoopRadius;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::LoopTruth;
using testing::TestTime;
using testing::World;

constexpr double kStep = 2.0 * M_PI * kLoopRadius / kNodesPerLap;
constexpr double kYawBiasPerStep = 0.002;
constexpr double kWingMinX = 13.207;
// Room A is in range from its first quarter lap; wing B, taller and past a narrow doorway, has
// no wall in range from anywhere in A. So A's submaps are what a frozen A covers, B's are novel.
constexpr double kRange = 11.0;

const World& TwoRooms() {
  static const World world{
      0.013,
      27.017,
      -4.011,
      14.009,
      {Eigen::AlignedBox2d(Eigen::Vector2d(1.51, 1.52), Eigen::Vector2d(2.32, 2.33)),
       Eigen::AlignedBox2d(Eigen::Vector2d(10.61, 7.52), Eigen::Vector2d(11.42, 8.33)),
       Eigen::AlignedBox2d(Eigen::Vector2d(6.01, 4.62), Eigen::Vector2d(6.82, 5.43)),
       Eigen::AlignedBox2d(Eigen::Vector2d(0.013, -4.011), Eigen::Vector2d(13.007, 0.021)),
       Eigen::AlignedBox2d(Eigen::Vector2d(0.013, 10.003), Eigen::Vector2d(13.007, 14.009)),
       Eigen::AlignedBox2d(Eigen::Vector2d(13.007, -4.011), Eigen::Vector2d(kWingMinX, 5.203)),
       Eigen::AlignedBox2d(Eigen::Vector2d(13.007, 6.797), Eigen::Vector2d(kWingMinX, 14.009))}};
  return world;
}

bool InWing(const Eigen::Affine2d& truth) { return truth.translation().x() > kWingMinX; }

// Arcs and straights sampled at the lap's node spacing; a positive angle turns left.
class PathBuilder {
 public:
  explicit PathBuilder(const Eigen::Affine2d& start) : pose_(start), poses_{start} {}

  PathBuilder& Straight(double length) { return Segment(length, 0.0); }
  PathBuilder& Arc(double angle, double radius) { return Segment(std::abs(angle) * radius, angle); }
  std::vector<Eigen::Affine2d> Take() { return std::move(poses_); }

 private:
  PathBuilder& Segment(double length, double angle) {
    double s = kStep - pending_;
    for (; s <= length + 1e-9; s += kStep) {
      poses_.push_back(Eigen::Affine2d(pose_ * Along(s, length, angle)));
    }
    pending_ = length - (s - kStep);
    pose_ = pose_ * Along(length, length, angle);
    return *this;
  }

  static Eigen::Affine2d Along(double s, double length, double angle) {
    if (angle == 0.0) {
      return transform::FromXYTheta(s, 0.0, 0.0);
    }
    const double theta = angle * s / length;
    const double radius = length / std::abs(angle);
    return transform::FromXYTheta(radius * std::sin(std::abs(theta)),
                                  std::copysign(radius * (1.0 - std::cos(theta)), angle), theta);
  }

  Eigen::Affine2d pose_;
  double pending_ = 0.0;
  std::vector<Eigen::Affine2d> poses_;
};

// A quarter of A's loop, out through the doorway, one and a half turns in B so the wing closes
// on itself, back through the doorway, then A's loop the other way round for `tail_steps`.
std::vector<Eigen::Affine2d> ExcursionTruth(int tail_steps) {
  PathBuilder path(LoopGroundTruth(0));
  path.Arc(M_PI / 2.0, kLoopRadius)
      .Arc(-M_PI / 2.0, 1.0)
      .Straight(7.5)
      .Arc(3.0 * M_PI, 3.0)
      .Arc(M_PI / 2.0, 1.5)
      .Straight(3.0)
      .Arc(-M_PI / 2.0, 1.5)
      .Straight(4.5)
      .Arc(M_PI / 2.0, 1.0)
      .Arc(-tail_steps * kStep / kLoopRadius, kLoopRadius);
  return path.Take();
}

int LastWingStep(const std::vector<Eigen::Affine2d>& truth) {
  int last = -1;
  for (size_t step = 0; step < truth.size(); ++step) {
    if (InWing(truth[step])) {
      last = static_cast<int>(step);
    }
  }
  return last;
}

// Local poses from identity, so the backend is seeded with truth.front(); a constant yaw bias
// per step is the lap fixture's drift model.
std::vector<Eigen::Affine2d> Odometry(const std::vector<Eigen::Affine2d>& truth,
                                      double yaw_bias_per_step) {
  std::vector<Eigen::Affine2d> poses{Eigen::Affine2d::Identity()};
  for (size_t i = 1; i < truth.size(); ++i) {
    poses.push_back(Eigen::Affine2d(poses.back() * truth[i - 1].inverse() * truth[i] *
                                    transform::FromXYTheta(0.0, 0.0, yaw_bias_per_step)));
  }
  return poses;
}

// Growth and settling thresholds small enough to play out within one excursion; the shipped
// growth tolerance already absorbs the wing's second half-turn (+18 m^2, then under 2 m^2 per
// submap). The covariance gate stays on but opened past the shipped 0.02 m / 0.005 rad to what
// a wing tied down only through the doorway reaches when it settles halfway round: 0.031 to
// 0.037 m, 0.0046 to 0.0051 rad. Feeding one step and draining before the next keeps the
// closure set the same run to run.
PoseGraphOption TestOption() {
  PoseGraphOption option;
  option.optimization.optimize_every_n_nodes = 10;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  option.checkpoint_min_interval = common::Duration::zero();
  option.session_manager.min_new_area = 5.0;
  option.session_manager.growth_tolerance = 20.0;
  // A submap's 20 nodes at 0.24 m stand in ~12 fresh 0.4 m cells: the excursion's own laps are
  // growth, the wing's second half-turn is not.
  option.session_manager.track_growth_tolerance = 0.5;
  option.session_manager.submaps_without_growth = 2;
  option.session_manager.freeze_judge.max_translation_stddev = 0.05;
  option.session_manager.freeze_judge.max_rotation_stddev = 0.01;
  return option;
}

// Session-local node index k was fed for step first_step + k.
double MeanNodeError(
    const PoseGraphData& graph, SessionId session, const std::vector<Eigen::Affine2d>& truth,
    int first_step,
    const std::function<bool(const Eigen::Affine2d&)>& include = [](const Eigen::Affine2d&) {
      return true;
    }) {
  double total = 0.0;
  int count = 0;
  for (const NodeId& id : graph.session(session).node_ids) {
    const Eigen::Affine2d& expected = truth.at(first_step + id.node_index);
    if (!include(expected)) {
      continue;
    }
    total += (graph.node(id).global_pose.translation() - expected.translation()).norm();
    ++count;
  }
  return count == 0 ? 0.0 : total / static_cast<double>(count);
}

std::vector<std::array<double, 6>> SnapshotSession(const PoseGraphData& graph, SessionId session) {
  std::vector<std::array<double, 6>> poses;
  const auto append = [&poses](const Eigen::Affine2d& pose) {
    poses.push_back({pose.linear()(0, 0), pose.linear()(0, 1), pose.linear()(1, 0),
                     pose.linear()(1, 1), pose.translation().x(), pose.translation().y()});
  };
  for (const SubmapId& id : graph.session(session).submap_ids) {
    append(graph.submap(id).global_pose);
  }
  for (const NodeId& id : graph.session(session).node_ids) {
    append(graph.node(id).global_pose);
  }
  return poses;
}

void ExpectBitwiseEqual(const std::vector<std::array<double, 6>>& before,
                        const std::vector<std::array<double, 6>>& after) {
  ASSERT_EQ(before.size(), after.size());
  for (size_t i = 0; i < before.size(); ++i) {
    for (size_t j = 0; j < before[i].size(); ++j) {
      EXPECT_EQ(before[i][j], after[i][j]) << "frozen pose " << i << " coefficient " << j;
    }
  }
}

// The frozen layer as the manager saw it when `session` was judged: every other frozen session.
FrozenCoverage CoverageExcluding(const PoseGraphData& graph, const PoseGraphOption& option,
                                 SessionId session) {
  FrozenCoverage coverage(option.session_manager.coverage_resolution,
                          option.session_manager.track_resolution);
  for (const auto& [id, data] : graph.sessions()) {
    if (data.frozen() && id != session) {
      coverage.AddSession(graph, id);
    }
  }
  return coverage;
}

int CountCoveredSubmaps(const PoseGraphData& graph, const PoseGraphOption& option,
                        SessionId session) {
  const FrozenCoverage coverage = CoverageExcluding(graph, option, session);
  int covered = 0;
  for (const SubmapId& id : graph.session(session).submap_ids) {
    if (coverage.Query(graph.submap(id)).covered_fraction() >=
        option.session_manager.frozen_covered_fraction) {
      ++covered;
    }
  }
  return covered;
}

constexpr int kBaseSteps = kNodesPerLap + 10;

// Room A mapped drift-free, closed by the revisit and frozen by hand: the base the boots below
// load. Everything is finished first, so nothing is handed over and the successor stays empty.
SessionId FreezeRoomA(const std::string& directory) {
  PoseGraphOption option = TestOption();
  option.session_manager.auto_freeze = false;
  PoseGraph backend(option, directory);
  const std::vector<Eigen::Affine2d> truth = LoopTruth(kBaseSteps);
  backend.Start(TestTime(0), truth.front());
  const SessionId base = *backend.session_manager().fed_session();
  Frontend frontend(backend, TwoRooms(), truth, Odometry(truth, 0.0), kRange);
  for (int step = 0; step < kBaseSteps; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  EXPECT_GT(backend.constraint_builder().num_constraints_added(), 0);
  frontend.FinishAll();
  backend.FreezeFedSession();
  backend.WaitUntilQuiescent();
  EXPECT_EQ(backend.session_manager().num_sessions_frozen(), 1);
  EXPECT_TRUE(backend.graph().session(base).frozen());
  return base;
}

// Nothing frozen yet, so the boot session carries the datum and freezes at its first finished
// submap. Its successor inherits the open submaps and maps the wing tied down at its head only;
// the wing's second half-turn adds nothing to the union, so the successor settles inside the
// wing and freezes there on its covariance alone, before its tail ever closes onto frozen truth.
TEST(FreezeE2eTest, FirstSessionFreezesAtItsFirstSubmapAndTheWingSessionOnSigmaAloneOnceSettled) {
  const PoseGraphOption option = TestOption();
  PoseGraph backend(option);
  const PoseGraphData& graph = backend.graph();
  const SessionManager& manager = backend.session_manager();
  const std::vector<Eigen::Affine2d> truth = ExcursionTruth(60);
  const int last_wing_step = LastWingStep(truth);
  backend.Start(TestTime(0), truth.front());
  const SessionId first = *manager.fed_session();
  Frontend frontend(backend, TwoRooms(), truth, Odometry(truth, kYawBiasPerStep), kRange);

  int step = 0;
  while (manager.num_sessions_frozen() == 0 && step < 40) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_EQ(manager.num_sessions_frozen(), 1)
      << "no bootstrap freeze; last verdict: " << ToString(manager.last_verdict().rejection);
  EXPECT_TRUE(manager.last_verdict().bootstrap);
  EXPECT_TRUE(graph.session(first).frozen());
  // The first submap finishes at node 20 and the next solve freezes it: one submap frozen, the
  // two open ones handed over.
  EXPECT_GE(step, 21);
  EXPECT_LE(step, 30);
  EXPECT_EQ(graph.session(first).submap_ids.size(), 1u);
  const SessionId second = *manager.fed_session();
  EXPECT_EQ(second.session_index, first.session_index + 1);
  EXPECT_EQ(graph.session(second).submap_ids.size(), 2u);
  const int second_start_step = step;
  const int judgements_at_bootstrap = manager.num_judgements();
  const std::vector<std::array<double, 6>> frozen_at_freeze = SnapshotSession(graph, first);

  // Out through the doorway and around the wing: growth keeps the judge quiet until the union
  // stands still, and the first verdict freezes the session where it is.
  double error_before_freeze = 0.0;
  while (manager.num_sessions_frozen() == 1 && step < static_cast<int>(truth.size())) {
    error_before_freeze = MeanNodeError(graph, second, truth, second_start_step, InWing);
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_EQ(manager.num_sessions_frozen(), 2) << "the wing session never froze; last verdict: "
                                              << ToString(manager.last_verdict().rejection);
  const int freeze_step = step - 1;
  EXPECT_TRUE(graph.session(second).frozen());
  EXPECT_FALSE(manager.last_verdict().bootstrap);
  EXPECT_EQ(manager.last_verdict().rejection, FreezeRejection::NONE);
  EXPECT_EQ(manager.num_judgements(), judgements_at_bootstrap + 1)
      << "the first verdict, at the entry that reached JUDGING, is the freeze";
  EXPECT_TRUE(InWing(truth[freeze_step]))
      << "frozen after the wing was left, at step " << freeze_step << " of " << last_wing_step;
  EXPECT_EQ(manager.fed_session()->session_index, second.session_index + 1);
  {
    // Frozen on the covariance alone: the head holds the frozen nodes, the tail has nothing
    // frozen to tie to.
    const std::map<SubmapId, int>& links = manager.last_verdict().frozen_links;
    ASSERT_GE(links.size(), 2u);
    EXPECT_GT(links.begin()->second, 0) << "the handed-over head is tied to frozen nodes";
    EXPECT_EQ(std::prev(links.end())->second, 0) << "the tail in the wing is untied";
  }
  // Frozen halfway round on a 0.03 m sigma, the wing keeps the 0.58 m the yaw bias gave it:
  // the gate reads the chain's noise, not its drift (ROADMAP open issue 1). The ceiling only
  // catches a divergence; the bitwise check below is what proves the freeze.
  const double error_in_wing = MeanNodeError(graph, second, truth, second_start_step, InWing);
  EXPECT_LT(error_in_wing, 1.0) << "before the freeze " << error_before_freeze;
  // The pass through known area is discarded on the way in; the one junction submap that alone
  // holds frozen nodes is the handover's known cost.
  EXPECT_LE(CountCoveredSubmaps(graph, option, second), 1);
  EXPECT_GT(graph.session(second).submap_ids.size(), 2u) << "the wing itself must survive";
  const std::vector<std::array<double, 6>> wing_at_freeze = SnapshotSession(graph, second);

  // The rest of the wing and the way back lie inside frozen area now: the successor localizes,
  // and its closures onto the frozen wing cannot move what is truth.
  const SessionId third = *manager.fed_session();
  for (; step < static_cast<int>(truth.size()); ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  EXPECT_EQ(manager.num_sessions_frozen(), 2);
  EXPECT_EQ(manager.num_judgements(), judgements_at_bootstrap + 1);
  ASSERT_EQ(manager.expansion().count(third), 1u);
  EXPECT_EQ(manager.expansion().at(third).phase, SessionManager::Phase::LOCALIZING);
  // Back in the room the successor snaps onto the frozen room, whatever the wing says.
  EXPECT_LT(MeanNodeError(graph, third, truth, freeze_step + 1,
                          [](const Eigen::Affine2d& pose) { return !InWing(pose); }),
            0.25);
  ExpectBitwiseEqual(frozen_at_freeze, SnapshotSession(graph, first));
  ExpectBitwiseEqual(wing_at_freeze, SnapshotSession(graph, second));
}

// A boot inside a frozen room that maps a new wing: the session settles inside the wing and
// freezes there, keeps the wing, and sheds what it saw of the room on its way out.
TEST(FreezeE2eTest, ExcursionFreezesInsideTheWingAndItsKnownAreaPassIsTrimmed) {
  const std::string directory = testing::MakeTempDir("evergreenslam_freeze_excursion");
  const SessionId base = FreezeRoomA(directory);

  const PoseGraphOption option = TestOption();
  PoseGraph backend(option, directory);
  const PoseGraphData& graph = backend.graph();
  const SessionManager& manager = backend.session_manager();
  const std::vector<Eigen::Affine2d> truth = ExcursionTruth(60);
  const int last_wing_step = LastWingStep(truth);
  backend.Start(TestTime(kBaseSteps), truth.front());
  ASSERT_TRUE(graph.session(base).frozen());
  const std::vector<std::array<double, 6>> base_at_boot = SnapshotSession(graph, base);
  const SessionId session = *manager.fed_session();
  Frontend frontend(backend, TwoRooms(), truth, Odometry(truth, kYawBiasPerStep), kRange);

  int step = 0;
  while (manager.num_sessions_frozen() == 0 && step < static_cast<int>(truth.size())) {
    frontend.Feed(step++);
    backend.WaitUntilQuiescent();
  }
  ASSERT_EQ(manager.num_sessions_frozen(), 1)
      << "the excursion never froze; last verdict: " << ToString(manager.last_verdict().rejection);
  const int freeze_step = step - 1;
  EXPECT_TRUE(graph.session(session).frozen());
  EXPECT_FALSE(manager.last_verdict().bootstrap);
  EXPECT_EQ(manager.last_verdict().rejection, FreezeRejection::NONE);
  EXPECT_EQ(manager.num_judgements(), 1) << "the first verdict is the freeze";
  EXPECT_TRUE(InWing(truth[freeze_step]))
      << "frozen after the wing was left, at step " << freeze_step << " of " << last_wing_step;
  EXPECT_EQ(manager.fed_session()->session_index, session.session_index + 1);

  // Entered with the drift the base's closures left and closed on itself, the wing is frozen
  // 0.23 m adrift; nothing corrects it after that.
  EXPECT_LT(MeanNodeError(graph, session, truth, 0, InWing), 0.4);
  EXPECT_EQ(CountCoveredSubmaps(graph, option, session), 0)
      << "a frozen submap lying in the base's area survived the overlap trim";
  EXPECT_GT(graph.session(session).submap_ids.size(), 2u) << "the wing itself must survive";
  for (const SubmapId& id : graph.session(session).submap_ids) {
    EXPECT_TRUE(graph.submap(id).submap->finished());
  }

  EXPECT_EQ(backend.map_manager()->num_frozen_files_written(), 1);
  const std::optional<MapManager::FileSummary> file =
      backend.map_manager()->InspectSessionFile(session);
  ASSERT_TRUE(file.has_value());
  EXPECT_TRUE(file->frozen);
  EXPECT_EQ(file->num_submaps, static_cast<int>(graph.session(session).submap_ids.size()));
  EXPECT_GT(file->num_constraints, 0);

  ExpectBitwiseEqual(base_at_boot, SnapshotSession(graph, base));
}

// A boot that only revisits the frozen room is localization: anchored by its first closure,
// bounded by the trimmer, checkpointed as usual, and never judged, let alone frozen.
TEST(FreezeE2eTest, ALocalizingSessionNeverFreezes) {
  const std::string directory = testing::MakeTempDir("evergreenslam_freeze_localizing");
  const SessionId base = FreezeRoomA(directory);

  const PoseGraphOption option = TestOption();
  PoseGraph backend(option, directory);
  const PoseGraphData& graph = backend.graph();
  const SessionManager& manager = backend.session_manager();
  const int num_steps = 3 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> truth = LoopTruth(num_steps);
  // Seeded a few decimetres off: the first closure has to snap the session onto the base.
  backend.Start(TestTime(kBaseSteps),
                Eigen::Affine2d(truth.front() * transform::FromXYTheta(0.3, -0.25, 0.1)));
  const SessionId session = *manager.fed_session();
  Frontend frontend(backend, TwoRooms(), truth, Odometry(truth, kYawBiasPerStep), kRange);

  for (int step = 0; step < num_steps; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }

  EXPECT_TRUE(HasFrozenLink(graph, session)) << "no closure anchored the boot onto the base";
  EXPECT_EQ(manager.num_sessions_frozen(), 0);
  EXPECT_EQ(manager.num_judgements(), 0) << "a session that grew nothing is never judged";
  EXPECT_FALSE(graph.session(session).frozen());
  EXPECT_EQ(*manager.fed_session(), session);
  ASSERT_EQ(manager.expansion().count(session), 1u);
  EXPECT_TRUE(manager.expansion().at(session).anchored);
  EXPECT_EQ(manager.expansion().at(session).phase, SessionManager::Phase::LOCALIZING);
  EXPECT_LT(manager.expansion().at(session).novel_area, option.session_manager.min_new_area);
  EXPECT_LT(MeanNodeError(graph, session, truth, 0), 0.25);

  EXPECT_GT(backend.trimmer().num_submaps_trimmed(), 0) << "trimming never engaged";
  EXPECT_EQ(backend.constraint_builder().num_constraints_dropped(), 0);
  const int created = graph.id_allocator().next_submap_index(session);
  const int surviving = static_cast<int>(graph.session(session).submap_ids.size());
  EXPECT_GT(created, surviving + 2) << "three laps over one spot have to shed submaps";
  // The two actives, the newest finished ones that never go, the ones behind them that lack
  // coverers yet, plus one whose footprint is not covered yet.
  EXPECT_LE(surviving, 2 + option.trimmer.selector.keep_newest_submaps +
                           option.trimmer.selector.min_covering_newer_submaps + 1)
      << "the localizing session's submap count is not bounded";

  EXPECT_GT(backend.map_manager()->num_checkpoints_written(), 0);
  EXPECT_EQ(backend.map_manager()->num_frozen_files_written(), 0);
  const std::optional<MapManager::FileSummary> file =
      backend.map_manager()->InspectSessionFile(session);
  ASSERT_TRUE(file.has_value());
  EXPECT_FALSE(file->frozen);
  EXPECT_GT(file->num_nodes, 0);
  EXPECT_TRUE(backend.map_manager()->InspectSessionFile(base)->frozen);
}

// The same excursion with auto_freeze off: one session for the whole run, no frozen file; the
// manual entry then runs the freeze sequence, overlap trim included.
TEST(FreezeE2eTest, AutoFreezeOffKeepsOneSession) {
  const std::string directory = testing::MakeTempDir("evergreenslam_freeze_manual");
  FreezeRoomA(directory);

  PoseGraphOption option = TestOption();
  option.session_manager.auto_freeze = false;
  PoseGraph backend(option, directory);
  const PoseGraphData& graph = backend.graph();
  const SessionManager& manager = backend.session_manager();
  const std::vector<Eigen::Affine2d> truth = ExcursionTruth(60);
  backend.Start(TestTime(kBaseSteps), truth.front());
  const SessionId session = *manager.fed_session();
  Frontend frontend(backend, TwoRooms(), truth, Odometry(truth, kYawBiasPerStep), kRange);

  for (int step = 0; step < static_cast<int>(truth.size()); ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  EXPECT_EQ(manager.num_sessions_frozen(), 0);
  EXPECT_EQ(manager.num_judgements(), 0);
  EXPECT_EQ(*manager.fed_session(), session);
  EXPECT_FALSE(graph.session(session).frozen());
  EXPECT_EQ(backend.map_manager()->num_frozen_files_written(), 0);
  ASSERT_TRUE(backend.map_manager()->InspectSessionFile(session).has_value());
  EXPECT_FALSE(backend.map_manager()->InspectSessionFile(session)->frozen);
  EXPECT_GT(CountCoveredSubmaps(graph, option, session), 0)
      << "the pass through the room has to be there for the manual freeze to shed";
  const size_t submaps_before = graph.session(session).submap_ids.size();

  backend.FreezeFedSession();
  backend.WaitUntilQuiescent();
  EXPECT_EQ(manager.num_sessions_frozen(), 1);
  EXPECT_EQ(manager.num_judgements(), 0) << "the manual entry asks no verdict";
  EXPECT_TRUE(graph.session(session).frozen());
  EXPECT_EQ(manager.fed_session()->session_index, session.session_index + 1);
  EXPECT_LT(graph.session(session).submap_ids.size(), submaps_before);
  EXPECT_EQ(CountCoveredSubmaps(graph, option, session), 0);
  EXPECT_EQ(backend.map_manager()->num_frozen_files_written(), 1);
  EXPECT_TRUE(backend.map_manager()->InspectSessionFile(session)->frozen);
}

}  // namespace
}  // namespace evergreenslam::lifelong

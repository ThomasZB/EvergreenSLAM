/**
 * @file pose_graph_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The whole lifelong chain through the real wiring: mapping, loop closure, freeze, trim,
 *        checkpoint, kill, reload, and anchoring back into the frozen base.
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "lifelong/sessions/frozen_links.h"
#include "mapping/grid_mapping/probability_values.h"
#include "testing/loop_scenario.h"

namespace evergreenslam::lifelong {
namespace {

// The simulated room, drifted odometry and frontend emulator live in testing/loop_scenario.h.
using testing::DriftedOdometry;
using testing::Frontend;
using testing::kNodesPerLap;
using testing::MeanNodeError;
using testing::TestTime;
using testing::World;

const World& kLoopRoom = testing::LoopRoom();

std::string MakeTempDir() { return testing::MakeTempDir("evergreenslam_backend_e2e"); }

// Options come from the shipped config through the real loader; only the search budget is
// reduced. Freezing runs as shipped: the boot session bootstraps at its first finished submap,
// and every later pass through that known area is localization, which never freezes.
TEST(PoseGraphE2eTest, FullChainSurvivesAKillAndAnchorsBackIntoTheFrozenBase) {
  const std::string directory = MakeTempDir();
  const std::string config_path = std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml";
  PoseGraphOption option = LoadPoseGraphOptionFromFile(config_path);
  // One match worker: this record asserts submap and checkpoint counts, so the match order and
  // therefore the closure set must not vary between runs.
  option.constraint_builder.num_match_workers = 1;
  option.constraint_builder.sampler_option.sampling_ratio = 0.25;
  option.constraint_builder.sampler_option.max_matches_per_round = 4;
  // The kill below must land within one solve cadence of a checkpoint.
  option.checkpoint_min_interval = common::Duration::zero();

  const int total_steps = 6 * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(total_steps);

  SessionId first_session;
  SessionId second_session;
  int second_session_start_step = 0;
  int watermark_node_index = 0;
  int watermark_submap_index = 0;
  int second_session_submaps_created = 0;
  int steps_fed = 0;
  double error_at_kill = 0.0;
  std::vector<Eigen::Vector2d> frozen_translations;

  {
    PoseGraph backend(option, directory);

    backend.Start(TestTime(0));
    Frontend frontend(backend, odometry);
    first_session = *backend.session_manager().fed_session();
    EXPECT_EQ(backend.map_manager()->boot_count(), 1) << "Start persists the boot count";

    // Bootstrap: the boot session freezes at its first finished submap, and the drifted lap goes
    // on in the successor, which closes the loop against that frozen submap on the revisit.
    int step = 0;
    // Paced like a real frontend, here and below: the backend keeps up with the feed. Flat out,
    // matches queue up behind dozens of keyframes and the trim that follows each solve deletes
    // what they name; that residual is accepted by design, not what this test measures.
    while (backend.session_manager().num_sessions_frozen() == 0 && step < kNodesPerLap) {
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1)
        << "the boot session never froze; last verdict: "
        << ToString(backend.session_manager().last_verdict().rejection);
    EXPECT_TRUE(backend.session_manager().last_verdict().bootstrap);
    EXPECT_TRUE(backend.graph().session(first_session).frozen());
    second_session = *backend.session_manager().fed_session();
    // The node fed at the freeze step still landed in the frozen session.
    second_session_start_step = step;
    EXPECT_EQ(second_session.session_index, first_session.session_index + 1);
    for (; step < kNodesPerLap + 30; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    backend.WaitUntilQuiescent();
    ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0)
        << "the revisit produced no loop closure";
    // Freezing finished every submap of the frozen session, trailing actives included.
    for (const SubmapId& id : backend.graph().session(first_session).submap_ids) {
      EXPECT_TRUE(backend.graph().submap(id).submap->finished());
    }
    for (const SubmapId& id : backend.graph().session(first_session).submap_ids) {
      frozen_translations.push_back(backend.graph().submap(id).global_pose.translation());
    }

    // Re-drive the same circle three times: a pass through frozen area is localization, so the
    // second session stays open however long it runs, and the trimmer has to bound its submaps.
    const int stage2_min = step + 3 * kNodesPerLap;
    for (; step < stage2_min; ++step) {
      frontend.Feed(step);
      backend.WaitUntilQuiescent();
    }
    backend.WaitUntilQuiescent();
    // Kill right behind a checkpoint: the trim that came with it is what bounds the count below.
    const int checkpoints_before = backend.map_manager()->num_checkpoints_written();
    while (backend.map_manager()->num_checkpoints_written() == checkpoints_before) {
      ASSERT_LT(step, stage2_min + option.optimization.optimize_every_n_nodes);
      frontend.Feed(step++);
      backend.WaitUntilQuiescent();
    }
    steps_fed = step;
    EXPECT_EQ(*backend.session_manager().fed_session(), second_session)
        << "a session inside known area never expands, so it never freezes";
    ASSERT_GT(backend.trimmer().num_submaps_trimmed(), 0) << "trimming never engaged";
    EXPECT_EQ(backend.constraint_builder().num_constraints_dropped(), 0)
        << "the trimmer deleted the endpoint of a match still in flight";
    second_session_submaps_created =
        backend.graph().id_allocator().next_submap_index(second_session);
    const int surviving =
        static_cast<int>(backend.graph().session(second_session).submap_ids.size());
    EXPECT_GT(second_session_submaps_created, surviving + 2)
        << "three laps over one spot have to shed submaps";
    // Right after a trim: the two actives, the newest finished ones that never go, and the ones
    // behind them that lack enough coverers yet, plus one whose footprint is not covered yet.
    EXPECT_LE(surviving, 2 + option.trimmer.selector.keep_newest_submaps +
                             option.trimmer.selector.min_covering_newer_submaps + 1)
        << "the active session's submap count is not bounded";

    ASSERT_GT(backend.map_manager()->num_checkpoints_written(), 0);
    const std::optional<MapManager::FileSummary> frozen_file =
        backend.map_manager()->InspectSessionFile(first_session);
    ASSERT_TRUE(frozen_file.has_value());
    EXPECT_TRUE(frozen_file->frozen);
    EXPECT_GT(frozen_file->num_constraints, 0) << "frozen constraints are stored in the file";

    error_at_kill = MeanNodeError(backend.graph(), second_session, second_session_start_step);
    watermark_node_index = backend.graph().id_allocator().next_node_index(second_session);
    watermark_submap_index = backend.graph().id_allocator().next_submap_index(second_session);
  }
  // The last checkpoint rode the last global solve, so the watermarks on disk may trail the ones
  // recorded above; the reloaded backend must never reuse a persisted index.

  PoseGraph backend(option, directory);
  backend.Start(TestTime(steps_fed));
  EXPECT_EQ(backend.map_manager()->boot_count(), 2) << "the reboot increments the boot count";
  const SessionId boot_session = *backend.session_manager().fed_session();
  EXPECT_EQ(boot_session.session_index, second_session.session_index + 1)
      << "boot has to open a fresh session";
  ASSERT_TRUE(backend.graph().HasSession(second_session));
  EXPECT_FALSE(backend.graph().session(second_session).frozen())
      << "the loaded unfrozen session floats instead of being fed or dropped";
  // The floating session's persisted watermarks survive even though nothing feeds it again.
  EXPECT_GT(backend.graph().id_allocator().next_node_index(second_session), 0);
  EXPECT_LE(backend.graph().id_allocator().next_node_index(second_session), watermark_node_index);
  EXPECT_GT(backend.graph().id_allocator().next_submap_index(second_session), 0);
  EXPECT_LE(backend.graph().id_allocator().next_submap_index(second_session),
            watermark_submap_index);
  // Every restored submap is finished now, so all of them can serve loop closure.
  for (const auto& [id, record] : backend.graph().submaps()) {
    EXPECT_TRUE(record.submap->finished());
  }

  ASSERT_FALSE(backend.graph().session(second_session).node_ids.empty());
  const NodeId last_restored = backend.graph().session(second_session).node_ids.back();
  EXPECT_LT((backend.graph().session(boot_session).local_to_global.translation() -
             backend.graph().node(last_restored).global_pose.translation())
                .norm(),
            1e-9);

  // The restarted frontend starts a fresh local frame at identity where the checkpoint left the
  // robot, so the drifted odometry stream is rebased onto the last restored keyframe.
  const int resume_step = second_session_start_step + last_restored.node_index;
  const Eigen::Affine2d resume_inverse = odometry[resume_step].inverse();
  std::vector<Eigen::Affine2d> rebased;
  rebased.reserve(odometry.size());
  for (const Eigen::Affine2d& pose : odometry) {
    rebased.push_back(Eigen::Affine2d(resume_inverse * pose));
  }

  // Loop search against the reloaded grids anchors the boot session onto the frozen base and
  // the floating one. Neither maps anything new, so neither freezes: a known-area pass is
  // localization, and the floating session is only judged when a constraint lands on it.
  Frontend frontend(backend, rebased);
  int step = steps_fed;
  const int stage3_end = std::min(steps_fed + kNodesPerLap, total_steps);
  for (; step < stage3_end; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
  }
  backend.WaitUntilQuiescent();
  ASSERT_GT(backend.constraint_builder().num_constraints_added(), 0)
      << "no loop closure against the reloaded submaps";
  EXPECT_TRUE(HasFrozenLink(backend.graph(), boot_session))
      << "the boot session never anchored onto the frozen base";
  EXPECT_EQ(backend.session_manager().num_sessions_frozen(), 0)
      << "nothing new was mapped, so nothing earned a freeze";
  EXPECT_FALSE(backend.graph().session(second_session).frozen());
  EXPECT_EQ(*backend.session_manager().fed_session(), boot_session);

  backend.Finish();

  // The frozen base held still through the reload and the second run's optimization, within the
  // serialization round trip (theta travels as one double, ~1 ulp).
  const std::vector<SubmapId>& first_ids = backend.graph().session(first_session).submap_ids;
  ASSERT_EQ(first_ids.size(), frozen_translations.size());
  for (size_t i = 0; i < first_ids.size(); ++i) {
    EXPECT_LT(
        (backend.graph().submap(first_ids[i]).global_pose.translation() - frozen_translations[i])
            .norm(),
        1e-12)
        << "frozen submap " << i << " moved";
  }

  // The drifted lap and its closure live in the successor; the frozen session is bitwise
  // constant (asserted above) and says nothing about recovery. Closure distributes the yaw-bias
  // residual around the ring: 0.174 m at the kill at the shipped 30-node cadence (0.145 m when
  // every keyframe solved: closures now land on poses left uncorrected until the next cadence).
  // The reload reproduces that to the digit, and the boot session's drifted closures then pull
  // the still-free floating session out to 0.276 m (0.211 m solving per keyframe).
  const double error_after =
      MeanNodeError(backend.graph(), second_session,
                    testing::SessionStartSteps(backend.graph()).at(second_session.session_index));
  std::cout << "e2e: second session mean error at kill " << error_at_kill << " m, after reload "
            << error_after << " m" << std::endl;
  EXPECT_LT(error_at_kill, 0.2);
  EXPECT_LT(error_after, 0.30) << "at kill " << error_at_kill << ", after " << error_after;

  // The assembled global map shows the room, not the drift.
  const mapping::GridMapu8 global_map = backend.AssembleGlobalMap();
  ASSERT_GT(global_map.width(), 0);
  ASSERT_GT(global_map.height(), 0);
  int occupied_near_walls = 0;
  int occupied_total = 0;
  for (int y = 0; y < global_map.height(); ++y) {
    for (int x = 0; x < global_map.width(); ++x) {
      const uint8_t value = global_map.GetValue(x, y);
      if (!mapping::IsKnownValue(value) || mapping::ValueToProbability(value) < 0.6) {
        continue;
      }
      ++occupied_total;
      const double wx = global_map.origin_x() + (x + 0.5) * global_map.resolution();
      const double wy = global_map.origin_y() + (y + 0.5) * global_map.resolution();
      const double to_boundary =
          std::min(std::min(std::abs(wx - kLoopRoom.min_x), std::abs(kLoopRoom.max_x - wx)),
                   std::min(std::abs(wy - kLoopRoom.min_y), std::abs(kLoopRoom.max_y - wy)));
      double to_pillar = std::numeric_limits<double>::max();
      for (const Eigen::AlignedBox2d& pillar : kLoopRoom.pillars) {
        to_pillar = std::min(to_pillar, pillar.exteriorDistance(Eigen::Vector2d(wx, wy)));
      }
      // The frozen poses carry the residual of the closure they were frozen with, plus the drift
      // baked inside each submap's grid.
      if (std::min(to_boundary, to_pillar) < 0.35) {
        ++occupied_near_walls;
      }
    }
  }
  ASSERT_GT(occupied_total, 0);
  EXPECT_GT(static_cast<double>(occupied_near_walls) / occupied_total, 0.85)
      << "the assembled map's occupied cells are not on the world's walls";
}

}  // namespace
}  // namespace evergreenslam::lifelong

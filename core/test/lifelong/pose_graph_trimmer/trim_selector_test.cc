/**
 * @file trim_selector_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Coverage-based selection: redundant submaps go, distant ones and the floor stay.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/trim_selector.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "common/time.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// A ring of points around `center`, deliberately off the resolution lattice.
sensor::PointCloud RingScan(const Eigen::Vector2d& center, double radius) {
  sensor::PointCloud cloud;
  for (int i = 0; i < 72; ++i) {
    const double angle = 2.0 * M_PI * i / 72.0;
    cloud.push_back(
        sensor::Point2d{center + radius * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  return cloud;
}

void AddSubmap(PoseGraphData& graph, SessionId session, int index, const Eigen::Vector2d& center,
               mapping::CastRaysMapping& inserter, bool observed,
               std::optional<Eigen::Vector2d> node_center, bool finished = true) {
  const SubmapId submap_id{session.session_index, index};
  const Eigen::Affine2d pose = transform::FromXYTheta(center.x(), center.y(), 0.0);
  auto submap = std::make_shared<mapping::Submap>(submap_id.submap_index, pose, kResolution);
  if (observed) {
    submap->InsertScan(pose, RingScan(Eigen::Vector2d::Zero(), 2.013), inserter);
  }
  if (finished) {
    submap->Finish();
  }

  SubmapRecord record;
  record.id = submap_id;
  record.submap = submap;
  record.local_pose = pose;
  record.global_pose = pose;
  graph.AddSubmap(record);

  if (node_center.has_value()) {
    Node node;
    node.id = NodeId{session.session_index, index};
    node.constant_data.time = TestTime(index);
    node.constant_data.local_pose = transform::FromXYTheta(node_center->x(), node_center->y(), 0.0);
    node.global_pose = node.constant_data.local_pose;
    graph.AddNode(node, {submap_id});
  }
}

// Co-located submaps end up with identical footprints, which is the redundancy the selector
// looks for.
void AddObservedSubmap(PoseGraphData& graph, SessionId session, int index,
                       const Eigen::Vector2d& center, mapping::CastRaysMapping& inserter) {
  AddSubmap(graph, session, index, center, inserter, true, center);
}

TrimSelector AllModeSelector() {
  TrimSelectorOption option;
  option.coverage_mode = CoverageMode::kAll;
  return TrimSelector(option);
}

bool Contains(const std::vector<SubmapId>& ids, const SubmapId& id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

// A half ring is lopsided, so turning the submap in place must move its footprint.
TEST(TrimSelectorTest, TheFootprintCacheFollowsARotationInPlace) {
  const Eigen::Affine2d pose = transform::FromXYTheta(1.013, -0.257, 0.0);
  auto submap = std::make_shared<mapping::Submap>(0, pose, kResolution);
  sensor::PointCloud half_ring;
  for (int i = 0; i < 36; ++i) {
    const double angle = M_PI * i / 36.0 + 0.011;
    half_ring.push_back(sensor::Point2d{2.013 * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  mapping::CastRaysMapping inserter;
  submap->InsertScan(pose, half_ring, inserter);
  submap->Finish();
  SubmapRecord record;
  record.id = SubmapId{0, 0};
  record.submap = submap;
  record.local_pose = pose;
  record.global_pose = pose;

  const TrimSelector selector;
  const std::set<std::int64_t> before = [&] {
    const std::vector<std::int64_t>& cells = selector.Footprint(record);
    return std::set<std::int64_t>(cells.begin(), cells.end());
  }();
  ASSERT_FALSE(before.empty());

  record.global_pose = transform::FromXYTheta(1.013, -0.257, M_PI);
  const std::vector<std::int64_t>& after = selector.Footprint(record);
  EXPECT_NE(std::set<std::int64_t>(after.begin(), after.end()), before);
}

TEST(TrimSelectorTest, PicksEveryRedundantSubmapOutsideTheNewestWindow) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d center(3.017, 2.503);
  for (int k = 0; k < 9; ++k) {
    AddObservedSubmap(graph, session, k, center, inserter);
  }

  const TrimSelector selector;
  const std::vector<SubmapId> selected = selector.Select(graph);

  // Nine co-located submaps, 7 and 8 neither go nor cover: 0..5 have a newer covering node,
  // 0..4 two newer covering submaps.
  ASSERT_EQ(selected.size(), 6u);
  for (int k = 0; k < 6; ++k) {
    EXPECT_EQ(selected[k], (SubmapId{session.session_index, k}));
  }
  const std::vector<SubmapId> all_mode = AllModeSelector().Select(graph);
  ASSERT_EQ(all_mode.size(), 5u);
  for (int k = 0; k < 5; ++k) {
    EXPECT_EQ(all_mode[k], (SubmapId{session.session_index, k}));
  }
}

TEST(TrimSelectorTest, TheSurvivorFloorCapsTheSelectionOldestFirst) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d center(3.017, 2.503);
  for (int k = 0; k < 9; ++k) {
    AddObservedSubmap(graph, session, k, center, inserter);
  }

  TrimSelectorOption option;
  option.min_surviving_submaps = 6;
  const std::vector<SubmapId> selected = TrimSelector(option).Select(graph);

  ASSERT_EQ(selected.size(), 3u);
  EXPECT_EQ(selected[0], (SubmapId{session.session_index, 0}));
  EXPECT_EQ(selected[1], (SubmapId{session.session_index, 1}));
  EXPECT_EQ(selected[2], (SubmapId{session.session_index, 2}));
}

TEST(TrimSelectorTest, AFrozenSessionNeverCoversAFloatingOne) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const Eigen::Vector2d shared(3.017, 2.503);
  const SessionId older = graph.StartNewSession(TestTime(0));
  AddObservedSubmap(graph, older, 0, shared, inserter);
  AddObservedSubmap(graph, older, 1, Eigen::Vector2d(50.011, 40.007), inserter);
  AddObservedSubmap(graph, older, 2, Eigen::Vector2d(70.013, 60.009), inserter);
  const SessionId newer = graph.StartNewSession(TestTime(10));
  for (int k = 0; k < 4; ++k) {
    AddObservedSubmap(graph, newer, k, shared, inserter);
  }
  const SubmapId revisited{older.session_index, 0};

  const TrimSelector selector;
  EXPECT_TRUE(Contains(selector.Select(graph), revisited));

  // A frozen session cannot move to correct what it excuses, and its higher session index makes
  // it "newer" than a floating session that may in fact predate it.
  graph.FreezeSession(newer);
  EXPECT_FALSE(Contains(selector.Select(graph), revisited));
}

TEST(TrimSelectorTest, NeverPicksASubmapNothingNewerCovers) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d near(3.017, 2.503);
  const Eigen::Vector2d far(50.011, 40.007);
  AddObservedSubmap(graph, session, 0, far, inserter);
  for (int k = 1; k < 8; ++k) {
    AddObservedSubmap(graph, session, k, near, inserter);
  }

  const TrimSelector selector;
  const std::vector<SubmapId> selected = selector.Select(graph);

  // The uncovered one is not eligible; with 6 and 7 out, 1..4 have a newer covering node.
  ASSERT_EQ(selected.size(), 4u);
  for (int k = 0; k < 4; ++k) {
    EXPECT_EQ(selected[k], (SubmapId{session.session_index, k + 1}));
  }
  // Area coverage needs two newer coverers among 1..5, which only 1..3 have.
  const std::vector<SubmapId> all_mode = AllModeSelector().Select(graph);
  ASSERT_EQ(all_mode.size(), 3u);
  for (int k = 0; k < 3; ++k) {
    EXPECT_EQ(all_mode[k], (SubmapId{session.session_index, k + 1}));
  }
}

TEST(TrimSelectorTest, AFrozenSessionYieldsNothing) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d center(3.017, 2.503);
  for (int k = 0; k < 9; ++k) {
    AddObservedSubmap(graph, session, k, center, inserter);
  }
  graph.FreezeSession(session);

  const TrimSelector selector;
  EXPECT_TRUE(selector.Select(graph).empty());
  EXPECT_TRUE(AllModeSelector().Select(graph).empty());
}

TEST(TrimSelectorTest, TheNewestFinishedSubmapsNeitherGoNorCover) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const Eigen::Vector2d center(3.017, 2.503);
  const SessionId older = graph.StartNewSession(TestTime(0));
  for (int k = 0; k < 3; ++k) {
    AddObservedSubmap(graph, older, k, center, inserter);
  }
  const SessionId newer = graph.StartNewSession(TestTime(10));
  for (int k = 0; k < 5; ++k) {
    AddObservedSubmap(graph, newer, k, center, inserter);
  }
  // Unfinished: outside the window and no coverer, so newer/3 and newer/4 stay the newest two.
  AddSubmap(graph, newer, 5, center, inserter, true, center, /*finished=*/false);

  // older/0, third newest of its session, is buried under newer/0..2; older/1 and older/2 are
  // just as buried but never go. newer/0 has two newer coverers, newer/1 only one: had newer/3
  // or newer/4 covered, it would have three.
  const std::vector<SubmapId> all_mode = AllModeSelector().Select(graph);
  ASSERT_EQ(all_mode.size(), 2u);
  EXPECT_EQ(all_mode[0], (SubmapId{older.session_index, 0}));
  EXPECT_EQ(all_mode[1], (SubmapId{newer.session_index, 0}));

  // Node coverage reaches newer/1 through newer/2's node; newer/2 has nothing newer covering.
  const std::vector<SubmapId> selected = TrimSelector().Select(graph);
  ASSERT_EQ(selected.size(), 3u);
  EXPECT_EQ(selected[0], (SubmapId{older.session_index, 0}));
  EXPECT_EQ(selected[1], (SubmapId{newer.session_index, 0}));
  EXPECT_EQ(selected[2], (SubmapId{newer.session_index, 1}));
}

TEST(TrimSelectorTest, AnyModeTrimsOnNodeCoverageAlone) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d near(3.017, 2.503);
  const Eigen::Vector2d far(50.011, 40.007);
  AddSubmap(graph, session, 0, far, inserter, true, near);
  for (int k = 1; k < 8; ++k) {
    AddObservedSubmap(graph, session, k, near, inserter);
  }

  const std::vector<SubmapId> selected = TrimSelector().Select(graph);
  ASSERT_EQ(selected.size(), 5u);
  EXPECT_EQ(selected[0], (SubmapId{session.session_index, 0}));

  // The conservative mode still demands the area signal too, so submap 0 stays; with 6 and 7
  // not covering, only 1..3 have two newer coverers.
  const std::vector<SubmapId> all_mode = AllModeSelector().Select(graph);
  ASSERT_EQ(all_mode.size(), 3u);
  EXPECT_EQ(all_mode[0], (SubmapId{session.session_index, 1}));
}

TEST(TrimSelectorTest, AnEmptySubmapGoesOnItsCoveredNodesAlone) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d near(3.017, 2.503);
  AddSubmap(graph, session, 0, near, inserter, false, near);
  for (int k = 1; k < 8; ++k) {
    AddObservedSubmap(graph, session, k, near, inserter);
  }

  // An empty grid contributes nothing to the map, so any-mode deletes it on the node signal.
  const std::vector<SubmapId> selected = TrimSelector().Select(graph);
  ASSERT_EQ(selected.size(), 5u);
  EXPECT_EQ(selected[0], (SubmapId{session.session_index, 0}));

  // The conservative mode keeps the old gate: an empty footprint disqualifies.
  const std::vector<SubmapId> all_mode = AllModeSelector().Select(graph);
  ASSERT_EQ(all_mode.size(), 3u);
  EXPECT_EQ(all_mode[0], (SubmapId{session.session_index, 1}));
}

TEST(TrimSelectorTest, ASubmapEmptiedOfNodesDoesNotCascade) {
  PoseGraphData graph;
  mapping::CastRaysMapping inserter;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Vector2d near(3.017, 2.503);
  const Eigen::Vector2d far(50.011, 40.007);
  // Submap 0 owns no nodes; a vacuously-true node signal would let any-mode cascade onto it.
  AddSubmap(graph, session, 0, far, inserter, true, std::nullopt);
  for (int k = 1; k < 8; ++k) {
    AddObservedSubmap(graph, session, k, near, inserter);
  }

  const std::vector<SubmapId> selected = TrimSelector().Select(graph);
  ASSERT_EQ(selected.size(), 4u);
  EXPECT_EQ(selected[0], (SubmapId{session.session_index, 1}));
}

}  // namespace
}  // namespace evergreenslam::lifelong

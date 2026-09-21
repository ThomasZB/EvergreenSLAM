/**
 * @file frozen_coverage_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The frozen cell set: built from frozen sessions only, queried at the record's pose.
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/frozen_coverage.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>

#include "common/time.h"
#include "lifelong/coarse_footprint.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kGridResolution = 0.05;
constexpr double kCoarseResolution = 0.15;
constexpr double kTrackResolution = 0.4;

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

// Deliberately off both the grid and the coarse lattice.
const Eigen::Vector2d kRoomA(3.017, 2.503);
const Eigen::Vector2d kRoomB(-7.281, 4.119);
const Eigen::Vector2d kRoomC(12.443, -9.037);

sensor::PointCloud RingScan(double radius) {
  sensor::PointCloud cloud;
  for (int i = 0; i < 72; ++i) {
    const double angle = 2.0 * M_PI * i / 72.0;
    cloud.push_back(sensor::Point2d{radius * Eigen::Vector2d(std::cos(angle), std::sin(angle))});
  }
  return cloud;
}

// One ring around each of `ring_centers`, each cast from its own centre so no ray crosses
// between them: the footprint is exactly the union of the rings' interiors.
SubmapRecord MakeRecord(SessionId session, int index, const Eigen::Vector2d& center,
                        const std::vector<Eigen::Vector2d>& ring_centers, bool finished = true) {
  const SubmapId id{session.session_index, index};
  const Eigen::Affine2d pose = transform::FromXYTheta(center.x(), center.y(), 0.0);
  auto submap = std::make_shared<mapping::Submap>(index, pose, kGridResolution);
  mapping::CastRaysMapping inserter;
  for (const Eigen::Vector2d& ring : ring_centers) {
    submap->InsertScan(pose * transform::FromXYTheta(ring.x(), ring.y(), 0.0), RingScan(2.013),
                       inserter);
  }
  if (finished) {
    submap->Finish();
  }
  SubmapRecord record;
  record.id = id;
  record.submap = submap;
  record.local_pose = pose;
  record.global_pose = pose;
  return record;
}

SubmapRecord AddRecord(PoseGraphData& graph, SessionId session, int index,
                       const Eigen::Vector2d& center) {
  const SubmapRecord record = MakeRecord(session, index, center, {Eigen::Vector2d::Zero()});
  graph.AddSubmap(record);
  return record;
}

std::set<std::int64_t> Cells(const SubmapRecord& record) {
  const std::vector<std::int64_t> cells = ComputeCoarseFootprint(record, kCoarseResolution);
  return {cells.begin(), cells.end()};
}

class FrozenCoverageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    session_a_ = graph_.StartNewSession(TestTime(0));
    session_b_ = graph_.StartNewSession(TestTime(1));
    live_ = graph_.StartNewSession(TestTime(2));
    submap_a_ = AddRecord(graph_, session_a_, 0, kRoomA);
    submap_b_ = AddRecord(graph_, session_b_, 0, kRoomB);
    AddRecord(graph_, live_, 0, kRoomC);
    graph_.FreezeSession(session_a_);
    graph_.FreezeSession(session_b_);
  }

  PoseGraphData graph_;
  SessionId session_a_;
  SessionId session_b_;
  SessionId live_;
  SubmapRecord submap_a_;
  SubmapRecord submap_b_;
  FrozenCoverage coverage_{kCoarseResolution, kTrackResolution};
};

TEST_F(FrozenCoverageTest, RebuildUnionsTheFrozenSessionsAndSkipsTheLiveOne) {
  coverage_.Rebuild(graph_);

  std::set<std::int64_t> expected = Cells(submap_a_);
  const std::set<std::int64_t> cells_b = Cells(submap_b_);
  expected.insert(cells_b.begin(), cells_b.end());
  ASSERT_FALSE(expected.empty());
  EXPECT_EQ(coverage_.size(), expected.size());
  EXPECT_DOUBLE_EQ(coverage_.resolution(), kCoarseResolution);

  const FrozenCoverage::Overlap live =
      coverage_.Query(MakeRecord(live_, 1, kRoomC, {Eigen::Vector2d::Zero()}));
  EXPECT_GT(live.cells, 0);
  EXPECT_EQ(live.covered, 0);
}

TEST_F(FrozenCoverageTest, ContainsIsTheFrozenCellSetItself) {
  coverage_.Rebuild(graph_);
  for (const std::int64_t cell : Cells(submap_a_)) {
    EXPECT_TRUE(coverage_.Contains(cell));
  }
  for (const std::int64_t cell : Cells(MakeRecord(live_, 1, kRoomC, {Eigen::Vector2d::Zero()}))) {
    EXPECT_FALSE(coverage_.Contains(cell));
  }
}

TEST_F(FrozenCoverageTest, RebuildStartsOver) {
  coverage_.Rebuild(graph_);
  const size_t once = coverage_.size();
  coverage_.Rebuild(graph_);
  EXPECT_EQ(coverage_.size(), once);
}

TEST_F(FrozenCoverageTest, QueriesFullyHalfAndNotCovered) {
  coverage_.Rebuild(graph_);

  const FrozenCoverage::Overlap full =
      coverage_.Query(MakeRecord(live_, 1, kRoomA, {Eigen::Vector2d::Zero()}));
  EXPECT_GT(full.cells, 0);
  EXPECT_EQ(full.covered, full.cells);
  EXPECT_DOUBLE_EQ(full.covered_fraction(), 1.0);
  EXPECT_DOUBLE_EQ(full.novel_area(kCoarseResolution), 0.0);

  // One ring inside A, one far outside: exactly the outside ring is novel.
  const Eigen::Vector2d away(30.0, 0.0);
  const FrozenCoverage::Overlap half =
      coverage_.Query(MakeRecord(live_, 2, kRoomA, {Eigen::Vector2d::Zero(), away}));
  const FrozenCoverage::Overlap outside = coverage_.Query(MakeRecord(live_, 3, kRoomA, {away}));
  EXPECT_EQ(half.covered, full.cells);
  EXPECT_EQ(half.cells, full.cells + outside.cells);
  EXPECT_NEAR(half.covered_fraction(), 0.5, 0.05);
  EXPECT_DOUBLE_EQ(half.novel_area(kCoarseResolution),
                   outside.cells * kCoarseResolution * kCoarseResolution);

  EXPECT_GT(outside.cells, 0);
  EXPECT_EQ(outside.covered, 0);
  EXPECT_DOUBLE_EQ(outside.covered_fraction(), 0.0);
}

TEST_F(FrozenCoverageTest, QueryFollowsTheGlobalPoseNotTheGrid) {
  coverage_.Rebuild(graph_);
  SubmapRecord moved = MakeRecord(live_, 1, kRoomA, {Eigen::Vector2d::Zero()});
  moved.global_pose = transform::FromXYTheta(kRoomA.x() + 40.0, kRoomA.y(), 0.0);
  const FrozenCoverage::Overlap overlap = coverage_.Query(moved);
  EXPECT_GT(overlap.cells, 0);
  EXPECT_EQ(overlap.covered, 0);
}

TEST_F(FrozenCoverageTest, AnEmptyFootprintCountsAsCovered) {
  coverage_.Rebuild(graph_);
  const FrozenCoverage::Overlap empty = coverage_.Query(MakeRecord(live_, 1, kRoomA, {}));
  EXPECT_EQ(empty.cells, 0);
  EXPECT_DOUBLE_EQ(empty.covered_fraction(), 1.0);
  EXPECT_DOUBLE_EQ(empty.novel_area(kCoarseResolution), 0.0);
}

TEST_F(FrozenCoverageTest, AddSessionExtendsTheLayer) {
  PoseGraphData graph;
  const SessionId first = graph.StartNewSession(TestTime(0));
  const SessionId second = graph.StartNewSession(TestTime(1));
  const SubmapRecord in_a = AddRecord(graph, first, 0, kRoomA);
  const SubmapRecord in_b = AddRecord(graph, second, 0, kRoomB);
  graph.FreezeSession(first);

  FrozenCoverage coverage(kCoarseResolution, kTrackResolution);
  coverage.Rebuild(graph);
  EXPECT_EQ(coverage.size(), Cells(in_a).size());
  EXPECT_EQ(coverage.Query(in_b).covered, 0);

  graph.FreezeSession(second);
  coverage.AddSession(graph, second);
  EXPECT_EQ(coverage.size(), Cells(in_a).size() + Cells(in_b).size());
  EXPECT_DOUBLE_EQ(coverage.Query(in_b).covered_fraction(), 1.0);
  EXPECT_DOUBLE_EQ(coverage.Query(in_a).covered_fraction(), 1.0);
}

// The track is the frozen sessions' node cells, read at the global pose.
TEST_F(FrozenCoverageTest, TheTrackHoldsTheFrozenNodesAndNotTheLiveOnes) {
  PoseGraphData graph;
  const SessionId frozen = graph.StartNewSession(TestTime(0));
  const SessionId live = graph.StartNewSession(TestTime(1));
  const Eigen::Vector2d frozen_at(5.213, -1.107);
  const Eigen::Vector2d live_at(-20.019, 8.421);
  const auto add_node = [&graph](SessionId session, const Eigen::Vector2d& at) {
    Node node;
    node.id = NodeId{session.session_index, 0};
    node.global_pose = transform::FromXYTheta(at.x(), at.y(), 0.0);
    graph.AddNode(node, {});
  };
  add_node(frozen, frozen_at);
  add_node(live, live_at);
  graph.FreezeSession(frozen);

  FrozenCoverage coverage(kCoarseResolution, kTrackResolution);
  coverage.Rebuild(graph);
  EXPECT_EQ(coverage.track_size(), 1u);
  EXPECT_TRUE(coverage.ContainsTrack(CoarseCellOf(frozen_at, kTrackResolution)));
  EXPECT_FALSE(coverage.ContainsTrack(CoarseCellOf(live_at, kTrackResolution)));
  EXPECT_FALSE(coverage.ContainsTrack(
      CoarseCellOf(frozen_at + Eigen::Vector2d(2.0 * kTrackResolution, 0.0), kTrackResolution)));
  EXPECT_DOUBLE_EQ(coverage.area(), 0.0) << "no grids, no area";

  graph.FreezeSession(live);
  coverage.AddSession(graph, live);
  EXPECT_EQ(coverage.track_size(), 2u);
  EXPECT_TRUE(coverage.ContainsTrack(CoarseCellOf(live_at, kTrackResolution)));
}

TEST_F(FrozenCoverageTest, GridlessRecordsAreSkipped) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  SubmapRecord record;
  record.id = SubmapId{session.session_index, 0};
  graph.AddSubmap(record);
  graph.FreezeSession(session);

  FrozenCoverage coverage(kCoarseResolution, kTrackResolution);
  coverage.Rebuild(graph);
  EXPECT_EQ(coverage.size(), 0u);
}

}  // namespace
}  // namespace evergreenslam::lifelong

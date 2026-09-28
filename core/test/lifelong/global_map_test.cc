/**
 * @file global_map_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The composite rule where submaps overlap, and the invariants the callers rely on.
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/global_map.h"

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <vector>

#include "common/time.h"
#include "mapping/grid_mapping/probability_values.h"
#include "mapping/submap.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;
// Off the resolution grid on purpose.
const Eigen::Vector2d kProbe(1.237, 0.813);

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

struct Observation {
  Eigen::Vector2d point;
  int num_hits = 0;
  int num_misses = 0;
};

std::shared_ptr<mapping::Submap> MakeSubmap(const SubmapId& id, const Eigen::Affine2d& local_pose,
                                            const std::vector<Observation>& observations,
                                            bool finished = true) {
  mapping::ProbabilityGrid grid(kResolution);
  const std::vector<uint8_t> hit_table =
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(mapping::kDefaultHitProbability));
  const std::vector<uint8_t> miss_table =
      mapping::ComputeLookupTableToApplyOdds(mapping::Odds(mapping::kDefaultMissProbability));
  for (const Observation& observation : observations) {
    grid.GrowToInclude(observation.point, observation.point);
    for (int i = 0; i < observation.num_hits; ++i) {
      grid.ApplyLookupTable(grid.ToCell(observation.point), hit_table);
      grid.FinishUpdate();
    }
    for (int i = 0; i < observation.num_misses; ++i) {
      grid.ApplyLookupTable(grid.ToCell(observation.point), miss_table);
      grid.FinishUpdate();
    }
  }
  return std::make_shared<mapping::Submap>(id.submap_index, local_pose, std::move(grid), 1,
                                           finished);
}

// The grid is in the submap frame, so content lands at global_pose * grid_point.
void AddSubmap(PoseGraphData& graph, SessionId session,
               const std::shared_ptr<mapping::Submap>& submap, const Eigen::Affine2d& global_pose) {
  SubmapRecord record;
  record.id = SubmapId{session.session_index, submap->local_index()};
  record.submap = submap;
  record.local_pose = submap->local_pose();
  record.global_pose = global_pose;
  graph.AddSubmap(record);
}

double ProbabilityAt(const mapping::GridMapu8& map, const Eigen::Vector2d& point) {
  const uint8_t value = map.GetValueAtPoint(point);
  EXPECT_TRUE(mapping::IsKnownValue(value)) << "cell is unknown at " << point.transpose();
  return mapping::ValueToProbability(value);
}

TEST(GlobalMap, AGrazeFromALaterSubmapDoesNotEraseAConvergedWall) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);

  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 60, 0}}),
            pose);
  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 0, 2}}),
            pose);

  EXPECT_GT(ProbabilityAt(AssembleGlobalMap(graph), kProbe), 0.8);
}

TEST(GlobalMap, ASweptReObservationDoesEraseAWall) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);

  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 60, 0}}),
            pose);
  // The obstacle is gone and the robot drove through where it stood: newest wins, not max.
  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 0, 60}}),
            pose);

  EXPECT_LT(ProbabilityAt(AssembleGlobalMap(graph), kProbe), 0.2);
}

TEST(GlobalMap, EquallyCertainCandidatesGoToTheNewest) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);

  // Both converge to the clamp, so the tie decides: a second session's observation is newer.
  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 60, 0}}),
            pose);
  const SessionId later = graph.StartNewSession(TestTime(100));
  AddSubmap(graph, later, MakeSubmap(graph.AllocateSubmapId(later), pose, {{kProbe, 0, 60}}), pose);

  EXPECT_LT(ProbabilityAt(AssembleGlobalMap(graph), kProbe), 0.2);
}

TEST(GlobalMap, UnknownNeverOverwritesKnown) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);

  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 60, 0}}),
            pose);
  AddSubmap(graph, session,
            MakeSubmap(graph.AllocateSubmapId(session), pose,
                       {{kProbe + Eigen::Vector2d(-1.4, 0.0), 0, 30},
                        {kProbe + Eigen::Vector2d(1.4, 0.0), 0, 30}}),
            pose);

  EXPECT_GT(ProbabilityAt(AssembleGlobalMap(graph), kProbe), 0.8);
}

TEST(GlobalMap, ContentFollowsTheOptimizedPose) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d local_pose = transform::FromXYTheta(0.3, -0.2, 0.4);
  const Eigen::Affine2d global_pose = transform::FromXYTheta(-2.1, 1.7, -0.9);

  AddSubmap(graph, session,
            MakeSubmap(graph.AllocateSubmapId(session), local_pose, {{kProbe, 60, 0}}),
            global_pose);

  // The local pose plays no part: the grid holds the probe in the submap frame already.
  const mapping::GridMapu8 map = AssembleGlobalMap(graph);
  const Eigen::Vector2d moved = global_pose * kProbe;
  EXPECT_GT(ProbabilityAt(map, moved), 0.8);
  EXPECT_FALSE(mapping::IsKnownValue(map.GetValueAtPoint(kProbe)));
  EXPECT_FALSE(
      mapping::IsKnownValue(map.GetValueAtPoint(global_pose * local_pose.inverse() * kProbe)));
}

TEST(GlobalMap, OnlyFinishedSkipsUnfinishedSubmaps) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);

  AddSubmap(graph, session,
            MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 60, 0}}, false), pose);

  EXPECT_TRUE(mapping::IsKnownValue(AssembleGlobalMap(graph).GetValueAtPoint(kProbe)));
  const mapping::GridMapu8 finished_only = AssembleGlobalMap(graph, 0.0, true);
  EXPECT_EQ(finished_only.width(), 0);
  EXPECT_EQ(finished_only.height(), 0);
}

// The backend assembles with only_finished, so its map differs from a full assembly wherever
// the dropped submap would have won the cell.
TEST(GlobalMap, OnlyFinishedDropsAMoreCertainUnfinishedCandidate) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);

  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 0, 2}}),
            pose);
  AddSubmap(graph, session,
            MakeSubmap(graph.AllocateSubmapId(session), pose, {{kProbe, 60, 0}}, false), pose);

  EXPECT_GT(ProbabilityAt(AssembleGlobalMap(graph), kProbe), 0.8);
  EXPECT_LT(ProbabilityAt(AssembleGlobalMap(graph, 0.0, true), kProbe), 0.6);
}

TEST(GlobalMap, ResolutionDefaultsToTheSubmapsAndCanBeOverridden) {
  PoseGraphData graph;
  const SessionId session = graph.StartNewSession(TestTime(0));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);
  // A patch rather than one cell: resampling is nearest neighbour.
  std::vector<Observation> patch;
  for (int y = -4; y <= 4; ++y) {
    for (int x = -4; x <= 4; ++x) {
      patch.push_back({kProbe + kResolution * Eigen::Vector2d(x, y), 60, 0});
    }
  }
  AddSubmap(graph, session, MakeSubmap(graph.AllocateSubmapId(session), pose, patch), pose);

  EXPECT_DOUBLE_EQ(AssembleGlobalMap(graph).resolution(), kResolution);
  const mapping::GridMapu8 coarse = AssembleGlobalMap(graph, 4 * kResolution);
  EXPECT_DOUBLE_EQ(coarse.resolution(), 4 * kResolution);
  EXPECT_LT(coarse.width(), AssembleGlobalMap(graph).width());
  EXPECT_GT(ProbabilityAt(coarse, kProbe), 0.8);
}

TEST(GlobalMap, SessionFilterAssemblesThatSessionAlone) {
  PoseGraphData graph;
  const SessionId first = graph.StartNewSession(TestTime(0));
  const SessionId second = graph.StartNewSession(TestTime(1));
  const Eigen::Affine2d pose = transform::FromXYTheta(0.0, 0.0, 0.0);
  const Eigen::Affine2d far = transform::FromXYTheta(20.0, 0.0, 0.0);
  AddSubmap(graph, first, MakeSubmap(graph.AllocateSubmapId(first), pose, {{kProbe, 60, 0}}), pose);
  AddSubmap(graph, second, MakeSubmap(graph.AllocateSubmapId(second), pose, {{kProbe, 60, 0}}),
            far);

  const mapping::GridMapu8 all = AssembleGlobalMap(graph, 0.0, true, std::nullopt);
  EXPECT_TRUE(mapping::IsKnownValue(all.GetValueAtPoint(kProbe)));
  EXPECT_TRUE(mapping::IsKnownValue(all.GetValueAtPoint(far * kProbe)));

  const mapping::GridMapu8 only_second = AssembleGlobalMap(graph, 0.0, true, second);
  EXPECT_TRUE(mapping::IsKnownValue(only_second.GetValueAtPoint(far * kProbe)));
  EXPECT_FALSE(mapping::IsKnownValue(only_second.GetValueAtPoint(kProbe)));
  EXPECT_LT(only_second.width(), all.width()) << "bounds come from the session's submaps alone";

  const mapping::GridMapu8 absent = AssembleGlobalMap(graph, 0.0, true, SessionId{7});
  EXPECT_EQ(absent.width(), 0);
  EXPECT_EQ(absent.height(), 0);
}

TEST(GlobalMap, EmptyGraphIsZeroSized) {
  PoseGraphData graph;
  graph.StartNewSession(TestTime(0));
  const mapping::GridMapu8 map = AssembleGlobalMap(graph);
  EXPECT_EQ(map.width(), 0);
  EXPECT_EQ(map.height(), 0);
}

}  // namespace
}  // namespace evergreenslam::lifelong

/**
 * @file relocalization_round_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief A hinted round reports every selected candidate exactly once, inline or from workers.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "../testing/loop_scenario.h"
#include "lifelong/constraints/constraint_builder.h"
#include "lifelong/pose_graph.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

using testing::LoopGroundTruth;
using testing::LoopRoom;
using testing::SimulateScan;
using testing::TestTime;

struct Report {
  bool accepted = false;
  bool after_return = false;
};

// A finished room submap in one session; a node of a second session at `kQueryStep`.
struct TwoSessions {
  static constexpr int kQueryStep = 5;

  TwoSessions() {
    const mapping::CastRaysMapping inserter;
    const SessionId base = pose_graph.StartNewSession(TestTime(0));
    auto room = std::make_shared<mapping::Submap>(0, LoopGroundTruth(0), testing::kResolution);
    for (int step = 0; step < 12; ++step) {
      room->InsertScan(LoopGroundTruth(step), SimulateScan(LoopRoom(), LoopGroundTruth(step)),
                       inserter);
    }
    room->Finish();
    testing::ExplicitInsertion base_node;
    base_node.node_id = NodeId{base.session_index, 0};
    base_node.node.time = TestTime(0);
    base_node.node.local_pose = LoopGroundTruth(0);
    base_node.node.point_cloud = SimulateScan(LoopRoom(), LoopGroundTruth(0));
    base_node.insertion_submaps = {{SubmapId{base.session_index, 0}, room}};
    testing::EnqueueExplicitInsertion(pose_graph, base_node);
    pose_graph.WaitUntilQuiescent();

    const SessionId fed = pose_graph.StartNewSession(TestTime(1));
    const Eigen::Affine2d truth = LoopGroundTruth(kQueryStep);
    query = NodeId{fed.session_index, 0};
    testing::ExplicitInsertion query_node;
    query_node.node_id = query;
    query_node.node.time = TestTime(1);
    query_node.node.local_pose = truth;
    query_node.node.point_cloud = SimulateScan(LoopRoom(), truth);
    query_node.insertion_submaps = {
        {SubmapId{fed.session_index, 0},
         std::make_shared<const mapping::Submap>(0, truth, testing::kResolution)}};
    testing::EnqueueExplicitInsertion(pose_graph, query_node);
    pose_graph.WaitUntilQuiescent();
  }

  PoseGraph pose_graph;
  NodeId query;
};

std::vector<Report> RunRound(int num_match_workers, const std::optional<Eigen::Affine2d>& prior,
                             int& selected) {
  TwoSessions world;
  ConstraintBuilderOption option;
  option.num_match_workers = num_match_workers;
  ConstraintBuilder builder(world.pose_graph, option);
  std::vector<Report> reports;
  bool returned = false;
  world.pose_graph.Enqueue([&] {
    selected = builder.SearchForNodeAroundOnTask(
        world.query, prior, -1,
        [&reports, &returned](bool accepted) { reports.push_back({accepted, returned}); });
    returned = true;
  });
  world.pose_graph.WaitUntilQuiescent();
  builder.WaitForMatches();
  world.pose_graph.WaitUntilQuiescent();
  return reports;
}

const Eigen::Affine2d kPrior = Eigen::Affine2d(LoopGroundTruth(TwoSessions::kQueryStep) *
                                               transform::FromXYTheta(0.3, -0.2, 0.1));

TEST(RelocalizationRoundE2eTest, InlineMatchesReportBeforeTheRoundReturns) {
  int selected = -1;
  const std::vector<Report> reports = RunRound(0, kPrior, selected);
  ASSERT_EQ(selected, 1);
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_TRUE(reports[0].accepted);
  EXPECT_FALSE(reports[0].after_return);
}

TEST(RelocalizationRoundE2eTest, WorkerMatchesReportOnTheBackendTaskAfterTheRoundReturns) {
  int selected = -1;
  const std::vector<Report> reports = RunRound(2, kPrior, selected);
  ASSERT_EQ(selected, 1);
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_TRUE(reports[0].accepted);
  EXPECT_TRUE(reports[0].after_return);
}

TEST(RelocalizationRoundE2eTest, APriorOutsideEverySubmapSelectsNothingAndReportsNothing) {
  for (const int workers : {0, 2}) {
    int selected = -1;
    const std::vector<Report> reports =
        RunRound(workers, transform::FromXYTheta(300.0, 40.0, 0.0), selected);
    EXPECT_EQ(selected, 0);
    EXPECT_TRUE(reports.empty());
  }
}

}  // namespace
}  // namespace evergreenslam::lifelong

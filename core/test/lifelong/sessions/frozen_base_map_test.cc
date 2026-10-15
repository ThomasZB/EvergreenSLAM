/**
 * @file frozen_base_map_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief A simulated lap frozen through the real wiring, then scans scored against it at the true
 * pose, off it, outside the map and with an object the map never saw.
 * @version 0.1
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/frozen_base_map.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "../testing/loop_scenario.h"
#include "lifelong/pose_graph.h"
#include "lifelong/pose_graph_option.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;
using testing::Frontend;
using testing::kNodesPerLap;
using testing::LoopGroundTruth;
using testing::LoopRoom;
using testing::LoopTruth;
using testing::SimulateScan;
using testing::TestTime;
using testing::World;

PoseGraphOption Option() {
  PoseGraphOption option =
      LoadPoseGraphOptionFromFile(std::string(EVERGREENSLAM_CONFIG_DIR) + "/evergreenslam.yaml");
  option.constraint_builder.num_match_workers = 1;
  option.session_manager.auto_freeze = false;
  option.trim = false;
  return option;
}

FrozenBaseMap::Agreement Evaluate(const PoseGraph& backend, const World& world,
                                  const Eigen::Affine2d& truth, const Eigen::Affine2d& claimed) {
  return backend.session_manager().base_map().Evaluate(
      sensor::TransformPointCloud(SimulateScan(world, truth), claimed));
}

class FrozenBaseMapE2eTest : public ::testing::Test {
 protected:
  void FreezeOneLap(PoseGraph& backend) {
    backend.Start(TestTime(0));
    Frontend frontend(backend, LoopTruth(kNodesPerLap));
    for (int step = 0; step < kNodesPerLap; ++step) {
      frontend.Feed(step);
    }
    backend.WaitUntilQuiescent();
    ASSERT_TRUE(backend.session_manager().base_map().grid().data().empty());
    backend.FreezeFedSession();
    backend.WaitUntilQuiescent();
    ASSERT_EQ(backend.session_manager().num_sessions_frozen(), 1);
  }

  const Eigen::Affine2d truth_ = LoopGroundTruth(20);
};

TEST_F(FrozenBaseMapE2eTest, TheTruePoseAgreesAndAnOffsetPoseDoesNot) {
  PoseGraph backend(Option());
  FreezeOneLap(backend);

  const FrozenBaseMap::Agreement at_truth = Evaluate(backend, LoopRoom(), truth_, truth_);
  ASSERT_TRUE(at_truth.score.has_value());
  EXPECT_GT(at_truth.coverage, 0.95);
  EXPECT_GT(*at_truth.score, 0.75);

  const Eigen::Affine2d shifted = truth_ * transform::FromXYTheta(0.5, 0.0, 0.0);
  const Eigen::Affine2d turned = truth_ * transform::FromXYTheta(0.0, 0.0, 10.0 * M_PI / 180.0);
  for (const Eigen::Affine2d& claimed : {shifted, turned}) {
    const FrozenBaseMap::Agreement off = Evaluate(backend, LoopRoom(), truth_, claimed);
    ASSERT_TRUE(off.score.has_value());
    EXPECT_LT(*off.score, *at_truth.score - 0.15);
  }
}

TEST_F(FrozenBaseMapE2eTest, OutsideTheMapNothingCanBeChecked) {
  PoseGraph backend(Option());
  FreezeOneLap(backend);
  const FrozenBaseMap::Agreement outside =
      Evaluate(backend, LoopRoom(), truth_, transform::FromXYTheta(100.0, 100.0, 0.0) * truth_);
  EXPECT_DOUBLE_EQ(outside.coverage, 0.0);
  EXPECT_FALSE(outside.score.has_value());
  EXPECT_FALSE(
      backend.session_manager().base_map().Evaluate(sensor::PointCloud()).score.has_value());
}

TEST_F(FrozenBaseMapE2eTest, AnObjectTheMapNeverSawLowersTheScoreButLeavesItDefined) {
  PoseGraph backend(Option());
  FreezeOneLap(backend);
  World moved = LoopRoom();
  // A cart parked inside the loop, on free space every lap scan saw through.
  moved.pillars.emplace_back(Eigen::Vector2d(7.81, 4.12), Eigen::Vector2d(8.63, 5.94));

  const FrozenBaseMap::Agreement clean = Evaluate(backend, LoopRoom(), truth_, truth_);
  const FrozenBaseMap::Agreement cluttered = Evaluate(backend, moved, truth_, truth_);
  ASSERT_TRUE(cluttered.score.has_value());
  EXPECT_GT(cluttered.coverage, 0.95);
  EXPECT_LT(*cluttered.score, *clean.score - 0.05);
}

TEST_F(FrozenBaseMapE2eTest, ABootRebuildsItFromTheLoadedFrozenSessions) {
  const std::string directory = testing::MakeTempDir("evergreenslam_frozen_base_map");
  FrozenBaseMap::Agreement before;
  {
    PoseGraph backend(Option(), directory);
    FreezeOneLap(backend);
    before = Evaluate(backend, LoopRoom(), truth_, truth_);
    backend.Finish();
  }
  PoseGraph rebooted(Option(), directory);
  rebooted.Start(TestTime(1000));
  rebooted.WaitUntilQuiescent();
  const FrozenBaseMap::Agreement after = Evaluate(rebooted, LoopRoom(), truth_, truth_);
  ASSERT_TRUE(after.score.has_value());
  EXPECT_DOUBLE_EQ(after.coverage, before.coverage);
  EXPECT_DOUBLE_EQ(*after.score, *before.score);
}

}  // namespace
}  // namespace evergreenslam::lifelong

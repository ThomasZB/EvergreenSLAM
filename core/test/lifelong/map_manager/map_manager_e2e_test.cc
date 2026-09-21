/**
 * @file map_manager_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Simulated laser, checkpoint, kill, load, keep mapping: the power loss story of M5.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "../testing/explicit_ingest.h"
#include "common/time.h"
#include "lifelong/constraints/constraint_builder.h"
#include "lifelong/map_manager/map_manager.h"
#include "lifelong/pose_graph.h"
#include "lifelong/sessions/session_manager.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

constexpr double kResolution = 0.05;
constexpr int kNodesPerSubmap = 10;
constexpr int kNumBeams = 360;
constexpr int kNumLoopNodes = 80;
constexpr int kNodesBeforeTheKill = 50;
constexpr double kLoopRadius = 3.1;
constexpr double kMaxRange = 12.0;

PoseGraphOption NoFreezeOption() {
  PoseGraphOption option;
  option.session_manager.auto_freeze = false;
  return option;
}

std::string MakeTempDir() {
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                     ("evergreenslam_map_e2e_" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path.string();
}

common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

struct World {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  std::vector<Eigen::AlignedBox2d> pillars;
};

double RangeToBox(const Eigen::Vector2d& origin, double angle, const Eigen::AlignedBox2d& box) {
  const Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
  double t_min = 0.0;
  double t_max = std::numeric_limits<double>::max();
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(direction[axis]) < 1e-9) {
      if (origin[axis] < box.min()[axis] || origin[axis] > box.max()[axis]) {
        return std::numeric_limits<double>::max();
      }
      continue;
    }
    double t1 = (box.min()[axis] - origin[axis]) / direction[axis];
    double t2 = (box.max()[axis] - origin[axis]) / direction[axis];
    if (t1 > t2) {
      std::swap(t1, t2);
    }
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
  }
  if (t_min > t_max || t_min <= 0.0) {
    return std::numeric_limits<double>::max();
  }
  return t_min;
}

double RangeToBoundary(const World& world, const Eigen::Vector2d& origin, double angle) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double range = std::numeric_limits<double>::max();
  if (dx > 1e-6) {
    range = std::min(range, (world.max_x - origin.x()) / dx);
  } else if (dx < -1e-6) {
    range = std::min(range, (world.min_x - origin.x()) / dx);
  }
  if (dy > 1e-6) {
    range = std::min(range, (world.max_y - origin.y()) / dy);
  } else if (dy < -1e-6) {
    range = std::min(range, (world.min_y - origin.y()) / dy);
  }
  for (const Eigen::AlignedBox2d& pillar : world.pillars) {
    range = std::min(range, RangeToBox(origin, angle, pillar));
  }
  return range;
}

sensor::PointCloud SimulateScan(const World& world, const Eigen::Affine2d& world_pose) {
  const double yaw = transform::GetYaw(world_pose);
  sensor::PointCloud cloud;
  for (int i = 0; i < kNumBeams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / kNumBeams;
    const double range = RangeToBoundary(world, world_pose.translation(), yaw + bearing);
    if (range > kMaxRange) {
      continue;
    }
    cloud.push_back(
        sensor::Point2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing))});
  }
  return cloud;
}

// Wall coordinates deliberately off the resolution lattice.
const World kLoopRoom{
    0.013,
    13.007,
    0.021,
    10.003,
    {Eigen::AlignedBox2d(Eigen::Vector2d(1.51, 1.52), Eigen::Vector2d(2.32, 2.33)),
     Eigen::AlignedBox2d(Eigen::Vector2d(10.61, 7.52), Eigen::Vector2d(11.42, 8.33)),
     Eigen::AlignedBox2d(Eigen::Vector2d(6.01, 4.62), Eigen::Vector2d(6.82, 5.43))}};

Eigen::Affine2d LoopGroundTruth(int index) {
  const double phi = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(kNumLoopNodes);
  return transform::FromXYTheta(6.4 + kLoopRadius * std::sin(phi),
                                1.9 + kLoopRadius * (1.0 - std::cos(phi)), phi);
}

std::vector<Eigen::Affine2d> DriftedLoopOdometry(double yaw_bias_per_step) {
  std::vector<Eigen::Affine2d> poses;
  poses.push_back(LoopGroundTruth(0));
  for (int i = 1; i < kNumLoopNodes; ++i) {
    const Eigen::Affine2d increment =
        Eigen::Affine2d(LoopGroundTruth(i - 1).inverse() * LoopGroundTruth(i));
    poses.push_back(Eigen::Affine2d(poses.back() * increment *
                                    transform::FromXYTheta(0.0, 0.0, yaw_bias_per_step)));
  }
  return poses;
}

// After a restart the frontend has no submap objects left, so it opens fresh ones from the ids
// the restored graph hands out; that is what `restart_at` reproduces.
class Feeder {
 public:
  Feeder(PoseGraph& pose_graph, SessionId session, const std::vector<Eigen::Affine2d>& odometry)
      : pose_graph_(&pose_graph), session_(session), odometry_(odometry) {}

  void Restart(PoseGraph& pose_graph) {
    pose_graph_ = &pose_graph;
    submaps_.clear();
    submap_ids_.clear();
    restarted_ = true;
  }

  void Feed(int index, ConstraintBuilder* constraint_builder) {
    PoseGraphData& graph = pose_graph_->mutable_graph();
    if (restarted_ && submaps_.empty()) {
      AddSubmap(graph.AllocateSubmapId(session_), odometry_[index]);
      fed_since_restart_ = 0;
    }
    const int newest = fed_since_restart_ / kNodesPerSubmap;
    while (static_cast<int>(submaps_.size()) <= newest) {
      AddSubmap(graph.AllocateSubmapId(session_), odometry_[index]);
      const int k = static_cast<int>(submaps_.size()) - 1;
      if (k >= 2) {
        submaps_[k - 2]->Finish();
      }
    }

    const sensor::PointCloud cloud = SimulateScan(kLoopRoom, LoopGroundTruth(index));
    testing::ExplicitInsertion insertion;
    insertion.node_id = graph.AllocateNodeId(session_);
    insertion.node.time = TestTime(index);
    insertion.node.local_pose = odometry_[index];
    insertion.node.point_cloud = cloud;
    for (int k = std::max(0, newest - 1); k <= newest; ++k) {
      submaps_[k]->InsertScan(odometry_[index], cloud, inserter_);
      insertion.insertion_submaps.emplace_back(submap_ids_[k], submaps_[k]);
    }
    last_node_id_ = insertion.node_id;
    testing::EnqueueExplicitInsertion(*pose_graph_, std::move(insertion));
    pose_graph_->WaitUntilQuiescent();
    if (constraint_builder != nullptr) {
      constraint_builder->SearchForNode(last_node_id_);
      pose_graph_->WaitUntilQuiescent();
    }
    ++fed_since_restart_;
  }

  void FinishAll() {
    for (const auto& submap : submaps_) {
      if (!submap->finished()) {
        submap->Finish();
      }
    }
  }

  const NodeId& last_node_id() const { return last_node_id_; }

 private:
  void AddSubmap(const SubmapId& id, const Eigen::Affine2d& local_pose) {
    submaps_.push_back(std::make_shared<mapping::Submap>(id.submap_index, local_pose, kResolution));
    submap_ids_.push_back(id);
  }

  PoseGraph* pose_graph_;
  SessionId session_;
  std::vector<Eigen::Affine2d> odometry_;
  mapping::CastRaysMapping inserter_;
  std::vector<std::shared_ptr<mapping::Submap>> submaps_;
  std::vector<SubmapId> submap_ids_;
  bool restarted_ = false;
  int fed_since_restart_ = 0;
  NodeId last_node_id_;
};

double MeanNodeError(const PoseGraphData& graph) {
  double total = 0.0;
  int count = 0;
  for (const auto& [id, node] : graph.nodes()) {
    total += (node.global_pose.translation() - LoopGroundTruth(id.node_index).translation()).norm();
    ++count;
  }
  return count == 0 ? 0.0 : total / static_cast<double>(count);
}

TEST(MapManagerE2eTest, CheckpointSurvivesAKillAndMappingContinuesOnTheRestoredMap) {
  const std::string directory = MakeTempDir();
  const std::vector<Eigen::Affine2d> odometry = DriftedLoopOdometry(0.0035);
  auto manager = std::make_shared<MapManager>(directory);

  int submaps_before = 0;
  int nodes_before = 0;
  double error_before = 0.0;
  {
    // The story is checkpoint/kill/restore of an unfrozen session, so freezing is held back.
    PoseGraph pose_graph(NoFreezeOption());
    const SessionId session = pose_graph.StartNewSession(TestTime(0));
    pose_graph.WaitUntilQuiescent();
    Feeder feeder(pose_graph, session, odometry);
    for (int i = 0; i < kNodesBeforeTheKill; ++i) {
      feeder.Feed(i, nullptr);
    }
    feeder.FinishAll();
    pose_graph.Optimize();
    pose_graph.WaitUntilQuiescent();

    manager->Checkpoint(pose_graph.graph(), session);
    submaps_before = static_cast<int>(pose_graph.graph().submaps().size());
    nodes_before = static_cast<int>(pose_graph.graph().nodes().size());
    error_before = MeanNodeError(pose_graph.graph());
    EXPECT_EQ(manager->num_checkpoints_written(), 1);
  }

  PoseGraph restarted(NoFreezeOption());
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(restarted.mutable_graph());
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
  const SessionId session = result->unfrozen_sessions.front().id;
  EXPECT_EQ(result->unfrozen_sessions.front().next_node_index, kNodesBeforeTheKill);
  EXPECT_EQ(result->unfrozen_sessions.front().next_submap_index, submaps_before);
  EXPECT_EQ(result->num_sessions, 1);
  EXPECT_EQ(result->num_frozen_sessions, 0);
  EXPECT_EQ(static_cast<int>(restarted.graph().submaps().size()), submaps_before);
  EXPECT_EQ(static_cast<int>(restarted.graph().nodes().size()), nodes_before);
  EXPECT_NEAR(MeanNodeError(restarted.graph()), error_before, 1e-9);
  ASSERT_TRUE(result->last_checkpoint_node.has_value());
  EXPECT_EQ(*result->last_checkpoint_node,
            (NodeId{session.session_index, kNodesBeforeTheKill - 1}));
  ASSERT_TRUE(result->last_checkpoint_global_pose.has_value());
  EXPECT_LT((result->last_checkpoint_global_pose->translation() -
             restarted.graph().node(*result->last_checkpoint_node).global_pose.translation())
                .norm(),
            1e-15);

  restarted.mutable_optimization().BuildFrom(restarted.mutable_graph());
  EXPECT_EQ(restarted.optimization().num_variables(), submaps_before + nodes_before);
  EXPECT_GT(restarted.optimization().num_residual_blocks(), 0);

  // No id may be reissued across the restart.
  const NodeId next_node = restarted.mutable_graph().AllocateNodeId(session);
  EXPECT_EQ(next_node.node_index, kNodesBeforeTheKill);
  const SubmapId next_submap = restarted.mutable_graph().AllocateSubmapId(session);
  EXPECT_EQ(next_submap.submap_index, submaps_before);

  Feeder feeder(restarted, session, odometry);
  feeder.Restart(restarted);
  ConstraintBuilder constraint_builder(restarted);
  for (int i = kNodesBeforeTheKill; i < kNumLoopNodes; ++i) {
    feeder.Feed(i, &constraint_builder);
  }
  feeder.FinishAll();
  restarted.Optimize();
  restarted.WaitUntilQuiescent();

  EXPECT_EQ(static_cast<int>(restarted.graph().nodes().size()),
            nodes_before + (kNumLoopNodes - kNodesBeforeTheKill));
  // The grids that came back off disk must be the ones written; a garbled grid matches nothing.
  EXPECT_GT(constraint_builder.num_constraints_added(), 0)
      << "no loop closure found against the restored submaps";
  for (int i = 0; i < kNodesBeforeTheKill; ++i) {
    EXPECT_TRUE(restarted.graph().HasNode(NodeId{session.session_index, i}));
  }
  // The probe above consumed an index; it is gone for good rather than handed out again.
  EXPECT_FALSE(restarted.graph().HasNode(next_node));
  EXPECT_FALSE(restarted.graph().HasSubmap(next_submap));
  const double error_after = MeanNodeError(restarted.graph());
  EXPECT_LT(error_after, 0.25) << "the trajectory after the restart is not sane";
  EXPECT_LT(error_after, error_before + 0.1);
  manager->Checkpoint(restarted.graph(), session);
  EXPECT_TRUE(reader.InspectSessionFile(session).has_value());
}

TEST(MapManagerE2eTest, TheFrozenFileIsWrittenFromTheFreezeCallback) {
  const std::string directory = MakeTempDir();
  const std::vector<Eigen::Affine2d> odometry = DriftedLoopOdometry(0.0);
  auto manager = std::make_shared<MapManager>(directory);

  PoseGraph pose_graph(NoFreezeOption());
  SessionManagerOption option;
  option.auto_freeze = false;
  SessionManager session_manager(pose_graph, option);
  session_manager.AddObserver(manager);
  const SessionId first = session_manager.Start(TestTime(0));
  pose_graph.WaitUntilQuiescent();

  Feeder feeder(pose_graph, first, odometry);
  for (int i = 0; i < 25; ++i) {
    feeder.Feed(i, nullptr);
  }
  feeder.FinishAll();
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  session_manager.FreezeFedSession();
  pose_graph.WaitUntilQuiescent();

  ASSERT_EQ(session_manager.num_sessions_frozen(), 1);
  EXPECT_EQ(manager->num_frozen_files_written(), 1);

  const std::optional<MapManager::FileSummary> summary = manager->InspectSessionFile(first);
  ASSERT_TRUE(summary.has_value());
  EXPECT_TRUE(summary->frozen);
  EXPECT_EQ(summary->num_nodes, 25);
  // Stored, not loaded: every constraint the frozen session owns has to be in its file.
  EXPECT_EQ(summary->num_constraints, static_cast<int>(pose_graph.graph().constraints().size()));
  EXPECT_GT(summary->num_constraints, 0);

  PoseGraphData loaded;
  MapManager reader(directory);
  const std::optional<MapManager::LoadResult> result = reader.Load(loaded);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->num_sessions, 2);
  EXPECT_EQ(result->num_frozen_sessions, 1);
  EXPECT_TRUE(loaded.session(first).frozen());
  EXPECT_TRUE(loaded.constraints().empty()) << "frozen constraints are stored, not loaded";
  EXPECT_EQ(loaded.session(first).node_ids.size(), 25u);
  ASSERT_EQ(result->unfrozen_sessions.size(), 1u);
  EXPECT_TRUE(loaded.session(result->unfrozen_sessions.front().id).node_ids.empty());
  EXPECT_FALSE(result->last_checkpoint_node.has_value())
      << "an empty replacement session leaves nothing to inherit an alignment from";
}

// A submap and a node born and trimmed between two checkpoints leave no trace in the surviving
// records, so only the persisted high water marks keep their indices retired.
TEST(MapManagerE2eTest, IndicesTrimmedBetweenCheckpointsAreNeverReissued) {
  const std::string directory = MakeTempDir();
  const std::vector<Eigen::Affine2d> odometry = DriftedLoopOdometry(0.0);
  MapManager manager(directory);

  PoseGraph pose_graph;
  const SessionId session = pose_graph.StartNewSession(TestTime(0));
  pose_graph.WaitUntilQuiescent();
  Feeder feeder(pose_graph, session, odometry);
  for (int i = 0; i < 20; ++i) {
    feeder.Feed(i, nullptr);
  }
  pose_graph.Optimize();
  pose_graph.WaitUntilQuiescent();
  manager.Checkpoint(pose_graph.graph(), session);

  const int submaps_before =
      static_cast<int>(pose_graph.graph().session(session).submap_ids.size());
  const int nodes_before = static_cast<int>(pose_graph.graph().session(session).node_ids.size());

  for (int i = 20; i < 40; ++i) {
    feeder.Feed(i, nullptr);
  }
  feeder.FinishAll();
  const std::vector<SubmapId> live = pose_graph.graph().session(session).submap_ids;
  ASSERT_EQ(live.size(), 4u);

  TrimRequest request;
  request.deleted_submap_ids.push_back(live[2]);
  request.deleted_submap_ids.push_back(live[3]);
  request.successors[live[2]] = live[1];
  request.successors[live[3]] = live[1];
  // Trimming recovers node-to-node constraints, which are INTER_SUBMAP whatever their endpoints.
  Constraint recovered;
  recovered.type = Constraint::Type::INTER_SUBMAP;
  recovered.from = VariableId::Of(pose_graph.graph().session(session).node_ids.front());
  recovered.to = VariableId::Of(pose_graph.graph().session(session).node_ids[1]);
  recovered.relative_pose = transform::FromXYTheta(0.37, -0.19, 0.083);
  recovered.sqrt_information = Eigen::Matrix3d::Identity() * 37.5;
  request.added_constraints.push_back(recovered);
  const TrimReport report = pose_graph.ApplyTrim(request);
  ASSERT_FALSE(report.deleted_node_ids.empty());

  manager.Checkpoint(pose_graph.graph(), session);
  const int next_submap_after_trim = pose_graph.graph().id_allocator().next_submap_index(session);
  const int next_node_after_trim = pose_graph.graph().id_allocator().next_node_index(session);
  EXPECT_GT(next_submap_after_trim, submaps_before);
  EXPECT_GT(next_node_after_trim, nodes_before);

  PoseGraphData loaded;
  MapManager reader(directory);
  ASSERT_TRUE(reader.Load(loaded).has_value());
  EXPECT_EQ(loaded.id_allocator().next_submap_index(session), next_submap_after_trim);
  EXPECT_EQ(loaded.id_allocator().next_node_index(session), next_node_after_trim);
  EXPECT_EQ(loaded.AllocateSubmapId(session).submap_index, next_submap_after_trim);
  EXPECT_EQ(loaded.AllocateNodeId(session).node_index, next_node_after_trim);
  EXPECT_EQ(loaded.constraints().size(), pose_graph.graph().constraints().size());
  EXPECT_TRUE(std::any_of(loaded.constraints().begin(), loaded.constraints().end(),
                          [&recovered](const Constraint& constraint) {
                            return constraint.type == Constraint::Type::INTER_SUBMAP &&
                                   constraint.from == recovered.from && constraint.to.has_value() &&
                                   *constraint.to == *recovered.to;
                          }));
  // The trimmed session's own timeline is preserved even though its newest node is gone.
  EXPECT_EQ(common::ToUnixNanos(pose_graph.graph().session(session).last_node_time),
            common::ToUnixNanos(loaded.session(session).last_node_time));
}

}  // namespace
}  // namespace evergreenslam::lifelong

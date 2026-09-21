/**
 * @file loop_scenario.h
 * @author hang chen (chen@hang.plus)
 * @brief Shared simulated-laser loop scenario and frontend emulator for the backend e2e tests.
 * @version 0.1
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_TEST_LIFELONG_TESTING_LOOP_SCENARIO_H_
#define EVERGREENSLAM_TEST_LIFELONG_TESTING_LOOP_SCENARIO_H_

#include <glog/logging.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph.h"
#include "mapping/grid_mapping/castrays_mapping.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong::testing {

constexpr double kResolution = 0.05;
constexpr int kNodesPerSubmap = 10;
constexpr int kNumBeams = 360;
constexpr int kNodesPerLap = 80;
constexpr double kLoopRadius = 3.1;
constexpr double kMaxRange = 12.0;
constexpr double kYawBiasPerStep = 0.004;

inline std::string MakeTempDir(const std::string& prefix) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / (prefix + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path.string();
}

inline common::Time TestTime(int index) {
  return common::FromUnixSeconds(1785000000.0) + common::FromSeconds(0.1 * index);
}

struct World {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  std::vector<Eigen::AlignedBox2d> pillars;
};

inline double RangeToBox(const Eigen::Vector2d& origin, double angle,
                         const Eigen::AlignedBox2d& box) {
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

inline double RangeToBoundary(const World& world, const Eigen::Vector2d& origin, double angle) {
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

inline sensor::PointCloud SimulateScan(const World& world, const Eigen::Affine2d& world_pose,
                                       double max_range = kMaxRange) {
  const double yaw = utils::transform::GetYaw(world_pose);
  sensor::PointCloud cloud;
  for (int i = 0; i < kNumBeams; ++i) {
    const double bearing = -M_PI + 2.0 * M_PI * i / kNumBeams;
    const double range = RangeToBoundary(world, world_pose.translation(), yaw + bearing);
    if (range > max_range) {
      continue;
    }
    cloud.push_back(
        sensor::Point2d{Eigen::Vector2d(range * std::cos(bearing), range * std::sin(bearing))});
  }
  return cloud;
}

// Wall coordinates deliberately off the resolution lattice.
inline const World& LoopRoom() {
  static const World room{
      0.013,
      13.007,
      0.021,
      10.003,
      {Eigen::AlignedBox2d(Eigen::Vector2d(1.51, 1.52), Eigen::Vector2d(2.32, 2.33)),
       Eigen::AlignedBox2d(Eigen::Vector2d(10.61, 7.52), Eigen::Vector2d(11.42, 8.33)),
       Eigen::AlignedBox2d(Eigen::Vector2d(6.01, 4.62), Eigen::Vector2d(6.82, 5.43))}};
  return room;
}

inline Eigen::Affine2d LoopGroundTruth(int step) {
  const double phi = 2.0 * M_PI * static_cast<double>(step) / static_cast<double>(kNodesPerLap);
  return utils::transform::FromXYTheta(6.4 + kLoopRadius * std::sin(phi),
                                       1.9 + kLoopRadius * (1.0 - std::cos(phi)), phi);
}

inline std::vector<Eigen::Affine2d> LoopTruth(int num_steps) {
  std::vector<Eigen::Affine2d> truth;
  truth.reserve(num_steps);
  for (int step = 0; step < num_steps; ++step) {
    truth.push_back(LoopGroundTruth(step));
  }
  return truth;
}

// One continuous drifted odometry stream over the whole scenario: the frontend's local frame
// never resets in these stories, so one array indexed by ground-truth step is the local frame.
inline std::vector<Eigen::Affine2d> DriftedOdometry(int num_steps) {
  std::vector<Eigen::Affine2d> poses;
  poses.push_back(LoopGroundTruth(0));
  for (int i = 1; i < num_steps; ++i) {
    const Eigen::Affine2d increment =
        Eigen::Affine2d(LoopGroundTruth(i - 1).inverse() * LoopGroundTruth(i));
    poses.push_back(Eigen::Affine2d(poses.back() * increment *
                                    utils::transform::FromXYTheta(0.0, 0.0, kYawBiasPerStep)));
  }
  return poses;
}

// Emulates what LocalTrajectoryBuilder hands the backend: session-agnostic keyframes with a
// builder-lifetime node_index and locally indexed submaps, two actives. The backend mints every
// graph id at ingest.
class Frontend {
 public:
  Frontend(PoseGraph& backend, const std::vector<Eigen::Affine2d>& odometry)
      : Frontend(backend, LoopRoom(), LoopTruth(static_cast<int>(odometry.size())), odometry) {}

  // Any world and any ground-truth path: odometry[step] is the local pose fed for truth[step].
  Frontend(PoseGraph& backend, const World& world, std::vector<Eigen::Affine2d> truth,
           std::vector<Eigen::Affine2d> odometry, double max_range = kMaxRange)
      : backend_(backend),
        world_(world),
        truth_(std::move(truth)),
        odometry_(std::move(odometry)),
        max_range_(max_range) {
    CHECK_EQ(truth_.size(), odometry_.size());
  }

  void Feed(int step) {
    if (submaps_.empty() || fed_into_newest_ >= kNodesPerSubmap) {
      const int local_index = next_local_index_++;
      submaps_.push_back(
          std::make_shared<mapping::Submap>(local_index, odometry_[step], kResolution));
      creation_steps_[local_index] = step;
      fed_into_newest_ = 0;
      if (submaps_.size() > 2) {
        submaps_.front()->Finish();
        submaps_.erase(submaps_.begin());
      }
    }
    ++fed_into_newest_;

    const sensor::PointCloud cloud = SimulateScan(world_, truth_.at(step), max_range_);
    mapping::LocalTrajectoryBuilder::InsertionResult result;
    result.node_index = next_node_index_++;
    result.node.time = TestTime(step);
    result.node.local_pose = odometry_[step];
    result.node.point_cloud = cloud;
    for (const auto& submap : submaps_) {
      submap->InsertScan(odometry_[step], cloud, inserter_);
      result.insertion_submaps.push_back(submap);
    }
    backend_.AddInsertionResult(result);
  }

  // The ground-truth step each submap was created at, keyed by local_index -- stable across
  // rotations, unlike graph ids.
  const std::map<int, int>& submap_creation_steps() const { return creation_steps_; }

  void FinishAll() {
    for (const auto& submap : submaps_) {
      if (!submap->finished()) {
        submap->Finish();
      }
    }
  }

 private:
  PoseGraph& backend_;
  World world_;
  std::vector<Eigen::Affine2d> truth_;
  std::vector<Eigen::Affine2d> odometry_;
  double max_range_;
  mapping::CastRaysMapping inserter_;
  std::vector<std::shared_ptr<mapping::Submap>> submaps_;
  std::map<int, int> creation_steps_;
  int next_node_index_ = 0;
  int next_local_index_ = 0;
  int fed_into_newest_ = 0;
};

// Session-local node index k was fed for ground-truth step first_step + k (0 for session 0; see
// SessionStartSteps for the rest).
inline double MeanNodeError(const PoseGraphData& graph, SessionId session, int first_step = 0) {
  double total = 0.0;
  int count = 0;
  for (const NodeId& id : graph.session(session).node_ids) {
    total += (graph.node(id).global_pose.translation() -
              LoopGroundTruth(first_step + id.node_index).translation())
                 .norm();
    ++count;
  }
  return count == 0 ? 0.0 : total / static_cast<double>(count);
}

// Ground-truth step of each session's node 0: sessions open in order and node indices are minted
// in feed order, so a session starts where the previous sessions' mint counts end.
inline std::map<int, int> SessionStartSteps(const PoseGraphData& graph) {
  std::map<int, int> starts;
  int start = 0;
  for (const auto& [id, session] : graph.sessions()) {
    starts[id.session_index] = start;
    start += graph.id_allocator().next_node_index(id);
  }
  return starts;
}

}  // namespace evergreenslam::lifelong::testing

#endif  // EVERGREENSLAM_TEST_LIFELONG_TESTING_LOOP_SCENARIO_H_

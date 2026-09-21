/**
 * @file submap.h
 * @author hang chen (chen@hang.plus)
 * @brief One bounded grid the pose graph can treat as a rigid body.
 * @version 0.1
 * @date 2026-08-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_SUBMAP_H_
#define EVERGREENSLAM_MAPPING_SUBMAP_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "mapping/grid_mapping/castrays_mapping.h"
#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_grid.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {

// A world point is global_pose * grid_point, with no local_pose involved.
//
// The only cross-thread accesses to an unfinished submap are FinishedCopy and SnapshotCopy.
class Submap {
 public:
  Submap(int local_index, const Eigen::Affine2d& local_pose, double resolution);
  Submap(int local_index, const Eigen::Affine2d& local_pose, ProbabilityGrid grid, int num_scans,
         bool finished);

  void Finish();
  const GridMapu8& Snapshot() const;
  // `point_cloud` in the sensor frame at `scan_local_pose`.
  void InsertScan(const Eigen::Affine2d& scan_local_pose, const sensor::PointCloud& point_cloud,
                  const CastRaysMapping& inserter);

  // Safe against a concurrent InsertScan from the frontend, which a plain copy is not.
  std::shared_ptr<Submap> FinishedCopy() const;
  // A consistent snapshot from any thread; costs a copy.
  GridMapu8 SnapshotCopy() const;

  // Not a graph id: the backend translates it at ingest, and it never changes.
  int local_index() const { return local_index_; }
  const Eigen::Affine2d& local_pose() const { return local_pose_; }
  const ProbabilityGrid& grid() const { return grid_; }
  int num_scans() const { return num_scans_.load(std::memory_order_relaxed); }
  bool finished() const { return finished_.load(std::memory_order_acquire); }

 private:
  const int local_index_;
  Eigen::Affine2d local_pose_;
  ProbabilityGrid grid_;
  std::atomic<int> num_scans_{0};
  // Release/acquire: a reader that sees true also sees the final grid and its snapshot.
  std::atomic<bool> finished_{false};

  // Serializes InsertScan against FinishedCopy/SnapshotCopy while the submap is unfinished.
  mutable std::mutex insert_mutex_;
  mutable std::optional<GridMapu8> snapshot_;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_SUBMAP_H_

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
#include <optional>

#include "mapping/grid_mapping/castrays_mapping.h"
#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_grid.h"
#include "mapping/trajectory_data.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {

class Submap {
 public:
  Submap(const SubmapId& id, const Eigen::Affine2d& local_pose, double resolution);

  void Finish();
  const GridMapu8& Snapshot() const;
  void InsertScan(const Eigen::Vector2d& origin, const sensor::PointCloud& point_cloud,
                  const CastRaysMapping& inserter);

  const SubmapId& id() const { return id_; }
  const Eigen::Affine2d& local_pose() const { return local_pose_; }
  const ProbabilityGrid& grid() const { return grid_; }
  int num_scans() const { return num_scans_; }
  bool finished() const { return finished_; }

 private:
  SubmapId id_;
  Eigen::Affine2d local_pose_;
  ProbabilityGrid grid_;
  int num_scans_ = 0;
  bool finished_ = false;

  mutable std::optional<GridMapu8> snapshot_;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_SUBMAP_H_

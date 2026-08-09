/**
 * @file active_map.h
 * @author hang chen (chen@hang.plus)
 * @brief The submaps currently being built.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_LASER_ODOMETRY_ACTIVE_MAP_H_
#define EVERGREENSLAM_MAPPING_LASER_ODOMETRY_ACTIVE_MAP_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "mapping/grid_mapping/castrays_mapping.h"
#include "mapping/laser_odometry/active_map_option.h"
#include "mapping/submap.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {

class ActiveMap {
 public:
  explicit ActiveMap(const ActiveMapOption& option = ActiveMapOption());

  void StartNewSession(int session_id);
  std::vector<std::shared_ptr<const Submap>> InsertScan(const Eigen::Vector2d& origin,
                                                        const sensor::PointCloud& point_cloud,
                                                        const Eigen::Affine2d& local_pose);

  // nullptr before the first scan.
  std::shared_ptr<const Submap> matching_submap() const;
  int session_id() const { return session_id_; }
  const std::vector<std::shared_ptr<Submap>>& submaps() const { return submaps_; }

 private:
  void AddSubmap(const Eigen::Affine2d& local_pose);

  ActiveMapOption option_;
  CastRaysMapping inserter_;
  std::vector<std::shared_ptr<Submap>> submaps_;
  int session_id_ = 0;
  int next_submap_index_ = 0;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_LASER_ODOMETRY_ACTIVE_MAP_H_

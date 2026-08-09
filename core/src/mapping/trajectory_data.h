/**
 * @file trajectory_data.h
 * @author hang chen (chen@hang.plus)
 * @brief Identifiers and records shared between local mapping and the pose graph.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_TRAJECTORY_DATA_H_
#define EVERGREENSLAM_MAPPING_TRAJECTORY_DATA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tuple>

#include "common/time.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {

// A session is one continuous run of mapping, and the unit that gets frozen, so ids carry it.
struct SubmapId {
  int session_id = 0;
  int submap_index = 0;

  bool operator==(const SubmapId& other) const {
    return session_id == other.session_id && submap_index == other.submap_index;
  }
  bool operator<(const SubmapId& other) const {
    return std::tie(session_id, submap_index) < std::tie(other.session_id, other.submap_index);
  }
};

struct NodeId {
  int session_id = 0;
  int node_index = 0;

  bool operator==(const NodeId& other) const {
    return session_id == other.session_id && node_index == other.node_index;
  }
  bool operator<(const NodeId& other) const {
    return std::tie(session_id, node_index) < std::tie(other.session_id, other.node_index);
  }
};

struct TrajectoryNode {
  common::Time time;
  Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();
  sensor::PointCloud point_cloud;
};

struct MatchingResult {
  common::Time time;
  Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();
  double match_score = 0.0;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_TRAJECTORY_DATA_H_

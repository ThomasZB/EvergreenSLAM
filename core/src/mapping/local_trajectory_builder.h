/**
 * @file local_trajectory_builder.h
 * @author hang chen (chen@hang.plus)
 * @brief Scan in, keyframe and submaps out. The whole foreground pipeline.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_LOCAL_TRAJECTORY_BUILDER_H_
#define EVERGREENSLAM_MAPPING_LOCAL_TRAJECTORY_BUILDER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <vector>

#include "common/time.h"
#include "debug/debug_sink.h"
#include "mapping/laser_odometry/active_map.h"
#include "mapping/laser_odometry/pose_optimization.h"
#include "mapping/local_trajectory_builder_option.h"
#include "mapping/trajectory_data.h"
#include "sensor/adaptive_voxel_filter.h"
#include "sensor/point_cloud.h"
#include "sensor/timed_point_cloud.h"
#include "sensor/voxel_filter.h"
#include "utils/filters/generic_tracking_filter.h"
#include "utils/filters/motion_filter.h"

namespace evergreenslam::mapping {

class LocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    NodeId node_id;
    TrajectoryNode node;
    MatchingResult matching_result;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  explicit LocalTrajectoryBuilder(
      const LocalTrajectoryBuilderOption& option = LocalTrajectoryBuilderOption());

  std::optional<InsertionResult> AddScan(common::Time time,
                                         const sensor::TimedPointCloud& point_cloud);

  void StartNewSession(int session_id);
  void SetDebugSink(std::shared_ptr<debug::DebugSink> debug_sink);

  const Eigen::Affine2d& local_pose() const { return local_pose_; }
  const ActiveMap& active_map() const { return active_map_; }

 private:
  std::optional<InsertionResult> AddUndistortedScan(common::Time time,
                                                    const sensor::PointCloud& point_cloud);
  sensor::PointCloud CropRange(const sensor::PointCloud& point_cloud) const;

  LocalTrajectoryBuilderOption option_;

  sensor::VoxelFilter voxel_filter_;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter_;
  utils::filters::GenericTrackingFilter tracking_filter_;
  PoseOptimization pose_optimization_;
  utils::filters::MotionFilter motion_filter_;
  ActiveMap active_map_;
  std::shared_ptr<debug::DebugSink> debug_sink_ = nullptr;

  Eigen::Matrix<double, 3, 3> measurement_covariance_;

  Eigen::Affine2d local_pose_ = Eigen::Affine2d::Identity();
  int session_id_ = 0;
  int next_node_index_ = 0;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_LOCAL_TRAJECTORY_BUILDER_H_

/**
 * @file local_trajectory_builder.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/local_trajectory_builder.h"

#include <glog/logging.h>

#include <utility>

#include "sensor/undistortion.h"
#include "utils/transform/transform.h"

namespace evergreenslam::mapping {
namespace {

Eigen::Matrix<double, 3, 3> MeasurementCovariance(
    const utils::filters::GenericTrackingFilterOption& option) {
  const double translation = option.measurement_translation_sigma;
  const double rotation = option.measurement_rotation_sigma;
  return Eigen::Vector3d(translation * translation, translation * translation, rotation * rotation)
      .asDiagonal();
}

}  // namespace

LocalTrajectoryBuilder::LocalTrajectoryBuilder(const LocalTrajectoryBuilderOption& option)
    : option_(option),
      voxel_filter_(option.voxel_size),
      adaptive_voxel_filter_(option.adaptive_voxel_filter_option),
      tracking_filter_(option.tracking_filter_option),
      pose_optimization_(option.pose_optimization_option),
      motion_filter_(option.motion_filter_option),
      active_map_(option.active_map_option),
      measurement_covariance_(MeasurementCovariance(option.tracking_filter_option)) {}

std::optional<LocalTrajectoryBuilder::InsertionResult> LocalTrajectoryBuilder::AddUndistortedScan(
    common::Time time, const sensor::PointCloud& point_cloud) {
  const sensor::PointCloud dense = voxel_filter_.Filter(CropRange(point_cloud));
  if (dense.empty()) {
    LOG(WARNING) << "empty scan after range crop and voxel filter, dropped";
    return std::nullopt;
  }
  const sensor::PointCloud filtered = adaptive_voxel_filter_.Filter(dense);

  Eigen::Affine2d pose = local_pose_;
  if (!tracking_filter_.empty()) {
    const Eigen::Matrix<double, 9, 1> state = tracking_filter_.PredictTime(time).state;
    pose = utils::transform::FromXYTheta(state(0), state(1), state(2));
  }

  double match_score = 0.0;
  const std::shared_ptr<const Submap> matching_submap = active_map_.matching_submap();
  if (matching_submap != nullptr && !matching_submap->grid().empty()) {
    const GridMapu8& grid_map = matching_submap->Snapshot();
    const Eigen::Affine2d predicted_pose = pose;
    Eigen::Affine2d coarse_pose = pose;
    match_score = pose_optimization_.Match(filtered, grid_map, pose, &coarse_pose);
    if (debug_sink_ != nullptr) {
      debug_sink_->PublishScanMatch(
          time, {{"predicted", predicted_pose}, {"coarse", coarse_pose}, {"matched", pose}},
          filtered, grid_map, match_score);
    }
  }

  local_pose_ = pose;
  tracking_filter_.Update({time, local_pose_, measurement_covariance_});

  if (motion_filter_.IsSimilar(time, local_pose_)) {
    return std::nullopt;
  }
  InsertionResult result;
  result.node_id = NodeId{session_id_, next_node_index_++};
  result.node.time = time;
  result.node.local_pose = local_pose_;
  result.node.point_cloud = filtered;
  result.matching_result = MatchingResult{time, local_pose_, match_score};
  result.insertion_submaps = active_map_.InsertScan(
      local_pose_.translation(), sensor::TransformPointCloud(dense, local_pose_), local_pose_);
  return result;
}

std::optional<LocalTrajectoryBuilder::InsertionResult> LocalTrajectoryBuilder::AddScan(
    common::Time time, const sensor::TimedPointCloud& point_cloud) {
  // With no motion estimate yet the identity pair degenerates to stripping the offsets.
  Eigen::Affine2d start_pose = Eigen::Affine2d::Identity();
  Eigen::Affine2d end_pose = Eigen::Affine2d::Identity();
  if (!tracking_filter_.empty() && !point_cloud.empty()) {
    const auto pose_at = [this](common::Time t) {
      const Eigen::Matrix<double, 9, 1> state = tracking_filter_.PredictTime(t).state;
      return utils::transform::FromXYTheta(state(0), state(1), state(2));
    };
    start_pose = pose_at(time + point_cloud.points().front().offset);
    end_pose = pose_at(time + point_cloud.points().back().offset);
  }
  return AddUndistortedScan(time, sensor::Undistort(point_cloud, start_pose, end_pose));
}

void LocalTrajectoryBuilder::StartNewSession(int session_id) {
  active_map_.StartNewSession(session_id);
  motion_filter_.Reset();
  tracking_filter_.Reset();
  session_id_ = session_id;
  next_node_index_ = 0;
}

void LocalTrajectoryBuilder::SetDebugSink(std::shared_ptr<debug::DebugSink> debug_sink) {
  debug_sink_ = debug_sink;
}

sensor::PointCloud LocalTrajectoryBuilder::CropRange(const sensor::PointCloud& point_cloud) const {
  sensor::PointCloud cropped;
  cropped.points().reserve(point_cloud.size());
  for (const auto& point : point_cloud) {
    const double range = point.point.norm();
    if (range >= option_.min_range && range <= option_.max_range) {
      cropped.push_back(point);
    }
  }
  return cropped;
}

}  // namespace evergreenslam::mapping

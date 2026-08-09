/**
 * @file undistortion.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sensor/undistortion.h"

#include "common/time.h"
#include "utils/transform/transform.h"

namespace evergreenslam::sensor {

PointCloud Undistort(const TimedPointCloud& point_cloud, const Eigen::Affine2d& start_pose,
                     const Eigen::Affine2d& end_pose) {
  PointCloud undistorted;
  if (point_cloud.empty()) {
    return undistorted;
  }
  undistorted.points().reserve(point_cloud.size());
  const common::Duration front_offset = point_cloud.points().front().offset;
  const double span = common::ToSeconds(point_cloud.points().back().offset - front_offset);
  const Eigen::Affine2d from_local = end_pose.inverse();
  for (const auto& point : point_cloud) {
    const double factor = span > 0.0 ? common::ToSeconds(point.offset - front_offset) / span : 1.0;
    const Eigen::Affine2d pose = utils::transform::Interpolate(start_pose, end_pose, factor);
    const Eigen::Vector2d corrected = from_local * (pose * point.point);
    undistorted.push_back({corrected});
  }
  return undistorted;
}

}  // namespace evergreenslam::sensor

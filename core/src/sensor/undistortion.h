/**
 * @file undistortion.h
 * @author hang chen (chen@hang.plus)
 * @brief Removes motion distortion from a scan taken while the body moved.
 * @version 0.1
 * @date 2026-08-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_SENSOR_UNDISTORTION_H_
#define EVERGREENSLAM_SENSOR_UNDISTORTION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sensor/point_cloud.h"
#include "sensor/timed_point_cloud.h"

namespace evergreenslam::sensor {

// start_pose / end_pose are the body poses at the first and the last point. The output is
// expressed in the body frame of end_pose, so downstream sees an ordinary rigid scan.
PointCloud Undistort(const TimedPointCloud& point_cloud, const Eigen::Affine2d& start_pose,
                     const Eigen::Affine2d& end_pose);

}  // namespace evergreenslam::sensor

#endif  // EVERGREENSLAM_SENSOR_UNDISTORTION_H_

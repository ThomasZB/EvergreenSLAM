/**
 * @file laser_scan_converter.h
 * @author hang chen (chen@hang.plus)
 * @brief sensor_msgs/LaserScan to the core point cloud type.
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_ROS2_LASER_SCAN_CONVERTER_H_
#define EVERGREENSLAM_ADAPTERS_ROS2_LASER_SCAN_CONVERTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "common/time.h"
#include "sensor/timed_point_cloud.h"

namespace evergreenslam::ros2 {

struct TimedScan {
  // Acquisition time of the last beam, not header.stamp (ROS stamps the first beam); the cloud's
  // offsets are relative to it, per the core convention.
  common::Time time;
  sensor::TimedPointCloud point_cloud;
};

// Points come out in the frame `base_from_laser` maps into, so the pose the pipeline tracks is the
// robot's rather than the sensor's, and comparing against a bag's own odometry needs no correction.
TimedScan FromLaserScan(const sensor_msgs::msg::LaserScan& scan,
                        const Eigen::Affine2d& base_from_laser);

common::Time FromRosTime(const builtin_interfaces::msg::Time& stamp);

builtin_interfaces::msg::Time ToRosTime(common::Time time);

}  // namespace evergreenslam::ros2

#endif  // EVERGREENSLAM_ADAPTERS_ROS2_LASER_SCAN_CONVERTER_H_

/**
 * @file laser_scan_converter.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "laser_scan_converter.h"

#include <cmath>

namespace evergreenslam::ros2 {

TimedScan FromLaserScan(const sensor_msgs::msg::LaserScan& scan,
                        const Eigen::Affine2d& base_from_laser) {
  const double last_beam_index =
      scan.ranges.empty() ? 0.0 : static_cast<double>(scan.ranges.size() - 1);
  TimedScan timed_scan;
  timed_scan.time =
      FromRosTime(scan.header.stamp) + common::FromSeconds(last_beam_index * scan.time_increment);
  timed_scan.point_cloud.points().reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    // A no return beam is NaN or +Inf, and every comparison against those is false, so a range
    // crop alone would pass them straight through into the matcher and the grid.
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    timed_scan.point_cloud.push_back(
        {base_from_laser * Eigen::Vector2d(range * std::cos(angle), range * std::sin(angle)),
         common::FromSeconds((static_cast<double>(i) - last_beam_index) * scan.time_increment)});
  }
  return timed_scan;
}

common::Time FromRosTime(const builtin_interfaces::msg::Time& stamp) {
  return common::FromUnixNanos(static_cast<int64_t>(stamp.sec) * common::kNanosPerSecond +
                               stamp.nanosec);
}

builtin_interfaces::msg::Time ToRosTime(common::Time time) {
  const int64_t nanos = common::ToUnixNanos(time);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanos / common::kNanosPerSecond);
  stamp.nanosec = static_cast<uint32_t>(nanos % common::kNanosPerSecond);
  return stamp;
}

}  // namespace evergreenslam::ros2

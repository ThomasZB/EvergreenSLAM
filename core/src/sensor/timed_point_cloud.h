/**
 * @file timed_point_cloud.h
 * @author hang chen (chen@hang.plus)
 * @brief A point cloud whose points carry per-point acquisition offsets.
 * @version 0.1
 * @date 2026-08-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_SENSOR_TIMED_POINT_CLOUD_H_
#define EVERGREENSLAM_SENSOR_TIMED_POINT_CLOUD_H_

#include <Eigen/Core>
#include <vector>

#include "common/time.h"

namespace evergreenslam::sensor {

struct TimedPoint2d {
  Eigen::Vector2d point;
  // Relative to the scan time: the last point is at zero, earlier ones negative.
  common::Duration offset;
};

class TimedPointCloud {
 public:
  using PointType = TimedPoint2d;

  TimedPointCloud() = default;
  ~TimedPointCloud() = default;

  size_t size() const;
  bool empty() const;
  const std::vector<PointType>& points() const { return points_; }

  const PointType& operator[](size_t i) const;

  using ConstIterator = std::vector<PointType>::const_iterator;
  ConstIterator begin() const;
  ConstIterator end() const;

  void push_back(PointType value);
  std::vector<PointType>& points() { return points_; }

 private:
  std::vector<PointType> points_;
};

}  // namespace evergreenslam::sensor

#endif  // EVERGREENSLAM_SENSOR_TIMED_POINT_CLOUD_H_

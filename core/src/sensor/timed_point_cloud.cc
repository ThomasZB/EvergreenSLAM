/**
 * @file timed_point_cloud.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sensor/timed_point_cloud.h"

namespace evergreenslam::sensor {

size_t TimedPointCloud::size() const { return points_.size(); }

bool TimedPointCloud::empty() const { return points_.empty(); }

const TimedPointCloud::PointType& TimedPointCloud::operator[](size_t i) const { return points_[i]; }

TimedPointCloud::ConstIterator TimedPointCloud::begin() const { return points_.begin(); }

TimedPointCloud::ConstIterator TimedPointCloud::end() const { return points_.end(); }

void TimedPointCloud::push_back(PointType value) { points_.push_back(value); }

}  // namespace evergreenslam::sensor

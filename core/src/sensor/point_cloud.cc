/**
 * @file point_cloud.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "sensor/point_cloud.h"

namespace evergreenslam::sensor {

size_t PointCloud::size() const { return points_.size(); }

bool PointCloud::empty() const { return points_.empty(); }

const PointCloud::PointType& PointCloud::operator[](size_t i) const { return points_[i]; }

PointCloud::ConstIterator PointCloud::begin() const { return points_.begin(); }

PointCloud::ConstIterator PointCloud::end() const { return points_.end(); }

void PointCloud::push_back(PointType value) { points_.push_back(value); }

PointCloud TransformPointCloud(const PointCloud& point_cloud, const Eigen::Affine2d& transform) {
  PointCloud transformed_point_cloud;
  transformed_point_cloud.points().reserve(point_cloud.size());
  for (const auto& point : point_cloud) {
    transformed_point_cloud.push_back(PointCloud::PointType{transform * point.point});
  }
  return transformed_point_cloud;
}

}  // namespace evergreenslam::sensor

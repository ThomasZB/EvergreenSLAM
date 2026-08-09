/**
 * @file point_cloud.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef EVERGREENSLAM_SENSOR_POINT_CLOUD_H_
#define EVERGREENSLAM_SENSOR_POINT_CLOUD_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace evergreenslam::sensor {

struct Point2d {
  Eigen::Vector2d point;
};

class PointCloud {
 public:
  using PointType = Point2d;

  PointCloud() = default;
  ~PointCloud() = default;

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

PointCloud TransformPointCloud(const PointCloud& point_cloud, const Eigen::Affine2d& transform);

}  // namespace evergreenslam::sensor

#endif  // EVERGREENSLAM_SENSOR_POINT_CLOUD_H_
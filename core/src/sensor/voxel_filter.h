/**
 * @file voxel_filter.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-07-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef EVERGREENSLAM_SENSOR_VOXEL_FILTER_H_
#define EVERGREENSLAM_SENSOR_VOXEL_FILTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
#include <unordered_map>
#include <vector>

#include "sensor/point_cloud.h"

namespace evergreenslam::sensor {

class VoxelFilter {
 public:
  VoxelFilter() = default;
  VoxelFilter(double voxel_size) : voxel_size_(voxel_size) {};
  ~VoxelFilter() = default;

  void SetVoxelSize(double voxel_size) { voxel_size_ = voxel_size; }

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  double voxel_size_ = 0.1;
};

}  // namespace evergreenslam::sensor

#endif  // EVERGREENSLAM_SENSOR_VOXEL_FILTER_H_
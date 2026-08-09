/**
 * @file voxel_filter.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2024-07-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "sensor/voxel_filter.h"

#include <cmath>
#include <cstdint>

namespace evergreenslam::sensor {

uint64_t GetVoxelCellIndex(const Eigen::Vector2d& point, const double resolution) {
  const Eigen::Array2d index = point.array() / resolution;
  const uint32_t x = static_cast<uint32_t>(std::lround(index.x()));
  const uint32_t y = static_cast<uint32_t>(std::lround(index.y()));
  return (static_cast<uint64_t>(x) << 32) | y;
}

template <class PointType, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(const PointType& point_cloud,
                                               PointFunction&& point_function,
                                               const double resolution) {
  std::minstd_rand0 generator;
  std::unordered_map<uint64_t, std::pair<int, int>> voxel_map;

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    const Eigen::Vector2d& point = point_function(point_cloud[i]);
    auto& voxel = voxel_map[GetVoxelCellIndex(point, resolution)];
    voxel.first++;
    if (voxel.first == 1) {
      voxel.second = i;
    } else {
      std::uniform_int_distribution<int> distribution(1, voxel.first);
      if (distribution(generator) == 1) {
        voxel.second = i;
      }
    }
  }

  std::vector<bool> indices(point_cloud.size(), false);
  for (const auto& voxel : voxel_map) {
    indices[voxel.second.second] = true;
  }
  return indices;
}

PointCloud VoxelFilter::Filter(const PointCloud& point_cloud) const {
  PointCloud filtered_point_cloud;
  filtered_point_cloud.points().reserve(point_cloud.size());
  const auto indices = RandomizedVoxelFilterIndices(
      point_cloud, [](const Point2d& point) { return point.point; }, voxel_size_);
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (indices[i]) {
      filtered_point_cloud.push_back(point_cloud[i]);
    }
  }
  return filtered_point_cloud;
}

}  // namespace evergreenslam::sensor
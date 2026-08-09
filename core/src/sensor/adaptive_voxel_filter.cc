/**
 * @file adaptive_voxel_filter.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "sensor/adaptive_voxel_filter.h"

#include <algorithm>
#include <cstddef>

#include "sensor/voxel_filter.h"

namespace evergreenslam::sensor {
namespace {

// Below this the filter no longer removes anything a 2D lidar produces, so halving further only
// burns passes over the cloud.
constexpr double kMinLength = 0.01;

}  // namespace

PointCloud AdaptiveVoxelFilter::Filter(const PointCloud& point_cloud) const {
  const size_t min_num_points = static_cast<size_t>(std::max(option_.min_num_points, 0));
  if (point_cloud.size() <= min_num_points) {
    return point_cloud;
  }

  PointCloud result = VoxelFilter(option_.max_length).Filter(point_cloud);
  if (result.size() >= min_num_points) {
    return result;
  }

  for (double high_length = option_.max_length; high_length > kMinLength; high_length /= 2.0) {
    double low_length = high_length / 2.0;
    result = VoxelFilter(low_length).Filter(point_cloud);
    if (result.size() < min_num_points) {
      continue;
    }
    // `low_length` keeps enough points and `high_length` does not, so the answer is between them.
    while ((high_length - low_length) / low_length > 0.1) {
      const double mid_length = (low_length + high_length) / 2.0;
      const PointCloud candidate = VoxelFilter(mid_length).Filter(point_cloud);
      if (candidate.size() >= min_num_points) {
        low_length = mid_length;
        result = candidate;
      } else {
        high_length = mid_length;
      }
    }
    return result;
  }
  // Even the finest voxel cannot reach the budget; that cloud is the best available.
  return result;
}

}  // namespace evergreenslam::sensor

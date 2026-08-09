/**
 * @file adaptive_voxel_filter.h
 * @author hang chen (chen@hang.plus)
 * @brief Voxel filter that picks its own voxel size to hit a point budget.
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_SENSOR_ADAPTIVE_VOXEL_FILTER_H_
#define EVERGREENSLAM_SENSOR_ADAPTIVE_VOXEL_FILTER_H_

#include "sensor/adaptive_voxel_filter_option.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::sensor {

// Returns the coarsest filtering no coarser than `max_length` that still keeps `min_num_points`.
// A fixed voxel size cannot do this: the same size yields a few dozen points in a corridor and
// thousands in an open hall, so matching cost swings with the room rather than with the config.
class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const AdaptiveVoxelFilterOption& option = AdaptiveVoxelFilterOption())
      : option_(option) {}
  ~AdaptiveVoxelFilter() = default;

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  AdaptiveVoxelFilterOption option_;
};

}  // namespace evergreenslam::sensor

#endif  // EVERGREENSLAM_SENSOR_ADAPTIVE_VOXEL_FILTER_H_

/**
 * @file castrays_mapping.h
 * @author hang chen (chen@hang.plus)
 * @brief Writes one scan into a ProbabilityGrid by ray casting.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_GRID_MAPPING_CASTRAYS_MAPPING_H_
#define EVERGREENSLAM_MAPPING_GRID_MAPPING_CASTRAYS_MAPPING_H_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "mapping/grid_mapping/castrays_mapping_option.h"
#include "mapping/grid_mapping/probability_grid.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {

// Both endpoints included.
std::vector<Eigen::Array2i> CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end);

class CastRaysMapping {
 public:
  explicit CastRaysMapping(const CastRaysMappingOption& option = CastRaysMappingOption());

  void Insert(const Eigen::Vector2d& origin, const sensor::PointCloud& point_cloud,
              ProbabilityGrid& grid) const;

 private:
  CastRaysMappingOption option_;
  std::vector<uint8_t> hit_table_;
  std::vector<uint8_t> miss_table_;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_GRID_MAPPING_CASTRAYS_MAPPING_H_

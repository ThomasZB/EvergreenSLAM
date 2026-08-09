/**
 * @file probability_grid.h
 * @author hang chen (chen@hang.plus)
 * @brief The one writable, growable map in the system.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_MAPPING_GRID_MAPPING_PROBABILITY_GRID_H_
#define EVERGREENSLAM_MAPPING_GRID_MAPPING_PROBABILITY_GRID_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::mapping {

// Insertion protocol, in this order: GrowToInclude() for the whole scan, then ApplyLookupTable()
// per cell, then FinishUpdate(). Growing reallocates and may move the origin, so a grow
// mid-insertion would invalidate the pending indices in update_indices_.
class ProbabilityGrid : public GridMap<uint8_t> {
 public:
  // Origin is meaningless until the first GrowToInclude().
  explicit ProbabilityGrid(double resolution)
      : ProbabilityGrid(std::vector<uint8_t>(), 0, 0, resolution, 0.0, 0.0) {}

  ProbabilityGrid(std::vector<uint8_t> data, int width, int height, double resolution,
                  double origin_x, double origin_y)
      : GridMap<uint8_t>(std::move(data), width, height, resolution, origin_x, origin_y,
                         kUnknownValue) {}
  ~ProbabilityGrid() = default;

  void GrowToInclude(const Eigen::Vector2d& min_point, const Eigen::Vector2d& max_point);
  bool ApplyLookupTable(const Eigen::Array2i& cell, const std::vector<uint8_t>& table);
  void FinishUpdate();

  inline double GetProbability(const Eigen::Array2i& cell) const {
    return ValueToProbability(GetValue(cell));
  }
  inline bool IsKnown(const Eigen::Array2i& cell) const {
    return IsInside(cell) && IsKnownValue(GetValue(cell));
  }

  inline const Eigen::AlignedBox2i& known_area() const { return known_area_; }
  bool empty() const { return known_area_.isEmpty(); }

  // Cropped to known_area(), which is lossless because cells outside it are
  // unknown and so is everything outside the snapshot. 0 x 0 when empty.
  GridMapu8 ToSnapshot() const;

 private:
  static constexpr int kGrowPaddingCells = 64;

  std::vector<int> update_indices_;
  Eigen::AlignedBox2i known_area_;
};

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_GRID_MAPPING_PROBABILITY_GRID_H_

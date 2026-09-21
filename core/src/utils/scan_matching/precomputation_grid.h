/**
 * @file precomputation_grid.h
 * @author hang chen (chen@hang.plus)
 * @brief Multi-resolution max-pooled grids for branch-and-bound scan matching.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_SCAN_MATCHING_PRECOMPUTATION_GRID_H_
#define EVERGREENSLAM_UTILS_SCAN_MATCHING_PRECOMPUTATION_GRID_H_

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::utils::scan_matching {

// Pooling raw values is valid because ValueToProbability() is monotone non-decreasing, so the
// pooled value's probability bounds every probability inside the window.
class PrecomputationGrid {
 public:
  static PrecomputationGrid FromGrid(const mapping::GridMapu8& grid);
  // Only powers of two are ever needed, so the four half-window lookups are exact.
  static PrecomputationGrid Doubled(const PrecomputationGrid& half);

  // Cells outside the base grid, and anchors outside the covered band, read kUnknownValue,
  // which the probability table maps to kMinProbability like every other matcher.
  inline uint8_t GetValue(int x, int y) const {
    const int local_x = x - offset_x_;
    const int local_y = y - offset_y_;
    if (local_x < 0 || local_x >= wide_width_ || local_y < 0 || local_y >= wide_height_) {
      return mapping::kUnknownValue;
    }
    return cells_[static_cast<size_t>(local_y) * wide_width_ + local_x];
  }
  inline uint8_t GetValue(const Eigen::Array2i& cell) const { return GetValue(cell.x(), cell.y()); }

  int pooling_width() const { return pooling_width_; }

 private:
  PrecomputationGrid(int pooling_width, int base_width, int base_height);

  int pooling_width_;
  int offset_x_;
  int offset_y_;
  int wide_width_;
  int wide_height_;
  std::vector<uint8_t> cells_;
};

// Depth d pools 2^d cells; depth 0 is the base grid itself.
class PrecomputationGridStack {
 public:
  PrecomputationGridStack(const mapping::GridMapu8& grid, int num_depths);

  const PrecomputationGrid& Get(int depth) const { return grids_[depth]; }
  int max_depth() const { return static_cast<int>(grids_.size()) - 1; }

 private:
  std::vector<PrecomputationGrid> grids_;
};

}  // namespace evergreenslam::utils::scan_matching

#endif  // EVERGREENSLAM_UTILS_SCAN_MATCHING_PRECOMPUTATION_GRID_H_

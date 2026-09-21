/**
 * @file precomputation_grid.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/scan_matching/precomputation_grid.h"

#include <glog/logging.h>

#include <algorithm>

namespace evergreenslam::utils::scan_matching {

PrecomputationGrid::PrecomputationGrid(int pooling_width, int base_width, int base_height)
    : pooling_width_(pooling_width),
      offset_x_(-pooling_width + 1),
      offset_y_(-pooling_width + 1),
      wide_width_(base_width + pooling_width - 1),
      wide_height_(base_height + pooling_width - 1),
      cells_(static_cast<size_t>(wide_width_) * wide_height_, mapping::kUnknownValue) {}

PrecomputationGrid PrecomputationGrid::FromGrid(const mapping::GridMapu8& grid) {
  DCHECK_EQ(grid.unknown_value(), mapping::kUnknownValue);
  PrecomputationGrid result(1, grid.width(), grid.height());
  result.cells_ = grid.data();
  return result;
}

PrecomputationGrid PrecomputationGrid::Doubled(const PrecomputationGrid& half) {
  const int base_width = half.wide_width_ - half.pooling_width_ + 1;
  const int base_height = half.wide_height_ - half.pooling_width_ + 1;
  const int step = half.pooling_width_;
  PrecomputationGrid result(2 * step, base_width, base_height);
  for (int y = 0; y < result.wide_height_; ++y) {
    const int cell_y = y + result.offset_y_;
    for (int x = 0; x < result.wide_width_; ++x) {
      const int cell_x = x + result.offset_x_;
      const uint8_t max_value =
          std::max(std::max(half.GetValue(cell_x, cell_y), half.GetValue(cell_x + step, cell_y)),
                   std::max(half.GetValue(cell_x, cell_y + step),
                            half.GetValue(cell_x + step, cell_y + step)));
      result.cells_[static_cast<size_t>(y) * result.wide_width_ + x] = max_value;
    }
  }
  return result;
}

PrecomputationGridStack::PrecomputationGridStack(const mapping::GridMapu8& grid, int num_depths) {
  CHECK_GT(num_depths, 0);
  grids_.reserve(num_depths);
  grids_.push_back(PrecomputationGrid::FromGrid(grid));
  for (int depth = 1; depth < num_depths; ++depth) {
    grids_.push_back(PrecomputationGrid::Doubled(grids_.back()));
  }
}

}  // namespace evergreenslam::utils::scan_matching

/**
 * @file probability_grid.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/probability_grid.h"

#include <glog/logging.h>

#include <algorithm>
#include <utility>

namespace evergreenslam::mapping {

ProbabilityGrid ProbabilityGrid::FromSnapshot(const GridMapu8& snapshot) {
  ProbabilityGrid grid(snapshot.data(), snapshot.width(), snapshot.height(), snapshot.resolution(),
                       snapshot.origin_x(), snapshot.origin_y());
  if (snapshot.width() > 0 && snapshot.height() > 0) {
    grid.known_area_.extend(Eigen::Vector2i(0, 0));
    grid.known_area_.extend(Eigen::Vector2i(snapshot.width() - 1, snapshot.height() - 1));
  }
  return grid;
}

void ProbabilityGrid::GrowToInclude(const Eigen::Vector2d& min_point,
                                    const Eigen::Vector2d& max_point) {
  CHECK(update_indices_.empty())
      << "growing mid-insertion would invalidate the pending cell indices";

  const Eigen::Array2i min_cell = ToCell(min_point);
  const Eigen::Array2i max_cell = ToCell(max_point);
  if (IsInside(min_cell) && IsInside(max_cell)) {
    return;
  }

  const int min_x = std::min(0, min_cell.x() - kGrowPaddingCells);
  const int min_y = std::min(0, min_cell.y() - kGrowPaddingCells);
  const int max_x = std::max(width_ - 1, max_cell.x() + kGrowPaddingCells);
  const int max_y = std::max(height_ - 1, max_cell.y() + kGrowPaddingCells);

  const int new_width = max_x - min_x + 1;
  const int new_height = max_y - min_y + 1;

  std::vector<uint8_t> new_data(static_cast<size_t>(new_width) * new_height, kUnknownValue);
  for (int y = 0; y < height_; ++y) {
    const auto source = data_.begin() + static_cast<size_t>(y) * width_;
    const auto destination = new_data.begin() + static_cast<size_t>(y - min_y) * new_width - min_x;
    std::copy(source, source + width_, destination);
  }

  origin_x_ += min_x * resolution_;
  origin_y_ += min_y * resolution_;
  if (!known_area_.isEmpty()) {
    known_area_.translate(Eigen::Vector2i(-min_x, -min_y));
  }

  data_ = std::move(new_data);
  width_ = new_width;
  height_ = new_height;
}

bool ProbabilityGrid::ApplyLookupTable(const Eigen::Array2i& cell,
                                       const std::vector<uint8_t>& table) {
  DCHECK_EQ(table.size(), static_cast<size_t>(kValueCount));
  DCHECK(IsInside(cell)) << "cell outside the grid; GrowToInclude() first";
  if (!IsInside(cell)) {
    return false;
  }

  const int index = cell.y() * width_ + cell.x();
  uint8_t& value = data_[index];
  if (value >= kUpdateMarker) {
    return false;
  }
  value = table[value];
  update_indices_.push_back(index);
  known_area_.extend(Eigen::Vector2i(cell.x(), cell.y()));
  return true;
}

void ProbabilityGrid::FinishUpdate() {
  for (const int index : update_indices_) {
    DCHECK_GE(data_[index], kUpdateMarker);
    data_[index] -= kUpdateMarker;
  }
  update_indices_.clear();
}

GridMapu8 ProbabilityGrid::ToSnapshot() const {
  DCHECK(update_indices_.empty()) << "snapshot mid-insertion carries markers";

  if (known_area_.isEmpty()) {
    return GridMapu8(std::vector<uint8_t>(), 0, 0, resolution_, origin_x_, origin_y_,
                     kUnknownValue);
  }

  const int min_x = known_area_.min().x();
  const int min_y = known_area_.min().y();
  const int width = known_area_.max().x() - min_x + 1;
  const int height = known_area_.max().y() - min_y + 1;

  std::vector<uint8_t> data(static_cast<size_t>(width) * height, kUnknownValue);
  for (int y = 0; y < height; ++y) {
    const auto source = data_.begin() + static_cast<size_t>(y + min_y) * width_ + min_x;
    std::copy(source, source + width, data.begin() + static_cast<size_t>(y) * width);
  }

  return GridMapu8(std::move(data), width, height, resolution_, origin_x_ + min_x * resolution_,
                   origin_y_ + min_y * resolution_, kUnknownValue);
}

}  // namespace evergreenslam::mapping

/**
 * @file grid_map.h
 * @author hang chen (chen@hang.plus)
 * @brief Read-only 2D grid, the form every scan matcher consumes.
 * @version 0.1
 * @date 2024-05-16
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef EVERGREENSLAM_MAPPING_GRID_MAPPING_GRID_MAP_H_
#define EVERGREENSLAM_MAPPING_GRID_MAPPING_GRID_MAP_H_

#include <Eigen/Core>
#include <cmath>
#include <utility>
#include <vector>

namespace evergreenslam::mapping {

template <typename T>
class GridMap {
 public:
  GridMap(std::vector<T> data, int width, int height, double resolution, double origin_x,
          double origin_y, T unknown_value)
      : data_(std::move(data)),
        width_(width),
        height_(height),
        resolution_(resolution),
        origin_x_(origin_x),
        origin_y_(origin_y),
        unknown_value_(unknown_value) {}
  ~GridMap() = default;

  inline Eigen::Array2i ToCell(const Eigen::Vector2d& point) const {
    return Eigen::Array2i(static_cast<int>(std::floor((point.x() - origin_x_) / resolution_)),
                          static_cast<int>(std::floor((point.y() - origin_y_) / resolution_)));
  }

  inline Eigen::Vector2d ToCenter(const Eigen::Array2i& cell) const {
    return Eigen::Vector2d(origin_x_ + (cell.x() + 0.5) * resolution_,
                           origin_y_ + (cell.y() + 0.5) * resolution_);
  }

  inline bool IsInside(int x, int y) const { return x >= 0 && x < width_ && y >= 0 && y < height_; }

  inline bool IsInside(const Eigen::Array2i& cell) const { return IsInside(cell.x(), cell.y()); }

  inline T GetValue(int x, int y) const {
    if (!IsInside(x, y)) {
      return unknown_value_;
    }
    return data_[static_cast<size_t>(y) * width_ + x];
  }

  inline T GetValue(const Eigen::Array2i& cell) const { return GetValue(cell.x(), cell.y()); }

  // Named apart from the cell overloads because an Eigen expression converts
  // to both Vector2d and Array2i, which would make the call ambiguous.
  inline T GetValueAtPoint(const Eigen::Vector2d& point) const { return GetValue(ToCell(point)); }

  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  int width() const { return width_; }
  int height() const { return height_; }
  T unknown_value() const { return unknown_value_; }
  const std::vector<T>& data() const { return data_; }

 protected:
  std::vector<T> data_;
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  T unknown_value_;
};

using GridMapu8 = GridMap<uint8_t>;

}  // namespace evergreenslam::mapping

#endif  // EVERGREENSLAM_MAPPING_GRID_MAPPING_GRID_MAP_H_

/**
 * @file view_frame.cc
 * @author hang chen (chen@hang.plus)
 * @brief World <-> pixel mapping of one north-up rendered view.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/render/view_frame.h"

#include <algorithm>
#include <cmath>

namespace evergreenslam::agent {
namespace {

constexpr int kPixelLimit = 1 << 24;

}  // namespace

Eigen::Array2i ViewFrame::ToPixel(const Eigen::Vector2d& xy) const {
  const auto to_int = [](double v) {
    return static_cast<int>(std::clamp(std::floor(v), -double{kPixelLimit}, double{kPixelLimit}));
  };
  return Eigen::Array2i(to_int((xy.x() - min_x) / metres_per_pixel),
                        to_int((max_y - xy.y()) / metres_per_pixel));
}

Eigen::Vector2d ViewFrame::PixelCenter(int col, int row) const {
  return Eigen::Vector2d(min_x + (col + 0.5) * metres_per_pixel,
                         max_y - (row + 0.5) * metres_per_pixel);
}

}  // namespace evergreenslam::agent

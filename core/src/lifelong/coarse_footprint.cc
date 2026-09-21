/**
 * @file coarse_footprint.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/coarse_footprint.h"

#include <glog/logging.h>

#include <cmath>
#include <unordered_set>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::lifelong {
namespace {

inline std::int64_t PackCell(int x, int y) {
  return (static_cast<std::int64_t>(x) << 32) ^ (static_cast<std::int64_t>(y) & 0xffffffffLL);
}

}  // namespace

std::int64_t CoarseCellOf(const Eigen::Vector2d& point, double resolution) {
  // floor, never truncation: half the points sit at negative coordinates.
  return PackCell(static_cast<int>(std::floor(point.x() / resolution)),
                  static_cast<int>(std::floor(point.y() / resolution)));
}

std::vector<std::int64_t> ComputeCoarseFootprint(const SubmapRecord& record, double resolution) {
  CHECK(record.submap != nullptr);
  const mapping::ProbabilityGrid& grid = record.submap->grid();
  const Eigen::Affine2d& grid_to_global = record.global_pose;

  std::unordered_set<std::int64_t> cells;
  const Eigen::AlignedBox2i& known = grid.known_area();
  if (!known.isEmpty()) {
    for (int y = known.min().y(); y <= known.max().y(); ++y) {
      for (int x = known.min().x(); x <= known.max().x(); ++x) {
        if (!mapping::IsKnownValue(grid.GetValue(x, y))) {
          continue;
        }
        const Eigen::Vector2d global_point = grid_to_global * grid.ToCenter(Eigen::Array2i(x, y));
        cells.insert(CoarseCellOf(global_point, resolution));
      }
    }
  }
  return {cells.begin(), cells.end()};
}

}  // namespace evergreenslam::lifelong

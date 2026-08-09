/**
 * @file castrays_mapping.cc
 * @author hang chen (chen@hang.plus)
 * @brief Writes one scan into a ProbabilityGrid by ray casting.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/castrays_mapping.h"

#include <algorithm>
#include <cstdlib>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::mapping {

std::vector<Eigen::Array2i> CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end) {
  int x = begin.x();
  int y = begin.y();
  const int x1 = end.x();
  const int y1 = end.y();

  const int dx = std::abs(x1 - x);
  // Negative on purpose: it lets one error term drive both axes in every
  // octant, with no slope comparison and no division.
  const int dy = -std::abs(y1 - y);
  const int sx = x < x1 ? 1 : -1;
  const int sy = y < y1 ? 1 : -1;
  int err = dx + dy;

  std::vector<Eigen::Array2i> ray;
  ray.reserve(static_cast<size_t>(std::max(dx, -dy)) + 1);
  while (true) {
    ray.emplace_back(x, y);
    if (x == x1 && y == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  return ray;
}

CastRaysMapping::CastRaysMapping(const CastRaysMappingOption& option)
    : option_(option),
      hit_table_(ComputeLookupTableToApplyOdds(Odds(option.hit_probability))),
      miss_table_(ComputeLookupTableToApplyOdds(Odds(option.miss_probability))) {}

void CastRaysMapping::Insert(const Eigen::Vector2d& origin, const sensor::PointCloud& point_cloud,
                             ProbabilityGrid& grid) const {
  if (point_cloud.empty()) {
    return;
  }

  Eigen::Vector2d min_point = origin;
  Eigen::Vector2d max_point = origin;
  for (const auto& point : point_cloud) {
    min_point = min_point.cwiseMin(point.point);
    max_point = max_point.cwiseMax(point.point);
  }
  const Eigen::Vector2d padding = Eigen::Vector2d::Constant(grid.resolution());
  grid.GrowToInclude(min_point - padding, max_point + padding);

  // Every cell coordinate is derived below this line: growing moves the grid
  // origin, so anything mapped before it would name a different cell.
  const Eigen::Array2i origin_cell = grid.ToCell(origin);

  // Hits first: the update marker means the first write to a cell wins for the
  // whole scan, and a real obstacle must not be downgraded by a beam that only
  // grazes it.
  for (const auto& point : point_cloud) {
    grid.ApplyLookupTable(grid.ToCell(point.point), hit_table_);
  }

  if (option_.insert_free_space) {
    for (const auto& point : point_cloud) {
      const std::vector<Eigen::Array2i> ray = CastRay(origin_cell, grid.ToCell(point.point));
      for (size_t i = 0; i + 1 < ray.size(); ++i) {
        grid.ApplyLookupTable(ray[i], miss_table_);
      }
    }
  }

  grid.FinishUpdate();
}

}  // namespace evergreenslam::mapping

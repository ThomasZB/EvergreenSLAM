/**
 * @file global_map.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/global_map.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::lifelong {
namespace {

struct Bounds {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  void Add(const Eigen::Vector2d& point) {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }

  bool empty() const { return min_x > max_x; }
};

Bounds SnapshotWorldBounds(const mapping::GridMapu8& snapshot, const Eigen::Affine2d& world) {
  Bounds bounds;
  const double x0 = snapshot.origin_x();
  const double y0 = snapshot.origin_y();
  const double x1 = x0 + snapshot.width() * snapshot.resolution();
  const double y1 = y0 + snapshot.height() * snapshot.resolution();
  bounds.Add(world * Eigen::Vector2d(x0, y0));
  bounds.Add(world * Eigen::Vector2d(x1, y0));
  bounds.Add(world * Eigen::Vector2d(x0, y1));
  bounds.Add(world * Eigen::Vector2d(x1, y1));
  return bounds;
}

}  // namespace

mapping::GridMapu8 AssembleGlobalMap(const PoseGraphData& graph, double resolution,
                                     bool only_finished) {
  Bounds bounds;
  double submap_resolution = 0.0;
  for (const auto& [id, record] : graph.submaps()) {
    if (only_finished && !record.submap->finished()) {
      continue;
    }
    const mapping::GridMapu8& snapshot = record.submap->Snapshot();
    if (snapshot.width() == 0 || snapshot.height() == 0) {
      continue;
    }
    submap_resolution = snapshot.resolution();
    const Bounds submap_bounds = SnapshotWorldBounds(snapshot, record.global_pose);
    bounds.Add(Eigen::Vector2d(submap_bounds.min_x, submap_bounds.min_y));
    bounds.Add(Eigen::Vector2d(submap_bounds.max_x, submap_bounds.max_y));
  }
  if (bounds.empty()) {
    return mapping::GridMapu8({}, 0, 0, resolution > 0.0 ? resolution : 0.05, 0.0, 0.0,
                              mapping::kUnknownValue);
  }
  const double cell_size = resolution > 0.0 ? resolution : submap_resolution;
  // One padding cell so a corner point on the outer edge still floors inside.
  const double origin_x = bounds.min_x - cell_size;
  const double origin_y = bounds.min_y - cell_size;
  const int width = static_cast<int>(std::ceil((bounds.max_x - origin_x) / cell_size)) + 1;
  const int height = static_cast<int>(std::ceil((bounds.max_y - origin_y) / cell_size)) + 1;

  std::vector<uint8_t> cells(static_cast<size_t>(width) * height, mapping::kUnknownValue);
  const auto to_cell_index = [&](double value, double origin) {
    return static_cast<int>(std::floor((value - origin) / cell_size));
  };

  // `>=` keeps the newest of equally certain candidates. Sampling output cells back into the
  // snapshot leaves no rotation holes, unlike pushing snapshot cells forward.
  for (const auto& [id, record] : graph.submaps()) {
    if (only_finished && !record.submap->finished()) {
      continue;
    }
    const mapping::GridMapu8& snapshot = record.submap->Snapshot();
    if (snapshot.width() == 0 || snapshot.height() == 0) {
      continue;
    }
    const Eigen::Affine2d& world = record.global_pose;
    const Eigen::Affine2d local = world.inverse();
    const Bounds submap_bounds = SnapshotWorldBounds(snapshot, world);
    const int x_begin = std::max(to_cell_index(submap_bounds.min_x, origin_x), 0);
    const int x_end = std::min(to_cell_index(submap_bounds.max_x, origin_x), width - 1);
    const int y_begin = std::max(to_cell_index(submap_bounds.min_y, origin_y), 0);
    const int y_end = std::min(to_cell_index(submap_bounds.max_y, origin_y), height - 1);
    for (int y = y_begin; y <= y_end; ++y) {
      for (int x = x_begin; x <= x_end; ++x) {
        const Eigen::Vector2d center(origin_x + (x + 0.5) * cell_size,
                                     origin_y + (y + 0.5) * cell_size);
        const Eigen::Vector2d in_submap = local * center;
        const uint8_t value = snapshot.GetValueAtPoint(in_submap);
        if (!mapping::IsKnownValue(value)) {
          continue;
        }
        uint8_t& cell = cells[static_cast<size_t>(y) * width + x];
        if (!mapping::IsKnownValue(cell) ||
            mapping::ValueConfidence(value) >= mapping::ValueConfidence(cell)) {
          cell = value;
        }
      }
    }
  }
  return mapping::GridMapu8(std::move(cells), width, height, cell_size, origin_x, origin_y,
                            mapping::kUnknownValue);
}

}  // namespace evergreenslam::lifelong

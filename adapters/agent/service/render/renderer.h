/**
 * @file renderer.h
 * @author hang chen (chen@hang.plus)
 * @brief GET /view renderer: a pure function from map, poses and markers to a north-up PNG.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_RENDERER_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_RENDERER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"
#include "sensor/point_cloud.h"
#include "service/render/canvas.h"
#include "service/render/view_frame.h"

namespace evergreenslam::agent {

struct RenderMarker {
  int number = 0;
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  std::string label;
  bool highlight = false;
};

// Layers: map, robot, scan, trail, places, target, session, submaps, zones. `map` or `session`
// draws `grid`; `places` draws the markers without highlight, `target` the highlighted ones;
// `scan` needs `robot`; `zones` hatches the keep-out polygons. Unknown names are ignored (the
// service validates them).
struct RenderInput {
  mapping::GridMapu8 grid{{}, 0, 0, 0.05, 0.0, 0.0, mapping::kUnknownValue};
  std::optional<Eigen::Affine2d> robot;
  std::optional<sensor::PointCloud> scan;
  std::vector<Eigen::Vector2d> trail;
  std::vector<RenderMarker> markers;
  std::vector<std::array<Eigen::Vector2d, 4>> submap_outlines;
  // Map-frame keep-out polygons; the legend also counts the zones that did not resolve.
  std::vector<std::vector<Eigen::Vector2d>> zones;
  int num_unresolved_zones = 0;
  bool zones_incomplete = false;
  std::set<std::string> layers;
  std::optional<double> ego_radius_m;
};

struct RenderOutput {
  std::vector<uint8_t> png;
  int width = 0;
  int height = 0;
  std::string legend;
};

RenderOutput Render(const RenderInput& input);

struct RenderedView {
  Canvas canvas;
  ViewFrame frame;
  std::string legend;
};

// `Render` before encoding; the raster is what tests inspect.
RenderedView RenderView(const RenderInput& input);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_RENDERER_H_

/**
 * @file renderer.cc
 * @author hang chen (chen@hang.plus)
 * @brief GET /view renderer: a pure function from map, poses and markers to a north-up PNG.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/render/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>
#include <utility>

#include "service/render/bitmap_font.h"
#include "service/render/grid_overlay.h"
#include "service/render/png_writer.h"

namespace evergreenslam::agent {
namespace {

constexpr int kMaxLongSidePx = 1024;
constexpr double kItemMarginM = 0.5;
constexpr double kEmptyViewSizeM = 10.0;
constexpr double kMarkerRadiusM = 0.15;
constexpr int kMinMarkerRadiusPx = 4;
constexpr int kBadgeScale = 2;
constexpr double kPi = 3.14159265358979323846;

double RobotArrowLengthPx(double metres_per_pixel) {
  return std::max(16.0, 0.6 / metres_per_pixel);
}

const Rgb kOccupied{0, 0, 0};
const Rgb kFree{255, 255, 255};
const Rgb kUnknown{205, 205, 205};
const Rgb kSubmap{0, 150, 150};
const Rgb kTrail{40, 90, 230};
const Rgb kScan{230, 0, 170};
const Rgb kRobot{0, 160, 0};
const Rgb kPlace{255, 140, 0};
const Rgb kTarget{220, 0, 0};
const Rgb kInk{0, 0, 0};

// Canonical order of the legend's `layers=`.
const char* const kLayerOrder[] = {"map",    "robot",  "scan",    "trail",
                                   "places", "target", "session", "submaps"};

enum class CellClass : uint8_t { kUnknown, kFree, kOccupied };

CellClass Classify(uint8_t value) {
  if (!mapping::IsKnownValue(value)) {
    return CellClass::kUnknown;
  }
  return mapping::ValueToProbability(value) > 0.5 ? CellClass::kOccupied : CellClass::kFree;
}

struct Bounds {
  Eigen::Vector2d min = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d max = Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());
  bool empty() const { return !(min.x() <= max.x() && min.y() <= max.y()); }
  void Extend(const Eigen::Vector2d& p, double margin) {
    min = min.cwiseMin(p - Eigen::Vector2d::Constant(margin));
    max = max.cwiseMax(p + Eigen::Vector2d::Constant(margin));
  }
};

struct Plan {
  bool grid = false;
  bool robot = false;
  bool scan = false;
  bool trail = false;
  bool places = false;
  bool target = false;
  bool submaps = false;
  bool ego = false;
  std::vector<std::string> drawn;
  std::vector<std::string> dropped;
};

Plan MakePlan(const RenderInput& input) {
  const auto has = [&input](const char* layer) { return input.layers.count(layer) > 0; };
  Plan plan;
  plan.grid = has("map") || has("session");
  plan.robot = has("robot") && input.robot.has_value();
  plan.scan = has("scan") && input.scan.has_value() && input.robot.has_value();
  plan.trail = has("trail");
  plan.places = has("places");
  plan.target = has("target");
  plan.submaps = has("submaps");
  plan.ego = input.ego_radius_m.has_value() && *input.ego_radius_m > 0.0 && input.robot;
  for (const char* layer : kLayerOrder) {
    if (!has(layer)) {
      continue;
    }
    const std::string name(layer);
    const bool lost = (name == "robot" && !plan.robot) || (name == "scan" && !plan.scan);
    (lost ? plan.dropped : plan.drawn).push_back(name);
  }
  if (input.ego_radius_m.has_value() && !plan.ego) {
    plan.dropped.push_back("ego");
  }
  return plan;
}

bool MarkerEnabled(const Plan& plan, const RenderMarker& marker) {
  return marker.highlight ? plan.target : plan.places;
}

ViewFrame MakeFrame(const RenderInput& input, const Plan& plan) {
  const mapping::GridMapu8& grid = input.grid;
  const bool has_cells = grid.width() > 0 && grid.height() > 0 && grid.resolution() > 0.0;
  const double resolution = has_cells ? grid.resolution() : 0.05;

  Bounds bounds;
  if (plan.ego) {
    const Eigen::Vector2d center = input.robot->translation();
    bounds.Extend(center, *input.ego_radius_m);
  } else {
    if (has_cells) {
      bounds.min = Eigen::Vector2d(grid.origin_x(), grid.origin_y());
      bounds.max = bounds.min + resolution * Eigen::Vector2d(grid.width(), grid.height());
    }
    if (plan.robot) {
      bounds.Extend(input.robot->translation(), kItemMarginM);
    }
    if (plan.trail) {
      for (const Eigen::Vector2d& p : input.trail) {
        bounds.Extend(p, kItemMarginM);
      }
    }
    for (const RenderMarker& marker : input.markers) {
      if (MarkerEnabled(plan, marker)) {
        bounds.Extend(marker.xy, kItemMarginM);
      }
    }
    if (plan.submaps) {
      for (const auto& corners : input.submap_outlines) {
        for (const Eigen::Vector2d& p : corners) {
          bounds.Extend(p, 0.0);
        }
      }
    }
    if (bounds.empty()) {
      const Eigen::Vector2d center =
          input.robot ? Eigen::Vector2d(input.robot->translation()) : Eigen::Vector2d::Zero();
      bounds.Extend(center, kEmptyViewSizeM / 2.0);
    }
  }

  const Eigen::Vector2d extent = bounds.max - bounds.min;
  // An integer number of cells per pixel keeps pixels on cell boundaries when uncropped.
  const double long_side_cells = extent.maxCoeff() / resolution;
  const int cells_per_pixel =
      std::max(1, static_cast<int>(std::ceil(long_side_cells / kMaxLongSidePx - 1e-9)));
  ViewFrame frame;
  frame.metres_per_pixel = resolution * cells_per_pixel;
  frame.width = std::clamp(static_cast<int>(std::ceil(extent.x() / frame.metres_per_pixel - 1e-9)),
                           1, kMaxLongSidePx);
  frame.height = std::clamp(static_cast<int>(std::ceil(extent.y() / frame.metres_per_pixel - 1e-9)),
                            1, kMaxLongSidePx);
  frame.min_x = bounds.min.x();
  frame.max_y = bounds.min.y() + frame.height * frame.metres_per_pixel;
  return frame;
}

std::vector<CellClass> PaintGrid(const mapping::GridMapu8& grid, const ViewFrame& frame,
                                 Canvas& canvas) {
  std::vector<CellClass> classes(static_cast<size_t>(frame.width) * frame.height,
                                 CellClass::kUnknown);
  const double mpp = frame.metres_per_pixel;
  const double resolution = grid.resolution();
  const bool pool = grid.width() > 0 && mpp > resolution * 1.5;
  for (int row = 0; row < frame.height; ++row) {
    for (int col = 0; col < frame.width; ++col) {
      CellClass cls = CellClass::kUnknown;
      if (!pool) {
        cls = Classify(grid.GetValueAtPoint(frame.PixelCenter(col, row)));
      } else {
        // Occupied wins so that one-cell walls survive the downsampling.
        const Eigen::Vector2d lower_left(frame.min_x + col * mpp, frame.max_y - (row + 1) * mpp);
        const Eigen::Array2i lo =
            grid.ToCell(lower_left + Eigen::Vector2d::Constant(0.25 * resolution));
        const Eigen::Array2i hi =
            grid.ToCell(lower_left + Eigen::Vector2d::Constant(mpp - 0.25 * resolution));
        for (int y = lo.y(); y <= hi.y() && cls != CellClass::kOccupied; ++y) {
          for (int x = lo.x(); x <= hi.x(); ++x) {
            const CellClass c = Classify(grid.GetValue(x, y));
            if (c == CellClass::kOccupied) {
              cls = c;
              break;
            }
            if (c == CellClass::kFree) {
              cls = c;
            }
          }
        }
      }
      classes[static_cast<size_t>(row) * frame.width + col] = cls;
      canvas.Set(col, row,
                 cls == CellClass::kOccupied ? kOccupied
                 : cls == CellClass::kFree   ? kFree
                                             : kUnknown);
    }
  }
  return classes;
}

void DrawRobot(const Eigen::Affine2d& robot, const ViewFrame& frame, Canvas& canvas) {
  const double mpp = frame.metres_per_pixel;
  const Eigen::Array2i center = frame.ToPixel(robot.translation());
  const Eigen::Vector2d heading = robot.linear().col(0).normalized();
  // Pixel rows grow downwards, so the image-space heading flips y.
  const Eigen::Vector2d dir(heading.x(), -heading.y());
  const Eigen::Vector2d normal(-dir.y(), dir.x());
  const double length = RobotArrowLengthPx(mpp);
  const Eigen::Vector2d c(center.x() + 0.5, center.y() + 0.5);
  canvas.FillTriangle(c + dir * length, c - dir * (0.35 * length) + normal * (0.4 * length),
                      c - dir * (0.35 * length) - normal * (0.4 * length), kRobot);
  const int radius = std::max(3, static_cast<int>(std::lround(0.15 / mpp)));
  canvas.FillDisc(center, radius + 1, kInk);
  canvas.FillDisc(center, radius, kRobot);
}

std::string BadgeText(const RenderMarker& marker) {
  return marker.number > 0 ? std::to_string(marker.number) : std::string("T");
}

// First candidate box around the dot that lies on free map pixels and clear of everything placed.
PixelBox PlaceBadge(const Eigen::Array2i& dot, int dot_radius, int w, int h,
                    const std::vector<CellClass>& classes, bool require_free,
                    const ViewFrame& frame, const std::vector<PixelBox>& blocked) {
  const auto box_at = [&](double cx, double cy) {
    const int x0 = static_cast<int>(std::lround(cx - w / 2.0));
    const int y0 = static_cast<int>(std::lround(cy - h / 2.0));
    return PixelBox{x0, y0, x0 + w, y0 + h};
  };
  const auto inside = [&](const PixelBox& b) {
    return b.x0 >= 0 && b.y0 >= 0 && b.x1 <= frame.width && b.y1 <= frame.height;
  };
  const auto clear = [&](const PixelBox& b) {
    for (const PixelBox& other : blocked) {
      if (b.Overlaps(other)) {
        return false;
      }
    }
    return true;
  };
  const auto on_free = [&](const PixelBox& b) {
    for (int y = b.y0; y < b.y1; ++y) {
      for (int x = b.x0; x < b.x1; ++x) {
        if (classes[static_cast<size_t>(y) * frame.width + x] != CellClass::kFree) {
          return false;
        }
      }
    }
    return true;
  };
  constexpr int kDirections = 16;
  for (const bool strict : {true, false}) {
    for (const int gap : {8, 16, 28, 44, 64}) {
      const double reach = dot_radius + gap + std::max(w, h) / 2.0;
      for (int i = 0; i < kDirections; ++i) {
        // Start at north-east and sweep clockwise.
        const double angle = kPi / 4.0 - 2.0 * kPi * i / kDirections;
        const PixelBox b =
            box_at(dot.x() + reach * std::cos(angle), dot.y() - reach * std::sin(angle));
        if (inside(b) && clear(b) && (!strict || !require_free || on_free(b))) {
          return b;
        }
      }
    }
  }
  return box_at(dot.x() + dot_radius + 8 + w / 2.0, dot.y() - dot_radius - 8 - h / 2.0);
}

void DrawMarkers(const RenderInput& input, const Plan& plan, const ViewFrame& frame,
                 const std::vector<CellClass>& classes, Canvas& canvas,
                 std::vector<PixelBox>& blocked) {
  const int radius = std::max(
      kMinMarkerRadiusPx, static_cast<int>(std::lround(kMarkerRadiusM / frame.metres_per_pixel)));
  std::vector<size_t> visible;
  for (size_t i = 0; i < input.markers.size(); ++i) {
    const RenderMarker& marker = input.markers[i];
    if (!MarkerEnabled(plan, marker)) {
      continue;
    }
    const Eigen::Array2i p = frame.ToPixel(marker.xy);
    if (!canvas.Contains(p.x(), p.y())) {
      continue;
    }
    visible.push_back(i);
    blocked.push_back(
        {p.x() - radius - 1, p.y() - radius - 1, p.x() + radius + 2, p.y() + radius + 2});
  }
  for (const size_t i : visible) {
    const RenderMarker& marker = input.markers[i];
    const Eigen::Array2i p = frame.ToPixel(marker.xy);
    const Rgb fill = marker.highlight ? kTarget : kPlace;
    if (marker.highlight) {
      canvas.StrokeCircle(p, radius + 4, 2, kTarget);
    }
    canvas.FillDisc(p, radius, kInk);
    canvas.FillDisc(p, radius - 1, fill);
  }
  for (const size_t i : visible) {
    const RenderMarker& marker = input.markers[i];
    const Eigen::Array2i p = frame.ToPixel(marker.xy);
    const std::string text = BadgeText(marker);
    const int w = TextWidth(text, kBadgeScale) + 6;
    const int h = TextHeight(kBadgeScale) + 6;
    const PixelBox box = PlaceBadge(p, radius, w, h, classes, plan.grid, frame, blocked);
    blocked.push_back(box);
    const Eigen::Array2i box_center((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2);
    canvas.Line(p, box_center, kInk);
    canvas.FillBox(box, kFree);
    canvas.StrokeBox(box, marker.highlight ? kTarget : kInk);
    canvas.Text({box.x0 + 3, box.y0 + 3}, text, kBadgeScale, kInk);
  }
}

std::string Sanitize(const std::string& text) {
  std::string out = text;
  for (char& c : out) {
    if (c < 0x20 || c > 0x7e || c == ';') {
      c = '?';
    }
  }
  return out;
}

std::string Join(const std::vector<std::string>& items) {
  std::string out;
  for (const std::string& item : items) {
    out += (out.empty() ? "" : ",") + item;
  }
  return out;
}

std::string MakeLegend(const RenderInput& input, const Plan& plan, const ViewFrame& frame) {
  std::string legend = "layers=" + Join(plan.drawn);
  if (!plan.dropped.empty()) {
    legend += "; dropped=" + Join(plan.dropped);
  }
  std::vector<RenderMarker> listed;
  std::copy_if(input.markers.begin(), input.markers.end(), std::back_inserter(listed),
               [&plan](const RenderMarker& marker) { return MarkerEnabled(plan, marker); });
  std::stable_sort(listed.begin(), listed.end(), [](const RenderMarker& a, const RenderMarker& b) {
    return a.number < b.number;
  });
  for (const RenderMarker& marker : listed) {
    legend += "; " + BadgeText(marker) + " " + Sanitize(marker.label);
    if (input.robot) {
      char distance[32];
      std::snprintf(distance, sizeof(distance), " %.1fm",
                    (marker.xy - input.robot->translation()).norm());
      legend += distance;
    }
    if (marker.highlight) {
      legend += " (target)";
    }
    const Eigen::Array2i p = frame.ToPixel(marker.xy);
    if (p.x() < 0 || p.y() < 0 || p.x() >= frame.width || p.y() >= frame.height) {
      legend += " (off view)";
    }
  }
  return legend;
}

}  // namespace

RenderedView RenderView(const RenderInput& input) {
  const Plan plan = MakePlan(input);
  const ViewFrame frame = MakeFrame(input, plan);
  RenderedView view{Canvas(frame.width, frame.height, kUnknown), frame, ""};
  Canvas& canvas = view.canvas;

  std::vector<CellClass> classes(static_cast<size_t>(frame.width) * frame.height,
                                 CellClass::kUnknown);
  if (plan.grid && input.grid.width() > 0 && input.grid.height() > 0) {
    classes = PaintGrid(input.grid, frame, canvas);
  }
  DrawGridLines(frame, kOccupied, canvas);

  if (plan.submaps) {
    for (const auto& corners : input.submap_outlines) {
      for (size_t i = 0; i < corners.size(); ++i) {
        canvas.Line(frame.ToPixel(corners[i]), frame.ToPixel(corners[(i + 1) % corners.size()]),
                    kSubmap);
      }
    }
  }
  if (plan.trail) {
    for (size_t i = 1; i < input.trail.size(); ++i) {
      canvas.Line(frame.ToPixel(input.trail[i - 1]), frame.ToPixel(input.trail[i]), kTrail, 2);
    }
    if (input.trail.size() == 1) {
      canvas.FillDisc(frame.ToPixel(input.trail.front()), 1, kTrail);
    }
  }
  if (plan.scan) {
    for (const sensor::Point2d& point : *input.scan) {
      canvas.FillDisc(frame.ToPixel(*input.robot * point.point), 1, kScan);
    }
  }

  std::vector<PixelBox> blocked;
  if (plan.robot) {
    DrawRobot(*input.robot, frame, canvas);
    const Eigen::Array2i p = frame.ToPixel(input.robot->translation());
    const int reach = static_cast<int>(RobotArrowLengthPx(frame.metres_per_pixel)) + 1;
    blocked.push_back({p.x() - reach, p.y() - reach, p.x() + reach + 1, p.y() + reach + 1});
  }
  DrawMarkers(input, plan, frame, classes, canvas, blocked);
  DrawScaleBar(frame, canvas);

  view.legend = MakeLegend(input, plan, frame);
  return view;
}

RenderOutput Render(const RenderInput& input) {
  RenderedView view = RenderView(input);
  RenderOutput output;
  output.width = view.canvas.width();
  output.height = view.canvas.height();
  output.png = EncodePng(view.canvas.rgb(), output.width, output.height);
  output.legend = std::move(view.legend);
  return output;
}

}  // namespace evergreenslam::agent

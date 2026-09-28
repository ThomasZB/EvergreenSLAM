/**
 * @file grid_overlay.cc
 * @author hang chen (chen@hang.plus)
 * @brief Metric grid lines with labels and a scale bar over a rendered view.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/render/grid_overlay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "service/render/bitmap_font.h"

namespace evergreenslam::agent {
namespace {

constexpr double kMinGridSpacingPx = 18.0;
constexpr double kMinLabelSpacingPx = 40.0;

const Rgb kGridLine{176, 200, 236};
const Rgb kGridLabel{60, 80, 130};
const Rgb kPaper{255, 255, 255};
const Rgb kInk{0, 0, 0};

double PickStep(const std::vector<double>& candidates, double mpp, double min_px) {
  for (const double step : candidates) {
    if (step / mpp >= min_px) {
      return step;
    }
  }
  return candidates.back();
}

std::string FormatMetres(double value) {
  char buffer[32];
  if (std::abs(value - std::round(value)) < 1e-6) {
    std::snprintf(buffer, sizeof(buffer), "%ld", std::lround(value));
  } else {
    std::snprintf(buffer, sizeof(buffer), "%.1f", value);
  }
  return buffer;
}

void DrawLabel(Canvas& canvas, const Eigen::Array2i& top_left, const std::string& text, Rgb ink) {
  canvas.FillBox({top_left.x() - 1, top_left.y() - 1, top_left.x() + TextWidth(text, 1) + 1,
                  top_left.y() + TextHeight(1) + 1},
                 kPaper);
  canvas.Text(top_left, text, 1, ink);
}

}  // namespace

void DrawGridLines(const ViewFrame& frame, Rgb keep, Canvas& canvas) {
  const double mpp = frame.metres_per_pixel;
  std::vector<double> steps;
  for (double decade = 1.0; decade <= 1e6; decade *= 10.0) {
    steps.insert(steps.end(), {decade, 2.0 * decade, 5.0 * decade});
  }
  const double step = PickStep(steps, mpp, kMinGridSpacingPx);
  const long step_units = std::lround(step);
  const long label_base = step_units % 5 == 0 ? step_units : step_units * 5;
  std::vector<double> label_steps;
  for (long factor = 1; factor <= 1000000; factor *= 10) {
    for (const long k : {1L, 2L, 5L}) {
      label_steps.push_back(static_cast<double>(label_base * k * factor));
    }
  }
  const long label_step = std::lround(PickStep(label_steps, mpp, kMinLabelSpacingPx));

  const double max_x = frame.min_x + frame.width * mpp;
  const double min_y = frame.max_y - frame.height * mpp;
  std::vector<std::pair<Eigen::Array2i, std::string>> labels;
  for (long k = static_cast<long>(std::ceil(frame.min_x / step)); k * step < max_x; ++k) {
    const int col = frame.ToPixel(Eigen::Vector2d(k * step, 0.0)).x();
    if (col < 0 || col >= frame.width) {
      continue;
    }
    for (int row = 0; row < frame.height; ++row) {
      if (canvas.Get(col, row) != keep) {
        canvas.Set(col, row, kGridLine);
      }
    }
    if ((k * step_units) % label_step == 0) {
      labels.emplace_back(Eigen::Array2i(col + 2, 2), std::to_string(k * step_units));
    }
  }
  for (long k = static_cast<long>(std::ceil(min_y / step)); k * step < frame.max_y; ++k) {
    const int row = frame.ToPixel(Eigen::Vector2d(0.0, k * step)).y();
    if (row < 0 || row >= frame.height) {
      continue;
    }
    for (int col = 0; col < frame.width; ++col) {
      if (canvas.Get(col, row) != keep) {
        canvas.Set(col, row, kGridLine);
      }
    }
    // Keep the top strip for the x labels.
    if ((k * step_units) % label_step == 0 && row - TextHeight(1) - 2 > TextHeight(1) + 3) {
      labels.emplace_back(Eigen::Array2i(2, row - TextHeight(1) - 1),
                          std::to_string(k * step_units));
    }
  }
  for (const auto& [top_left, text] : labels) {
    DrawLabel(canvas, top_left, text, kGridLabel);
  }
}

void DrawScaleBar(const ViewFrame& frame, Canvas& canvas) {
  const double mpp = frame.metres_per_pixel;
  double length_m = 0.0;
  for (double decade = 0.1; decade <= 1e6; decade *= 10.0) {
    for (const double k : {1.0, 2.0, 5.0}) {
      if (k * decade / mpp <= frame.width / 4.0) {
        length_m = k * decade;
      }
    }
  }
  if (length_m <= 0.0) {
    return;
  }
  const int length_px = static_cast<int>(std::lround(length_m / mpp));
  const std::string text = FormatMetres(length_m) + "m";
  const int x0 = 6;
  const int bar_y = frame.height - 6;
  const int text_y = bar_y - 4 - TextHeight(1);
  canvas.FillBox({x0 - 3, text_y - 2, x0 + std::max(length_px, TextWidth(text, 1)) + 4, bar_y + 4},
                 kPaper);
  canvas.FillBox({x0, bar_y - 1, x0 + length_px + 1, bar_y + 1}, kInk);
  canvas.FillBox({x0, bar_y - 4, x0 + 1, bar_y + 1}, kInk);
  canvas.FillBox({x0 + length_px, bar_y - 4, x0 + length_px + 1, bar_y + 1}, kInk);
  canvas.Text({x0, text_y}, text, 1, kInk);
}

}  // namespace evergreenslam::agent

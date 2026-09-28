/**
 * @file canvas.cc
 * @author hang chen (chen@hang.plus)
 * @brief RGB raster with clipped, non-antialiased drawing primitives in pixel coordinates.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/render/canvas.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "service/render/bitmap_font.h"

namespace evergreenslam::agent {

Canvas::Canvas(int width, int height, Rgb background)
    : width_(std::max(width, 0)),
      height_(std::max(height, 0)),
      rgb_(static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3) {
  for (size_t i = 0; i < rgb_.size(); i += 3) {
    rgb_[i] = background.r;
    rgb_[i + 1] = background.g;
    rgb_[i + 2] = background.b;
  }
}

Rgb Canvas::Get(int x, int y) const {
  if (!Contains(x, y)) {
    return Rgb{};
  }
  const size_t i = (static_cast<size_t>(y) * width_ + x) * 3;
  return Rgb{rgb_[i], rgb_[i + 1], rgb_[i + 2]};
}

void Canvas::Set(int x, int y, Rgb color) {
  if (!Contains(x, y)) {
    return;
  }
  const size_t i = (static_cast<size_t>(y) * width_ + x) * 3;
  rgb_[i] = color.r;
  rgb_[i + 1] = color.g;
  rgb_[i + 2] = color.b;
}

void Canvas::FillBox(const PixelBox& box, Rgb color) {
  const int x0 = std::max(box.x0, 0);
  const int x1 = std::min(box.x1, width_);
  const int y0 = std::max(box.y0, 0);
  const int y1 = std::min(box.y1, height_);
  for (int y = y0; y < y1; ++y) {
    for (int x = x0; x < x1; ++x) {
      Set(x, y, color);
    }
  }
}

void Canvas::StrokeBox(const PixelBox& box, Rgb color) {
  FillBox({box.x0, box.y0, box.x1, box.y0 + 1}, color);
  FillBox({box.x0, box.y1 - 1, box.x1, box.y1}, color);
  FillBox({box.x0, box.y0, box.x0 + 1, box.y1}, color);
  FillBox({box.x1 - 1, box.y0, box.x1, box.y1}, color);
}

void Canvas::Line(const Eigen::Array2i& from, const Eigen::Array2i& to, Rgb color, int thickness) {
  const int offset = (thickness - 1) / 2;
  int x = from.x();
  int y = from.y();
  const int dx = std::abs(to.x() - x);
  const int dy = -std::abs(to.y() - y);
  const int sx = x < to.x() ? 1 : -1;
  const int sy = y < to.y() ? 1 : -1;
  const bool same_side_outside = (x < 0 && to.x() < 0) || (y < 0 && to.y() < 0) ||
                                 (x >= width_ && to.x() >= width_) ||
                                 (y >= height_ && to.y() >= height_);
  if (same_side_outside) {
    return;
  }
  int error = dx + dy;
  while (true) {
    FillBox({x - offset, y - offset, x - offset + thickness, y - offset + thickness}, color);
    if (x == to.x() && y == to.y()) {
      break;
    }
    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y += sy;
    }
  }
}

void Canvas::FillDisc(const Eigen::Array2i& center, int radius, Rgb color) {
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy <= radius * radius + radius) {
        Set(center.x() + dx, center.y() + dy, color);
      }
    }
  }
}

void Canvas::StrokeCircle(const Eigen::Array2i& center, int radius, int thickness, Rgb color) {
  const int outer = radius * radius + radius;
  const int inner_radius = std::max(radius - thickness, 0);
  const int inner = inner_radius * inner_radius + inner_radius;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int d2 = dx * dx + dy * dy;
      if (d2 <= outer && d2 > inner) {
        Set(center.x() + dx, center.y() + dy, color);
      }
    }
  }
}

void Canvas::FillTriangle(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
                          const Eigen::Vector2d& c, Rgb color) {
  const auto edge = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q,
                       const Eigen::Vector2d& r) {
    return (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x());
  };
  const double area = edge(a, b, c);
  if (area == 0.0) {
    return;
  }
  const int x0 = std::max(static_cast<int>(std::floor(std::min({a.x(), b.x(), c.x()}))), 0);
  const int x1 = std::min(static_cast<int>(std::ceil(std::max({a.x(), b.x(), c.x()}))), width_);
  const int y0 = std::max(static_cast<int>(std::floor(std::min({a.y(), b.y(), c.y()}))), 0);
  const int y1 = std::min(static_cast<int>(std::ceil(std::max({a.y(), b.y(), c.y()}))), height_);
  for (int y = y0; y < y1; ++y) {
    for (int x = x0; x < x1; ++x) {
      const Eigen::Vector2d p(x + 0.5, y + 0.5);
      const double w0 = edge(b, c, p) / area;
      const double w1 = edge(c, a, p) / area;
      const double w2 = edge(a, b, p) / area;
      if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) {
        Set(x, y, color);
      }
    }
  }
}

void Canvas::Text(const Eigen::Array2i& top_left, const std::string& text, int scale, Rgb color) {
  int pen_x = top_left.x();
  for (const char c : text) {
    const GlyphRows rows = Glyph(c);
    for (int row = 0; row < kGlyphHeight; ++row) {
      for (int col = 0; col < kGlyphWidth; ++col) {
        if ((rows[row] >> (kGlyphWidth - 1 - col)) & 1) {
          FillBox({pen_x + col * scale, top_left.y() + row * scale, pen_x + (col + 1) * scale,
                   top_left.y() + (row + 1) * scale},
                  color);
        }
      }
    }
    pen_x += (kGlyphWidth + 1) * scale;
  }
}

}  // namespace evergreenslam::agent

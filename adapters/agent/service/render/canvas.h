/**
 * @file canvas.h
 * @author hang chen (chen@hang.plus)
 * @brief RGB raster with clipped, non-antialiased drawing primitives in pixel coordinates.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_CANVAS_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_CANVAS_H_

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

namespace evergreenslam::agent {

struct Rgb {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  bool operator==(const Rgb& other) const { return r == other.r && g == other.g && b == other.b; }
  bool operator!=(const Rgb& other) const { return !(*this == other); }
};

// Half-open pixel box [x0, x1) x [y0, y1).
struct PixelBox {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  bool Overlaps(const PixelBox& other) const {
    return x0 < other.x1 && other.x0 < x1 && y0 < other.y1 && other.y0 < y1;
  }
};

// Pixel coordinates are (column, row), row 0 at the top. Everything clips to the image.
class Canvas {
 public:
  Canvas(int width, int height, Rgb background);

  int width() const { return width_; }
  int height() const { return height_; }
  const std::vector<uint8_t>& rgb() const { return rgb_; }

  bool Contains(int x, int y) const { return x >= 0 && x < width_ && y >= 0 && y < height_; }
  // Outside the image reads as black.
  Rgb Get(int x, int y) const;
  void Set(int x, int y, Rgb color);

  void FillBox(const PixelBox& box, Rgb color);
  void StrokeBox(const PixelBox& box, Rgb color);
  // Square brush of `thickness` pixels.
  void Line(const Eigen::Array2i& from, const Eigen::Array2i& to, Rgb color, int thickness = 1);
  void FillDisc(const Eigen::Array2i& center, int radius, Rgb color);
  void StrokeCircle(const Eigen::Array2i& center, int radius, int thickness, Rgb color);
  // Corners in continuous pixel coordinates; fills the pixels whose centres lie inside.
  void FillTriangle(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c,
                    Rgb color);
  // Diagonal hatch lines `period` pixels apart over the pixels whose centres lie inside the
  // polygon (even-odd rule); corners in continuous pixel coordinates.
  void HatchPolygon(const std::vector<Eigen::Vector2d>& corners, int period, Rgb color);
  // `top_left` is the first glyph's top-left pixel.
  void Text(const Eigen::Array2i& top_left, const std::string& text, int scale, Rgb color);

 private:
  int width_;
  int height_;
  std::vector<uint8_t> rgb_;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_CANVAS_H_

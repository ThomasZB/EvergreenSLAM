/**
 * @file renderer_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief View renderer on a synthetic 20 x 20 m room with a wall.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/render/renderer.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::agent {
namespace {

constexpr double kResolution = 0.05;
constexpr double kOrigin = -10.0;

const Rgb kBlack{0, 0, 0};
const Rgb kWhite{255, 255, 255};
const Rgb kGrey{205, 205, 205};
const Rgb kRobotGreen{0, 160, 0};

// Free for x >= -8 m, unknown strip on the left, a wall at x ~ 3.03 m and a block near the top.
mapping::GridMapu8 MakeRoom(int cells_x = 400, int cells_y = 400) {
  std::vector<uint8_t> data(static_cast<size_t>(cells_x) * cells_y, mapping::kUnknownValue);
  const uint8_t free_value = mapping::ProbabilityToValue(0.2);
  const uint8_t occupied_value = mapping::ProbabilityToValue(0.9);
  const auto at = [&](int x, int y) -> uint8_t& {
    return data[static_cast<size_t>(y) * cells_x + x];
  };
  for (int y = 0; y < cells_y; ++y) {
    for (int x = 40; x < cells_x; ++x) {
      at(x, y) = free_value;
    }
  }
  // One cell thick; every other cell carries the update marker, which must not change its class.
  const int wall_x = static_cast<int>(std::floor((3.03 - kOrigin) / kResolution));
  for (int y = 80; y < 320; ++y) {
    at(wall_x, y) = y % 2 == 0 ? occupied_value : occupied_value + mapping::kUpdateMarker;
  }
  for (int y = 387; y < 393; ++y) {
    for (int x = 327; x < 333; ++x) {
      at(x, y) = occupied_value;
    }
  }
  return mapping::GridMapu8(std::move(data), cells_x, cells_y, kResolution, kOrigin, kOrigin,
                            mapping::kUnknownValue);
}

Eigen::Affine2d Pose(double x, double y, double theta) {
  Eigen::Affine2d pose = Eigen::Affine2d::Identity();
  pose.translation() = Eigen::Vector2d(x, y);
  pose.linear() = Eigen::Rotation2Dd(theta).toRotationMatrix();
  return pose;
}

Rgb PixelAt(const RenderedView& view, const Eigen::Vector2d& xy) {
  const Eigen::Array2i p = view.frame.ToPixel(xy);
  return view.canvas.Get(p.x(), p.y());
}

uint32_t ReadBigEndian(const std::vector<uint8_t>& bytes, size_t offset) {
  return (static_cast<uint32_t>(bytes[offset]) << 24) |
         (static_cast<uint32_t>(bytes[offset + 1]) << 16) |
         (static_cast<uint32_t>(bytes[offset + 2]) << 8) | static_cast<uint32_t>(bytes[offset + 3]);
}

void MaybeDump(const RenderOutput& output, const std::string& name) {
  const char* dir = std::getenv("EGS_RENDER_DUMP_DIR");
  if (dir == nullptr) {
    return;
  }
  std::ofstream file(std::string(dir) + "/" + name + ".png", std::ios::binary);
  file.write(reinterpret_cast<const char*>(output.png.data()),
             static_cast<std::streamsize>(output.png.size()));
}

// 400 cells fit twice into 1024 px: two pixels per cell.
TEST(RendererTest, MapIsTrinaryNorthUpAndUpscaled) {
  RenderInput input{MakeRoom()};
  input.layers = {"map"};
  const RenderedView view = RenderView(input);

  EXPECT_EQ(view.canvas.width(), 800);
  EXPECT_EQ(view.canvas.height(), 800);
  EXPECT_EQ(view.frame.pixels_per_cell, 2);
  EXPECT_DOUBLE_EQ(view.frame.metres_per_pixel, kResolution / 2.0);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(3.03, 0.47)), kBlack);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(3.03, 0.52)), kBlack);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(1.53, 0.47)), kWhite);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(-9.33, 0.47)), kGrey);
  // Row 0 is max y: the block centred at (6.5, 9.5) lands near the top, not the bottom.
  EXPECT_EQ(view.canvas.Get(660, 20), kBlack);
  EXPECT_EQ(view.canvas.Get(660, 774), kWhite);
}

TEST(RendererTest, PngHeaderMatchesRaster) {
  RenderInput input{MakeRoom()};
  input.layers = {"map"};
  const RenderOutput output = Render(input);
  MaybeDump(output, "map");

  ASSERT_GT(output.png.size(), 33u);
  const uint8_t signature[] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  for (size_t i = 0; i < sizeof(signature); ++i) {
    EXPECT_EQ(output.png[i], signature[i]);
  }
  EXPECT_EQ(std::string(output.png.begin() + 12, output.png.begin() + 16), "IHDR");
  EXPECT_EQ(ReadBigEndian(output.png, 16), 800u);
  EXPECT_EQ(ReadBigEndian(output.png, 20), 800u);
  EXPECT_EQ(output.width, 800);
  EXPECT_EQ(output.height, 800);
  EXPECT_EQ(std::string(output.png.end() - 8, output.png.end() - 4), "IEND");
}

TEST(RendererTest, RobotArrowPointsAlongHeading) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "robot"};
  input.robot = Pose(1.53, -3.03, M_PI / 2.0);
  const RenderedView view = RenderView(input);

  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(1.53, -3.03)), kRobotGreen);
  // 16 px at 2 px per cell: the arrow reaches 0.4 m ahead.
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(1.53, -2.73)), kRobotGreen);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(1.53, -3.63)), kWhite);
}

// Upscaling adds map detail, not bigger glyphs: the arrow is 16 px long at 1, 2 and 4 px per cell.
TEST(RendererTest, RobotGlyphKeepsItsPixelSize) {
  for (const int cells : {1000, 400, 200}) {
    RenderInput input{MakeRoom(cells, cells)};
    input.layers = {"map", "robot"};
    input.robot = Pose(1.53, -3.03, 0.0);
    const RenderedView view = RenderView(input);
    const Eigen::Array2i center = view.frame.ToPixel(input.robot->translation());
    EXPECT_EQ(view.frame.pixels_per_cell, std::min(4, 1024 / cells));
    EXPECT_EQ(view.canvas.Get(center.x() + 14, center.y()), kRobotGreen) << cells;
    EXPECT_NE(view.canvas.Get(center.x() + 18, center.y()), kRobotGreen) << cells;
  }
}

TEST(RendererTest, ScanDotsLandOnTheWall) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "robot", "scan"};
  input.robot = Pose(1.53, 0.0, 0.0);
  sensor::PointCloud scan;
  scan.push_back({Eigen::Vector2d(1.5, 0.23)});
  input.scan = scan;
  const RenderedView view = RenderView(input);

  const Rgb dot = PixelAt(view, Eigen::Vector2d(3.03, 0.23));
  EXPECT_NE(dot, kBlack);
  EXPECT_NE(dot, kWhite);
  EXPECT_EQ(view.legend, "layers=map,robot,scan");
}

// 10 m = 200 cells: the upscale is capped at four pixels per cell.
TEST(RendererTest, EgoCropIsUpscaledAtMostFourTimes) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "robot"};
  input.robot = Pose(1.53, -3.03, 0.0);
  input.ego_radius_m = 5.0;
  const RenderedView view = RenderView(input);

  EXPECT_EQ(view.canvas.width(), 800);
  EXPECT_EQ(view.canvas.height(), 800);
  EXPECT_EQ(view.frame.pixels_per_cell, 4);
  EXPECT_EQ(view.canvas.Get(400, 400), kRobotGreen);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(3.02, -3.53)), kBlack);
}

TEST(RendererTest, LongSideIsCappedAndThinWallsSurvive) {
  RenderInput input{MakeRoom(2400, 1200)};
  input.layers = {"map"};
  const RenderedView view = RenderView(input);

  EXPECT_EQ(view.canvas.width(), 800);
  EXPECT_EQ(view.canvas.height(), 400);
  EXPECT_EQ(view.frame.pixels_per_cell, 1);
  EXPECT_DOUBLE_EQ(view.frame.metres_per_pixel, 3 * kResolution);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(3.03, 0.47)), kBlack);
}

TEST(RendererTest, LegendListsMarkersWithDistanceFromRobot) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "robot", "places"};
  input.robot = Pose(2.0, -3.0, 0.0);
  input.markers.push_back({2, Eigen::Vector2d(-2.0, 1.0), "places/kitchen", false});
  input.markers.push_back({1, Eigen::Vector2d(5.0, 5.0), "places/dock", false});
  const RenderOutput output = Render(input);
  MaybeDump(output, "places");

  EXPECT_EQ(output.legend, "layers=map,robot,places; 1 places/dock 8.5m; 2 places/kitchen 5.7m");
}

TEST(RendererTest, TargetNeedsItsLayerAndMissingRobotIsNoted) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "robot", "scan", "target"};
  input.markers.push_back({1, Eigen::Vector2d(5.53, 5.47), "places/dock", false});
  input.markers.push_back({2, Eigen::Vector2d(-2.0, 1.0), "places/kitchen", true});
  const RenderedView view = RenderView(input);

  EXPECT_EQ(view.legend, "layers=map,target; dropped=robot,scan; 2 places/kitchen (target)");
  EXPECT_NE(PixelAt(view, Eigen::Vector2d(-2.0, 1.0)), kWhite);
  EXPECT_EQ(PixelAt(view, Eigen::Vector2d(5.53, 5.47)), kWhite);
}

TEST(RendererTest, EgoMarksMarkersOutsideTheCrop) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "robot", "places", "trail", "submaps"};
  input.robot = Pose(1.53, -3.03, 0.3);
  input.ego_radius_m = 3.0;
  input.markers.push_back({1, Eigen::Vector2d(0.5, -2.0), "places/dock", false});
  input.markers.push_back({2, Eigen::Vector2d(-6.0, 6.0), "places/kitchen", false});
  input.trail = {Eigen::Vector2d(-1.0, -4.0), Eigen::Vector2d(0.0, -3.5),
                 Eigen::Vector2d(1.53, -3.03)};
  input.submap_outlines.push_back({Eigen::Vector2d(-2.0, -6.0), Eigen::Vector2d(2.5, -6.0),
                                   Eigen::Vector2d(2.5, -1.0), Eigen::Vector2d(-2.0, -1.0)});
  const RenderOutput output = Render(input);
  MaybeDump(output, "ego");

  EXPECT_EQ(output.width, 480);
  EXPECT_EQ(output.legend,
            "layers=map,robot,trail,places,submaps; 1 places/dock 1.5m; 2 places/kitchen 11.8m "
            "(off view)");
}

// No badge may be filled over the robot glyph, including the target when the robot stands on it.
TEST(RendererTest, BadgesNeverCoverTheRobot) {
  const std::vector<Eigen::Vector2d> robots = {Eigen::Vector2d(1.53, -3.03),
                                               Eigen::Vector2d(-9.9, 9.9)};
  const std::vector<Eigen::Vector2d> offsets = {
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.3, 0.0), Eigen::Vector2d(0.0, -0.4),
      Eigen::Vector2d(-0.6, 0.2)};
  for (const double ego : {0.0, 7.5}) {
    for (const Eigen::Vector2d& at : robots) {
      for (const Eigen::Vector2d& offset : offsets) {
        RenderInput input{MakeRoom()};
        input.robot = Pose(at.x(), at.y(), 0.7);
        if (ego > 0.0) {
          input.ego_radius_m = ego;
        }
        input.markers.push_back({0, at + offset, "places/dock", true});
        input.markers.push_back({1, at - offset, "places/hall", false});
        input.markers.push_back({2, at + Eigen::Vector2d(0.2, 0.5), "places/desk", false});
        // The trail spans the markers, so both renders share one frame.
        for (const RenderMarker& marker : input.markers) {
          input.trail.push_back(marker.xy);
        }
        input.layers = {"map", "robot", "trail"};
        const RenderedView robot_only = RenderView(input);
        input.layers.insert({"target", "places"});
        const RenderedView view = RenderView(input);
        ASSERT_EQ(view.canvas.width(), robot_only.canvas.width());
        ASSERT_EQ(view.canvas.height(), robot_only.canvas.height());
        int covered = 0;
        for (int row = 0; row < view.canvas.height(); ++row) {
          for (int col = 0; col < view.canvas.width(); ++col) {
            if (robot_only.canvas.Get(col, row) == kRobotGreen &&
                view.canvas.Get(col, row) == kWhite) {
              ++covered;
            }
          }
        }
        EXPECT_EQ(covered, 0) << "ego " << ego << " robot " << at.transpose() << " offset "
                              << offset.transpose();
      }
    }
  }
}

// A frame barely larger than the robot glyph leaves no in-frame spot off the robot: badges of
// places in its south-west corner must go past the frame edge instead of onto the robot.
TEST(RendererTest, TinyFrameBadgesStillAvoidTheRobot) {
  const Eigen::Vector2d at(1.53, -3.03);
  RenderInput input{MakeRoom()};
  input.robot = Pose(at.x(), at.y(), M_PI);
  input.ego_radius_m = 0.3;
  input.markers.push_back({0, at + Eigen::Vector2d(-0.27, -0.27), "places/dock", true});
  input.markers.push_back({1, at + Eigen::Vector2d(-0.25, -0.2), "places/hall", false});
  input.layers = {"map", "robot"};
  const RenderedView robot_only = RenderView(input);
  input.layers.insert({"target", "places"});
  const RenderedView view = RenderView(input);
  ASSERT_EQ(view.canvas.width(), robot_only.canvas.width());
  ASSERT_LE(view.canvas.width(), 64);
  int robot_pixels = 0;
  int covered = 0;
  for (int row = 0; row < view.canvas.height(); ++row) {
    for (int col = 0; col < view.canvas.width(); ++col) {
      if (robot_only.canvas.Get(col, row) == kRobotGreen) {
        ++robot_pixels;
        covered += view.canvas.Get(col, row) == kWhite ? 1 : 0;
      }
    }
  }
  EXPECT_GT(robot_pixels, 0);
  EXPECT_EQ(covered, 0);
}

TEST(RendererTest, ZonesAreHatchedInsideTheirPolygonOnly) {
  RenderInput input{MakeRoom()};
  input.layers = {"map", "zones"};
  // A triangle, so a bounding-box fill would show up in its empty half.
  input.zones.push_back(
      {Eigen::Vector2d(0.33, 0.33), Eigen::Vector2d(2.33, 0.33), Eigen::Vector2d(0.33, 2.33)});
  input.num_unresolved_zones = 2;
  const RenderOutput output = Render(input);
  MaybeDump(output, "zones");
  const RenderedView view = RenderView(input);

  const Rgb zone = PixelAt(view, Eigen::Vector2d(1.33, 0.33));
  EXPECT_NE(zone, kWhite);
  EXPECT_NE(zone, kBlack);
  int inside = 0;
  int inside_marked = 0;
  int outside_marked = 0;
  for (double x = 0.5; x < 2.3; x += 0.05) {
    for (double y = 0.5; y < 2.3; y += 0.05) {
      const bool marked = PixelAt(view, Eigen::Vector2d(x, y)) == zone;
      if (x + y < 2.5) {
        ++inside;
        inside_marked += marked ? 1 : 0;
      } else if (x + y > 2.9) {
        outside_marked += marked ? 1 : 0;
      }
    }
  }
  EXPECT_GT(inside_marked, inside / 5);
  EXPECT_LT(inside_marked, inside);
  EXPECT_EQ(outside_marked, 0);
  EXPECT_EQ(view.legend, "layers=map,zones; zones: 1 keepout, 2 unresolvable");

  input.layers = {"map"};
  EXPECT_EQ(PixelAt(RenderView(input), Eigen::Vector2d(1.33, 0.33)), kWhite);
}

}  // namespace
}  // namespace evergreenslam::agent

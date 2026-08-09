/**
 * @file grid_map_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/grid_map.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

namespace evergreenslam::mapping {
namespace {

constexpr double kResolution = 0.05;
constexpr int kWidth = 10;
constexpr int kHeight = 8;
constexpr double kOriginX = -0.25;
constexpr double kOriginY = -0.20;
constexpr uint8_t kUnknown = 255;

GridMapu8 MakeGrid() {
  std::vector<uint8_t> data(static_cast<size_t>(kWidth) * kHeight, 0);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      data[static_cast<size_t>(y) * kWidth + x] = static_cast<uint8_t>(y * kWidth + x);
    }
  }
  return GridMapu8(std::move(data), kWidth, kHeight, kResolution, kOriginX, kOriginY, kUnknown);
}

TEST(GridMapTest, CellRoundTripIncludingNegativeWorldCoordinates) {
  const GridMapu8 grid = MakeGrid();
  for (int y = -3; y < kHeight + 3; ++y) {
    for (int x = -3; x < kWidth + 3; ++x) {
      const Eigen::Array2i cell(x, y);
      const Eigen::Array2i round_tripped = grid.ToCell(grid.ToCenter(cell));
      EXPECT_EQ(round_tripped.x(), x) << "cell " << x << ", " << y;
      EXPECT_EQ(round_tripped.y(), y) << "cell " << x << ", " << y;
    }
  }
}

TEST(GridMapTest, JustBelowOriginIsCellMinusOneAndOutside) {
  const GridMapu8 grid = MakeGrid();
  // Truncation would fold this half cell onto cell 0 and call it inside.
  const Eigen::Vector2d point(kOriginX - 0.5 * kResolution, kOriginY - 0.5 * kResolution);
  const Eigen::Array2i cell = grid.ToCell(point);
  EXPECT_EQ(cell.x(), -1);
  EXPECT_EQ(cell.y(), -1);
  EXPECT_FALSE(grid.IsInside(cell));

  const Eigen::Vector2d mixed(kOriginX - 0.5 * kResolution, kOriginY + 1.5 * kResolution);
  const Eigen::Array2i mixed_cell = grid.ToCell(mixed);
  EXPECT_EQ(mixed_cell.x(), -1);
  EXPECT_EQ(mixed_cell.y(), 1);
  EXPECT_FALSE(grid.IsInside(mixed_cell));

  // A full cell below the origin must not alias onto cell -0 either.
  const Eigen::Vector2d lower(kOriginX - 1.5 * kResolution, kOriginY);
  EXPECT_EQ(grid.ToCell(lower).x(), -2);
}

TEST(GridMapTest, WorldPointsBelowOriginMapToNegativeCells) {
  const GridMapu8 grid = MakeGrid();
  for (int i = 1; i <= 20; ++i) {
    const float offset = static_cast<float>(i) * 0.01f;
    const Eigen::Vector2d point(kOriginX - offset, kOriginY - offset);
    const Eigen::Array2i cell = grid.ToCell(point);
    EXPECT_LT(cell.x(), 0) << "offset " << offset;
    EXPECT_LT(cell.y(), 0) << "offset " << offset;
    EXPECT_FALSE(grid.IsInside(cell)) << "offset " << offset;
  }
}

TEST(GridMapTest, ToCenterOfToCellStaysWithinHalfAResolution) {
  const GridMapu8 grid = MakeGrid();
  for (int i = -40; i <= 40; ++i) {
    for (int j = -40; j <= 40; j += 7) {
      const Eigen::Vector2d point(static_cast<float>(i) * 0.013f, static_cast<float>(j) * 0.011f);
      const Eigen::Vector2d center = grid.ToCenter(grid.ToCell(point));
      EXPECT_NEAR(center.x(), point.x(), 0.5 * kResolution + 1e-5f)
          << "point " << point.x() << ", " << point.y();
      EXPECT_NEAR(center.y(), point.y(), 0.5 * kResolution + 1e-5f)
          << "point " << point.x() << ", " << point.y();
    }
  }
}

TEST(GridMapTest, IsInsideMatchesTheHalfOpenExtent) {
  const GridMapu8 grid = MakeGrid();
  EXPECT_TRUE(grid.IsInside(0, 0));
  EXPECT_TRUE(grid.IsInside(kWidth - 1, kHeight - 1));
  EXPECT_FALSE(grid.IsInside(-1, 0));
  EXPECT_FALSE(grid.IsInside(0, -1));
  EXPECT_FALSE(grid.IsInside(kWidth, 0));
  EXPECT_FALSE(grid.IsInside(0, kHeight));
}

TEST(GridMapTest, GetValueOutsideReturnsUnknown) {
  const GridMapu8 grid = MakeGrid();
  EXPECT_FLOAT_EQ(grid.GetValue(-1, 0), kUnknown);
  EXPECT_FLOAT_EQ(grid.GetValue(0, -1), kUnknown);
  EXPECT_FLOAT_EQ(grid.GetValue(kWidth, kHeight - 1), kUnknown);
  EXPECT_FLOAT_EQ(grid.GetValue(kWidth - 1, kHeight), kUnknown);
  EXPECT_FLOAT_EQ(grid.GetValueAtPoint(Eigen::Vector2d(kOriginX - kResolution, kOriginY)),
                  kUnknown);
  EXPECT_FLOAT_EQ(
      grid.GetValueAtPoint(Eigen::Vector2d(kOriginX + (kWidth + 1) * kResolution, kOriginY)),
      kUnknown);
  EXPECT_FLOAT_EQ(grid.unknown_value(), kUnknown);
}

TEST(GridMapTest, GetValueInsideIsRowMajor) {
  const GridMapu8 grid = MakeGrid();
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const float expected = static_cast<float>(y * kWidth + x);
      EXPECT_FLOAT_EQ(grid.GetValue(x, y), expected);
      EXPECT_FLOAT_EQ(grid.GetValueAtPoint(grid.ToCenter(Eigen::Array2i(x, y))), expected);
    }
  }
}

}  // namespace
}  // namespace evergreenslam::mapping

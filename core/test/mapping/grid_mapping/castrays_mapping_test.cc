/**
 * @file castrays_mapping_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/castrays_mapping.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "mapping/grid_mapping/probability_grid.h"
#include "mapping/grid_mapping/probability_values.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::mapping {
namespace {

using Cells = std::vector<std::pair<int, int>>;

// Array2i has no bool operator==, so cells are compared as pairs, which also
// makes gtest print the whole ray on a mismatch.
Cells AsPairs(const std::vector<Eigen::Array2i>& cells) {
  Cells pairs;
  pairs.reserve(cells.size());
  for (const Eigen::Array2i& cell : cells) {
    pairs.emplace_back(cell.x(), cell.y());
  }
  return pairs;
}

Cells Ray(int begin_x, int begin_y, int end_x, int end_y) {
  return AsPairs(CastRay(Eigen::Array2i(begin_x, begin_y), Eigen::Array2i(end_x, end_y)));
}

constexpr float kResolution = 0.1f;
const Eigen::Vector2d kSensorOrigin(0.f, 0.f);

sensor::PointCloud WallScan() {
  // Three points on a wall about a metre away, spread far enough apart in y
  // that no ray ends on another ray's free cells.
  sensor::PointCloud cloud;
  cloud.push_back(sensor::Point2d{Eigen::Vector2d(1.07f, 0.02f)});
  cloud.push_back(sensor::Point2d{Eigen::Vector2d(1.07f, 0.32f)});
  cloud.push_back(sensor::Point2d{Eigen::Vector2d(1.07f, -0.28f)});
  return cloud;
}

TEST(CastRayTest, HorizontalRay) {
  const Cells expected = {{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  EXPECT_EQ(Ray(0, 0, 3, 0), expected);
}

TEST(CastRayTest, DiagonalRay) {
  const Cells expected = {{0, 0}, {1, 1}, {2, 2}, {3, 3}};
  EXPECT_EQ(Ray(0, 0, 3, 3), expected);
}

TEST(CastRayTest, DegenerateRayIsASingleCell) {
  const Cells expected = {{0, 0}};
  EXPECT_EQ(Ray(0, 0, 0, 0), expected);
  const Cells shifted = {{-5, 7}};
  EXPECT_EQ(Ray(-5, 7, -5, 7), shifted);
}

TEST(CastRayTest, ShallowOctant) {
  // Slope 0.4 never lands on a half cell boundary, so every round to nearest
  // Bresenham agrees on this one; (4, 2) would leave the ties to the variant.
  const Cells expected = {{0, 0}, {1, 0}, {2, 1}, {3, 1}, {4, 2}, {5, 2}};
  EXPECT_EQ(Ray(0, 0, 5, 2), expected);
}

TEST(CastRayTest, SteepOctant) {
  const Cells expected = {{0, 0}, {0, 1}, {1, 2}, {1, 3}, {2, 4}, {2, 5}};
  EXPECT_EQ(Ray(0, 0, 2, 5), expected);
}

TEST(CastRayTest, HalfSlopeOctantStaysOnTheLine) {
  // Slope 0.5 puts every other step exactly on a tie, so only the properties
  // shared by all Bresenham variants are asserted.
  const Cells ray = Ray(0, 0, 4, 2);
  ASSERT_EQ(ray.size(), 5u);
  EXPECT_EQ(ray.front(), std::make_pair(0, 0));
  EXPECT_EQ(ray.back(), std::make_pair(4, 2));
  for (size_t i = 0; i < ray.size(); ++i) {
    const int x = static_cast<int>(i);
    EXPECT_EQ(ray[i].first, x);
    EXPECT_GE(ray[i].second, x / 2);
    EXPECT_LE(ray[i].second, (x + 1) / 2);
    if (i > 0) {
      EXPECT_GE(ray[i].second, ray[i - 1].second);
    }
  }
}

TEST(CastRayTest, NegativeDirections) {
  const Cells horizontal = {{0, 0}, {-1, 0}, {-2, 0}, {-3, 0}};
  EXPECT_EQ(Ray(0, 0, -3, 0), horizontal);

  const Cells vertical = {{0, 0}, {0, -1}, {0, -2}, {0, -3}};
  EXPECT_EQ(Ray(0, 0, 0, -3), vertical);

  const Cells diagonal = {{2, 1}, {1, 0}, {0, -1}, {-1, -2}};
  EXPECT_EQ(Ray(2, 1, -1, -2), diagonal);

  const Cells octant = {{0, 0}, {-1, 0}, {-2, -1}, {-3, -1}, {-4, -2}, {-5, -2}};
  EXPECT_EQ(Ray(0, 0, -5, -2), octant);
}

TEST(CastRayTest, EndpointsAreIncludedInOrder) {
  const std::vector<Eigen::Array2i> ray = CastRay(Eigen::Array2i(-7, 3), Eigen::Array2i(11, -9));
  ASSERT_FALSE(ray.empty());
  EXPECT_EQ(ray.front().x(), -7);
  EXPECT_EQ(ray.front().y(), 3);
  EXPECT_EQ(ray.back().x(), 11);
  EXPECT_EQ(ray.back().y(), -9);
  EXPECT_EQ(ray.size(), 19u);
  for (size_t i = 1; i < ray.size(); ++i) {
    EXPECT_EQ(ray[i].x() - ray[i - 1].x(), 1);
    const int dy = ray[i].y() - ray[i - 1].y();
    EXPECT_GE(dy, -1);
    EXPECT_LE(dy, 0);
  }
}

TEST(CastRaysMappingTest, ScanMarksEndpointsOccupiedAndTheBeamFree) {
  ProbabilityGrid grid(kResolution);
  ASSERT_TRUE(grid.empty());

  const CastRaysMapping mapping{CastRaysMappingOption()};
  const sensor::PointCloud cloud = WallScan();
  mapping.Insert(kSensorOrigin, cloud, grid);

  ASSERT_FALSE(grid.empty());
  const Eigen::Array2i origin_cell = grid.ToCell(kSensorOrigin);

  for (size_t i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector2d& point = cloud[i].point;
    const Eigen::Array2i end_cell = grid.ToCell(point);
    ASSERT_TRUE(grid.IsInside(end_cell)) << "point " << i;
    EXPECT_TRUE(grid.IsKnown(end_cell)) << "point " << i;
    // An inverted convention shows up right here.
    EXPECT_GT(grid.GetProbability(end_cell), 0.5f) << "point " << i;

    const std::vector<Eigen::Array2i> ray = CastRay(origin_cell, end_cell);
    ASSERT_GT(ray.size(), 2u) << "point " << i;
    for (size_t j = 1; j + 1 < ray.size(); ++j) {
      EXPECT_TRUE(grid.IsKnown(ray[j])) << "point " << i << ", cell " << j;
      EXPECT_LT(grid.GetProbability(ray[j]), 0.5f) << "point " << i << ", cell " << j;
    }

    for (int beyond = 1; beyond <= 3; ++beyond) {
      const Eigen::Array2i cell = end_cell + Eigen::Array2i(beyond, 0);
      EXPECT_FALSE(grid.IsKnown(cell)) << "point " << i << ", beyond " << beyond;
      EXPECT_NEAR(grid.GetProbability(cell), kUnknownProbability, 1e-6f)
          << "point " << i << ", beyond " << beyond;
    }
  }
}

TEST(CastRaysMappingTest, RepeatedInsertionsSaturateWithoutOvershooting) {
  ProbabilityGrid grid(kResolution);
  const CastRaysMapping mapping{CastRaysMappingOption()};
  const sensor::PointCloud cloud = WallScan();
  const Eigen::Vector2d& wall_point = cloud[0].point;

  float hit_probability = 0.f;
  float free_probability = 0.f;
  for (int i = 0; i < 60; ++i) {
    mapping.Insert(kSensorOrigin, cloud, grid);

    // The grid may have moved its origin, so the cells are looked up afresh.
    const Eigen::Array2i origin_cell = grid.ToCell(kSensorOrigin);
    const Eigen::Array2i end_cell = grid.ToCell(wall_point);
    const std::vector<Eigen::Array2i> ray = CastRay(origin_cell, end_cell);
    ASSERT_GT(ray.size(), 2u);
    const Eigen::Array2i free_cell = ray[ray.size() / 2];

    const float hit = grid.GetProbability(end_cell);
    const float freed = grid.GetProbability(free_cell);
    EXPECT_LE(hit, kMaxProbability + 1e-6f) << "iteration " << i;
    EXPECT_GE(freed, kMinProbability - 1e-6f) << "iteration " << i;
    if (i > 0) {
      EXPECT_GE(hit, hit_probability - 1e-6f) << "iteration " << i;
      EXPECT_LE(freed, free_probability + 1e-6f) << "iteration " << i;
    }
    hit_probability = hit;
    free_probability = freed;
  }

  EXPECT_NEAR(hit_probability, kMaxProbability, 1e-4f);
  EXPECT_LT(free_probability, 0.2f);
}

TEST(CastRaysMappingTest, HitOnlyModeLeavesTheBeamUnknown) {
  ProbabilityGrid grid(kResolution);
  CastRaysMappingOption options;
  options.insert_free_space = false;
  const CastRaysMapping mapping{options};
  const sensor::PointCloud cloud = WallScan();
  mapping.Insert(kSensorOrigin, cloud, grid);

  const Eigen::Array2i origin_cell = grid.ToCell(kSensorOrigin);
  const Eigen::Array2i end_cell = grid.ToCell(cloud[0].point);
  EXPECT_GT(grid.GetProbability(end_cell), 0.5f);

  const std::vector<Eigen::Array2i> ray = CastRay(origin_cell, end_cell);
  ASSERT_GT(ray.size(), 2u);
  for (size_t j = 1; j + 1 < ray.size(); ++j) {
    EXPECT_FALSE(grid.IsKnown(ray[j])) << "cell " << j;
  }
}

}  // namespace
}  // namespace evergreenslam::mapping

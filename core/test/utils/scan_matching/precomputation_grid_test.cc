/**
 * @file precomputation_grid_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/scan_matching/precomputation_grid.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::utils::scan_matching {
namespace {

constexpr int kWidth = 23;
constexpr int kHeight = 17;
constexpr int kNumDepths = 5;

mapping::GridMapu8 MakeRandomGrid(uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> value(0, mapping::kMaxValue);
  std::uniform_int_distribution<int> keep(0, 3);
  std::vector<uint8_t> data(static_cast<size_t>(kWidth) * kHeight, mapping::kUnknownValue);
  for (uint8_t& cell : data) {
    // A mix of unknown and known cells, so window maxima cross the unknown/known boundary too.
    if (keep(rng) != 0) {
      cell = static_cast<uint8_t>(value(rng));
    }
  }
  return mapping::GridMapu8(std::move(data), kWidth, kHeight, 0.05, -0.31, 0.27,
                            mapping::kUnknownValue);
}

uint8_t NaiveWindowMax(const mapping::GridMapu8& grid, int x, int y, int window) {
  uint8_t max_value = mapping::kUnknownValue;
  for (int dy = 0; dy < window; ++dy) {
    for (int dx = 0; dx < window; ++dx) {
      max_value = std::max(max_value, grid.GetValue(x + dx, y + dy));
    }
  }
  return max_value;
}

TEST(PrecomputationGridTest, EveryLevelEqualsTheNaiveWindowMaximumEverywhere) {
  for (uint32_t seed : {7u, 8u, 9u}) {
    const mapping::GridMapu8 grid = MakeRandomGrid(seed);
    const PrecomputationGridStack stack(grid, kNumDepths);
    ASSERT_EQ(stack.max_depth(), kNumDepths - 1);
    for (int depth = 0; depth < kNumDepths; ++depth) {
      const PrecomputationGrid& level = stack.Get(depth);
      const int window = 1 << depth;
      ASSERT_EQ(level.pooling_width(), window);
      // Anchors beyond the covered band included, where reads must fall back to unknown.
      for (int y = -window - 2; y < kHeight + 3; ++y) {
        for (int x = -window - 2; x < kWidth + 3; ++x) {
          EXPECT_EQ(static_cast<int>(level.GetValue(x, y)),
                    static_cast<int>(NaiveWindowMax(grid, x, y, window)))
              << "seed " << seed << ", depth " << depth << ", anchor " << x << ", " << y;
        }
      }
    }
  }
}

// The property branch and bound relies on: a coarse window value bounds every full-resolution
// value inside its window from above.
TEST(PrecomputationGridTest, CoarseValuesBoundEveryFineValueInTheirWindow) {
  const mapping::GridMapu8 grid = MakeRandomGrid(42);
  const PrecomputationGridStack stack(grid, kNumDepths);
  for (int depth = 1; depth < kNumDepths; ++depth) {
    const PrecomputationGrid& coarse = stack.Get(depth);
    const PrecomputationGrid& fine = stack.Get(0);
    const int window = 1 << depth;
    for (int y = -window; y < kHeight + 2; ++y) {
      for (int x = -window; x < kWidth + 2; ++x) {
        const int coarse_value = coarse.GetValue(x, y);
        for (int dy = 0; dy < window; ++dy) {
          for (int dx = 0; dx < window; ++dx) {
            ASSERT_GE(coarse_value, static_cast<int>(fine.GetValue(x + dx, y + dy)))
                << "depth " << depth << ", anchor " << x << ", " << y << ", offset " << dx << ", "
                << dy;
          }
        }
      }
    }
  }
}

TEST(PrecomputationGridTest, DepthZeroIsTheBaseGridIncludingOutsideReads) {
  const mapping::GridMapu8 grid = MakeRandomGrid(3);
  const PrecomputationGridStack stack(grid, 1);
  const PrecomputationGrid& level = stack.Get(0);
  for (int y = -2; y < kHeight + 2; ++y) {
    for (int x = -2; x < kWidth + 2; ++x) {
      EXPECT_EQ(static_cast<int>(level.GetValue(Eigen::Array2i(x, y))),
                static_cast<int>(grid.GetValue(x, y)));
    }
  }
}

TEST(PrecomputationGridTest, OutOfMapReadsAreUnknownWhichReadsAsMinProbability) {
  const mapping::GridMapu8 grid = MakeRandomGrid(11);
  const PrecomputationGridStack stack(grid, 3);
  for (int depth = 0; depth < 3; ++depth) {
    const uint8_t outside = stack.Get(depth).GetValue(-100, 500);
    EXPECT_EQ(static_cast<int>(outside), static_cast<int>(mapping::kUnknownValue));
    EXPECT_NEAR(mapping::ValueToProbability(outside), mapping::kMinProbability, 1e-9);
  }
}

}  // namespace
}  // namespace evergreenslam::utils::scan_matching

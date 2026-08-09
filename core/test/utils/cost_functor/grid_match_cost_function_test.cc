/**
 * @file grid_match_cost_function_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/cost_functor/grid_match_cost_function.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "mapping/grid_mapping/probability_values.h"
#include "utils/transform/transform.h"

namespace evergreenslam::utils::cost_functor {
namespace {

constexpr double kResolution = 0.05;
constexpr int kSize = 20;
constexpr double kOrigin = -0.5;
constexpr double kWeight = 2.0;

mapping::GridMapu8 MakeGrid() {
  std::vector<uint8_t> data(static_cast<size_t>(kSize) * kSize, mapping::kUnknownValue);
  data[10 * kSize + 10] = mapping::ProbabilityToValue(mapping::kMaxProbability);
  return mapping::GridMapu8(std::move(data), kSize, kSize, kResolution, kOrigin, kOrigin,
                            mapping::kUnknownValue);
}

double ResidualAt(const mapping::GridMapu8& grid, const Eigen::Vector2d& sensor_point) {
  sensor::PointCloud cloud;
  cloud.push_back(sensor::Point2d{sensor_point});

  std::unique_ptr<ceres::CostFunction> cost_function(
      CreateGridMatchCostFunction(cloud, grid, kWeight));

  const std::array<double, 3> pose = {0.0, 0.0, 0.0};
  const double* parameters[] = {pose.data()};
  double residual = 0.0;
  EXPECT_TRUE(cost_function->Evaluate(parameters, &residual, nullptr));
  return residual;
}

// Pins the value->cost lookup and the cell indexing together: an inverted
// table, an off-by-one on kPadding or a mis-sized table all move these.
TEST(GridMatchCostFunctionTest, ResidualIsLowOnOccupiedAndHighOnUnknown) {
  const mapping::GridMapu8 grid = MakeGrid();

  // Centre of the occupied cell, in the sensor frame, with the pose at
  // identity so the sensor frame is the grid frame.
  const Eigen::Vector2d occupied = grid.ToCenter(Eigen::Array2i(10, 10));
  EXPECT_NEAR(ResidualAt(grid, occupied), kWeight * (1.0 - mapping::kMaxProbability), 1e-6);

  // Far enough from the single occupied cell that bicubic support cannot
  // reach it.
  const Eigen::Vector2d unknown = grid.ToCenter(Eigen::Array2i(3, 3));
  EXPECT_NEAR(ResidualAt(grid, unknown), kWeight * (1.0 - mapping::kUnknownProbability), 1e-6);

  const Eigen::Vector2d outside(kOrigin - 5.0, kOrigin - 5.0);
  EXPECT_NEAR(ResidualAt(grid, outside), kWeight * (1.0 - mapping::kUnknownProbability), 1e-6);
}

TEST(GridMatchCostFunctionTest, ResidualRisesMovingOffTheOccupiedCell) {
  const mapping::GridMapu8 grid = MakeGrid();
  const Eigen::Vector2d occupied = grid.ToCenter(Eigen::Array2i(10, 10));

  const double on_cell = ResidualAt(grid, occupied);
  // Bicubic interpolation passes through the node values, so every other cell
  // centre already reads exactly unknown; the gradient only exists between
  // cells. Sampling on a neighbouring centre would prove nothing.
  const double half_cell_away =
      ResidualAt(grid, occupied + Eigen::Vector2d(0.5 * kResolution, 0.0));
  EXPECT_GT(half_cell_away, on_cell);

  const double far_away = ResidualAt(grid, occupied + Eigen::Vector2d(3 * kResolution, 0.0));
  EXPECT_GT(far_away, half_cell_away);
  EXPECT_NEAR(far_away, kWeight * (1.0 - mapping::kUnknownProbability), 1e-6);
}

// A read table sized to the 128 stored values would index out of bounds here.
TEST(GridMatchCostFunctionTest, MarkedCellValuesStayInTheReadTable) {
  for (int value = 0; value < mapping::kReadTableSize; ++value) {
    const double probability = mapping::ValueToProbability(static_cast<uint8_t>(value));
    EXPECT_GE(probability, mapping::kMinProbability - 1e-12) << "value " << value;
    EXPECT_LE(probability, mapping::kMaxProbability + 1e-12) << "value " << value;
  }
  for (int value = 0; value < mapping::kValueCount; ++value) {
    EXPECT_DOUBLE_EQ(
        mapping::ValueToProbability(static_cast<uint8_t>(value)),
        mapping::ValueToProbability(static_cast<uint8_t>(value + mapping::kUpdateMarker)));
  }
}

}  // namespace
}  // namespace evergreenslam::utils::cost_functor

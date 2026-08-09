/**
 * @file grid_match_cost_function.cc
 * @author hang chen (chen@hang.plus)
 * @brief Residual between a point cloud and an occupancy grid map.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/cost_functor/grid_match_cost_function.h"

#include <ceres/cubic_interpolation.h>
#include <glog/logging.h>

#include <array>
#include <climits>

#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::utils::cost_functor {
namespace {

class GridMatchCostFunction {
 public:
  GridMatchCostFunction(const sensor::PointCloud& point_cloud, const mapping::GridMapu8& grid_map,
                        const double point_weight)
      : point_cloud_(point_cloud),
        grid_array_adapter_(grid_map),
        interpolator_(grid_array_adapter_),
        point_weight_(point_weight),
        origin_x_(grid_map.origin_x()),
        origin_y_(grid_map.origin_y()),
        resolution_(grid_map.resolution()) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    const T x = pose[0];
    const T y = pose[1];
    const T theta = pose[2];

    const T cos_theta = ceres::cos(theta);
    const T sin_theta = ceres::sin(theta);

    for (size_t i = 0; i < point_cloud_.size(); i++) {
      const T point_x = T(point_cloud_[i].point.x());
      const T point_y = T(point_cloud_[i].point.y());

      const T grid_x = cos_theta * point_x - sin_theta * point_y + x;
      const T grid_y = sin_theta * point_x + cos_theta * point_y + y;

      interpolator_.Evaluate(
          (grid_y - origin_y_) / resolution_ + static_cast<double>(kPadding) - 0.5,
          (grid_x - origin_x_) / resolution_ + static_cast<double>(kPadding) - 0.5, &residual[i]);
      residual[i] = point_weight_ * residual[i];
    }

    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const mapping::GridMapu8& grid_map) : grid_map_(grid_map) {
      DCHECK_EQ(grid_map.unknown_value(), mapping::kUnknownValue);
      // Indexed by the raw cell byte, marker included, so it spans the byte.
      for (int value = 0; value < mapping::kReadTableSize; ++value) {
        cost_table_[value] = 1.0 - mapping::ValueToProbability(static_cast<uint8_t>(value));
      }
    }

    void GetValue(const int row, const int column, double* const value) const {
      *value = cost_table_[grid_map_.GetValue(column - kPadding, row - kPadding)];
    }

    int NumRows() const { return grid_map_.height() + 2 * kPadding; }
    int NumCols() const { return grid_map_.width() + 2 * kPadding; }

   private:
    const mapping::GridMapu8& grid_map_;
    std::array<double, mapping::kReadTableSize> cost_table_;
  };

  const sensor::PointCloud& point_cloud_;
  const GridArrayAdapter grid_array_adapter_;
  ceres::BiCubicInterpolator<GridArrayAdapter> interpolator_;
  double point_weight_;
  double origin_x_;
  double origin_y_;
  double resolution_;
};

}  // namespace

ceres::CostFunction* CreateGridMatchCostFunction(const sensor::PointCloud& point_cloud,
                                                 const mapping::GridMapu8& grid_map,
                                                 const double point_weight) {
  return new ceres::AutoDiffCostFunction<GridMatchCostFunction, ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new GridMatchCostFunction(point_cloud, grid_map, point_weight), point_cloud.size());
}

}  // namespace evergreenslam::utils::cost_functor

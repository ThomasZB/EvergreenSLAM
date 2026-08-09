/**
 * @file grid_match_cost_function.h
 * @author hang chen (chen@hang.plus)
 * @brief Residual between a point cloud and an occupancy grid map.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_COST_FUNCTOR_GRID_MATCH_COST_FUNCTION_H_
#define EVERGREENSLAM_UTILS_COST_FUNCTOR_GRID_MATCH_COST_FUNCTION_H_

#include <ceres/ceres.h>

#include "mapping/grid_mapping/grid_map.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::utils::cost_functor {

ceres::CostFunction* CreateGridMatchCostFunction(const sensor::PointCloud& point_cloud,
                                                 const mapping::GridMapu8& grid_map,
                                                 const double point_weight);

}  // namespace evergreenslam::utils::cost_functor

#endif  // EVERGREENSLAM_UTILS_COST_FUNCTOR_GRID_MATCH_COST_FUNCTION_H_

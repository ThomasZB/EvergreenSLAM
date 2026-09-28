/**
 * @file global_map.h
 * @author hang chen (chen@hang.plus)
 * @brief One assembled occupancy grid from every session's submaps at their optimized poses.
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_GLOBAL_MAP_H_
#define EVERGREENSLAM_LIFELONG_GLOBAL_MAP_H_

#include <optional>

#include "lifelong/ids.h"
#include "lifelong/pose_graph_data.h"
#include "mapping/grid_mapping/grid_map.h"

namespace evergreenslam::lifelong {

mapping::GridMapu8 AssembleGlobalMap(const PoseGraphData& graph, double resolution = 0.0,
                                     bool only_finished = false);
mapping::GridMapu8 AssembleGlobalMap(const PoseGraphData& graph, double resolution,
                                     bool only_finished, std::optional<SessionId> only_session);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_GLOBAL_MAP_H_

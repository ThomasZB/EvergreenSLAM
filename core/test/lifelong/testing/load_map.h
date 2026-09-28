/**
 * @file load_map.h
 * @author hang chen (chen@hang.plus)
 * @brief Test-side Load for fixtures that only care whether a map came back.
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_TEST_LIFELONG_TESTING_LOAD_MAP_H_
#define EVERGREENSLAM_TEST_LIFELONG_TESTING_LOAD_MAP_H_

#include <optional>
#include <utility>
#include <variant>

#include "lifelong/map_manager/map_manager.h"
#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong::testing {

// Empty for a fresh directory and for a failure alike.
inline std::optional<MapManager::LoadResult> LoadMap(MapManager& manager, PoseGraphData& graph) {
  MapManager::LoadOutcome outcome = manager.Load(graph);
  if (!std::holds_alternative<MapManager::LoadResult>(outcome)) {
    return std::nullopt;
  }
  return std::get<MapManager::LoadResult>(std::move(outcome));
}

inline std::optional<MapManager::LoadFailure::Reason> LoadFailureOf(MapManager& manager,
                                                                    PoseGraphData& graph) {
  MapManager::LoadOutcome outcome = manager.Load(graph);
  if (!std::holds_alternative<MapManager::LoadFailure>(outcome)) {
    return std::nullopt;
  }
  return std::get<MapManager::LoadFailure>(outcome).reason;
}

}  // namespace evergreenslam::lifelong::testing

#endif  // EVERGREENSLAM_TEST_LIFELONG_TESTING_LOAD_MAP_H_

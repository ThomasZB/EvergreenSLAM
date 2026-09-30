/**
 * @file service_context.cc
 * @author hang chen (chen@hang.plus)
 * @brief What the endpoints share: the backend, the host hooks, the memory tree and worker state.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/service_context.h"

#include <filesystem>
#include <utility>

namespace evergreenslam::agent {

ServiceContext::ServiceContext(lifelong::PoseGraph& pose_graph, AgentServiceHooks hooks,
                               PlaceStore places, std::string map_root, std::string map_name)
    : pose_graph(pose_graph),
      hooks(std::move(hooks)),
      places(std::move(places)),
      sandbox(this->places.memory_dir()),
      zones(this->places.memory_dir()),
      map_root(std::move(map_root)),
      map_name(std::move(map_name)),
      snapshots((std::filesystem::path(this->places.map_dir()) / "snapshots").string()),
      next_view_seq(NextSequenceNumber(views_dir())) {}

std::string ServiceContext::views_dir() const {
  return (std::filesystem::path(places.map_dir()) / "views").string();
}

}  // namespace evergreenslam::agent

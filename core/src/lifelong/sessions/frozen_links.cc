/**
 * @file frozen_links.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/sessions/frozen_links.h"

#include <vector>

namespace evergreenslam::lifelong {

std::map<SubmapId, int> CountFrozenLinks(const PoseGraphData& graph, SessionId id) {
  std::map<SubmapId, int> links;
  // A node's session says nothing about which submaps hold it: a transferred submap keeps
  // node_ids left behind in the session it came from.
  std::map<NodeId, std::vector<SubmapId>> holders;
  for (const SubmapId& submap_id : graph.session(id).submap_ids) {
    links.emplace(submap_id, 0);
    for (const NodeId& node_id : graph.submap(submap_id).node_ids) {
      holders[node_id].push_back(submap_id);
    }
  }

  const auto frozen = [&graph](const VariableId& variable) {
    return graph.session(variable.session()).frozen();
  };
  const auto count = [&](const VariableId& variable) {
    if (variable.kind == VariableId::Kind::SUBMAP) {
      const auto it = links.find(variable.submap_id());
      if (it != links.end()) {
        ++it->second;
      }
      return;
    }
    const auto it = holders.find(variable.node_id());
    if (it == holders.end()) {
      return;
    }
    for (const SubmapId& submap_id : it->second) {
      ++links.at(submap_id);
    }
  };

  for (const Constraint& constraint : graph.constraints()) {
    const bool from_frozen = frozen(constraint.from);
    if (!constraint.to.has_value()) {
      // A PRIOR pins its variable to the global frame, which is what the frozen layer defines.
      if (!from_frozen) {
        count(constraint.from);
      }
      continue;
    }
    if (from_frozen == frozen(*constraint.to)) {
      continue;
    }
    count(from_frozen ? *constraint.to : constraint.from);
  }
  return links;
}

bool HasFrozenLink(const PoseGraphData& graph, SessionId id) {
  for (const auto& [submap_id, links] : CountFrozenLinks(graph, id)) {
    if (links > 0) {
      return true;
    }
  }
  // A PRIOR on a node no surviving submap of the session holds still pins the session.
  for (const Constraint& constraint : graph.constraints()) {
    if (!constraint.to.has_value() && constraint.from.session() == id) {
      return true;
    }
  }
  return false;
}

}  // namespace evergreenslam::lifelong

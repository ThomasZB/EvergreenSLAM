/**
 * @file drift_tracker.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/drift_tracker.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <set>

namespace evergreenslam::lifelong {
namespace {

template <typename Key>
std::optional<double> FindPath(const std::map<Key, double>& paths, const Key& key) {
  const auto it = paths.find(key);
  return it == paths.end() ? std::nullopt : std::optional<double>(it->second);
}

template <typename Map, typename Alive>
void SweepMap(Map& map, const Alive& alive) {
  for (auto it = map.begin(); it != map.end();) {
    it = alive(it->first) ? std::next(it) : map.erase(it);
  }
}

}  // namespace

DriftTracker::DriftTracker(const ConstraintBuilderOption& option)
    : candidate_slack_per_meter_(option.candidate_slack_per_meter),
      max_candidate_slack_(option.max_candidate_slack),
      min_loop_node_gap_(option.min_loop_node_gap) {}

void DriftTracker::RecordNode(const PoseGraphData& graph, const NodeId& node_id) {
  const SessionId session = SessionOf(node_id);
  const Eigen::Vector2d position = graph.node(node_id).constant_data.local_pose.translation();
  double& path_length = path_length_[session];
  const auto last = last_node_position_.find(session);
  if (last != last_node_position_.end()) {
    path_length += (position - last->second).norm();
  }
  last_node_position_[session] = position;
  path_at_node_[node_id] = path_length;
  for (const auto& [submap_id, record] : graph.submaps()) {
    if (record.node_ids.count(node_id) > 0) {
      path_at_submap_[submap_id] = path_length;
    }
  }
}

double DriftTracker::Slack(const PoseGraphData& graph, const NodeId& node_id,
                           const SubmapId& submap_id) const {
  const std::optional<double> node_path = PathAtNode(node_id);
  const std::optional<double> submap_path = PathAtSubmap(submap_id);
  const std::optional<double> node_drift = DriftOf(graph, SessionOf(node_id), node_path);
  const std::optional<double> submap_drift = DriftOf(graph, SessionOf(submap_id), submap_path);
  if (!node_drift.has_value() || !submap_drift.has_value()) {
    return max_candidate_slack_;
  }
  // Sessions never share an odometer, so their drifts add; within one session the path between
  // the endpoints bounds them.
  double drift = *node_drift + *submap_drift;
  if (SessionOf(node_id) == SessionOf(submap_id) && node_path.has_value() &&
      submap_path.has_value()) {
    drift = std::min(drift, std::abs(*node_path - *submap_path));
  }
  return std::min(candidate_slack_per_meter_ * drift, max_candidate_slack_);
}

void DriftTracker::NoteClosure(const PoseGraphData& graph, const NodeId& node_id,
                               const SubmapId& submap_id) {
  if (!IsValidClosure(graph, node_id, submap_id)) {
    return;
  }
  NoteSessionClosure(graph, SessionOf(node_id), PathAtNode(node_id).value_or(0.0));
  NoteSessionClosure(graph, SessionOf(submap_id), PathAtSubmap(submap_id).value_or(0.0));
}

void DriftTracker::Sweep(const PoseGraphData& graph) {
  const auto has_session = [&graph](SessionId id) { return graph.HasSession(id); };
  SweepMap(path_at_node_, [&graph](const NodeId& id) { return graph.HasNode(id); });
  SweepMap(path_at_submap_, [&graph](const SubmapId& id) { return graph.HasSubmap(id); });
  SweepMap(path_length_, has_session);
  SweepMap(last_node_position_, has_session);
  SweepMap(path_at_last_closure_, has_session);
}

bool DriftTracker::IsValidClosure(const PoseGraphData& graph, const NodeId& node_id,
                                  const SubmapId& submap_id) const {
  if (!(SessionOf(node_id) == SessionOf(submap_id))) {
    return true;
  }
  // A transferred submap keeps its old session's nodes; only members of the node's own session
  // measure the gap.
  const std::set<NodeId>& members = graph.submap(submap_id).node_ids;
  const auto first = members.lower_bound(NodeId{node_id.session_id, 0});
  const auto last = members.lower_bound(NodeId{node_id.session_id + 1, 0});
  if (first == last) {
    return true;
  }
  const int max_index = std::prev(last)->node_index;
  const int gap = node_id.node_index > max_index ? node_id.node_index - max_index
                                                 : first->node_index - node_id.node_index;
  return gap >= min_loop_node_gap_;
}

// A frozen endpoint is truth and drifts nothing; a loaded frozen map has no path record either,
// and giving it the cap would put every one of its submaps behind a 37 m window.
std::optional<double> DriftTracker::DriftOf(const PoseGraphData& graph, SessionId session,
                                            std::optional<double> path) const {
  if (graph.session(session).frozen()) {
    return 0.0;
  }
  const auto closure = path_at_last_closure_.find(session);
  if (closure == path_at_last_closure_.end()) {
    return path;
  }
  return path.has_value() ? std::max(0.0, *path - closure->second) : 0.0;
}

void DriftTracker::NoteSessionClosure(const PoseGraphData& graph, SessionId session, double path) {
  if (graph.session(session).frozen()) {
    return;
  }
  double& recorded = path_at_last_closure_[session];
  recorded = std::max(recorded, path);
}

std::optional<double> DriftTracker::PathAtNode(const NodeId& node_id) const {
  return FindPath(path_at_node_, node_id);
}

std::optional<double> DriftTracker::PathAtSubmap(const SubmapId& submap_id) const {
  return FindPath(path_at_submap_, submap_id);
}

}  // namespace evergreenslam::lifelong

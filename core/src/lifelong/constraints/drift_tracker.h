/**
 * @file drift_tracker.h
 * @author hang chen (chen@hang.plus)
 * @brief Path each session walked since its last valid loop closure and since its last match to the
 * frozen base, and the candidate slack it earns.
 * @version 0.1
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_CONSTRAINTS_DRIFT_TRACKER_H_
#define EVERGREENSLAM_LIFELONG_CONSTRAINTS_DRIFT_TRACKER_H_

#include <Eigen/Core>
#include <map>
#include <optional>

#include "lifelong/constraints/constraint_builder_option.h"
#include "lifelong/ids.h"
#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

// Backend task only: every call reads the graph.
class DriftTracker {
 public:
  explicit DriftTracker(const ConstraintBuilderOption& option);

  void RecordNode(const PoseGraphData& graph, const NodeId& node_id);
  double Slack(const PoseGraphData& graph, const NodeId& node_id, const SubmapId& submap_id) const;
  void NoteClosure(const PoseGraphData& graph, const NodeId& node_id, const SubmapId& submap_id);
  void Sweep(const PoseGraphData& graph);
  void InheritSession(SessionId from, SessionId to, const std::map<SubmapId, SubmapId>& submaps,
                      const std::map<NodeId, NodeId>& nodes);
  std::optional<double> SinceBaseMatch(SessionId session) const;

 private:
  bool IsValidClosure(const PoseGraphData& graph, const NodeId& node_id,
                      const SubmapId& submap_id) const;
  std::optional<double> DriftOf(const PoseGraphData& graph, SessionId session,
                                std::optional<double> path) const;
  void NoteSessionClosure(const PoseGraphData& graph, SessionId session, double path);
  void NoteBaseMatch(SessionId session, std::optional<double> path);
  std::optional<double> PathAtNode(const NodeId& node_id) const;
  std::optional<double> PathAtSubmap(const SubmapId& submap_id) const;

  double candidate_slack_per_meter_;
  double max_candidate_slack_;
  int min_loop_node_gap_;
  std::map<SessionId, double> path_length_;
  std::map<SessionId, Eigen::Vector2d> last_node_position_;
  std::map<NodeId, double> path_at_node_;
  std::map<SubmapId, double> path_at_submap_;
  std::map<SessionId, double> path_at_last_closure_;
  std::map<SessionId, double> path_at_last_base_match_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_CONSTRAINTS_DRIFT_TRACKER_H_

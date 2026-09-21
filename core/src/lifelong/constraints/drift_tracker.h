/**
 * @file drift_tracker.h
 * @author hang chen (chen@hang.plus)
 * @brief Path each session walked since its last valid loop closure, and the candidate slack it
 * earns.
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
  // Drift is the path walked since the session's last valid closure, not since its start: a
  // closed loop ties everything before it. An unfrozen endpoint with neither a path record nor a
  // closure gets the full cap, since its drift across a shutdown is unknown.
  double Slack(const PoseGraphData& graph, const NodeId& node_id, const SubmapId& submap_id) const;
  // Counts only when the pair is a loop under min_loop_node_gap; cross-session pairs always are.
  void NoteClosure(const PoseGraphData& graph, const NodeId& node_id, const SubmapId& submap_id);
  // Trimming and session GC remove ids without telling the tracker.
  void Sweep(const PoseGraphData& graph);

 private:
  bool IsValidClosure(const PoseGraphData& graph, const NodeId& node_id,
                      const SubmapId& submap_id) const;
  std::optional<double> DriftOf(const PoseGraphData& graph, SessionId session,
                                std::optional<double> path) const;
  void NoteSessionClosure(const PoseGraphData& graph, SessionId session, double path);
  std::optional<double> PathAtNode(const NodeId& node_id) const;
  std::optional<double> PathAtSubmap(const SubmapId& submap_id) const;

  double candidate_slack_per_meter_;
  double max_candidate_slack_;
  int min_loop_node_gap_;
  // Per session: a session opened by a new boot shares no frame with anything before it, so
  // each runs its own odometer from zero.
  std::map<SessionId, double> path_length_;
  std::map<SessionId, Eigen::Vector2d> last_node_position_;
  std::map<NodeId, double> path_at_node_;
  std::map<SubmapId, double> path_at_submap_;
  std::map<SessionId, double> path_at_last_closure_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_CONSTRAINTS_DRIFT_TRACKER_H_

/**
 * @file relocalization_reporter.h
 * @author hang chen (chen@hang.plus)
 * @brief Relocalization outcomes from finished match rounds.
 * @version 0.1
 * @date 2026-10-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_REPORTING_RELOCALIZATION_REPORTER_H_
#define EVERGREENSLAM_LIFELONG_REPORTING_RELOCALIZATION_REPORTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

#include "common/time.h"
#include "lifelong/backend_handles.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/reporting/event_reporter.h"
#include "lifelong/reporting/pose_graph_events.h"

namespace evergreenslam::lifelong {

struct RelocalizationResult {
  NodeId node_id;
  common::Time time;
  // Absent for a global search.
  std::optional<Eigen::Affine2d> prior;
  int candidates = 0;
  int constraints_added = 0;
  // How far the round moved the node; set when a constraint landed.
  std::optional<PoseCorrection> correction;
  std::optional<ScoredPose> best;
};

class RelocalizationReporter {
 public:
  // Called on the backend task: a listener must not block.
  using Listener = std::function<void(const RelocalizationResult&)>;

  RelocalizationReporter(const PoseGraphData& graph, EventReporter& events);

  void AddListener(Listener listener);
  void OnMatchRoundFinished(const MatchRoundSummary& summary);

 private:
  const PoseGraphData& graph_;
  EventReporter& events_;
  std::mutex listeners_mutex_;
  std::vector<Listener> listeners_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_REPORTING_RELOCALIZATION_REPORTER_H_

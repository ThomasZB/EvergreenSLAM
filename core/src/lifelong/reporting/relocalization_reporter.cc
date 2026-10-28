/**
 * @file relocalization_reporter.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-10-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/reporting/relocalization_reporter.h"

#include <utility>

namespace evergreenslam::lifelong {

RelocalizationReporter::RelocalizationReporter(const PoseGraphData& graph, EventReporter& events)
    : graph_(graph), events_(events) {}

void RelocalizationReporter::AddListener(Listener listener) {
  std::lock_guard<std::mutex> lock(listeners_mutex_);
  listeners_.push_back(std::move(listener));
}

void RelocalizationReporter::OnMatchRoundFinished(const MatchRoundSummary& summary, bool reseeded) {
  // A node gone mid-round went with a dropped fed session, which already ended the request.
  if (summary.source == MatchRoundSummary::Source::GRAPH || !summary.node_id.has_value() ||
      !graph_.HasNode(*summary.node_id)) {
    return;
  }
  RelocalizationResult result;
  result.node_id = *summary.node_id;
  result.time = events_.SensorNow();
  result.prior = summary.prior;
  result.candidates = summary.candidates;
  result.constraints_added = summary.constraints_added;
  result.reseeded = reseeded;
  if (summary.constraints_added > 0 || reseeded) {
    result.correction =
        CorrectionBetween(summary.node_global_pose, graph_.node(result.node_id).global_pose);
  }
  result.best = summary.best;

  PoseGraphEvent event;
  event.time = result.time;
  event.kind = PoseGraphEvent::Kind::RELOCALIZATION;
  event.candidates = result.candidates;
  event.constraints_added = result.constraints_added;
  event.reseeded = result.reseeded;
  event.correction = result.correction;
  events_.Append(std::move(event));

  std::vector<Listener> listeners;
  {
    std::lock_guard<std::mutex> lock(listeners_mutex_);
    listeners = listeners_;
  }
  for (const Listener& listener : listeners) {
    listener(result);
  }
}

}  // namespace evergreenslam::lifelong

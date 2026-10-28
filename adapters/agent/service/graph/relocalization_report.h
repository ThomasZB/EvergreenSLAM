/**
 * @file relocalization_report.h
 * @author hang chen (chen@hang.plus)
 * @brief Relocalization requests and their outcomes, and backend events as agents read them.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_RELOCALIZATION_REPORT_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_RELOCALIZATION_REPORT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "common/time.h"
#include "lifelong/anchors/anchor_store.h"
#include "lifelong/pose_graph.h"
#include "lifelong/reporting/pose_graph_events.h"
#include "lifelong/reporting/relocalization_reporter.h"
#include "service/http/json_writer.h"

namespace evergreenslam::agent {

using RelocalizationId = int64_t;

struct RelocalizationStatus {
  enum class State { PENDING, MATCHED, RESEEDED, NO_MATCH, SUPERSEDED };

  RelocalizationId id = 0;
  std::string kind;
  std::optional<lifelong::AnchorId> anchor;
  std::optional<Eigen::Affine2d> prior;
  State state = State::PENDING;
  int candidates = 0;
  int constraints_added = 0;
  std::optional<lifelong::PoseCorrection> correction;
  std::optional<double> best_score;
  std::optional<common::Time> requested_at;
  std::optional<common::Time> resolved_at;
  std::optional<RelocalizationId> superseded_by;
};

// Backend task only. Core reports rounds, not requests: a result goes to the open request when
// their priors agree, so of two back-to-back requests with one prior the newer may take the
// older's round. A fed-session drop ends the open request, as core then forgets it.
class RelocalizationRequests {
 public:
  // Registers the listener; results landing after the service is gone are dropped.
  static std::shared_ptr<RelocalizationRequests> Attach(lifelong::PoseGraph& pose_graph);

  // Right after SetInitialPose / RelocalizeGlobally, in the same task.
  RelocalizationId Open(std::string kind, std::optional<lifelong::AnchorId> anchor,
                        const std::optional<Eigen::Affine2d>& prior);
  std::optional<RelocalizationStatus> Find(RelocalizationId id);
  std::optional<RelocalizationStatus> Latest();

 private:
  explicit RelocalizationRequests(lifelong::PoseGraph& pose_graph) : pose_graph_(pose_graph) {}

  void OnResult(const lifelong::RelocalizationResult& result);
  void CatchUpOnDrops();
  void SupersedeOpen(std::optional<RelocalizationId> by, common::Time time);

  lifelong::PoseGraph& pose_graph_;
  RelocalizationId next_id_ = 1;
  int64_t next_event_seq_ = 0;
  std::deque<RelocalizationStatus> recent_;
};

void WriteRelocalization(JsonWriter& writer, const std::optional<RelocalizationStatus>& status);
void WriteEvent(JsonWriter& writer, const lifelong::PoseGraphEvent& event);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_RELOCALIZATION_REPORT_H_

/**
 * @file relocalization_report.h
 * @author hang chen (chen@hang.plus)
 * @brief Relocalization outcomes and backend events as agents read them.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_RELOCALIZATION_REPORT_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_RELOCALIZATION_REPORT_H_

#include <map>
#include <mutex>
#include <optional>
#include <string>

#include "lifelong/anchors/anchor_store.h"
#include "lifelong/pose_graph_events.h"
#include "lifelong/relocalization_tracker.h"
#include "service/http/json_writer.h"

namespace evergreenslam::agent {

struct RelocalizationRequest {
  std::string kind;
  std::optional<lifelong::AnchorId> anchor;
};

class RelocalizationRequests {
 public:
  void Remember(lifelong::RelocalizationId id, RelocalizationRequest request);
  std::optional<RelocalizationRequest> Find(lifelong::RelocalizationId id) const;

 private:
  mutable std::mutex mutex_;
  std::map<lifelong::RelocalizationId, RelocalizationRequest> requests_;
};

void WriteRelocalization(JsonWriter& writer,
                         const std::optional<lifelong::RelocalizationStatus>& status,
                         const RelocalizationRequests& requests);
void WriteEvent(JsonWriter& writer, const lifelong::PoseGraphEvent& event);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_RELOCALIZATION_REPORT_H_

/**
 * @file event_reporter.h
 * @author hang chen (chen@hang.plus)
 * @brief Turns backend state changes into the bounded event log.
 * @version 0.1
 * @date 2026-10-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_REPORTING_EVENT_REPORTER_H_
#define EVERGREENSLAM_LIFELONG_REPORTING_EVENT_REPORTER_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <ostream>

#include "common/time.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/reporting/pose_graph_events.h"
#include "lifelong/sessions/session_manager.h"

namespace evergreenslam::lifelong {

struct ReportingOption {
  // A republish moving the fed session's newest node more than either logs a POSE_CORRECTION.
  double pose_correction_translation = 0.3;
  double pose_correction_rotation = 0.2;

  friend std::ostream& operator<<(std::ostream& os, const ReportingOption& option) {
    os << "ReportingOption:" << std::endl;
    os << "  pose_correction_translation: " << option.pose_correction_translation << std::endl;
    os << "  pose_correction_rotation: " << option.pose_correction_rotation << std::endl;
    return os;
  }
};

ReportingOption LoadReportingOption(const YAML::Node& node);

// Backend task only, except Since.
class EventReporter : public SessionObserver {
 public:
  EventReporter(const PoseGraphData& graph, const ReportingOption& option);

  void OnSessionFrozen(const PoseGraphData& graph, SessionId id) override;
  // Before the graph erases the session: its nodes may hold the newest sensor time.
  void OnSessionRemoved(SessionId id);
  void OnFedSessionDropped(SessionId dropped, SessionId fed_now, common::Time time);
  void OnAlignmentPublished(SessionId fed, const std::optional<Eigen::Affine2d>& session_to_global);

  void Append(PoseGraphEvent event) { events_.Append(std::move(event)); }
  PoseGraphEvents::Page Since(int64_t seq) const { return events_.Since(seq); }
  common::Time SensorNow() const;

 private:
  const PoseGraphData& graph_;
  const ReportingOption option_;
  PoseGraphEvents events_;
  std::optional<SessionId> published_fed_;
  std::optional<Eigen::Affine2d> published_alignment_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_REPORTING_EVENT_REPORTER_H_

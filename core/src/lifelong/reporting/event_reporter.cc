/**
 * @file event_reporter.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-10-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/reporting/event_reporter.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "utils/config/yaml_utils.h"

namespace evergreenslam::lifelong {

ReportingOption LoadReportingOption(const YAML::Node& node) {
  using utils::config::LoadOption;
  ReportingOption option;
  option.pose_correction_translation =
      LoadOption(node, "pose_correction_translation", option.pose_correction_translation);
  option.pose_correction_rotation =
      LoadOption(node, "pose_correction_rotation", option.pose_correction_rotation);
  return option;
}

EventReporter::EventReporter(const PoseGraphData& graph, const ReportingOption& option)
    : graph_(graph), option_(option) {}

void EventReporter::OnSessionFrozen(const PoseGraphData& graph, SessionId id) {
  PoseGraphEvent event;
  event.time = SensorNow();
  event.kind = PoseGraphEvent::Kind::SESSION_FROZEN;
  event.session = id;
  Append(std::move(event));
}

void EventReporter::OnSessionRemoved(SessionId id) {
  PoseGraphEvent event;
  event.time = SensorNow();
  event.kind = PoseGraphEvent::Kind::SESSION_REMOVED;
  event.session = id;
  Append(std::move(event));
}

void EventReporter::OnFedSessionDropped(SessionId dropped, SessionId fed_now, common::Time time) {
  PoseGraphEvent event;
  event.time = time;
  event.kind = PoseGraphEvent::Kind::FED_SESSION_DROPPED;
  event.session = dropped;
  event.fed_now = fed_now;
  Append(std::move(event));
}

void EventReporter::OnAlignmentPublished(SessionId fed,
                                         const std::optional<Eigen::Affine2d>& session_to_global) {
  const std::optional<Eigen::Affine2d> previous = published_alignment_;
  const bool same_session = published_fed_ == fed;
  published_fed_ = fed;
  published_alignment_ = session_to_global;
  const std::vector<NodeId>& node_ids = graph_.session(fed).node_ids;
  if (!same_session || !previous.has_value() || !session_to_global.has_value() ||
      node_ids.empty()) {
    return;
  }
  const Node& newest = graph_.node(node_ids.back());
  const Eigen::Affine2d& local = newest.constant_data.local_pose;
  const PoseCorrection correction =
      CorrectionBetween(*previous * local, *session_to_global * local);
  if (correction.translation <= option_.pose_correction_translation &&
      correction.rotation <= option_.pose_correction_rotation) {
    return;
  }
  PoseGraphEvent event;
  event.time = newest.constant_data.time;
  event.kind = PoseGraphEvent::Kind::POSE_CORRECTION;
  event.session = fed;
  event.correction = correction;
  Append(std::move(event));
}

common::Time EventReporter::SensorNow() const {
  common::Time newest;
  for (const auto& [id, session] : graph_.sessions()) {
    newest = std::max(newest, session.last_node_time);
  }
  return newest;
}

}  // namespace evergreenslam::lifelong

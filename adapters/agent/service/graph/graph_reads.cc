/**
 * @file graph_reads.cc
 * @author hang chen (chen@hang.plus)
 * @brief Reads shared by the endpoints: robot pose, anchor states and phases as agents see them.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/graph/graph_reads.h"

#include <utility>

#include "lifelong/sessions/frozen_links.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {

RobotReading ReadRobotOnTask(const lifelong::PoseGraph& pose_graph, const AgentServiceHooks& hooks,
                             std::optional<sensor::PointCloud>* scan) {
  RobotReading reading;
  if (scan != nullptr) {
    scan->reset();
  }
  const std::optional<lifelong::SessionId> fed = pose_graph.session_manager().fed_session();
  if (!hooks.current_frame || !fed.has_value() || !pose_graph.graph().HasSession(*fed)) {
    return reading;
  }
  std::optional<HostFrame> frame = hooks.current_frame();
  if (!frame.has_value()) {
    return reading;
  }
  const StampedPose& stamped = frame->pose;
  const lifelong::PoseGraphData& graph = pose_graph.graph();
  const Eigen::Affine2d session_to_global =
      graph.ComputeSessionToGlobal(*fed).value_or(graph.session(*fed).local_to_global);
  reading.pose = Eigen::Affine2d(session_to_global * stamped.local_pose);
  reading.scan_match = ScanMatch{frame->match_score, frame->match_score_avg};
  const std::optional<common::Time> newest = NewestNodeTime(graph);
  if (newest.has_value()) {
    reading.keyframe_age_s = common::ToSeconds(stamped.time - *newest);
  }
  if (scan != nullptr) {
    *scan = std::move(frame->scan);
  }
  return reading;
}

std::optional<common::Time> NewestNodeTime(const lifelong::PoseGraphData& graph) {
  std::optional<common::Time> newest;
  for (const auto& [id, session] : graph.sessions()) {
    if (session.node_ids.empty()) {
      continue;
    }
    const common::Time time = graph.node(session.node_ids.back()).constant_data.time;
    if (!newest.has_value() || time > *newest) {
      newest = time;
    }
  }
  return newest;
}

const char* SessionPhase(const lifelong::PoseGraph& pose_graph, lifelong::SessionId id) {
  const auto& expansion = pose_graph.session_manager().expansion();
  const auto it = expansion.find(id);
  return PhaseName(it == expansion.end() ? lifelong::SessionManager::Phase::BOOTSTRAP
                                         : it->second.phase);
}

int NumSolves(const lifelong::PoseGraph& pose_graph) {
  return pose_graph.optimization().num_solves();
}

BaseAlignment ReadBaseAlignmentOnTask(const lifelong::PoseGraph& pose_graph) {
  BaseAlignment alignment;
  const lifelong::PoseGraphData& graph = pose_graph.graph();
  for (const auto& [id, session] : graph.sessions()) {
    alignment.has_frozen_base = alignment.has_frozen_base || session.frozen();
  }
  const std::optional<lifelong::SessionId> fed = pose_graph.session_manager().fed_session();
  alignment.aligned_to_base = fed.has_value() && lifelong::HasFrozenLink(graph, *fed);
  return alignment;
}

const char* AgentState(const lifelong::ResolvedAnchor& anchor) {
  switch (anchor.state) {
    case lifelong::AnchorState::BOUND:
      return anchor.frozen ? "frozen" : "pending";
    case lifelong::AnchorState::REBOUND:
      return anchor.frozen ? "frozen" : "rebound";
    case lifelong::AnchorState::ORPHAN:
      return "orphan";
  }
  return "orphan";
}

std::optional<std::string> OrphanReasonName(const lifelong::ResolvedAnchor& anchor) {
  if (anchor.orphan_reason == lifelong::OrphanReason::NONE) {
    return std::nullopt;
  }
  return std::string(lifelong::ToString(anchor.orphan_reason));
}

const char* PhaseName(lifelong::SessionManager::Phase phase) {
  using Phase = lifelong::SessionManager::Phase;
  switch (phase) {
    case Phase::BOOTSTRAP:
      return "BOOTSTRAP";
    case Phase::LOCALIZING:
      return "LOCALIZING";
    case Phase::EXPANDING:
      return "EXPANDING";
    case Phase::NO_GROWTH:
      return "NO_GROWTH";
    case Phase::JUDGING:
      return "JUDGING";
  }
  return "BOOTSTRAP";
}

std::vector<PlaceRow> ResolvePlaces(
    const std::vector<PlaceFile>& places,
    const std::map<lifelong::AnchorId, lifelong::ResolvedAnchor>& resolved) {
  std::vector<PlaceRow> rows;
  rows.reserve(places.size());
  for (const PlaceFile& place : places) {
    PlaceRow row;
    row.path = place.path;
    row.anchor = place.anchor;
    const auto it = resolved.find(place.anchor);
    if (it == resolved.end()) {
      // A place.yaml naming an id the store never issued or lost: nothing to resolve.
      row.state = "orphan";
      row.orphan_reason = "unknown_anchor";
    } else {
      row.state = AgentState(it->second);
      row.orphan_reason = OrphanReasonName(it->second);
      row.pose = it->second.global_pose;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

std::map<lifelong::AnchorId, lifelong::ResolvedAnchor> ById(
    const std::vector<lifelong::ResolvedAnchor>& resolved) {
  std::map<lifelong::AnchorId, lifelong::ResolvedAnchor> by_id;
  for (const lifelong::ResolvedAnchor& anchor : resolved) {
    by_id.emplace(anchor.id, anchor);
  }
  return by_id;
}

void WritePose(JsonWriter& writer, std::string_view key,
               const std::optional<Eigen::Affine2d>& pose) {
  writer.Key(key);
  if (!pose.has_value()) {
    writer.Null();
    return;
  }
  writer.BeginObject()
      .Field("x", pose->translation().x())
      .Field("y", pose->translation().y())
      .Field("theta", utils::transform::GetYaw(*pose))
      .EndObject();
}

void WriteSubmapId(JsonWriter& writer, std::string_view key, const lifelong::SubmapId& id) {
  writer.Key(key).BeginArray().Int(id.session_id).Int(id.submap_index).EndArray();
}

void WriteScanMatch(JsonWriter& writer, const std::optional<ScanMatch>& scan_match) {
  writer.Key("scan_match");
  if (!scan_match.has_value()) {
    writer.Null();
    return;
  }
  writer.BeginObject().Field("score", scan_match->score).Field("avg", scan_match->avg).EndObject();
}

}  // namespace evergreenslam::agent

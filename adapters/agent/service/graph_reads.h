/**
 * @file graph_reads.h
 * @author hang chen (chen@hang.plus)
 * @brief Reads shared by the endpoints: robot pose, anchor states and phases as agents see them.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_READS_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_READS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "common/time.h"
#include "lifelong/anchors/anchor_store.h"
#include "lifelong/pose_graph.h"
#include "sensor/point_cloud.h"
#include "service/json_writer.h"
#include "service/place_store.h"
#include "service/service_hooks.h"

namespace evergreenslam::agent {

struct ScanMatch {
  double score = 0.0;
  double avg = 0.0;
};

struct RobotReading {
  std::optional<Eigen::Affine2d> pose;
  std::optional<double> keyframe_age_s;
  // From the same host frame as `pose`; empty whenever it is.
  std::optional<ScanMatch> scan_match;
};

struct BaseAlignment {
  bool has_frozen_base = false;
  // HasFrozenLink(graph, fed): the robot pose and the frozen anchors share one frame.
  bool aligned_to_base = false;
};

// ---- Backend task only.

// The fed session's current alignment times the host's local pose, both read now. `scan`, when
// given, receives the scan of that same host frame, empty whenever the pose is.
RobotReading ReadRobotOnTask(const lifelong::PoseGraph& pose_graph, const AgentServiceHooks& hooks,
                             std::optional<sensor::PointCloud>* scan = nullptr);
std::optional<common::Time> NewestNodeTime(const lifelong::PoseGraphData& graph);
// expansion() entry, BOOTSTRAP when absent.
const char* SessionPhase(const lifelong::PoseGraph& pose_graph, lifelong::SessionId id);
int NumSolves(const lifelong::PoseGraph& pose_graph);
BaseAlignment ReadBaseAlignmentOnTask(const lifelong::PoseGraph& pose_graph);

// ---- Anywhere.

// pending, frozen, rebound or orphan (API.md, "Anchor state").
const char* AgentState(const lifelong::ResolvedAnchor& anchor);
std::optional<std::string> OrphanReasonName(const lifelong::ResolvedAnchor& anchor);
const char* PhaseName(lifelong::SessionManager::Phase phase);

std::vector<PlaceRow> ResolvePlaces(
    const std::vector<PlaceFile>& places,
    const std::map<lifelong::AnchorId, lifelong::ResolvedAnchor>& resolved);
std::map<lifelong::AnchorId, lifelong::ResolvedAnchor> ById(
    const std::vector<lifelong::ResolvedAnchor>& resolved);

void WritePose(JsonWriter& writer, std::string_view key,
               const std::optional<Eigen::Affine2d>& pose);
void WriteSubmapId(JsonWriter& writer, std::string_view key, const lifelong::SubmapId& id);
void WriteScanMatch(JsonWriter& writer, const std::optional<ScanMatch>& scan_match);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_READS_H_

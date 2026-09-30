/**
 * @file service_hooks.h
 * @author hang chen (chen@hang.plus)
 * @brief What the host injects into the agent service: the frontend's pose and scan, map switch
 * and fed-session drop.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERVICE_HOOKS_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERVICE_HOOKS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <optional>
#include <string>

#include "common/time.h"
#include "lifelong/ids.h"
#include "lifelong/pose_graph.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::agent {

struct StampedPose {
  common::Time time;
  // Frontend local frame.
  Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();
};

struct HostFrame {
  StampedPose pose;
  // Robot frame, at `pose`.
  sensor::PointCloud scan;
  // The frontend's MatchingResult::match_score, 0..1: agreement with the local submap only, so it
  // stays high under a wrong global pose. The average is MatchScoreAverage's (host side).
  double match_score = 0.0;
  double match_score_avg = 0.0;
};

struct MapSwitchRequest {
  std::string name;
  bool create = false;
};

// The fed session, dropped in process.
struct SessionDropRequest {
  lifelong::SessionId id;
};

// `current_frame` runs inside a backend task and may take only the host's own mutex: never
// enqueue on or wait for the PoseGraph from it. Pose and scan are read under one lock, or a
// scan-thread update between two reads would draw scan k+1 at pose k.
// `switch_map` runs on the service's worker thread, never inside a task: it records the request
// and returns at once; the host then stops this service, finishes the graph and boots the named
// map, so the port disappears and comes back. Empty: the host cannot switch.
// `drop_session` runs on the worker thread outside any task; the host blocks its frontend, calls
// PoseGraph::DropFedSession, replaces the frontend, and returns the result. Empty: cannot drop.
struct AgentServiceHooks {
  std::function<std::optional<HostFrame>()> current_frame;
  std::function<bool(const MapSwitchRequest&)> switch_map;
  std::function<lifelong::PoseGraph::DropFedSessionResult(const SessionDropRequest&)> drop_session;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERVICE_HOOKS_H_

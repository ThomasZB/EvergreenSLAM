/**
 * @file agent_host.cc
 * @author hang chen (chen@hang.plus)
 * @brief The hosts' side of the agent service: owns it and feeds its pose and scan hooks.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "agent_host.h"

#include <iostream>
#include <utility>

#include "service/place_store.h"

namespace evergreenslam::ros2 {

namespace {

agent::AgentServiceOption ServiceOption(const AgentHostOption& option) {
  agent::AgentServiceOption service_option;
  service_option.bind = option.bind;
  service_option.port = option.port;
  service_option.map_root = option.map_root;
  service_option.map_name = option.map_name;
  return service_option;
}

}  // namespace

std::unique_ptr<AgentHost> AgentHost::Create(const AgentHostOption& option,
                                             lifelong::PoseGraph& pose_graph,
                                             MapSwitchHook switch_map,
                                             SessionDropHook drop_session) {
  if (option.port <= 0) {
    return nullptr;
  }
  if (pose_graph.map_manager() == nullptr) {
    std::cerr << "agent service off: it needs a map root (map_root)" << std::endl;
    return nullptr;
  }
  return std::make_unique<AgentHost>(option, pose_graph, std::move(switch_map),
                                     std::move(drop_session));
}

AgentHost::AgentHost(const AgentHostOption& option, lifelong::PoseGraph& pose_graph,
                     MapSwitchHook switch_map, SessionDropHook drop_session)
    : zones_(agent::PlaceStore(pose_graph.map_manager()->directory()).memory_dir()),
      service_(std::make_unique<agent::AgentService>(
          pose_graph, Hooks(std::move(switch_map), std::move(drop_session)),
          ServiceOption(option))) {}

AgentHost::~AgentHost() { Stop(); }

void AgentHost::Update(common::Time time, const Eigen::Affine2d& local_pose,
                       const sensor::TimedPointCloud& scan, std::optional<double> match_score) {
  sensor::PointCloud cloud;
  cloud.points().reserve(scan.size());
  for (const sensor::TimedPoint2d& point : scan) {
    cloud.push_back({point.point});
  }
  std::lock_guard<std::mutex> lock(mutex_);
  agent::HostFrame frame{agent::StampedPose{time, local_pose}, std::move(cloud)};
  if (match_score.has_value()) {
    frame.match_score = *match_score;
    frame.match_score_avg = match_score_average_.Add(time, *match_score);
  } else if (frame_.has_value()) {
    frame.match_score = frame_->match_score;
    frame.match_score_avg = frame_->match_score_avg;
  }
  frame_ = std::move(frame);
}

void AgentHost::OnBackendStarted() { service_->OnBackendStarted(); }

void AgentHost::OnFedSessionDropped() {
  std::lock_guard<std::mutex> lock(mutex_);
  frame_.reset();
  match_score_average_ = agent::MatchScoreAverage();
}

void AgentHost::Stop() { service_->Stop(); }

agent::AgentServiceHooks AgentHost::Hooks(MapSwitchHook switch_map, SessionDropHook drop_session) {
  agent::AgentServiceHooks hooks;
  hooks.switch_map = std::move(switch_map);
  hooks.drop_session = std::move(drop_session);
  hooks.current_frame = [this]() -> std::optional<agent::HostFrame> {
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_;
  };
  return hooks;
}

}  // namespace evergreenslam::ros2

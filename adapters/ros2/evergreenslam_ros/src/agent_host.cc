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
                                             MapSwitchHook switch_map) {
  if (option.port <= 0) {
    return nullptr;
  }
  if (pose_graph.map_manager() == nullptr) {
    std::cerr << "agent service off: it needs a map root (map_root)" << std::endl;
    return nullptr;
  }
  return std::make_unique<AgentHost>(option, pose_graph, std::move(switch_map));
}

AgentHost::AgentHost(const AgentHostOption& option, lifelong::PoseGraph& pose_graph,
                     MapSwitchHook switch_map)
    : service_(std::make_unique<agent::AgentService>(pose_graph, Hooks(std::move(switch_map)),
                                                     ServiceOption(option))) {}

AgentHost::~AgentHost() { Stop(); }

void AgentHost::Update(common::Time time, const Eigen::Affine2d& local_pose,
                       const sensor::TimedPointCloud& scan) {
  sensor::PointCloud cloud;
  cloud.points().reserve(scan.size());
  for (const sensor::TimedPoint2d& point : scan) {
    cloud.push_back({point.point});
  }
  std::lock_guard<std::mutex> lock(mutex_);
  frame_ = agent::HostFrame{agent::StampedPose{time, local_pose}, std::move(cloud)};
}

void AgentHost::OnBackendStarted() { service_->OnBackendStarted(); }

void AgentHost::Stop() { service_->Stop(); }

agent::AgentServiceHooks AgentHost::Hooks(MapSwitchHook switch_map) {
  agent::AgentServiceHooks hooks;
  hooks.switch_map = std::move(switch_map);
  hooks.current_frame = [this]() -> std::optional<agent::HostFrame> {
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_;
  };
  return hooks;
}

}  // namespace evergreenslam::ros2

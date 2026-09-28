/**
 * @file agent_host.h
 * @author hang chen (chen@hang.plus)
 * @brief The hosts' side of the agent service: owns it and feeds its pose and scan hooks.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_ROS2_AGENT_HOST_H_
#define EVERGREENSLAM_ADAPTERS_ROS2_AGENT_HOST_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "common/time.h"
#include "lifelong/pose_graph.h"
#include "sensor/point_cloud.h"
#include "sensor/timed_point_cloud.h"
#include "service/agent_service.h"

namespace evergreenslam::ros2 {

struct AgentHostOption {
  // 0 turns the service off.
  int port = 0;
  std::string bind = "127.0.0.1";
  std::string map_root;
  std::string map_name;
};

using MapSwitchHook = std::function<bool(const agent::MapSwitchRequest&)>;

// Contract: adapters/agent/API.md "Host injection". The scan loop calls Update; the service's
// hooks read copies under mutex_, which is never held across a PoseGraph call.
class AgentHost {
 public:
  // nullptr when the port is 0 or `pose_graph` persists nothing (no map_manager()). An empty
  // `switch_map` means the host cannot switch maps.
  static std::unique_ptr<AgentHost> Create(const AgentHostOption& option,
                                           lifelong::PoseGraph& pose_graph,
                                           MapSwitchHook switch_map = {});

  AgentHost(const AgentHostOption& option, lifelong::PoseGraph& pose_graph,
            MapSwitchHook switch_map);
  ~AgentHost();

  AgentHost(const AgentHost&) = delete;
  AgentHost& operator=(const AgentHost&) = delete;

  // `scan` is in the robot frame at `local_pose`, the builder's pose after that scan.
  void Update(common::Time time, const Eigen::Affine2d& local_pose,
              const sensor::TimedPointCloud& scan);
  // Right after PoseGraph::Start.
  void OnBackendStarted();
  // Before PoseGraph::Finish, or before the PoseGraph is destroyed. Idempotent.
  void Stop();

  // -1 when binding failed.
  int port() const { return service_->port(); }

 private:
  agent::AgentServiceHooks Hooks(MapSwitchHook switch_map);

  std::mutex mutex_;
  std::optional<agent::HostFrame> frame_;
  // Last: its worker calls the hooks above until Stop() joins it.
  std::unique_ptr<agent::AgentService> service_;
};

}  // namespace evergreenslam::ros2

#endif  // EVERGREENSLAM_ADAPTERS_ROS2_AGENT_HOST_H_

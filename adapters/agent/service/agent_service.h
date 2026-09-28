/**
 * @file agent_service.h
 * @author hang chen (chen@hang.plus)
 * @brief The agent layer's HTTP server: PoseGraph operations and the memory/ tree for `egs`.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_AGENT_SERVICE_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_AGENT_SERVICE_H_

#include <memory>
#include <string>

#include "lifelong/pose_graph.h"
#include "service/service_hooks.h"

namespace evergreenslam::agent {

struct AgentServiceOption {
  std::string bind = "127.0.0.1";
  // 0 binds an ephemeral port.
  int port = 0;
  // Empty: no map root known, /maps answers not_supported.
  std::string map_root;
  std::string map_name;
};

// Contract: adapters/agent/API.md. Construct only with a PoseGraph that has a map_manager().
class AgentService {
 public:
  // Creates memory/places/, installs memory/README.md and listens.
  AgentService(lifelong::PoseGraph& pose_graph, AgentServiceHooks hooks,
               const AgentServiceOption& option);
  ~AgentService();

  AgentService(const AgentService&) = delete;
  AgentService& operator=(const AgentService&) = delete;

  // After PoseGraph::Start; task endpoints answer 503 until then.
  void OnBackendStarted();
  // Before PoseGraph::Finish. Idempotent.
  void Stop();

  // The bound port, -1 when binding failed.
  int port() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_AGENT_SERVICE_H_

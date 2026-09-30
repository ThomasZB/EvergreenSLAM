/**
 * @file service_context.h
 * @author hang chen (chen@hang.plus)
 * @brief What the endpoints share: the backend, the host hooks, the memory tree and worker state.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERVICE_CONTEXT_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERVICE_CONTEXT_H_

#include <atomic>
#include <string>

#include "lifelong/pose_graph.h"
#include "service/graph/here_tracker.h"
#include "service/graph/snapshot_exporter.h"
#include "service/http/service_hooks.h"
#include "service/memory/fs_sandbox.h"
#include "service/memory/place_store.h"
#include "service/memory/zone_store.h"

namespace evergreenslam::agent {

struct ServiceContext {
  // `places` must be installed before construction: the sandbox resolves memory/ at once.
  ServiceContext(lifelong::PoseGraph& pose_graph, AgentServiceHooks hooks, PlaceStore places,
                 std::string map_root, std::string map_name);

  bool accepting_tasks() const { return backend_started.load() && !stopping.load(); }
  std::string views_dir() const;

  lifelong::PoseGraph& pose_graph;
  const AgentServiceHooks hooks;
  const PlaceStore places;
  const FsSandbox sandbox;
  const ZoneStore zones;
  // Empty when the host named no map root.
  const std::string map_root;
  const std::string map_name;

  std::atomic<bool> backend_started{false};
  std::atomic<bool> stopping{false};
  // Set once switch_map accepted a request; this service is torn down before it would clear.
  std::atomic<bool> switch_pending{false};

  // Worker-thread state: httplib runs one worker, so none of this is shared.
  HereTracker here;
  int closures_baseline = 0;
  SnapshotExporter snapshots;
  int next_view_seq = 1;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERVICE_CONTEXT_H_

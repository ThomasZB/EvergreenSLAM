/**
 * @file endpoints.h
 * @author hang chen (chen@hang.plus)
 * @brief Route registration, one function per area of adapters/agent/API.md.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_ENDPOINTS_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_ENDPOINTS_H_

#include "service/http/routes.h"
#include "service/http/service_context.h"

namespace evergreenslam::agent {

// /here, /place/save, /anchors, /anchors/{id}, /init-pose.
void RegisterPlaceEndpoints(Routes& server, ServiceContext& context);
// /status, /sessions/**, /relocalize, /checkpoint.
void RegisterSessionEndpoints(Routes& server, ServiceContext& context);
// /snapshot, /snapshots/file, and /view when the renderer is built.
void RegisterViewEndpoints(Routes& server, ServiceContext& context);
// /fs/ls, tree, cat, grep, write, append, mkdir, mv, rm.
void RegisterFsEndpoints(Routes& server, ServiceContext& context);
// /maps, /maps/new, /maps/open.
void RegisterMapsEndpoints(Routes& server, ServiceContext& context);
// /zones.
void RegisterZonesEndpoints(Routes& server, ServiceContext& context);
// /pose, concurrent with the serial handlers.
void RegisterPoseEndpoints(Routes& server, ServiceContext& context);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_ENDPOINTS_H_

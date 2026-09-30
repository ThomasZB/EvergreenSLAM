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

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_ENDPOINTS_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_ENDPOINTS_H_

#include "service/httplib_include.h"
#include "service/service_context.h"

namespace evergreenslam::agent {

// /here, /place/save, /anchors, /anchors/{id}, /init-pose.
void RegisterPlaceEndpoints(httplib::Server& server, ServiceContext& context);
// /status, /sessions/**, /relocalize, /checkpoint.
void RegisterSessionEndpoints(httplib::Server& server, ServiceContext& context);
// /snapshot, and /view when the renderer is built.
void RegisterMapEndpoints(httplib::Server& server, ServiceContext& context);
// /fs/ls, tree, cat, write, append, mkdir, mv, rm.
void RegisterFsEndpoints(httplib::Server& server, ServiceContext& context);
// /maps, /maps/new, /maps/open.
void RegisterMapsEndpoints(httplib::Server& server, ServiceContext& context);
// /zones.
void RegisterZonesEndpoints(httplib::Server& server, ServiceContext& context);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_ENDPOINTS_H_

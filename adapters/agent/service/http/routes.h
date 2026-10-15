/**
 * @file routes.h
 * @author hang chen (chen@hang.plus)
 * @brief Route registration: every handler runs on the serial executor unless registered as
 * concurrent.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_ROUTES_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_ROUTES_H_

#include <string>

#include "service/http/http_reply.h"
#include "service/http/httplib_include.h"
#include "service/http/serial_executor.h"

namespace evergreenslam::agent {

class Routes {
 public:
  Routes(httplib::Server& server, SerialExecutor& serial) : server_(server), serial_(serial) {}

  // Serial: 503 busy when the executor's wait line is full.
  Routes& Get(const std::string& pattern, RouteHandler handler);
  Routes& Post(const std::string& pattern, RouteHandler handler);
  // On the pool thread beside the serial handlers: may touch only thread-safe state.
  Routes& ConcurrentGet(const std::string& pattern, RouteHandler handler);

 private:
  RouteHandler Serialized(RouteHandler handler);

  httplib::Server& server_;
  SerialExecutor& serial_;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_ROUTES_H_

/**
 * @file http_reply.h
 * @author hang chen (chen@hang.plus)
 * @brief Receipts, error bodies, parameter parsing and the route guards every endpoint uses.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_HTTP_REPLY_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_HTTP_REPLY_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "service/http/httplib_include.h"
#include "service/http/json_writer.h"
#include "service/http/request_error.h"
#include "service/http/service_context.h"

namespace evergreenslam::agent {

using RouteHandler = std::function<void(const httplib::Request&, httplib::Response&)>;

// Opens the object with `ok` and `reason` (null when ok); the caller adds fields and closes it.
JsonWriter BeginReceipt(const std::optional<std::string>& refusal);
void SendJson(httplib::Response& response, JsonWriter& writer, int status = 200);
void SendError(httplib::Response& response, int status, const std::string& reason,
               const std::string& detail);

// Answers RequestError with 400.
RouteHandler Guarded(RouteHandler handler);
// Guarded, and 503 not_started before OnBackendStarted() or once Stop() began.
RouteHandler TaskRoute(const ServiceContext& context, RouteHandler handler);

std::optional<std::string> OptionalParam(const httplib::Request& request, const std::string& name);
std::string RequiredParam(const httplib::Request& request, const std::string& name);
double ParseDouble(const std::string& name, const std::string& text);
int64_t ParseInt(const std::string& name, const std::string& text);
bool ParseBool(const std::string& name, const std::string& text);
bool BoolParam(const httplib::Request& request, const std::string& name, bool fallback);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_HTTP_REPLY_H_

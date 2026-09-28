/**
 * @file http_reply.cc
 * @author hang chen (chen@hang.plus)
 * @brief Receipts, error bodies, parameter parsing and the route guards every endpoint uses.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/http_reply.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace evergreenslam::agent {

JsonWriter BeginReceipt(const std::optional<std::string>& refusal) {
  JsonWriter writer;
  writer.BeginObject().Field("ok", !refusal.has_value()).OptionalField("reason", refusal);
  return writer;
}

void SendJson(httplib::Response& response, JsonWriter& writer, int status) {
  response.status = status;
  response.set_content(writer.str(), "application/json");
}

void SendError(httplib::Response& response, int status, const std::string& reason,
               const std::string& detail) {
  JsonWriter writer = BeginReceipt(reason);
  writer.Field("detail", detail).EndObject();
  SendJson(response, writer, status);
}

RouteHandler Guarded(RouteHandler handler) {
  return
      [handler = std::move(handler)](const httplib::Request& request, httplib::Response& response) {
        try {
          handler(request, response);
        } catch (const RequestError& error) {
          SendError(response, 400, error.reason, error.detail);
        }
      };
}

RouteHandler TaskRoute(const ServiceContext& context, RouteHandler handler) {
  return [&context, guarded = Guarded(std::move(handler))](const httplib::Request& request,
                                                           httplib::Response& response) {
    if (!context.accepting_tasks()) {
      SendError(response, 503, "not_started", "the SLAM backend is not running");
      return;
    }
    guarded(request, response);
  };
}

std::optional<std::string> OptionalParam(const httplib::Request& request, const std::string& name) {
  if (!request.has_param(name)) {
    return std::nullopt;
  }
  return request.get_param_value(name);
}

std::string RequiredParam(const httplib::Request& request, const std::string& name) {
  std::optional<std::string> value = OptionalParam(request, name);
  if (!value.has_value() || value->empty()) {
    throw RequestError("bad_param", "missing parameter '" + name + "'");
  }
  return *value;
}

double ParseDouble(const std::string& name, const std::string& text) {
  char* end = nullptr;
  errno = 0;
  const double value = std::strtod(text.c_str(), &end);
  if (text.empty() || end != text.c_str() + text.size() || errno != 0 || !std::isfinite(value)) {
    throw RequestError("bad_param", "'" + name + "' is not a number: " + text);
  }
  return value;
}

int64_t ParseInt(const std::string& name, const std::string& text) {
  char* end = nullptr;
  errno = 0;
  const long long value = std::strtoll(text.c_str(), &end, 10);
  if (text.empty() || end != text.c_str() + text.size() || errno != 0) {
    throw RequestError("bad_param", "'" + name + "' is not an integer: " + text);
  }
  return static_cast<int64_t>(value);
}

bool ParseBool(const std::string& name, const std::string& text) {
  if (text == "1" || text == "true") {
    return true;
  }
  if (text == "0" || text == "false") {
    return false;
  }
  throw RequestError("bad_param", "'" + name + "' is not a boolean: " + text);
}

bool BoolParam(const httplib::Request& request, const std::string& name, bool fallback) {
  const std::optional<std::string> value = OptionalParam(request, name);
  return value.has_value() && !value->empty() ? ParseBool(name, *value) : fallback;
}

}  // namespace evergreenslam::agent

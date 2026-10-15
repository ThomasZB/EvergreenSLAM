/**
 * @file routes.cc
 * @author hang chen (chen@hang.plus)
 * @brief Route registration: every handler runs on the serial executor unless registered as
 * concurrent.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/http/routes.h"

#include <utility>

namespace evergreenslam::agent {

Routes& Routes::Get(const std::string& pattern, RouteHandler handler) {
  server_.Get(pattern, Serialized(std::move(handler)));
  return *this;
}

Routes& Routes::Post(const std::string& pattern, RouteHandler handler) {
  server_.Post(pattern, Serialized(std::move(handler)));
  return *this;
}

Routes& Routes::ConcurrentGet(const std::string& pattern, RouteHandler handler) {
  server_.Get(pattern, std::move(handler));
  return *this;
}

RouteHandler Routes::Serialized(RouteHandler handler) {
  return [&serial = serial_, handler = std::move(handler)](const httplib::Request& request,
                                                           httplib::Response& response) {
    if (!serial.Run([&] { handler(request, response); })) {
      SendError(response, 503, "busy", "other requests are waiting; retry shortly");
    }
  };
}

}  // namespace evergreenslam::agent

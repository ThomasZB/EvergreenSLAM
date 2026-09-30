/**
 * @file request_error.h
 * @author hang chen (chen@hang.plus)
 * @brief A request the service rejects before acting: answered as HTTP 400 {ok, reason, detail}.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_REQUEST_ERROR_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_REQUEST_ERROR_H_

#include <string>
#include <utility>

namespace evergreenslam::agent {

// Thrown by parameter and path checks; every route handler catches it.
struct RequestError {
  RequestError(std::string reason, std::string detail)
      : reason(std::move(reason)), detail(std::move(detail)) {}

  std::string reason;
  std::string detail;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_REQUEST_ERROR_H_

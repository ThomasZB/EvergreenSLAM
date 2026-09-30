/**
 * @file service_test_client.h
 * @author hang chen (chen@hang.plus)
 * @brief HTTP client and a simulated host for the agent service e2e tests.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_TEST_SERVICE_TEST_CLIENT_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_TEST_SERVICE_TEST_CLIENT_H_

#include <gtest/gtest.h>

#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>

#include "lifelong/anchors/anchor_scenario.h"
#include "service/agent_service.h"
#include "service/graph/match_score_average.h"
#include "service/http/httplib_include.h"

namespace evergreenslam::agent::testing {

struct Reply {
  int status = 0;
  std::string body;
  std::string content_type;
  httplib::Headers headers;

  std::string Header(const std::string& name) const {
    const auto it = headers.find(name);
    return it == headers.end() ? std::string() : it->second;
  }
};

class TestClient {
 public:
  explicit TestClient(int port) : client_("127.0.0.1", port) { client_.set_read_timeout(120, 0); }

  Reply Get(const std::string& path) { return Wrap(client_.Get(path)); }
  Reply Post(const std::string& path, const httplib::Params& params = {}) {
    return Wrap(client_.Post(path, params));
  }
  Reply PostBody(const std::string& path, const std::string& body) {
    return Wrap(client_.Post(path, body, "application/octet-stream"));
  }

 private:
  static Reply Wrap(const httplib::Result& result) {
    Reply reply;
    if (result) {
      reply.status = result->status;
      reply.body = result->body;
      reply.content_type = result->get_header_value("Content-Type");
      reply.headers = result->headers;
    }
    return reply;
  }

  httplib::Client client_;
};

// Index of the bracket closing the one at `open`, skipping strings.
inline size_t MatchingClose(const std::string& body, size_t open) {
  int depth = 0;
  bool in_string = false;
  for (size_t i = open; i < body.size(); ++i) {
    const char c = body[i];
    if (in_string) {
      if (c == '\\') {
        ++i;
      } else if (c == '"') {
        in_string = false;
      }
    } else if (c == '"') {
      in_string = true;
    } else if (c == '{' || c == '[') {
      ++depth;
    } else if ((c == '}' || c == ']') && --depth == 0) {
      return i;
    }
  }
  return body.size();
}

// Enough JSON reading for receipts: `path` is dotted keys, each searched inside the object or
// array the previous one opens, so a null parent is missing and never falls through to a sibling.
inline std::optional<size_t> FindValue(const std::string& body, const std::string& path) {
  size_t position = 0;
  size_t limit = body.size();
  size_t begin = 0;
  while (begin <= path.size()) {
    size_t end = path.find('.', begin);
    if (end == std::string::npos) {
      end = path.size();
    }
    const std::string key = "\"" + path.substr(begin, end - begin) + "\":";
    position = body.find(key, position);
    if (position == std::string::npos || position + key.size() >= limit) {
      return std::nullopt;
    }
    position += key.size();
    if (end < path.size()) {
      if (body[position] != '{' && body[position] != '[') {
        return std::nullopt;
      }
      limit = MatchingClose(body, position);
    }
    begin = end + 1;
  }
  return position;
}

inline double JsonNumber(const std::string& body, const std::string& path) {
  const std::optional<size_t> position = FindValue(body, path);
  EXPECT_TRUE(position.has_value()) << path << " missing in " << body;
  return position.has_value() ? std::strtod(body.c_str() + *position, nullptr) : 0.0;
}

inline std::string JsonRaw(const std::string& body, const std::string& path) {
  const std::optional<size_t> position = FindValue(body, path);
  EXPECT_TRUE(position.has_value()) << path << " missing in " << body;
  if (!position.has_value()) {
    return "";
  }
  if (body[*position] == '"') {
    return body.substr(*position + 1, body.find('"', *position + 1) - *position - 1);
  }
  const size_t end = body.find_first_of(",}]", *position);
  return body.substr(*position, end - *position);
}

// What the bag main or live node would inject: the frontend's newest pose and scan, by copy under
// the host's own mutex.
class SimulatedHost {
 public:
  void Update(common::Time time, const Eigen::Affine2d& local_pose, const sensor::PointCloud& scan,
              double match_score = 0.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_ = HostFrame{StampedPose{time, local_pose}, scan, match_score,
                       average_.Add(time, match_score)};
  }

  AgentServiceHooks Hooks() {
    AgentServiceHooks hooks;
    hooks.current_frame = [this]() -> std::optional<HostFrame> {
      std::lock_guard<std::mutex> lock(mutex_);
      return frame_;
    };
    return hooks;
  }

 private:
  std::mutex mutex_;
  std::optional<HostFrame> frame_;
  MatchScoreAverage average_;
};

}  // namespace evergreenslam::agent::testing

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_TEST_SERVICE_TEST_CLIENT_H_

/**
 * @file backend_task.h
 * @author hang chen (chen@hang.plus)
 * @brief One PoseGraph task per request, waited on with a promise and future.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_BACKEND_TASK_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_BACKEND_TASK_H_

#include <future>
#include <type_traits>

#include "lifelong/pose_graph.h"

namespace evergreenslam::agent {

// No timeout: the backend always drains its queue before Finish, and Stop() runs before that.
template <typename Function>
auto RunOnBackend(lifelong::PoseGraph& pose_graph, Function function)
    -> std::invoke_result_t<Function> {
  using Result = std::invoke_result_t<Function>;
  std::promise<Result> promise;
  std::future<Result> future = promise.get_future();
  pose_graph.Enqueue([&promise, &function] {
    if constexpr (std::is_void_v<Result>) {
      function();
      promise.set_value();
    } else {
      promise.set_value(function());
    }
  });
  return future.get();
}

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_BACKEND_TASK_H_

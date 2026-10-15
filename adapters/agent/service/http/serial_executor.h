/**
 * @file serial_executor.h
 * @author hang chen (chen@hang.plus)
 * @brief Runs the service's handlers one at a time, in arrival order, with a bounded wait line.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERIAL_EXECUTOR_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERIAL_EXECUTOR_H_

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>

namespace evergreenslam::agent {

class SerialExecutor {
 public:
  explicit SerialExecutor(int max_waiting) : max_waiting_(max_waiting) {}

  SerialExecutor(const SerialExecutor&) = delete;
  SerialExecutor& operator=(const SerialExecutor&) = delete;

  bool Run(const std::function<void()>& function);

 private:
  const int max_waiting_;
  std::mutex mutex_;
  std::condition_variable turn_;
  uint64_t next_ticket_ = 0;
  uint64_t now_serving_ = 0;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTP_SERIAL_EXECUTOR_H_

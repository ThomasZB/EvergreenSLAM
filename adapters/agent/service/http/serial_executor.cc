/**
 * @file serial_executor.cc
 * @author hang chen (chen@hang.plus)
 * @brief Runs the service's handlers one at a time, in arrival order, with a bounded wait line.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/http/serial_executor.h"

namespace evergreenslam::agent {
namespace {

class TurnEnd {
 public:
  TurnEnd(std::mutex& mutex, std::condition_variable& turn, uint64_t& now_serving)
      : mutex_(mutex), turn_(turn), now_serving_(now_serving) {}
  ~TurnEnd() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++now_serving_;
    }
    turn_.notify_all();
  }

  TurnEnd(const TurnEnd&) = delete;
  TurnEnd& operator=(const TurnEnd&) = delete;

 private:
  std::mutex& mutex_;
  std::condition_variable& turn_;
  uint64_t& now_serving_;
};

}  // namespace

bool SerialExecutor::Run(const std::function<void()>& function) {
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (next_ticket_ - now_serving_ > static_cast<uint64_t>(max_waiting_)) {
      return false;
    }
    const uint64_t ticket = next_ticket_++;
    turn_.wait(lock, [this, ticket] { return now_serving_ == ticket; });
  }
  // A handler may throw past its guard (httplib answers 500); the next caller must still run.
  const TurnEnd end(mutex_, turn_, now_serving_);
  function();
  return true;
}

}  // namespace evergreenslam::agent

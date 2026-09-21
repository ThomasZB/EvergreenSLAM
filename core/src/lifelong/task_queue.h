/**
 * @file task_queue.h
 * @author hang chen (chen@hang.plus)
 * @brief Single consumer work queue: one backend thread behind Enqueue.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_TASK_QUEUE_H_
#define EVERGREENSLAM_LIFELONG_TASK_QUEUE_H_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace evergreenslam::lifelong {

class TaskQueue {
 public:
  using Task = std::function<void()>;

  TaskQueue();
  ~TaskQueue();

  void Enqueue(Task task);
  void Drain();
  // Queued plus in flight, so 0 means quiescent.
  size_t pending_count() const;

 private:
  void ConsumerLoop();

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::condition_variable became_idle_;
  std::deque<Task> tasks_;
  bool executing_ = false;
  bool shutdown_ = false;
  std::thread consumer_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_TASK_QUEUE_H_

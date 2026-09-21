/**
 * @file task_queue.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/task_queue.h"

#include <glog/logging.h>

#include <utility>

namespace evergreenslam::lifelong {

TaskQueue::TaskQueue() : consumer_([this] { ConsumerLoop(); }) {}

TaskQueue::~TaskQueue() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
  }
  work_available_.notify_all();
  consumer_.join();
}

void TaskQueue::Enqueue(Task task) {
  CHECK(task != nullptr);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(!shutdown_) << "Enqueue after the queue started shutting down";
    tasks_.push_back(std::move(task));
  }
  work_available_.notify_one();
}

void TaskQueue::Drain() {
  CHECK(std::this_thread::get_id() != consumer_.get_id()) << "Drain is not reentrant";
  std::unique_lock<std::mutex> lock(mutex_);
  became_idle_.wait(lock, [this] { return tasks_.empty() && !executing_; });
}

size_t TaskQueue::pending_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return tasks_.size() + (executing_ ? 1 : 0);
}

void TaskQueue::ConsumerLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    work_available_.wait(lock, [this] { return !tasks_.empty() || shutdown_; });
    if (tasks_.empty()) {
      // Quit only once the queue is empty, so queued work never gets lost.
      return;
    }
    Task task = std::move(tasks_.front());
    tasks_.pop_front();
    executing_ = true;
    lock.unlock();
    task();
    lock.lock();
    executing_ = false;
    if (tasks_.empty()) {
      became_idle_.notify_all();
    }
    LOG_EVERY_N(INFO, 100) << "TaskQueue: " << tasks_.size() << " tasks pending";
  }
}

}  // namespace evergreenslam::lifelong

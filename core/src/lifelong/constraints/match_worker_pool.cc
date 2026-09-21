/**
 * @file match_worker_pool.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/match_worker_pool.h"

#include <glog/logging.h>

#include <algorithm>
#include <utility>

namespace evergreenslam::lifelong {

MatchWorkerPool::MatchWorkerPool(int num_workers) {
  CHECK_GT(num_workers, 0);
  workers_.reserve(num_workers);
  for (int i = 0; i < num_workers; ++i) {
    workers_.emplace_back([this] { WorkerLoop(); });
  }
}

MatchWorkerPool::~MatchWorkerPool() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
  }
  work_available_.notify_all();
  for (std::thread& worker : workers_) {
    worker.join();
  }
}

void MatchWorkerPool::Submit(std::function<void()> job) {
  CHECK(job != nullptr);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    CHECK(!shutdown_) << "Submit after the pool started shutting down";
    jobs_.push_back(std::move(job));
  }
  work_available_.notify_one();
}

void MatchWorkerPool::WaitIdle() const {
  std::unique_lock<std::mutex> lock(mutex_);
  became_idle_.wait(lock, [this] { return jobs_.empty() && num_running_ == 0; });
}

size_t MatchWorkerPool::pending_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return jobs_.size() + num_running_;
}

int MatchWorkerPool::DefaultNumWorkers() {
  const unsigned hardware = std::thread::hardware_concurrency();
  return static_cast<int>(std::min(4u, std::max(1u, hardware)));
}

void MatchWorkerPool::WorkerLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    work_available_.wait(lock, [this] { return !jobs_.empty() || shutdown_; });
    if (jobs_.empty()) {
      return;
    }
    std::function<void()> job = std::move(jobs_.front());
    jobs_.pop_front();
    ++num_running_;
    lock.unlock();
    job();
    lock.lock();
    --num_running_;
    if (jobs_.empty() && num_running_ == 0) {
      became_idle_.notify_all();
    }
    LOG_EVERY_N(INFO, 100) << "MatchWorkerPool: " << jobs_.size() << " jobs pending";
  }
}

}  // namespace evergreenslam::lifelong

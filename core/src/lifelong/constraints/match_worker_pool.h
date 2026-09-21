/**
 * @file match_worker_pool.h
 * @author hang chen (chen@hang.plus)
 * @brief Worker threads for the per-pair loop match chain, which is stateless per pair.
 * @version 0.1
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_CONSTRAINTS_MATCH_WORKER_POOL_H_
#define EVERGREENSLAM_LIFELONG_CONSTRAINTS_MATCH_WORKER_POOL_H_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace evergreenslam::lifelong {

// Jobs must read only immutable data pinned by the submitter and never touch the pose graph.
class MatchWorkerPool {
 public:
  explicit MatchWorkerPool(int num_workers);
  ~MatchWorkerPool();

  void Submit(std::function<void()> job);
  // Pipeline quiescence additionally needs the backend queue drained: jobs enqueue results there.
  void WaitIdle() const;

  int num_workers() const { return static_cast<int>(workers_.size()); }
  size_t pending_count() const;

  static int DefaultNumWorkers();

 private:
  void WorkerLoop();

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  mutable std::condition_variable became_idle_;
  std::deque<std::function<void()>> jobs_;
  int num_running_ = 0;
  bool shutdown_ = false;
  std::vector<std::thread> workers_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_CONSTRAINTS_MATCH_WORKER_POOL_H_

/**
 * @file match_worker_pool_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The pool's contract: parallel execution, WaitIdle quiescence, lossless shutdown.
 * @version 0.1
 * @date 2026-08-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/constraints/match_worker_pool.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace evergreenslam::lifelong {
namespace {

TEST(MatchWorkerPool, RunsEveryJobAndWaitIdleMeansIdle) {
  MatchWorkerPool pool(3);
  EXPECT_EQ(pool.num_workers(), 3);
  std::atomic<int> count{0};
  for (int i = 0; i < 200; ++i) {
    pool.Submit([&count] { ++count; });
  }
  pool.WaitIdle();
  EXPECT_EQ(count.load(), 200);
  EXPECT_EQ(pool.pending_count(), 0u);
}

TEST(MatchWorkerPool, WaitIdleWaitsForRunningJobsNotJustQueuedOnes) {
  MatchWorkerPool pool(2);
  std::atomic<int> done{0};
  for (int i = 0; i < 4; ++i) {
    pool.Submit([&done] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      ++done;
    });
  }
  pool.WaitIdle();
  EXPECT_EQ(done.load(), 4);
}

TEST(MatchWorkerPool, JobsActuallyRunConcurrently) {
  MatchWorkerPool pool(2);
  std::atomic<int> inside{0};
  std::atomic<bool> overlapped{false};
  for (int i = 0; i < 8; ++i) {
    pool.Submit([&inside, &overlapped] {
      if (inside.fetch_add(1) > 0) {
        overlapped = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      inside.fetch_sub(1);
    });
  }
  pool.WaitIdle();
  EXPECT_TRUE(overlapped.load()) << "two workers never ran at the same time";
}

TEST(MatchWorkerPool, SubmitIsThreadSafeAcrossProducers) {
  MatchWorkerPool pool(4);
  std::atomic<int> count{0};
  std::vector<std::thread> producers;
  for (int t = 0; t < 4; ++t) {
    producers.emplace_back([&pool, &count] {
      for (int i = 0; i < 100; ++i) {
        pool.Submit([&count] { ++count; });
      }
    });
  }
  for (std::thread& producer : producers) {
    producer.join();
  }
  pool.WaitIdle();
  EXPECT_EQ(count.load(), 400);
}

TEST(MatchWorkerPool, DestructorFinishesQueuedJobs) {
  std::atomic<int> count{0};
  {
    MatchWorkerPool pool(2);
    for (int i = 0; i < 32; ++i) {
      pool.Submit([&count] {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        ++count;
      });
    }
    // No WaitIdle: a submitted job must still deliver.
  }
  EXPECT_EQ(count.load(), 32);
}

TEST(MatchWorkerPool, DefaultNumWorkersIsSmallAndPositive) {
  const int workers = MatchWorkerPool::DefaultNumWorkers();
  EXPECT_GE(workers, 1);
  EXPECT_LE(workers, 4);
}

}  // namespace
}  // namespace evergreenslam::lifelong

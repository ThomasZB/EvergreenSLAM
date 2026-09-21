/**
 * @file task_queue_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Drain semantics, which every backend module is developed against.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/task_queue.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

namespace evergreenslam::lifelong {
namespace {

// FIFO on one consumer, Drain blocks until the queue is empty and the in-flight task finished.
TEST(TaskQueueTest, RunsEverythingInEnqueueOrderOffTheCallerThread) {
  TaskQueue queue;
  std::vector<int> order;  // consumer-only until Drain returns
  std::thread::id consumer_id;
  for (int i = 0; i < 100; ++i) {
    queue.Enqueue([&order, &consumer_id, i] {
      order.push_back(i);
      consumer_id = std::this_thread::get_id();
    });
  }
  queue.Drain();
  ASSERT_EQ(order.size(), 100u);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(order[i], i);
  }
  EXPECT_NE(consumer_id, std::this_thread::get_id());
  EXPECT_EQ(queue.pending_count(), 0u);
}

TEST(TaskQueueTest, DrainWaitsForTheInFlightTaskAndItsSpawn) {
  TaskQueue queue;
  std::atomic<int> done{0};
  queue.Enqueue([&queue, &done] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    queue.Enqueue([&done] { ++done; });
    ++done;
  });
  queue.Drain();
  EXPECT_EQ(done.load(), 2);
  EXPECT_EQ(queue.pending_count(), 0u);
}

TEST(TaskQueueTest, EnqueueIsThreadSafeAcrossProducers) {
  TaskQueue queue;
  std::atomic<int> count{0};
  std::vector<std::thread> producers;
  for (int t = 0; t < 4; ++t) {
    producers.emplace_back([&queue, &count] {
      for (int i = 0; i < 250; ++i) {
        queue.Enqueue([&count] { ++count; });
      }
    });
  }
  for (std::thread& producer : producers) {
    producer.join();
  }
  queue.Drain();
  EXPECT_EQ(count.load(), 1000);
}

TEST(TaskQueueTest, DestructorFinishesQueuedWorkAndJoins) {
  std::atomic<int> count{0};
  {
    TaskQueue queue;
    for (int i = 0; i < 50; ++i) {
      queue.Enqueue([&count] {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ++count;
      });
    }
    // No Drain: destruction alone must not lose a task.
  }
  EXPECT_EQ(count.load(), 50);
}

TEST(TaskQueueDeathTest, DrainFromInsideATaskIsRejected) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_DEATH(
      {
        TaskQueue queue;
        queue.Enqueue([&queue] { queue.Drain(); });
        queue.Drain();
      },
      "not reentrant");
}

}  // namespace
}  // namespace evergreenslam::lifelong

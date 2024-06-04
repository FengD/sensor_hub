#include "common/thread_safe_queue.h"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

namespace crdc {
namespace airi {
namespace common {

class ThreadSafeQueueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Add elements to the queue
    for (int i = 0; i < 5; ++i) {
      queue.enqueue(i);
    }
  }

  ThreadSafeQueue<int> queue;
};

TEST_F(ThreadSafeQueueTest, EnqueueDequeue) {
  // Enqueue a new element
  queue.enqueue(5);

  // Dequeue elements and check their values
  int value;
  for (int i = 0; i < 6; ++i) {
    ASSERT_TRUE(queue.dequeue(&value));
    ASSERT_EQ(value, i);
  }

  // Check if the queue is empty
  ASSERT_TRUE(queue.empty());
}

TEST_F(ThreadSafeQueueTest, WaitDequeue) {
  std::thread thread([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    queue.enqueue(5);
  });

  int value;
  ASSERT_TRUE(queue.wait_dequeue(&value));
  ASSERT_EQ(value, 0);

  ASSERT_TRUE(queue.wait_dequeue(&value));
  ASSERT_EQ(value, 5);

  thread.join();
}

TEST_F(ThreadSafeQueueTest, WaitForDequeue) {
  int value;
  ASSERT_TRUE(queue.wait_for_dequeue(&value, 100));
  ASSERT_EQ(value, 0);

  ASSERT_FALSE(queue.wait_for_dequeue(&value, 100));
}

TEST_F(ThreadSafeQueueTest, ClearQueue) {
  queue.clear();
  ASSERT_TRUE(queue.empty());
}

}  // namespace common
}  // namespace airi
}  // namespace crdc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "common/concurrent_queue.h"
#include <gtest/gtest.h>
#include <thread>

namespace sensor {
namespace hub {
namespace common {

class ConcurrentQueueTest : public ::testing::Test {
 protected:
  ConcurrentQueue<int> queue_;
};

TEST_F(ConcurrentQueueTest, PushPop) {
  std::thread t1([this]() {
    for (int i = 0; i < 100; ++i) {
      queue_.push(i);
    }
  });

  std::thread t2([this]() {
    for (int i = 0; i < 100; ++i) {
      int data;
      queue_.pop(&data);
      EXPECT_GE(data, 0);
      EXPECT_LE(data, 99);
    }
  });

  t1.join();
  t2.join();
}

TEST_F(ConcurrentQueueTest, Empty) {
  EXPECT_TRUE(queue_.empty());
  queue_.push(1);
  EXPECT_FALSE(queue_.empty());
}

TEST_F(ConcurrentQueueTest, Size) {
  EXPECT_EQ(queue_.size(), 0);
  queue_.push(1);
  EXPECT_EQ(queue_.size(), 1);
}

TEST_F(ConcurrentQueueTest, Clear) {
  queue_.push(1);
  queue_.push(2);
  queue_.clear();
  EXPECT_TRUE(queue_.empty());
}

class FixedSizeConQueueTest : public ::testing::Test {
 protected:
  FixedSizeConQueue<int> queue_{5};
};

TEST_F(FixedSizeConQueueTest, PushPop) {
  std::thread t1([this]() {
    for (int i = 0; i < 10; ++i) {
      queue_.push(i);
    }
  });

  std::thread t2([this]() {
    for (int i = 0; i < 10; ++i) {
      int data;
      queue_.pop(&data);
      EXPECT_GE(data, 0);
      EXPECT_LE(data, 9);
    }
  });

  t1.join();
  t2.join();
}

TEST_F(FixedSizeConQueueTest, Full) {
  for (int i = 0; i < 5; ++i) {
    queue_.push(i);
  }
  EXPECT_TRUE(queue_.full());
}

}  // namespace common
}  // namespace hub
}  // namespace sensor

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

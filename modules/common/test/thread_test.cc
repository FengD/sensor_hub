#include "common/thread.h"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

namespace crdc {
namespace airi {
namespace common {

class TestThread : public Thread {
 public:
  explicit TestThread(bool joinable = false, const std::string& name = "TestThread")
    : Thread(joinable, name) {}

  int get_counter() const { return counter_; }

 protected:
  void run() override {
    for (int i = 0; i < 5; ++i) {
      ++counter_;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

 private:
  int counter_ = 0;
};

class ThreadTest : public ::testing::Test {};

TEST_F(ThreadTest, StartJoin) {
  TestThread t(true);
  ASSERT_FALSE(t.is_alive());
  t.start();
  ASSERT_FALSE(t.is_alive());
  t.join();
  ASSERT_FALSE(t.is_alive());
  ASSERT_EQ(t.get_counter(), 5);
}

TEST_F(ThreadTest, ThreadName) {
  TestThread t;
  ASSERT_EQ(t.get_thread_name(), "TestThread");
  t.set_thread_name("NewTestThread");
  ASSERT_EQ(t.get_thread_name(), "NewTestThread");
}

TEST_F(ThreadTest, ThreadPriority) {
  TestThread t;
  ASSERT_EQ(t.get_priority(), 0);
  t.set_priority(10);
  ASSERT_EQ(t.get_priority(), 10);
}

}  // namespace common
}  // namespace airi
}  // namespace crdc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
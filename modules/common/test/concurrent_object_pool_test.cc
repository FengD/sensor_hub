#include "common/concurrent_object_pool.h"
#include <gtest/gtest.h>
#include <thread>
#include <vector>

namespace crdc {
namespace airi {
namespace common {

class TestObject {
 public:
  TestObject() : value_(0) {}
  explicit TestObject(int value) : value_(value) {}

  int GetValue() const { return value_; }
  void SetValue(int value) { value_ = value; }

 private:
  int value_;
};

TEST(CCObjectPoolTest, TestObjectPoolFunctionality) {
  constexpr uint32_t kPoolSize = 100;
  auto object_pool = std::make_shared<crdc::airi::common::CCObjectPool<TestObject>>(kPoolSize);
  object_pool->ConstructAll(42);

  // get object
  std::vector<std::shared_ptr<TestObject>> objects;
  for (uint32_t i = 0; i < kPoolSize; ++i) {
    auto obj = object_pool->GetObject();
    ASSERT_NE(obj, nullptr);
    ASSERT_EQ(obj->GetValue(), 42);
    objects.push_back(std::move(obj));
  }

  // pool is empty
  auto exhausted_obj = object_pool->GetObject();
  ASSERT_EQ(exhausted_obj, nullptr);

  // relese object
  objects.clear();
  auto recycled_obj = object_pool->GetObject();
  ASSERT_NE(recycled_obj, nullptr);
  ASSERT_EQ(recycled_obj->GetValue(), 42);
}

TEST(CCObjectPoolTest, TestObjectPoolMultiThreaded) {
  constexpr uint32_t kPoolSize = 100;
  constexpr uint32_t kThreadCount = 10;
  auto object_pool = std::make_shared<crdc::airi::common::CCObjectPool<TestObject>>(kPoolSize);
  object_pool->ConstructAll(42);

  auto test_func = [&object_pool]() {
    for (int i = 0; i < 1000; ++i) {
      auto obj = object_pool->GetObject();
      if (obj) {
        ASSERT_EQ(obj->GetValue(), 42);
      }
    }
  };

  std::vector<std::thread> threads;
  for (uint32_t i = 0; i < kThreadCount; ++i) {
    threads.emplace_back(test_func);
  }

  for (auto &thread : threads) {
    thread.join();
  }
}

TEST(CCObjectPoolTest, TestConstructObject) {
  constexpr uint32_t kPoolSize = 100;
  auto object_pool = std::make_shared<crdc::airi::common::CCObjectPool<TestObject>>(kPoolSize);

  std::vector<std::shared_ptr<TestObject>> objects;
  for (uint32_t i = 0; i < kPoolSize; ++i) {
    auto obj = object_pool->ConstructObject(84);
    ASSERT_NE(obj, nullptr);
    ASSERT_EQ(obj->GetValue(), 84);
    objects.push_back(std::move(obj));
  }

  auto exhausted_obj = object_pool->ConstructObject(84);
  ASSERT_EQ(exhausted_obj, nullptr);

  // release and check the constructor
  objects.clear();
  auto recycled_obj = object_pool->ConstructObject(84);
  ASSERT_NE(recycled_obj, nullptr);
  ASSERT_EQ(recycled_obj->GetValue(), 84);
}

// TEST(CCObjectPoolTest, TestEmptyPool) {
//   auto object_pool = std::make_shared<crdc::airi::common::CCObjectPool<TestObject>>(0);
//   auto obj = object_pool->GetObject();
//   ASSERT_EQ(obj, nullptr);
// }

// TEST(CCObjectPoolTest, TestMaxPoolSize) {
//   constexpr uint32_t kMaxPoolSize = std::numeric_limits<uint32_t>::max();
//   auto object_pool = std::make_shared<crdc::airi::common::CCObjectPool<TestObject>>(kMaxPoolSize);
// }

// TEST(CCObjectPoolTest, TestReleaseUnallocatedObject) {
//   constexpr uint32_t kPoolSize = 100;
//   auto object_pool = std::make_shared<crdc::airi::common::CCObjectPool<TestObject>>(kPoolSize);
//   TestObject unallocated_object;

//   // object_pool->ReleaseObject(&unallocated_object);
// }

}  // namespace common
}  // namespace airi
}  // namespace crdc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "common/singleton.h"
#include <gtest/gtest.h>

namespace sensor {
namespace hub {
namespace common {

class SampleSingleton {
  MAKE_SINGLETON(SampleSingleton);

 public:
  int get_value() const { return value_; }
  void set_value(int value) { value_ = value; }

 private:
  int value_ = 0;
};

class SingletonTest : public ::testing::Test {
};

TEST_F(SingletonTest, SingletonInstance) {
  SampleSingleton* instance1 = Singleton<SampleSingleton>::get();
  SampleSingleton* instance2 = Singleton<SampleSingleton>::get();

  ASSERT_EQ(instance1, instance2);

  instance1->set_value(42);
  ASSERT_EQ(instance1->get_value(), 42);
  ASSERT_EQ(instance2->get_value(), 42);
}

}  // namespace common
}  // namespace hub
}  // namespace sensor

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

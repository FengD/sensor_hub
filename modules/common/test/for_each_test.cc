#include "common/for_each.h"
#include <gtest/gtest.h>
#include <vector>

namespace sensor {
namespace hub {
namespace common {

class ForEachTest : public ::testing::Test {
};

TEST_F(ForEachTest, Vector) {
  std::vector<int> vec = {0, 1, 2, 3, 4};
  int counter = 0;
  std::vector<int>::iterator iter;
  FOR_EACH(iter, vec.begin(), vec.end()) {
    ASSERT_EQ(*iter, vec[counter]);
    counter++;
  }
}

TEST_F(ForEachTest, Integer) {
  int begin = 0;
  int end = 5;
  int counter = 0;
  int i = 0;
  FOR_EACH(i, begin, end) {
    ASSERT_EQ(i, counter);
    counter++;
  }
}

}  // namespace common
}  // namespace hub
}  // namespace sensor

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "util/util.h"
#include <gtest/gtest.h>
#include <time.h>
#include <string>
#include <vector>
#include <algorithm>

namespace {

class UtilTest : public ::testing::Test {
 protected:
  UtilTest() {
    // You can do set-up work for each test here.
  }

  ~UtilTest() override {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:
  void SetUp() override {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test suite for Util.
};

TEST_F(UtilTest, TestCalcTimeDiff) {
  struct timespec lo = {5, 500000000};
  struct timespec hi = {7, 300000000};
  struct timespec diff;
  calc_time_diff(&lo, &hi, &diff);
  EXPECT_EQ(diff.tv_sec, 1);
  EXPECT_EQ(diff.tv_nsec, 800000000);
}

TEST_F(UtilTest, TestSplit) {
  std::string s = "This is a test string.";
  std::string seperator = " ";
  std::vector<std::string> result;
  split(s, seperator, &result);
  std::vector<std::string> expected = {"This", "is", "a", "test", "string."};
  EXPECT_EQ(result, expected);
}

TEST_F(UtilTest, TestGetAllFilesInFolder) {
  std::string folder = "test_folder";
  std::vector<std::string> files;
  getAllFilesInFolder(folder, &files);
  std::sort(files.begin(), files.end());
  std::vector<std::string> expected = {};
  EXPECT_EQ(files, expected);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

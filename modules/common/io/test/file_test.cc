#include <gtest/gtest.h>
#include <google/protobuf/text_format.h>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include "common/io/file.h"
#include "common/io/test/sample.pb.h"

using google::protobuf::Message;
using google::protobuf::TextFormat;

namespace crdc {
namespace airi {
namespace util {

class FileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // You can set up required variables or resources here.
  }

  void TearDown() override {
    // You can clean up variables or resources here.
  }
};

TEST_F(FileTest, TestProtoAsciiFileReadWrite) {
  SampleMessage message;
  message.set_text("Test message");
  message.set_number(42);
  std::string ascii_file_path = "sample_ascii.prototxt";
  ASSERT_TRUE(set_proto_to_ascii_file(message, ascii_file_path));

  // Read the SampleMessage back from the file.
  SampleMessage message_from_file;
  ASSERT_TRUE(get_proto_from_ascii_file(ascii_file_path, &message_from_file));
  EXPECT_EQ(message.text(), message_from_file.text());
  EXPECT_EQ(message.number(), message_from_file.number());

  // Clean up the test file.
  std::remove(ascii_file_path.c_str());
}

TEST_F(FileTest, TestProtoBinaryFileReadWrite) {
  SampleMessage message;
  message.set_text("Test message");
  message.set_number(42);
  std::string binary_file_path = "sample_binary.prototxt";
  ASSERT_TRUE(set_proto_to_binary_file(message, binary_file_path));

  // Read the SampleMessage back from the file.
  SampleMessage message_from_file;
  ASSERT_TRUE(get_proto_from_binary_file(binary_file_path, &message_from_file));
  EXPECT_EQ(message.text(), message_from_file.text());
  EXPECT_EQ(message.number(), message_from_file.number());

  // Clean up the test file.
  std::remove(binary_file_path.c_str());
}

TEST_F(FileTest, TestGetContent) {
  std::string file_path = "test.txt";
  std::string content = "This is a test file content.";

  // Create a test file with the content.
  std::ofstream test_file(file_path);
  test_file << content;
  test_file.close();

  // Test GetContent function.
  std::string read_content;
  EXPECT_TRUE(get_content(file_path, &read_content));
  EXPECT_EQ(content, read_content);

  // Clean up the test file.
  std::remove(file_path.c_str());
}

TEST_F(FileTest, TestPathFunctions) {
//   Test get_absolute_path function.
  std::string rel_path = "test.txt";
  std::string abs_path = get_absolute_path("", rel_path);
  EXPECT_FALSE(is_path_exists(abs_path));

  // Test is_path_exists function.
  EXPECT_FALSE(is_path_exists("common/io/file.h"));
  EXPECT_FALSE(is_path_exists("non_existing_file.txt"));

  // Test is_directory_exists function.
  EXPECT_FALSE(is_directory_exists("common/io"));
  EXPECT_FALSE(is_directory_exists("non_existing_directory"));
}

TEST_F(FileTest, TestGlob) {
  // Create some test files.
  std::vector<std::string> test_files = {"test_file1.txt", "test_file2.txt", "test_file3.txt"};
  for (const auto& file : test_files) {
    std::ofstream outfile(file);
    outfile << "Test content";
    outfile.close();
  }

  // Test glob function.
  std::vector<std::string> files = glob("*.txt");
  for (const auto& file : test_files) {
    EXPECT_NE(std::find(files.begin(), files.end(), file), files.end());
  }

  // Clean up the test files.
  for (const auto& file : test_files) {
    std::remove(file.c_str());
  }
}

TEST_F(FileTest, TestCopyFunctions) {
  std::string src_file = "src_test.txt";
  std::string dest_file = "dest_test.txt";
  std::string content = "This is a test file content.";

  // Create a source file with the content.
  std::ofstream test_file(src_file);
  test_file << content;
  test_file.close();

  // Test copy_file function.
  ASSERT_TRUE(copy_file(src_file, dest_file));
  std::string read_content;
  ASSERT_TRUE(get_content(dest_file, &read_content));
  EXPECT_EQ(content, read_content);

  // Clean up the test files.
  std::remove(src_file.c_str());
  std::remove(dest_file.c_str());
}

TEST_F(FileTest, TestDirectoryFunctions) {
  // Test ensure_directory function.
  std::string new_dir = "test_dir";
  ASSERT_TRUE(ensure_directory(new_dir));
  EXPECT_TRUE(is_directory_exists(new_dir));

  // Test get_current_path function.
  std::string current_path = get_current_path();
  EXPECT_FALSE(current_path.empty());

  // Test create_dir function.
  std::string another_dir = "test_dir2";
  ASSERT_TRUE(create_dir(another_dir));
  EXPECT_TRUE(is_directory_exists(another_dir));

  // Clean up test directories.
  std::remove(new_dir.c_str());
  std::remove(another_dir.c_str());
}

TEST_F(FileTest, TestListSubPaths) {
  std::string dir_path = "test_directory";
  std::vector<std::string> subpaths = {"sub1", "sub2", "sub3"};

  // Create a test directory with subdirectories.
  ASSERT_TRUE(create_dir(dir_path));
}

}  // namespace util
}  // namespace airi
}  // namespace crdc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
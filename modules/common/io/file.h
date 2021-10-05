// Copyright (C) 2021 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: file

#pragma once

#include <string>
#include <vector>
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "gflags/gflags.h"

namespace crdc {
namespace airi {
namespace util {

// @brief file type: file or directory
enum FileType {
    TYPE_FILE,
    TYPE_DIR
};

bool set_proto_to_ascii_file(const google::protobuf::Message& message,
                             const int& file_descriptor);

bool set_proto_to_ascii_file(const google::protobuf::Message& message,
                             const std::string& file_name);

bool get_proto_from_ascii_file(const std::string& file_name,
                               google::protobuf::Message *message);

bool set_proto_to_binary_file(const google::protobuf::Message& message,
                             const std::string& file_name);

bool get_proto_from_binary_file(const std::string& file_name,
                                google::protobuf::Message *message);

bool get_proto_from_file(const std::string& file_name,
                         google::protobuf::Message *message);

bool get_content(const std::string& file_name,
                 std::string *content);

std::string get_absolute_path(const std::string& prefix, const std::string& relative_path);

bool is_path_exists(const std::string& path);

bool is_directory_exists(const std::string& directory_path);

std::vector<std::string> glob(const std::string& pattern);

bool copy_file(const std::string& from, const std::string& to);

bool copy_directory(const std::string& from, const std::string& to);

bool copy(const std::string& from, const std::string& to);

bool ensure_directory(const std::string& directory_path);

std::string get_current_path();

bool create_dir(const std::string& directory_path);

}  // namespace util
}  // namespace airi
}  // namespace crdc
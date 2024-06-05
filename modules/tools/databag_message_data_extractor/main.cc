// Copyright (C) 2021 FengD
// License: Modified BSD Software License Agreement
// Author: Binqi Fan, Zilou Cao, Jianfei Jiang
// Description: databag_message_data_extractor

#include <gflags/gflags.h>
#include <omp.h>
#include <sys/types.h>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include "camera_drivers/proto/camera_component_config.pb.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "camera_drivers/proto/encoder_config.pb.h"
#include "common/common.h"
#include "ins_drivers/proto/ins_component_config.pb.h"
#include "ins_drivers/proto/ins_config.pb.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "tools/databag_message_data_extractor/data_extractor/data_extractor.h"
#include "tools/databag_message_data_extractor/sensor/point_cloud.h"

#define MODULE "extract_frame"
DEFINE_string(input_file, "data_packet/",
              "input file recorded by cyber_record");

namespace crdc {
namespace airi {

void creat_folder(const std::string &folder_path) {
  std::string command;
  command = "mkdir -p " + folder_path;
  if (system(command.c_str())) {
    LOG(WARNING) << "[TypeConvertor] Failed to create dir " << folder_path;
  }
}

void extract_packet(const std::string &path) {
  std::string temp_path(path);
  auto pos = path.rfind('/');
  if (pos == path.length()-1) {
    temp_path = path.substr(0, pos);  // remove the last '/' if path is end with '/'
  }
  boost::filesystem::path boost_path(temp_path);
  std::string parent_path = boost_path.stem().string();
  std::string save_path = std::getenv("SAVE_PATH") == NULL ? "." : std::getenv("SAVE_PATH");
  std::string folder_parent_path =
  save_path +"/" + parent_path + "/extracted_data/";
  std::cout << "save_path: " << folder_parent_path << " , input_file: " << path << std::endl;
  std::vector<std::string> lidar_channel_names;
#ifndef WITH_ROS2
  apollo::cyber::record::RecordReader reader_(path);
  std::set<std::string> test = reader_.GetChannelList();
  for (auto it : test) {
    if (it.find("LIDAR") != std::string::npos || it.find("RADAR") != std::string::npos) {
       lidar_channel_names.push_back(it);
    }
    creat_folder(folder_parent_path + it);
  }
#else
    rosbag2_cpp::Reader reader_;
    reader_.open(path);  // open file
    std::unordered_map<std::string, std::string> topic_and_type;
    auto topics = reader_.get_all_topics_and_types();
    for (const auto & topic_with_type : topics) {
        std::string channel_name = topic_with_type.name;
        if (channel_name.find("LIDAR") != std::string::npos ||
            channel_name.find("RADAR") != std::string::npos) {
          lidar_channel_names.push_back(channel_name);
        }
    }
    reader_.close();
#endif
  std::string format = std::getenv("FORMAT") == NULL ? "ASCII" : std::getenv("FORMAT");
  transform(format.begin(), format.end(), format.begin(), ::tolower);  // case-insensitive
  std::string type = std::getenv("TYPE") == NULL ? "png" : std::getenv("TYPE");
  transform(type.begin(), type.end(), type.begin(), ::tolower);
  pcd::PointCloud save_lidar_pcd(path, folder_parent_path, format, lidar_channel_names);
#ifndef WITH_ROS2
  data_extractor::DataExtractor data_extractor;
  int cyber_status = data_extractor.init_cyber_env();
  if (apollo::cyber::SUCC != cyber_status) {
    throw("Fatal Error: cyber init failed!");
  }
  int camera_status =
      data_extractor.extract_frame(path, folder_parent_path, type);
  LOG(INFO) << "CYBERRT EXTRACT FRAMES" << std::endl;
  if (apollo::cyber::SUCC != camera_status) {
    throw("Fatal Error: listener init failed!");
  }
#else
  data_extractor::DataExtractor data_extractor;
  data_extractor.extract_frame(path, folder_parent_path, type);
#endif
}

int extract_frames(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
#ifndef WITH_ROS2
  apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
  apollo::cyber::Init(MODULE);
#endif
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  std::string data_dir(FLAGS_input_file);
  std::vector<std::string> files;
  boost::filesystem::recursive_directory_iterator end_iter;
  for (boost::filesystem::recursive_directory_iterator iter(data_dir);
       iter != end_iter; iter++) {
    if (boost::filesystem::is_regular_file(*iter)) {
      files.push_back((*iter).path().string());
    }
  }
#ifndef WITH_ROS2
  int thread_num = omp_get_max_threads();
  LOG(INFO) << thread_num << " workers working on parallel!";
  thread_num = thread_num > 16 ? thread_num - 6 : 16;
  #pragma omp parallel for num_threads(thread_num)
  for (size_t i = 0; i < files.size(); ++i) {
    extract_packet(files[i]);
  }
  return apollo::cyber::SUCC;
#else
  for (size_t i = 0; i < files.size(); ++i) {
    extract_packet(files[i]);
  }
  return 0;
#endif
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char *argv[]) { return crdc::airi::extract_frames(argc, argv); }

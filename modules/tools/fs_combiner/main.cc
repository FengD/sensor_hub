// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / JianFei JIANG
// Description: sensor extract main

#include <gflags/gflags.h>
#include <omp.h>
#include <vector>
#include "common/common.h"
#include "tools/fs_combiner/fs_combiner.h"

#ifndef WITH_ROS2
#define MODULE "TYPECONVERTOR"
#else
DEFINE_string(module, "TYPECONVERTOR", "The name of this module");
#endif
DEFINE_string(ins_config_file,
             "params/drivers/ins/databag_message_type_convertor/ins_config.prototxt",
              "path of config file");
DEFINE_string(lidar_config_file,
              "params/drivers/lidar/databag_message_type_convertor/lidar_config.prototxt",
              "path of config file");
DEFINE_string(camera_config_file,
              "params/drivers/camera/databag_message_type_convertor/camera_config.prototxt",
              "path of config file");
DEFINE_string(input_file, "data_packet/", "input file recorded by cyber_record/Ros2bag");
DEFINE_string(input_file_lidar, "data_packet/", "input file recorded by cyber_record/Ros2bag");

namespace crdc {
namespace airi {
namespace sensor {

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!std::getenv("CRDC_WS")) {
    LOG(FATAL) << "[TYPECONVERTOR] CRDC_WS not setting!";
  } else {
    LOG(INFO) << "[TYPECONVERTOR] Current CRDC_WS: "
              << std::string(std::getenv("CRDC_WS"));
  }
#ifndef WITH_ROS2
  apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
  apollo::cyber::Init(MODULE);
#else
  rclcpp::init(argc, argv);
  common::Singleton<crdc::airi::TypeConverAFOutput>::get()->init(FLAGS_module);
#endif
  std::string data_dir(FLAGS_input_file);
  std::string data_dir_lidar(FLAGS_input_file_lidar); // data folder path
  crdc::airi::FSCombiner write_channel;
  std::vector<std::string> files;
  std::vector<std::string> files_lidar;
  boost::filesystem::recursive_directory_iterator end_iter;
  boost::filesystem::recursive_directory_iterator end_iter_lidar;
  for (boost::filesystem::recursive_directory_iterator iter(data_dir);
       iter != end_iter; iter++) {
    if (boost::filesystem::is_regular_file(*iter)) {
      files.push_back((*iter).path().string());
    }
  }
  for (boost::filesystem::recursive_directory_iterator iter(data_dir_lidar);
       iter != end_iter_lidar; iter++) {
    // std::cout << "2!!!!!!!!!!!" << std::endl;
    if (boost::filesystem::is_regular_file(*iter)) {
      files_lidar.push_back((*iter).path().string());
    }
  }
  // TODO(yuan.sun) use omp_get_max_threads
  // int thread_num = omp_get_max_threads();
  // LOG(INFO) << thread_num << " workers working on parallel!";
  // thread_num = thread_num > 16 ? thread_num - 6 : 16;
  // #pragma omp parallel for num_threads(thread_num)
  for (size_t i = 0; i < files.size(); ++i) {
     write_channel.write_packet(files[i], files_lidar[i]);
  }

  return 0;
}

}  // namespace sensor
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) { return crdc::airi::sensor::main(argc, argv); }

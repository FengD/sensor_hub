// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / JianFei JIANG
// Description: sensor data extract

#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "camera_drivers/proto/camera_component_config.pb.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "common/common.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "cyber/sensor_proto/image.pb.h"
#include "cyber/sensor_proto/ins.pb.h"
#include "ins_drivers/parser/parser.h"
#include "ins_drivers/proto/ins_component_config.pb.h"
#include "ins_drivers/proto/ins_config.pb.h"
#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "tools/databag_message_data_extractor/data_extractor/data_extractor.h"
#include "tools/databag_message_data_extractor/sensor/point_cloud.h"

using apollo::cyber::record::RecordReader;

namespace crdc {
namespace airi {

class Sensor : public common::Thread {
 public:
  Sensor(const InsComponentConfig& config, const std::string& type);
  Sensor(const CameraComponentConfig& config, const std::string& type);
  Sensor(const LidarComponentConfig& config, const std::string& type);
  virtual ~Sensor() = default;
  void stop();

  std::string get_name() const { return sensor_name_; }

 private:
  /**
   * @brief The run function execute the whole process of the ins loop process.
   */
  void run() override;

  /**
   * @brief Init the encoder if defined.
   */
  bool init_parser();

  /**
   * @brief Init config and file save path
   */
  template <typename T>
  std::string init(const T& config) {
    stop_ = false;
    packets_ = nullptr;
    folder_path_ = std::string(std::getenv("SAVE_PATH")) + "/extracted_data/" +
                   config.channel_name();
    creat_folder(folder_path_);

    set_thread_name(config.frame_id());

    if (config.has_priority()) {
      set_priority(config.priority());
    }

    std::string config_file_path =
        std::string(std::getenv("CRDC_WS")) + '/' + config.config_file();

    if (!crdc::airi::util::is_path_exists(config_file_path)) {
      LOG(FATAL) << "[" << get_thread_name()
                 << "] proto file not exits: " << config_file_path;
      return "error";
    } else {
      return config_file_path;
    }
  }

  /**
   * @brief Init the input if defined.
   */
  template <typename T>
  bool init_input(const T& input_config);

  /**
   * @brief cyber reader first data file 
   */
  template <typename T>
  void init_reader_data(const T& config) {
    file_index_ = 0;
    if (config.cyber_config().file_path(file_index_).find(".record.0") !=
        std::string::npos) {
      reader_ = std::make_shared<RecordReader>(
          config.cyber_config().file_path(file_index_));
    } else {
      std::string data_dir(config.cyber_config().file_path(file_index_));
      boost::filesystem::recursive_directory_iterator end_iter;
      for (boost::filesystem::recursive_directory_iterator iter(data_dir);
           iter != end_iter; iter++) {
        if (boost::filesystem::is_regular_file(*iter) &&
            (*iter).path().string().find(".record.0") != std::string::npos) {
          pcaket_path_.push_back((*iter).path().string());
        }
      }
      reader_ = std::make_shared<RecordReader>(pcaket_path_.at(file_index_));
    }
  }

   /**
   * @brief Cyber reader file to get sensor data
   */
  template <typename T>
  void reader_data(const T& config) {
    file_index_++;
    if (config.cyber_config().file_path(0).find(".record.0") !=
        std::string::npos) {
      if (file_index_ < config.cyber_config().file_path().size()) {
        reader_ = std::make_shared<RecordReader>(
            config.cyber_config().file_path(file_index_));
        LOG(ERROR) << "[INPUT_PACKET] "
                   << config.cyber_config().file_path(file_index_);
      } else {
        LOG(ERROR) << "[" << config.frame_id() << "] "
                   << "Read Cyber Files Finished!";
      }
    } else {
      if (file_index_ < pcaket_path_.size()) {
        reader_ = std::make_shared<RecordReader>(pcaket_path_.at(file_index_));
        LOG(ERROR) << "[INPUT_PACKET] " << pcaket_path_.at(file_index_);
      } else {
        LOG(ERROR) << "[" << config.frame_id() << "] "
                   << "Read Cyber Files Finished!";
      }
    }
  }

  /**
   * @brief get sensor data, need to be redefined in subclass
   * @param packet lists
   * @return status
   */
  template <typename T>
  int get_data(Packet** packet, const T& config);

  /**
   * @brief Check if t
   * @return statushe packet pool is full.
   */
  template <typename T>
  bool is_packet_pool_full(const T& config) {
    return packet_cur_index_ >= config.pool_size();
  }

  /**
   * @brief get raw packet
   * @return packet
   */
  template <typename T>
  Packet* get_raw_packet(const T& config) {
    if (packet_cur_index_ >= config.pool_size()) {
      LOG(ERROR) << "[" << config.frame_id() << "] pool is full.";
      return nullptr;
    }
    return packets_->mutable_packet(packet_cur_index_++);
  }

  /**
   * @brief init the pool
   * @return status
   */
  template <typename T>
  bool init_pool(const T& config) {
    packet_cur_index_ = 0;
    packets_ = std::make_shared<Packets>();
    for (auto i = 0; i < config.pool_size(); ++i) {
      auto packet = packets_->add_packet();
      packet->mutable_data()->reserve(config.packet_size());
      packet->set_version(0x0001);
    }
    return true;
  }

  /**
   * @brief Get all the data packets
   * @param packet
   * @return packet size
   */
  int32_t get_packets(std::shared_ptr<Packets>* packets) {
    *packets = packets_;
    return packet_cur_index_;
  }

  /**
   * @brief Clear the packet pool
   */
  void clear_pool() { packet_cur_index_ = 0; }

  /**
   * @brief creat folder for extract data 
   */
  void creat_folder(const std::string& folder_path) {
    std::string command;
    command = "mkdir -p " + folder_path;
    system(command.c_str());
  }

  bool stop_;
  int cur_index_, file_index_;
  int32_t packet_cur_index_;
  std::string sensor_name_, sensor_type_, folder_path_;

  InsConfig ins_config_;
  InsInputConfig ins_input_config_;
  InsComponentConfig ins_component_config_;
  std::shared_ptr<InsParser> ins_parser_;

  LidarConfig lidar_config_;
  LidarInputConfig lidar_input_config_;
  LidarComponentConfig lidar_component_config_;
  std::shared_ptr<LidarParser> lidar_parser_;

  CameraConfig camera_config_;
  CameraInputConfig camera_input_config_;
  CameraComponentConfig camera_component_config_;

  Packets proto_packets_;
  std::shared_ptr<RecordReader> reader_;
  std::shared_ptr<Packets> packets_;
  crdc::airi::pcd::PointCloud cloudtopcd_;
  crdc::airi::data_extractor::DataExtractor cyber_tool_;
  std::vector<std::string> pcaket_path_;
};

}  // namespace airi
}  // namespace crdc

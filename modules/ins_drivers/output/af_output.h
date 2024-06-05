// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Zilou Cao
// Description: AF_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msg/msg/packets.hpp"
#include "sensor_msg/msg/ins.hpp"

namespace crdc {
namespace airi {

class InsAFOutput {
 public:
  InsAFOutput() = default;
  virtual ~InsAFOutput() = default;

  /**
   * @brief Init the af output
   * @param the name of the node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[INS_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[INS_OUTPUT] Node name: " << node_name;
    node_ = std::make_shared<rclcpp::Node>(node_name);
    ins_data_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::Ins>(node_));
    packets_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::Packets>(node_));
    return true;
  }

  /**
   * @brief Send the ins data message by topic
   * @param topic name
   * @param proto ins data ptr
   * @return status
   */
  bool write_ins_data(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::Ins>& ins_msg) {
    ins_msg->header.module_name = "InsDriver";
    ins_msg->header.timestamp_sec = static_cast<double>(get_now_microsecond()) / 1000000;
    return ins_data_writer_ptr_->write(topic, ins_msg);
  }

  /**
   * @brief Send the packets message by topic
   * @param topic name
   * @param proto packets ptr
   * @return status
   */
  bool write_packet(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::Packets>& packets_msg) {
    return packets_writer_ptr_->write(topic, packets_msg);
  }

 private:
  friend class common::Singleton<InsAFOutput>;
  template <typename MessageT>
  class AFChannelWriter {
   public:
    explicit AFChannelWriter(std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    bool write(const std::string& topic, const std::shared_ptr<MessageT>& msg_ptr) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (writer_.find(topic) == writer_.end()) {
        writer_[topic] = node_->create_publisher<MessageT>(topic, rclcpp::QoS(10));
      }
      writer_[topic]->publish(*msg_ptr);
      return true;
    }

   private:
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<MessageT>>> writer_;
    std::shared_ptr<rclcpp::Node> node_;
    std::mutex mutex_;
  };

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Ins>> ins_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Packets>> packets_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc

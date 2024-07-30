// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Zilou Cao
// Description: af2.0_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace crdc {
namespace airi {

class CameraROSOutput {
 public:
  CameraROSOutput() = default;
  virtual ~CameraROSOutput() = default;

  /**
   * @brief Init the af output
   * @param the name of node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[CAMERA_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[CAMERA_OUTPUT] Node name: " << node_name;
    node_ = std::make_shared<rclcpp::Node>(node_name);
    image_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::Image>(node_));
    encode_image_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::CompressedImage>(node_));
    return true;
  }

  bool write_image(const std::string& topic, sensor_msgs::msg::Image& msg) {
    return image_writer_ptr_->write(topic, msg);
  }

  bool write_image(const std::string& topic, sensor_msgs::msg::CompressedImage& msg) {
    return encode_image_writer_ptr_->write(topic, msg);
  }

  std::shared_ptr<rclcpp::Node> get_node() {
    return node_;
  }

 private:
  friend class common::Singleton<CameraROSOutput>;
  template <typename T>
  class AFChannelWriter {
   public:
    explicit AFChannelWriter(std::shared_ptr<rclcpp::Node>& node) : node_(node) {}
    bool write(const std::string& topic, const T& msg) {
      if (writer_.find(topic) == writer_.end()) {
        writer_[topic] = node_->create_publisher<T>(topic, rclcpp::QoS(10));
      }
      writer_[topic]->publish(msg);
      return true;
    }
   private:
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<T>>> writer_;
    std::shared_ptr<rclcpp::Node> node_;
  };
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::Image>> image_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::CompressedImage>> encode_image_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc

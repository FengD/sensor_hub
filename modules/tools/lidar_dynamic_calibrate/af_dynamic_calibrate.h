// Copyright (C) 2023 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Yuan Sun
// Description: af2.0_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define SUB_NODE_NAME "boundary_visu"

namespace crdc {
namespace airi {

class DynamicCalibrateOutput {
 public:
  DynamicCalibrateOutput() = default;
  virtual ~DynamicCalibrateOutput() = default;

  /**
   * @brief Init the af output
   * @param the name of node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[AF_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[AF_OUTPUT] Node name: " << node_name;
    node_ = std::make_shared<rclcpp::Node>(node_name);
    cloud_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::PointCloud2>(node_));
    return true;
  }
  /**
   * @brief Send the cloud message by topic
   * @param topic name
   * @param proto cloud ptr
   * @return status
   */
  bool write_cloud(const std::string& topic,
      const std::shared_ptr<sensor_msgs::msg::PointCloud2>& cloud_msg) {
    return cloud_writer_ptr_->write(topic, cloud_msg);
  }

  std::shared_ptr<rclcpp::Node> get_node() {
    return node_;
  }

 private:
  friend class common::Singleton<DynamicCalibrateOutput>;
  template <typename T>
  class AFChannelWriter {
   public:
    explicit AFChannelWriter(std::shared_ptr<rclcpp::Node>& node) : node_(node) {}
    bool write(const std::string& topic, const std::shared_ptr<T>& msg_ptr) {
      if (writer_.find(topic) == writer_.end()) {
        writer_[topic] = node_->create_publisher<T>(topic, rclcpp::QoS(10));
      }
      writer_[topic]->publish(*msg_ptr);
      return true;
    }

   private:
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<T>>> writer_;
    std::shared_ptr<rclcpp::Node> node_;
  };

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::PointCloud2>> cloud_writer_ptr_;
};

class AFNode {
 public:
  AFNode() {
    node_ = std::make_shared<rclcpp::Node>(SUB_NODE_NAME);
  }

  std::shared_ptr<rclcpp::Node> get_node() {
    return node_;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace airi
}  // namespace crdc

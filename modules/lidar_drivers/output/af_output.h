// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING, Zilou Cao
// Description: cyber_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msg/msg/pose.hpp"
#include "sensor_msg/msg/packets.hpp"
#include "sensor_msg/msg/point_cloud2.hpp"
#include "sensor_msg/msg/point_clouds2.hpp"

namespace crdc {
namespace airi {

class LidarAFOutput {
 public:
  LidarAFOutput() = default;
  virtual ~LidarAFOutput() = default;

  /**
   * @brief Init the cyber output
   * @param the name of the node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[LIDAR_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[LIDAR_OUTPUT] Node name: " << node_name;
    node_ = std::make_shared<rclcpp::Node>(node_name);
    cloud_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::PointCloud2>(node_));
    packets_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::Packets>(node_));
    fusion_clouds_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::PointClouds2>(node_));
    return true;
  }

  // void subscribe_pose(const std::string& topic,
  //       const std::function<void(const std::shared_ptr<sensor_msg::msg::Pose>&)>& reader_func) {
  //   rclcpp::Subscription<sensor_msg::msg::Pose>::SharedPtr pose_listened =
  //             node_.create_subscription<sensor_msg::msg::Pose>(topic, 10, reader_func);
  // }

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

  /**
   * @brief Send the merged cloud
   * @param the cloud list
   * @return status
   */
  bool write_fusion_clouds(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::PointClouds2>& clouds_msg) {
    return fusion_clouds_writer_ptr_->write(topic, clouds_msg);
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
  friend class common::Singleton<LidarAFOutput>;
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
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::PointCloud2>> cloud_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Packets>> packets_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::PointClouds2>> fusion_clouds_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc

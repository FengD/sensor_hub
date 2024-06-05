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
#include "sensor_msg/msg/image.hpp"
#include "sensor_msg/msg/perception_obstacles.hpp"
#include "sensor_msg/msg/marker_list.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace crdc {
namespace airi {

class TypeConverAFOutput {
 public:
  TypeConverAFOutput() = default;
  virtual ~TypeConverAFOutput() = default;

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
    percption_obstacles_writer_ptr_.reset(
          new AFChannelWriter<sensor_msg::msg::PerceptionObstacles>(node_));
    percption_markerlist_writer_ptr_.reset(
          new AFChannelWriter<sensor_msg::msg::MarkerList>(node_));
    image_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::Image>(node_));
    percption_image_writer_ptr_.reset(
          new AFChannelWriter<sensor_msgs::msg::Image>(node_));
    marker_array_writer_ptr_.reset(
          new AFChannelWriter<visualization_msgs::msg::MarkerArray>(node_));
    marker_writer_ptr_.reset(
          new AFChannelWriter<visualization_msgs::msg::Marker>(node_));
    polygon_stamped_writer_ptr_.reset(
          new AFChannelWriter<geometry_msgs::msg::PolygonStamped>(node_));
    cloud_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::PointCloud2>(node_));
    encode_image_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::CompressedImage>(node_));
    return true;
  }

  bool write_polygon_stamped(const std::string& topic, geometry_msgs::msg::PolygonStamped& msg) {
    return polygon_stamped_writer_ptr_->write(topic, msg);
  }

  bool write_marker(const std::string& topic, visualization_msgs::msg::Marker& msg) {
    return marker_writer_ptr_->write(topic, msg);
  }

  bool write_image(const std::string& topic, sensor_msg::msg::Image& msg) {
    return image_writer_ptr_->write(topic, msg);
  }

  bool write_marker_array(const std::string& topic, visualization_msgs::msg::MarkerArray& msg) {
    return marker_array_writer_ptr_->write(topic, msg);
  }

  bool write_obstacles(const std::string& topic, sensor_msg::msg::PerceptionObstacles& msg) {
    msg.header.timestamp_sec = static_cast<double>(get_now_microsecond()) / 1000000;
    return percption_obstacles_writer_ptr_->write(topic, msg);
  }

  bool write_markerlists(const std::string& topic, sensor_msg::msg::MarkerList& msg) {
    return percption_markerlist_writer_ptr_->write(topic, msg);
  }

  bool write_images(const std::string& topic, sensor_msgs::msg::Image& msg) {
    return percption_image_writer_ptr_->write(topic, msg);
  }

  bool write_encodeimage(const std::string& topic, sensor_msgs::msg::CompressedImage& msg) {
    return encode_image_writer_ptr_->write(topic, msg);
  }
  /**
   * @brief Send the cloud message by topic
   * @param topic name
   * @param proto cloud ptr
   * @return status
   */
  bool write_cloud(const std::string& topic, sensor_msgs::msg::PointCloud2& cloud_msg) {
    cloud_msg.header.stamp.sec = get_now_microsecond() / 1000000;
    cloud_msg.header.stamp.nanosec = get_now_microsecond() % 1000000 * 1000;
    return cloud_writer_ptr_->write(topic, cloud_msg);
  }

  std::shared_ptr<rclcpp::Node> get_node() {
    return node_;
  }

 private:
  friend class common::Singleton<TypeConverAFOutput>;
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
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::PerceptionObstacles>>
          percption_obstacles_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::MarkerList>>
          percption_markerlist_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Image>> image_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::Image>>
          percption_image_writer_ptr_;
  std::shared_ptr<AFChannelWriter<visualization_msgs::msg::MarkerArray>>
          marker_array_writer_ptr_;
  std::shared_ptr<AFChannelWriter<visualization_msgs::msg::Marker>>
          marker_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::PolygonStamped>>
          polygon_stamped_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::PointCloud2>> cloud_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::CompressedImage>> encode_image_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc

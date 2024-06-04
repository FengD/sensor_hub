// Copyright (C) 2021 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Zilou Cao
// Description: parse ins data

#pragma once

#include <gflags/gflags.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <memory>
#include <vector>
#include "perception_framework/perception_framework.h"
#include "tools/databag_message_data_extractor/sensor/point_type.h"
#ifndef WITH_ROS2
#include "cyber/cyber.h"
#include "cyber/common/log.h"
#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_base.h"
#include "cyber/record/file/record_file_reader.h"
#else
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msg/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#endif
#ifndef WITH_ROS2
using PointCloud2 = crdc::airi::PointCloud2;
#else
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using CustomPointCloud2 = sensor_msg::msg::PointCloud2;
#endif

namespace crdc {
namespace airi {
namespace pcd {
class PointCloud {
 public:
    PointCloud() = default;
    PointCloud(const std::string &input_file, const std::string &save_path,
               const std::string &format, const std::vector<std::string> &channel_name);
    virtual ~PointCloud() = default;
    template <typename T>
    void save_pcd(const T &cloud);
#ifdef WITH_ROS2
    void save_ros2_pcd();
    template <typename T>
    void extract_points(const T &pcd2_msg, const std::string &channel_name);
    template <typename T>
    void extract_point_cloud(T &pcd2_msg, std::string channel_name, std::string timestamp);
#else
    void save();
    void transform_to_pcd(const std::shared_ptr<PointCloud2> &pcd2_msg,
                          const std::string &channel_name);
    void transform_to_pcd(const std::shared_ptr<PointCloud2> &pcd2_msg,
                                  const std::string &format,
                                  const std::string &file_name);
#endif
 private:
    std::string file_name_;
    std::string input_file_;
    std::string timestamp_;
    std::string save_path_;
    std::string type_;
    std::string format_;
    std::vector<std::string> channel_name_;
    const std::string underline_ = "_";
    const std::string pcd_extension_ = ".pcd";
    const std::string sensor_msg_type_ = "sensor_msgs/msg/PointCloud2";
    const std::string custom_msg_type_ = "sensor_msg/msg/PointCloud2";
    const std::string format_ascii_ = "ascii";
    const std::string format_binary_ = "binary";
    const std::string file_separator_ = "/";
    const std::string channel_type_lidar_ = "LIDAR";
    const std::string channel_type_radar_ = "RADAR";
    std::unordered_map<std::string, std::string> topic_and_type;
    std::string result_str_;
#ifdef WITH_ROS2
    sensor_msg::msg::PointCloud2 lidar_message_custom;
    sensor_msgs::msg::PointCloud2 lidar_message;
    geometry_msgs::msg::PoseStamped adu_msg_;
#endif
};

}  // namespace pcd
}  // namespace airi
}  // namespace crdc

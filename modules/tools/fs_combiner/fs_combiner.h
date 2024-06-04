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
#include <boost/filesystem.hpp>
#include "camera_drivers/proto/camera_component_config.pb.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "common/common.h"
#include "ins_drivers/parser/parser.h"
#include "lidar_drivers/parser/parser.h"
#include "ins_drivers/proto/ins_component_config.pb.h"
#include "ins_drivers/proto/ins_config.pb.h"
#include "lidar_drivers/compensator/compensator.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#ifndef WITH_ROS2
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "cyber/sensor_proto/image.pb.h"
#include "cyber/sensor_proto/ins.pb.h"
#include "cyber/sensor_proto/perception_obstacle.pb.h"
#include "cyber/sensor_proto/header.pb.h"
#include "cyber/sensor_proto/geometry.pb.h"
#else
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msg/msg/point_clouds2.hpp"
#include "sensor_msg/msg/packets.hpp"
#include "sensor_msg/msg/packet.hpp"
#include "sensor_msg/msg/point_field.hpp"
#include "sensor_msg/msg/ins.hpp"
#include "tools/fs_combiner/image_decode_fs.h"
#endif

#ifdef WITH_ROS2
using Packet = sensor_msg::msg::Packet;
using Packets = sensor_msg::msg::Packets;
#endif

namespace crdc {
namespace airi {

class FSCombiner {
 public:
  FSCombiner() = default;
  virtual ~FSCombiner() = default;
  void write_packet(const std::string& path, const std::string& path_lidar);

 private:
#ifndef WITH_ROS2
  void extract_cloud(const crdc::airi::Packets& packets,
                    apollo::cyber::record::RecordWriter& writer,
                    const std::shared_ptr<crdc::airi::LidarParser>& lidar_parser,
                    const std::string& channel_name, const std::string& frame_id);

  void extract_ins(const crdc::airi::Packets& packets,
                   apollo::cyber::record::RecordWriter& writer,
                   const std::shared_ptr<crdc::airi::InsParser>& ins_parser,
                   const std::string& channel_name, const std::string& frame_id);

  void extract_packet(const std::string& input_packet_path,
                    const std::string& save_pcaket_path);

#else
  void extract_ros2packet(const std::string& input_packet_path, const std::string& input_packet_path_lidar,
                                 const std::string& save_pcaket_path);
  template <typename T>
  void deserialized_msg_to_bag(rosbag2_cpp::Writer& writer, std::string& channel_type,
                               std::string& channel_name, T& message,
                               rcutils_time_point_value_t& time_stamp);

  template <typename T>                             
  void timestamp_align(rosbag2_cpp::Reader& reader_lidar, rosbag2_cpp::Writer& writer, auto bag_message,
                                std::unordered_map<std::string, std::string> topic_and_type_lidar,
                                std::unordered_map<std::string, std::string> topic_and_type, 
                                T message, std::string topic_name);
  std::string path_;
  bool deflag_;
  bool play_save_flag_;
  int diff_timestamp_thres_;
  std::string ispacket_;
  std::string record_type_;
  cv::Mat process_img_ld_;
  cv::Mat process_img_fs_;
  std::vector<sensor_msgs::msg::CompressedImage> msgs_fs_mask_;
  std::vector<geometry_msgs::msg::PolygonStamped> msgs_fs_visu_;

  sensor_msg::msg::Packets packets_msg_;
  sensor_msg::msg::Image image_msg_;
  sensor_msg::msg::Image encodeframe_;
  sensor_msg::msg::MarkerList ld_perception_out_;
  sensor_msg::msg::PerceptionObstacles fs_perception_out_;
  sensor_msg::msg::Image ld_process_image_msg_;
  sensor_msg::msg::Image fs_process_image_msg_;

  sensor_msgs::msg::Image image_msgs_;
  sensor_msgs::msg::CompressedImage pressed_msg_;
  sensor_msgs::msg::Image encodeframes_;
  sensor_msgs::msg::CompressedImage ld_process_image_msgs_;
  sensor_msgs::msg::CompressedImage fs_process_image_msgs_;
  visualization_msgs::msg::Marker ld_marker_msgs_;
  geometry_msgs::msg::PolygonStamped fs_polygon_msg_;
  geometry_msgs::msg::PolygonStamped fs_perc_polygon_msg_;
  geometry_msgs::msg::PolygonStamped fs_fusion_lidar_msg_;
  geometry_msgs::msg::PolygonStamped fs_fusion_camera_msg_;
  std::shared_ptr<crdc::airi::ImageDecodeFS> image_decode_;
#endif
  std::shared_ptr<crdc::airi::LidarParser> init_lidar_parser(
      const std::string& lidar_config_path, std::string frame_id);

  std::shared_ptr<crdc::airi::InsParser> init_ins_parser(
      const std::string& ins_config_path, std::string frame_id);

  void creat_folder(const std::string& folder_path);

  Compensator compensator_;
};

}  // namespace airi
}  // namespace crdc

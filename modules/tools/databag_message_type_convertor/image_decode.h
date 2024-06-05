// Copyright (C) 2021 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: Decode Image

#pragma once

#include <stdio.h>
#include <stdlib.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
#include <libavutil/pixdesc.h>
}

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <map>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include "camera_drivers/proto/camera_component_config.pb.h"
#include "camera_drivers/proto/camera_config.pb.h"
#include "common/common.h"
#ifdef WITH_ROS2
#include "tools/databag_message_type_convertor/af_output.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "sensor_msg/msg/image.hpp"
#include "sensor_msg/msg/sensor_header.hpp"
#include "sensor_msg/msg/marker_list.hpp"
#include "sensor_msg/msg/perception_obstacles.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#endif

namespace crdc {
namespace airi {
#ifdef WITH_ROS2

class ImageDecode {
 public:
  ImageDecode() = default;
  virtual ~ImageDecode() = default;
  void write_packet(const std::string &path);

  void decode(rclcpp::SerializedMessage& extracted_serialized_msg, std::string topic_name,
              sensor_msgs::msg::Image& decode_msg, std::string path, bool& deflag,
              sensor_msgs::msg::CompressedImage& encodeframes);
  void decode(rclcpp::SerializedMessage& extracted_serialized_msg, std::string topic_name,
              sensor_msg::msg::Image& decode_msg, std::string path, bool& deflag,
              sensor_msg::msg::Image& encodeframe);
  template <typename T>
  void decode_h264frame(std::string& image_name, T& decode_msg, bool& deflag);
  template <typename T>
  void decode_cvframe(std::string topic_name, cv::Mat& image_jpeg_,
                      std::string& image_name, T& decode_msg, bool& deflag);
  void imagepublish(sensor_msg::msg::Image& image_msg_, cv::Mat& process_img_,
                    std::string channel_name, sensor_msg::msg::Image& encode_msg);
  void imagepublish(sensor_msgs::msg::CompressedImage& image_msg_, cv::Mat& process_img_,
                    std::string channel_name, sensor_msgs::msg::CompressedImage& encode_msg);
  void ffmpeg_init();
  void DecodeInit();
  void get_img_ld(cv::Mat& img_ld);
  void get_img_fs(cv::Mat& img_fs);
  void clear_image_map();

 private:
  AVCodec *codec;
  AVCodecContext *context;
  AVFrame *frame;
  AVPacket *packet;
  std::string path_;
  std::string save_mask_;
  std::map<std::string, cv::Mat> ld_image_map_;
  std::map<std::string, cv::Mat> fs_image_map_;
  std::map<std::string, cv::Mat> ld_mask_map_;
  std::map<std::string, cv::Mat> fs_mask_map_;
  cv::Mat process_img_ld_;
  cv::Mat process_img_fs_;
  cv::Mat rear_image_crop_;
  cv::Mat decode_ld_mask_8uc1_;
  cv::Mat decode_fs_mask_8uc1_;
  std::vector<int32_t> params_;
  cv::Mat ld_filter_mask;
};
#endif

}  // namespace airi
}  // namespace crdc

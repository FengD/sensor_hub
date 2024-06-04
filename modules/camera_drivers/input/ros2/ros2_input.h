#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include "common/common.h"
#include "camera_drivers/input/input.h"
#ifdef WITH_ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#endif


namespace crdc {
namespace airi {

class Ros2Input : public CameraInput {
 public:
  Ros2Input() = default;
  virtual ~Ros2Input() = default;
  bool camera_init() override;
  bool camera_start() override;
  bool camera_stop() override;
  bool get_topic_name(std::string* topic_name,
                      std::shared_ptr<const CameraRawData>& raw_data) override;

 private:
  std::unordered_map<std::string, std::string> topic_and_type_;
  common::ThreadSafeQueue<std::string> ros2_topic_queue_;
#ifdef WITH_ROS2
  rosbag2_cpp::Reader reader_;
  sensor_msgs::msg::Image sensor_msgs_image_;
  sensor_msgs::msg::CompressedImage sensor_msgs_compressed_image_;
#endif
  void bgr_to_nv12(cv::Mat& bgr_image, cv::Mat& nv12_image);
  void parse_rosbag();
  void get_data();
};

}  // namespace airi
}  // namespace crdc

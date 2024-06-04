#pragma once

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pcl/common/common.h>
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "tools/service_server/data_encode_decode.h"
#include "tools/service_server/calib_utils.h"
#ifdef WITH_ROS2
using Marker = visualization_msgs::msg::Marker;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
#endif

namespace crdc {
namespace airi {

class DynamicCalibration {
 public:
  DynamicCalibration();
  virtual ~DynamicCalibration() = default;

  void init(const std::string &frame_id);

  void process(std::shared_ptr<LidarPointCloud> cloud);

  std::string name() const {
    return "DynamicCalibration";
  }

  void receive_boundary_marker(Marker boundary_msg);

  void af_spin(std::shared_ptr<rclcpp::Node> node);

  void write_to_prototxt(CalibrationConfig &lidar_config);

  std::string get_save_path();

  int get_process_state();

  void reset_param();

 private:
  std::string current_time_str_, result_save_path_, file_save_prefix_;
  std::shared_ptr<CalibrationUtils> calib_utils_;
  void convert_pcl_to_image(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  void parse_point_to_pcl_cloud(std::shared_ptr<PointCloud2> &pcd2_msg,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  void get_the_calibrated_cloud(std::shared_ptr<PointCloud2> &cloud_msg, float y_correction);
  cv::Mat rotatedImage;
  pcl::PointXYZI point_;
  std::shared_ptr<DataEncoderDecoder> data_converter_;
  std::shared_ptr<rclcpp::Node> node_;
  std::thread thread_af_spin_;
  std::shared_ptr<rclcpp::Subscription<Marker>> subscription_;
  std::shared_ptr<LidarPointCloud> cloud_;
  std::string cali_path_;
  CalibrationConfig lidar_config_;
  LiDynamicCalibrateConfig calibrate_config_;
  std::vector<double> yaw_array, b_array;
  int current_frame_num_;
  int count_num_;
  int stop_frame_nums_;
  int process_state_;
  float y_current_;
  float y_average_;
  float b_current_;
  float b_total_;
  float b_average_;
  float oldyawangle_;
  float newyawangle_;
  float range_;
  bool flag_;
};

}  // namespace airi
}  // namespace crdc
